/*
 * Frontier Tranzport to Ardour interface using OSC, stuart@ashbysoft.com, Aug 2024
 * Cmd syntax to match Ardour 6 release
 * inspired by the work of the Ardour team regarding the mapping and display
 * used in the native integration (which seems no longer available in Ubuntu?)
 *
 * might consider externalising the cmd syntax to fit with other DAWs?
 *
 * Based on earlier work:
 * tranzport 0.1 <tranzport.sf.net>
 * oct 18, 2005
 * arthur@artcmusic.com
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <usb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <time.h>

// ** config
// OSC destination
#define OSC_SERVER_HOST "127.0.0.1"
#define OSC_SERVER_PORT 3819
#define OSC_SRC_PORT 48909

// ** protocol

// Tranzport USB configutation
#define VENDORID  0x165b
#define PRODUCTID 0x8101

#define READ_ENDPOINT  0x81
#define WRITE_ENDPOINT 0x02

#define USB_WRITE_TIMEOUT 200 // ms - should not be reached unless USB is overloaded/down
#define USB_READ_TIMEOUT 50 // ms - controls polling loop speed, needs longer at startup to establish radio channel?
#define USB_READ_TIMEOUT_ERR -110 // we should expect this code during read polling, it's not an error

#define MIN_CTRL_INTERVAL_SEC 0.25 // throttle the rate of gain, pan messages to avoid crashing either end

// Tranzport native message syntax:
/*
 * 8-byte messages. 
 * Payload syntax for recv is: <B.?><B.status><4B.bigendian.buttons><B.datawheel><B.?>
 * buttons state is stored in a 32-bit word. Msg is sent on each state change. Might be possible that more than one changed?
 * datawheel value is 7-bit signed int. Msg is sent at max rate whilst moving and indicates how many clicks up/down since last msg
 *
 * Payload for send is: <B.0><B.[0=light|1=lcd]><B.address>[<B.[1=on|0=off]><3B.0>|<4B.chars>]<B.0>
 * where: address is either the light enum or the lcd cell position [0-9]. Cells are arranged 5 x 2 from top left.
 */
enum {
	LIGHT_RECORD = 0,
	LIGHT_TRACKREC,
	LIGHT_TRACKMUTE,
	LIGHT_TRACKSOLO,
	LIGHT_ANYSOLO,
	LIGHT_LOOP,
	LIGHT_PUNCH
};

#define BUTTONMASK_BATTERY     0x00004000
#define BUTTONMASK_BACKLIGHT   0x00008000
#define BUTTONMASK_TRACKLEFT   0x04000000
#define BUTTONMASK_TRACKRIGHT  0x40000000
#define BUTTONMASK_TRACKREC    0x00040000
#define BUTTONMASK_TRACKMUTE   0x00400000
#define BUTTONMASK_TRACKSOLO   0x00000400
#define BUTTONMASK_UNDO        0x80000000
#define BUTTONMASK_IN          0x02000000
#define BUTTONMASK_OUT         0x20000000
#define BUTTONMASK_PUNCH       0x00800000
#define BUTTONMASK_LOOP        0x00080000
#define BUTTONMASK_PREV        0x00020000
#define BUTTONMASK_ADD         0x00200000
#define BUTTONMASK_NEXT        0x00000200
#define BUTTONMASK_REWIND      0x01000000
#define BUTTONMASK_FASTFORWARD 0x10000000
#define BUTTONMASK_STOP        0x00010000
#define BUTTONMASK_PLAY        0x00100000
#define BUTTONMASK_RECORD      0x00000100
#define BUTTONMASK_SHIFT       0x08000000

#define STATUS_OFFLINE 0xff
#define STATUS_ONLINE  0x01

// ** end of protocol

// ** types
// tranzport USB connection handle
struct tranzport_s {
	struct usb_device *dev;
	usb_dev_handle *udev;
};

typedef struct tranzport_s tranzport_t;

// osc socket handle
struct osc_s {
	struct sockaddr_in addr;
	int sock;
};

typedef struct osc_s osc_t;

// osc transport state
struct txprt_s {
	int record;
	int loop;
	float jumps; // accumulates datawheel movements during rate limiting
	float jogs;
	char last_bbt[9]; // store in LCD format '000|00  ' only update LCD if changed to limit rate
	double next_jump_sec_out; // rate limit osc output
	double next_jog_sec_out;
};

typedef struct txprt_s txprt_t;

// osc track state
#define MIN_GAIN -50.0 	// dB
#define MAX_GAIN 6.0
#define MIN_PAN 0.0	// 0-1
#define MAX_PAN 1.0

struct track_s {
	char name[64];
	int recenable;
	int mute;
	int solo;
	float gain;
	float pan;
	double next_gain_sec_in; // limit recv rate from osc
	double next_pan_sec_in;
	double next_gain_sec_out; // limit send rate to osc
	double next_pan_sec_out;
};

typedef struct track_s track_t;

// ** functions
// declarations
int update_strip(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int update_position(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int update_record(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int update_loop(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int update_all_solos(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int update_heartbeat(tranzport_t *z, char *msg, char *fmt, unsigned char *val);
int hello_lcd(tranzport_t *z);

// utilities
void log_entry(FILE *fp, char *format, va_list ap)
{
	vfprintf(fp, format, ap);
	fputc('\n', fp);
}

void log_error(char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	log_entry(stderr, format, ap);
	va_end(ap);
}

int debug_lvl = 0;
void debug(int lvl, char *format, ...)
{
	if(lvl>debug_lvl) return;
	va_list ap;
	va_start(ap, format);
	log_entry(stderr, format, ap);
	va_end(ap);
}

void vlog_error(char *format, va_list ap)
{
	log_entry(stderr, format, ap);
}

void die(char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	vlog_error(format, ap);
	va_end(ap);
	exit(1);
}

float getInt(unsigned char *val) {
  // convert from big-endian (network btye order)
  const int32_t i = (int32_t)ntohl(*((uint32_t *) val));
  return i;
}

float getFloat(unsigned char *val) {
  // convert from big-endian (network btye order)
  const uint32_t i = ntohl(*((uint32_t *) val));
  return *((float *) (&i));
}

int too_soon(double *next_sec) {
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double now_sec = now.tv_sec + now.tv_nsec / 1000000000.0;
	debug(3,"now:%lf, nxt:%lf",now_sec,*next_sec);
	int b = now_sec > *next_sec ? 1 : 0;
	if(b) *next_sec = now_sec + MIN_CTRL_INTERVAL_SEC;
	return !b;
}

// OSC
void osc_send(osc_t *osc, char *osc_addr, char *osc_type, void *val) {
	char msg[32];
	size_t l;
	
	memset(msg, 0, 32);
	
	// insert addr
	if(osc_addr==NULL) { log_error("no osc_addr"); return; }
	l = strlen(osc_addr);
	if(l>31) { log_error("osc_addr too long:%s",osc_addr); return; } // need space for the NULL!
	memcpy(msg, osc_addr, l);
	l = (l + 4) & ~0x3; // round up to next multiple of 4 boundary
	
	// if there is a value, insert type and val
	if(osc_type!=NULL) {
		size_t tl = strlen(osc_type);
		if(l+tl > 30) { log_error("osc_type too long:%s",osc_type); return; } // need space for , and NULL
		msg[l++] = ',';
		memcpy(msg+l,osc_type,tl);
		l = (l + tl + 4) & ~0x3; // round up again
		// TODO insert something other than int and float! also multiple values!
		switch(osc_type[0]) {
			case 'i': {
				*((uint32_t *) (msg+l)) = htonl(*(int *)val);
				l+=4;
				break;
			}
			case 'f': {
				*((uint32_t *) (msg+l)) = htonl(*((uint32_t *)val));
				l+=4;
				break;
			}
			default: {
				log_error("osc unknown format:%s",osc_type);
				return;
			}
		}
	}
	
	// send the msg
	debug(1,"osc_send l:%ld txt=%s", l, msg);
	for(int i=0; i<l; i++) { debug(3," %2.2x",(uint8_t)msg[i]); }
	sendto(osc->sock, msg, l, 0, (struct sockaddr *)&osc->addr, sizeof(osc->addr));
}

void do_osc_init(osc_t *osc) {
	// /set_surface to configure feedback we want
	// no banks, strip types audio+midi tracks+busses+control+master, feedback buttons+controls+master+bbt+ssid in address, gain in db. Dont change anything else!
	int i = 0;
	char *config = "/set_surface/0/63/63";
	debug(1,"osc init: %s",config);
	osc_send(osc, config, "i", &i);
}

int do_osc_input(tranzport_t *z, osc_t *osc) {
	// poll socket and process any packets
	char msg[64];
	char *fmt;
	int r, err = 0;
	unsigned char *val;
	
	while ((r = recv(osc->sock, msg, 64, MSG_DONTWAIT)) > 0) {
		debug(2,"osc_recv l=%d txt=%s ",r, msg);
		for(int i=0; i<r; i++) { debug(3," %2.2x",(uint8_t)msg[i]); }
		// parse msg - TODO multiple values
		// locate format str after ','
		int p = 0;
		while(p < r) {
			debug(3,"a%d=%2.2x",p,msg[p]);
			if(','==msg[p++]) 
				break;
		}
		if(p == r) {
			log_error("cannot find OSC format marker in msg: %s",msg);
			return -1;
		}
		fmt = msg+p;
		// step to end of fmt str
		while(p < r) {
			debug(3,"f%d=%2.2x",p,msg[p]);
			if('\0'==msg[p]) 
				break;
			p++;
		}
		// step to next 4 byte boundary to find val
		p = (p+4) & ~0x3;
		if(p >= r) {
			log_error("cannot find OSC data value in msg: %s",msg);
			return -1;
		}
		val = (unsigned char *)msg+p;
		debug(3,"v%d=%2.2x",p,*val);
		// check for string or binary - ONLY handling 4 byte vals for now!
		if('s'==*fmt) {
			debug(2,"s fmt=%s val=%s",fmt, val);
		} else if(debug_lvl>0) {
			char vstr[64];
			sprintf(vstr,"%2.2x %2.2x %2.2x %2.2x", *val, *(val+1), *(val+2), *(val+3));
			debug(2,"b fmt=%s val=%s",fmt,vstr);
		}
		// update track state
		if(strncmp("/strip",msg,6)==0)
			err += update_strip(z, msg, fmt, val);
		// update position lcd
		if(strncmp("/position",msg,9)==0)
			err +=  update_position(z, msg, fmt, val);
		// update other controls and stuff
		if(strncmp("/rec_enable_toggle",msg,18)==0)
			err +=  update_record(z, msg, fmt, val);
		if(strncmp("/loop_toggle",msg,18)==0)
			err +=  update_loop(z, msg, fmt, val);
		if(strncmp("/cancel_all_solos",msg,17)==0)
			err +=  update_all_solos(z, msg, fmt, val);
		if(strncmp("/heartbeat",msg,10)==0)
			err +=  update_heartbeat(z, msg, fmt, val);
		if(err<0) {
			// dump any remaining packets
			int dumped = 0;
			while(recv(osc->sock, msg, 64, MSG_DONTWAIT) > 0){dumped++;}
			debug(1,"tranzport write err, dumped %d osc pkts",dumped);
			return err; // quit early if there is an error
		}
	}
	return 0;
}

// tranzport
tranzport_t *open_tranzport_core(struct usb_device *dev)
{
	tranzport_t *z;
	int val;

	debug(1,"open USB devnum:%d path:%s",dev->devnum, dev->filename);
	z = malloc(sizeof(tranzport_t));
	if (!z)
		die("not enough memory");
	memset(z, 0, sizeof(tranzport_t));

	z->dev = dev;
	z->udev = usb_open(z->dev);
	if (!z->udev)
		die("unable to open tranzport");

	val = usb_claim_interface(z->udev, 0);
	if (val < 0)
		die("unable to claim tranzport");

	return z;
}

tranzport_t *open_tranzport()
{
	struct usb_bus *bus;
	struct usb_device *dev;

	usb_init();
	usb_find_busses();
	usb_find_devices();

	debug(2,"search for tranzport USBID:%x,%x",VENDORID,PRODUCTID);
	for(bus=usb_busses; bus; bus=bus->next) {
		for(dev=bus->devices; dev; dev=dev->next) {
			if (dev->descriptor.idVendor != VENDORID)
				continue;
			if (dev->descriptor.idProduct != PRODUCTID)
				continue;

			return open_tranzport_core(dev);
		}
	}

	die("can't find tranzport");
	return 0;
}

void close_tranzport(tranzport_t *z)
{
	int val;

	val = usb_release_interface(z->udev, 0);
	if (val < 0)
		log_error("unable to release tranzport");

	val = usb_close(z->udev);
	if (val < 0)
		log_error("unable to close tranzport");

	free(z);
}

int tranzport_write_core(tranzport_t *z, uint8_t *cmd)
{
	int val;
	val = usb_interrupt_write(z->udev, WRITE_ENDPOINT, (char *)cmd, 8, USB_WRITE_TIMEOUT);
	if(val<0 || val!=8)
		debug(2,"USB write err:%d",val);
	if (val < 0)
		return val;
	if (val != 8)
		return -1;
	return 0;
}

int tranzport_lcdwrite(tranzport_t *z, uint8_t cell, char *text)
{
	uint8_t cmd[8];

	debug(3,"LCD cell=%d txt=%s",cell, text);
	if (cell > 9) {
		return -1;
	}

	cmd[0] = 0x00;
	cmd[1] = 0x01;
	cmd[2] = cell;
	cmd[3] = text[0];
	cmd[4] = text[1];
	cmd[5] = text[2];
	cmd[6] = text[3];
	cmd[7] = 0x00;

	return tranzport_write_core(z, cmd);
}

int tranzport_lighton(tranzport_t *z, uint8_t light)
{
	uint8_t cmd[8];

	debug(3,"lightOn %d",light);
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = light;
	cmd[3] = 0x01;
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;

	return tranzport_write_core(z, cmd);
}

int tranzport_lightoff(tranzport_t *z, uint8_t light)
{
	uint8_t cmd[8];

	debug(3,"lightOff %d",light);
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = light;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;

	return tranzport_write_core(z, cmd);
}

// variable timeout function
int tranzport_read_core(tranzport_t *z, uint8_t *status, uint32_t *buttons, uint8_t *datawheel, int timeout)
{
	uint8_t buf[8];
	int val;

	memset(buf, 0, 8);
	val = usb_interrupt_read(z->udev, READ_ENDPOINT, (char *)buf, 8, timeout);
	if(val!=USB_READ_TIMEOUT_ERR && val!=8)
		debug(2,"USB read err:%d",val);
	if (val < 0)
		return val;
	if (val != 8)
		return -1; // we didn't get all the bytes

	*status = buf[1];

	*buttons = 0;
	*buttons |= buf[2] << 24;
	*buttons |= buf[3] << 16;
	*buttons |= buf[4] << 8;
	*buttons |= buf[5];

	*datawheel = buf[6];

	return 0;
}

// fixed timeout function
int tranzport_read(tranzport_t *z, uint8_t *s, uint32_t *b, uint8_t *d) {
	return tranzport_read_core(z, s, b, d, USB_READ_TIMEOUT);
}

void do_tranzport_init(tranzport_t *z) {
	uint32_t b = 0;
	uint8_t s,d = 0;
	char c[] = "-\\|/";
	int x = 0;
	// wait for online status, then send hello_lcd
	log_error("Waiting for online status");
	do {
		tranzport_read_core(z, &s, &b, &d, 1000); // use longer timeout here
		fprintf(stderr,"Offline%c\r",c[x]); x = (x+1)%4;
	} while(s==STATUS_OFFLINE);
	// it's ready!
	log_error("Online! lets go..");
}

// Tranzport update logic
int track_selected = 0; // set from Ardour
int max_track = 1;
track_t track;
txprt_t txprt;
int ctrl = 0;

int update_strip(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	char strip_addr[32];
	char *tok;
	char txt[5];
	int r = 0;
	
	// skip /strip
	tok = strchr(msg+1,'/');
	debug(2,"strip: subaddr=%s",tok);

	// extract selected track - assumes ONLY ONE can be active at a time! And it's set first before other values are sent..
	if(strncmp("/select",tok,7)==0) {
		tok = strchr(tok+1,'/'); // skip '/'
		int ti = atoi(tok+1); // skip '/'
		float f = getFloat(val);
		debug(2,"select: trk=%s f=%f",tok,f);
		if(f>0) {
			track_selected=ti;
			sprintf(txt, "%02u  ", (uint8_t)track_selected);
			r += tranzport_lcdwrite(z, 0, "Trk:");
			r += tranzport_lcdwrite(z, 1, txt);
			if(r!=0) debug(1,"LCD Trk update failed");
			debug(1,"set track:%d",track_selected);
		}
		// update max track num
		if(ti>max_track) max_track=ti;
		return r;
	}
	if(track_selected == 0) // cannot process strip until we have the track selected
		return 0;
	// extract name & control states for selected track if we have one
	sprintf(strip_addr,"/name/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// set name
		memset(track.name,0,64);
		strncpy(track.name,(char *)val,63);
		r += tranzport_lcdwrite(z, 2, " ID:");
		for(int i=0; i<4; i++) {txt[i]=track.name[i]=='\0' ? ' ' : track.name[i]; }
		txt[5]='\0';
		r += tranzport_lcdwrite(z, 3, txt);
		for(int i=0; i<4; i++) {txt[i]=track.name[i+4]=='\0' ? ' ' : track.name[i+4]; }
		txt[5]='\0';
		r += tranzport_lcdwrite(z, 4, txt);
		if(r!=0) debug(1,"LCD ID update failed");
		debug(1,"set name:%s",track.name);
		return r;
	}
	sprintf(strip_addr,"/gain/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// throttle osc input rate by discarding messages
		if(too_soon(&track.next_gain_sec_in))
			return 0;
		// set gain
		char dtxt[9];
		memset(dtxt,0,9);
		track.gain = getFloat(val);
		sprintf(dtxt, "%+06.2f", track.gain); // split over two txt segments
		r += tranzport_lcdwrite(z, 5, "Vol:");
		for(int i=0; i<4; i++) {txt[i]=dtxt[i]=='\0' ? ' ' : dtxt[i]; }
		r += tranzport_lcdwrite(z, 6, txt);
		for(int i=0; i<4; i++) {txt[i]=dtxt[i+4]=='\0' ? ' ' : dtxt[i+4]; }
		r += tranzport_lcdwrite(z, 7, txt);
		if(r!=0) debug(1,"LCD Vol update failed");
		debug(1,"set gain:%f",track.gain);
		return r;
	}
	sprintf(strip_addr,"/pan_stereo_position/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// throttle osc input rate by discarding messages
		if(too_soon(&track.next_pan_sec_in))
			return 0;
		// set pan
		track.pan = getFloat(val);
		sprintf(txt, "%04.2f", track.pan);
		r += tranzport_lcdwrite(z, 8, "Pan:");
		r += tranzport_lcdwrite(z, 9, txt);
		if(r!=0) debug(1,"LCD Pan update failed");
		debug(1,"set pan:%f",track.pan);
		return r;
	}
	sprintf(strip_addr,"/mute/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// set mute
		track.mute = getInt(val) > 0 ? 1 : 0;
		if(track.mute) r += tranzport_lighton(z, LIGHT_TRACKMUTE); else r += tranzport_lightoff(z, LIGHT_TRACKMUTE);
		if(r!=0) debug(1,"Light mute update failed");
		debug(1,"set mute:%d",track.mute);
		return r;
	}
	sprintf(strip_addr,"/solo/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// set solo
		track.solo = getInt(val) > 0 ? 1 : 0;
		if(track.solo) r += tranzport_lighton(z, LIGHT_TRACKSOLO); else r += tranzport_lightoff(z, LIGHT_TRACKSOLO);
		if(r!=0) debug(1,"Light solo update failed");
		debug(1,"set solo:%d",track.solo);
		return r;
	}
	sprintf(strip_addr,"/recenable/%d",track_selected);
	if(strcmp(strip_addr,tok)==0) {
		// set recenable
		track.recenable = getInt(val) > 0 ? 1 : 0;
		if(track.recenable) r += tranzport_lighton(z, LIGHT_TRACKREC); else r += tranzport_lightoff(z, LIGHT_TRACKREC);
		if(r!=0) debug(1,"Light recenable update failed");
		debug(1,"set recenable:%d",track.recenable);
		return r;
	}
	// unknown msg
	return 0;
}

int update_position(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	// check we have bbt format
	char *tok;
	char txt[5];
	int r = 0;
	
	tok = strchr(msg+1,'/');
	debug(2,"position: subaddr=%s",tok);

	// just print timecode to lcd..
	if(strncmp("/bbt",tok,4)==0) {
		val[6]=' ';val[7]=' '; val[8]='\0'; // discard the ticks part and add two spaces to clear the display
		// throttle rate by discarding messages with same bar|beat value
		if(strcmp(txprt.last_bbt,(char *)val)==0)
			return 0;
		strcpy(txprt.last_bbt,(char *)val);
		r += tranzport_lcdwrite(z, 2, "Pos:");
		for(int i=0; i<4; i++) {txt[i]=val[i]=='\0' ? ' ' : val[i]; }
		txt[5]='\0';
		r += tranzport_lcdwrite(z, 3, txt);
		for(int i=0; i<4; i++) {txt[i]=val[i+4]=='\0' ? ' ' : val[i+4]; }
		txt[5]='\0';
		r += tranzport_lcdwrite(z, 4, txt);
		if(r!=0) debug(1,"LCD Pos update failed");
		debug(1,"set pos:%s",val);
		return r;
	}	
	// unknown msg
	return 0;
}

int update_record(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	int r = 0;
	// just turn on/off light for now
	if(getFloat(val)>0) r += tranzport_lighton(z, LIGHT_RECORD); else r += tranzport_lightoff(z, LIGHT_RECORD);
	if(r!=0) debug(1,"Light record update failed");
	return r;
}

int update_loop(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	int r = 0;
	// just turn on/off light for now
	if(getFloat(val)>0) r += tranzport_lighton(z, LIGHT_LOOP); else r += tranzport_lightoff(z, LIGHT_LOOP);
	if(r!=0) debug(1,"Light loop update failed");
	return r;
}

int update_all_solos(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	int r = 0;
	// just turn on/off light for now
	if(getFloat(val)>0) r += tranzport_lighton(z, LIGHT_ANYSOLO); else r += tranzport_lightoff(z, LIGHT_ANYSOLO);
	if(r!=0) debug(1,"Light anysolo update failed");
	return r;
}

int update_heartbeat(tranzport_t *z, char *msg, char *fmt, unsigned char *val) {
	char txt[5];
	int r = 0;
	// add pulse to track selected field
	sprintf(txt, "%02u %c", (uint8_t)track_selected, getFloat(val)>0?0x1f:0x16);
	r += tranzport_lcdwrite(z, 1, txt);	
	if(r!=0) debug(1,"LCD heartbeat update failed");
	return r;
}

int hello_lcd(tranzport_t *z)
{
	int r = 0;
	r += tranzport_lcdwrite(z, 0, "   *");
	r += tranzport_lcdwrite(z, 1, "Ashb");
	r += tranzport_lcdwrite(z, 2, "ysof");
	r += tranzport_lcdwrite(z, 3, "t   ");
	r += tranzport_lcdwrite(z, 4, "    ");

	r += tranzport_lcdwrite(z, 5, "    ");
	r += tranzport_lcdwrite(z, 6, "Welc");
	r += tranzport_lcdwrite(z, 7, "ome!");
	r += tranzport_lcdwrite(z, 8, "    ");
	r += tranzport_lcdwrite(z, 9, "    ");
	if(r!=0) debug(1,"LCD hello update failed");
	return r;
}

// OSC update logic
void buttons_core(osc_t *osc, uint32_t buttons, uint32_t buttonmask, char *osc_addr, char *osc_type, void *val, char *str)
{
	if (buttons & buttonmask)
		if(osc_addr!=NULL) {
			fprintf(stderr,"%16.16s\r",str);
			osc_send(osc, osc_addr, osc_type, val);
		}
}

void do_track_step(osc_t *osc, uint32_t buttons) {
	int i = 1;
	char track_addr[32];
	
	if(buttons & BUTTONMASK_TRACKLEFT) track_selected--;
	if(buttons & BUTTONMASK_TRACKRIGHT) track_selected++;
	if ( track_selected < 1) track_selected = 1;
	if ( track_selected > max_track) track_selected = max_track;
	sprintf(track_addr,"/strip/select/%d",track_selected);
	buttons_core(osc, buttons, BUTTONMASK_TRACKLEFT | BUTTONMASK_TRACKRIGHT, track_addr, "i", &i, "trackchange");
}

void do_stateful_buttons(tranzport_t *z, osc_t *osc, uint32_t buttons) {
	int r = 0;
	// Punch - use as edit ctrl on/off for track
	if(buttons&BUTTONMASK_PUNCH) {
		ctrl=!ctrl;
		debug(1,"edit ctrl :%d",ctrl);
		if(ctrl) r += tranzport_lighton(z, LIGHT_PUNCH); else r += tranzport_lightoff(z, LIGHT_PUNCH);
		if(r!=0) debug(1,"Light punch update failed");
	}
	
	// track rec arm
	if(buttons&BUTTONMASK_TRACKREC) {
		track.recenable=!track.recenable;
		debug(1,"recenable:%d",track.recenable);
		buttons_core( osc, buttons, BUTTONMASK_TRACKREC, "/select/recenable", "i", &track.recenable, "trackrec");
	}
	// track mute
	if(buttons&BUTTONMASK_TRACKMUTE) {
		track.mute=!track.mute;
		debug(1,"mute:%d",track.mute);
		buttons_core( osc, buttons, BUTTONMASK_TRACKMUTE, "/select/mute", "i", &track.mute, "trackmute");
	}
	// track solo
	if(buttons&BUTTONMASK_TRACKSOLO) {
		// hacky! shifted this does /cancel_all_solos
		if(buttons&BUTTONMASK_SHIFT) {
			int i = 1;
			buttons_core( osc, buttons, BUTTONMASK_TRACKSOLO, "/cancel_all_solos", "i", &i, "cancelsolos");
		} else {
			track.solo=!track.solo;
			debug(1,"solo:%d",track.solo);
			buttons_core( osc, buttons, BUTTONMASK_TRACKSOLO, "/select/solo", "i", &track.solo, "tracksolo");
		}
	}
}

void do_buttons(tranzport_t *z, osc_t *osc, uint32_t buttons, uint8_t datawheel)
{
	int i = 1;

	debug(1,"buttons:");
	// dynamic button logic
	do_track_step( osc, buttons);
	do_stateful_buttons(z, osc, buttons);
	if(buttons&BUTTONMASK_SHIFT) {
		// shifted button actions
		buttons_core( osc, buttons, BUTTONMASK_UNDO, "/redo", "i", &i, "redo");
		buttons_core( osc, buttons, BUTTONMASK_ADD, "/remove_marker", "i", &i, "unmark");
		buttons_core( osc, buttons, BUTTONMASK_RECORD, "/save_state", "i", &i, "save");
		buttons_core( osc, buttons, BUTTONMASK_REWIND, "/goto_start", "i", &i, "goto start");
		buttons_core( osc, buttons, BUTTONMASK_FASTFORWARD, "/goto_end", "i", &i, "goto end");
		buttons_core( osc, buttons, BUTTONMASK_STOP, "/toggle_click", "i", &i, "click");
		if(ctrl) {
			// track edit ctrl enabled
			if (datawheel) {
				float d = ((signed char)datawheel) / 20.0f;
				track.pan -= d; // maps physical ctrl direction 
				if(track.pan > MAX_PAN) track.pan = MAX_PAN; // clamp movement
				if(track.pan < MIN_PAN) track.pan = MIN_PAN;
				if(too_soon(&track.next_pan_sec_out)) {
					return; // don't send it yet
				}
				fprintf(stderr,"      pan=%+06.2f\r", track.pan);
				osc_send(osc, "/select/pan_stereo_position", "f", &track.pan);
			}
		} else {
			if (datawheel) {
				float d = ((signed char)datawheel) / 20.0f;
				txprt.jogs += d;
				if(too_soon(&txprt.next_jog_sec_out)) {
					return; // don't send it yet
				}
				// send and clear
				fprintf(stderr,"      jog=%+06.2f\r", txprt.jogs);
				osc_send(osc, "/jog", "f", &txprt.jogs);
				txprt.jogs = 0.0;
			}
		}
	} else {
		// unshifted button actions
		buttons_core( osc, buttons, BUTTONMASK_BATTERY, NULL, NULL, NULL, "battery");
		buttons_core( osc, buttons, BUTTONMASK_BACKLIGHT, NULL, NULL, NULL, "backlight");
		buttons_core( osc, buttons, BUTTONMASK_UNDO, "/undo", "i", &i, "undo");
		buttons_core( osc, buttons, BUTTONMASK_IN, "/toggle_punch_in", "i", &i, "in");
		buttons_core( osc, buttons, BUTTONMASK_OUT, "/toggle_punch_out", "i", &i, "out");
		buttons_core( osc, buttons, BUTTONMASK_LOOP, "/loop_toggle", "i", &i, "loop");
		buttons_core( osc, buttons, BUTTONMASK_PREV, "/prev_marker", "i", &i, "prev");
		buttons_core( osc, buttons, BUTTONMASK_ADD, "/add_marker", "i", &i, "mark");
		buttons_core( osc, buttons, BUTTONMASK_NEXT, "/next_marker", "i", &i, "next");
		buttons_core( osc, buttons, BUTTONMASK_REWIND, "/rewind", "i", &i, "rewind");
		buttons_core( osc, buttons, BUTTONMASK_FASTFORWARD, "/ffwd", "i", &i, "fastforward");
		buttons_core( osc, buttons, BUTTONMASK_STOP, "/transport_stop", "i", &i, "stop");
		buttons_core( osc, buttons, BUTTONMASK_PLAY, "/transport_play", "i", &i, "play");
		buttons_core( osc, buttons, BUTTONMASK_RECORD, "/rec_enable_toggle", "i", &i, "play");
		if(ctrl) {
			// track edit ctrl enabled
			if (datawheel) {
				float d = ((signed char)datawheel) / 2.0f;
				track.gain += d;
				if(track.gain > MAX_GAIN) track.gain = MAX_GAIN; // clamp movement
				if(track.gain < MIN_GAIN) track.gain = MIN_GAIN;
				if(too_soon(&track.next_gain_sec_out)) {
					return; // don't send it yet
				}
				fprintf(stderr,"      vol=%+06.2f\r", track.gain);
				osc_send(osc, "/select/gain", "f", &track.gain);
			}
		} else {
			if (datawheel) {
				float d = ((signed char)datawheel) / 1.0f;
				txprt.jumps += d;
				if(too_soon(&txprt.next_jump_sec_out)) {
					return; // don't send it yet
				}
				// send and clear
				fprintf(stderr,"      bar=%+06.2f\r", txprt.jumps);
				osc_send(osc, "/jump_bars", "f", &txprt.jumps);
				txprt.jumps = 0.0;
			}
		}
	}
}

// ** MAIN ***
int main(int argc, char **argv)
{
	// tranzport vars
	tranzport_t *z;
	uint8_t status;
	uint32_t buttons;
	uint8_t datawheel;
	int val;
	
	printf("Tranzport OSC bridge starting..\n\n");
	// debug
	if(argc>1) {
		debug_lvl = atoi(argv[1]);
		log_error("debug_lvl: %d",debug_lvl);
	}
	
	// OSC vars
	osc_t osc;
	struct sockaddr_in src;

	// open USB endpoint
	debug(1,"open USB endpoint");
	z = open_tranzport();
	
	// open OSC socket
	debug(1,"open UDP socket");
	osc.sock = socket(AF_INET,SOCK_DGRAM,0);
	
	if(osc.sock<0)
		die("cannot open OSC socket");

	// bind to local port no to stop Arbour seeing a new surface every time it runs!
	src.sin_family = AF_INET;
	src.sin_addr.s_addr = htonl(INADDR_ANY);
	src.sin_port = htons(OSC_SRC_PORT);
	debug(1,"bind socket port");
	if(bind(osc.sock, (struct sockaddr *)&src, sizeof(src)) < 0)
		die("cannot bind OSC socket to port");
	// set dest addr and port
	osc.addr.sin_family = AF_INET;
	osc.addr.sin_addr.s_addr = inet_addr(OSC_SERVER_HOST);
	osc.addr.sin_port = htons(OSC_SERVER_PORT);

	// prep tranzport and OSC surface
	debug(1,"init Tranzport");
	do_tranzport_init(z);
	while(hello_lcd(z)<0)
		do_tranzport_init(z); // write failed, do init again
	// clear edit/punch light as not controlled by Ardour
	tranzport_lightoff(z, LIGHT_PUNCH);
	debug(1,"init OSC");
	// reset rate limiters
	track.next_gain_sec_in = 0.0;
	track.next_pan_sec_in = 0.0;
	track.next_gain_sec_out = 0.0;
	track.next_pan_sec_out = 0.0;
	txprt.next_jump_sec_out = 0.0;
	txprt.next_jog_sec_out = 0.0;
	txprt.jumps = 0.0;
	txprt.jogs = 0.0;
	strcpy(txprt.last_bbt,"000|00  ");
	do_osc_init(&osc);

	for(;;) {
		// non-blocking read of OSC socket and blocking write to tranzport if data
		if(do_osc_input(z, &osc) < 0) {
			log_error("RESET! Be careful!");
			do_tranzport_init(z); // write failed, wait for online again
		}
		// blocking read of USB endpoint with short timeout
		val = tranzport_read(z, &status, &buttons, &datawheel);
		if (val < 0) {
			continue; // no data
		}
		
		if (status == STATUS_OFFLINE) {
			do_tranzport_init(z); // wait for online again
			continue; // no data
		}

		if (status == STATUS_ONLINE) {
			do_osc_init(&osc); // reset OSC when we come back online
		}
		// send to OSC only if we got data
		do_buttons(z, &osc, buttons, datawheel);
	}
	
	close(osc.sock);

	close_tranzport(z);

	return 0;
}

