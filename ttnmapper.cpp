/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello, world!", that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 *******************************************************************************/


#include <stdio.h>    // for printf()
#include <fcntl.h>    // for open()
#include <termios.h>  // for termios
#include <unistd.h>   // for write()
#include <string.h>   // for strlen()
#include <time.h>   // for nanosleep()
#include <stdlib.h>   // for exit()
#include <stdbool.h>  // for bool
#include <sys/time.h> // for struct timeval and gettimeofday()
#include <cmath>   //for isnan

#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>
#include <gps.h>
#define SENT_EVERY_X_MINUTES 2  /* 3 minimum for SF12 (5 recommended), 1 minimum for SF7 (2 recommended) */

#include "my_config.h" // do not forget to add this file, see the README.md

struct gps_data_t my_gps_data;

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

u4_t cntr=0;
u1_t mydata[] = {"Hello, world!                               "};
static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins = {
  .nss = 6,
  .rxtx = UNUSED_PIN, // Not connected on RFM92/RFM95
  .rst = 0,  // Needed on RFM92/RFM95
  .dio = {7,4,5}
};

void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          fprintf(stdout, "Event EV_TXCOMPLETE, time: %d\n", millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              fprintf(stdout, "Data Received!\n");
          }
          break;
       default:
          break;
    }
}

void logMsg(const char *message)
{
  int fd = open("/var/log/lorawanmapper.log", O_WRONLY | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (fd == -1)
    printf("error opening log file\n");
  else {
    printf(message);
    write(fd, message, strlen(message));
    if (fsync(fd) == -1) {
      printf("fsync error\n");
    }
    close(fd);
    if (close(fd) == -1) {
      printf("close error\n");
    }
  }
}


void delay_ms(unsigned int ms)
{
  struct timespec ts;
  ts.tv_sec = 0;
  if (ms >= 200) {
    int counter = ms / 200;
    ts.tv_nsec = 100000000L;
    for (int i = 0; i < counter; i++) {
      nanosleep(&ts, (struct timespec *)NULL); // wait 100ms
    }
  } else {
    ts.tv_nsec = ms * 1000000L;
    nanosleep(&ts, (struct timespec *)NULL);
  }
}
bool init_gps(void)
{
  int rc;
  for (int i = 0; i < 10; i++) {
    rc = gps_open("localhost", "2947", &my_gps_data);
    if (rc != -1) {
      gps_stream(&my_gps_data, WATCH_ENABLE | WATCH_JSON, NULL);
      return true;
    }
    sleep(3);
  }
  printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
  return false;
}

void flush_gps_data(void)
{
  while (gps_read(&my_gps_data) > 0) {
    ;
  } 
}

bool fetch_gps(void)
{
  for (int i = 0; i < 3; i++) {
    int rc = gps_read(&my_gps_data);
    if (rc > 0) {
      while (rc > 0) {
        // flush old data
        rc = gps_read(&my_gps_data);
      }
      if ((my_gps_data.status == STATUS_FIX) &&
        (my_gps_data.fix.mode == MODE_3D) &&
        !std::isnan(my_gps_data.fix.time) &&
        !std::isnan(my_gps_data.fix.latitude) &&
        !std::isnan(my_gps_data.fix.longitude) &&
        !std::isnan(my_gps_data.fix.altitude) &&
        !std::isnan(my_gps_data.dop.hdop)) {
          return true;
        }
    }
    delay_ms(500);
  }
  return false;
}


static void do_send(osjob_t* j){
      time_t t=time(NULL);
      fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(&t));
      // Show TX channel (channel numbers are local to LMIC)
      // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      fprintf(stdout, "OP_TXRXPEND, not sending");
    } else {
      // Prepare upstream data transmission at the next possible time.
	      
      char buf[10];
      //sprintf(buf, "Hell[%d]", cntr++);
      if (fetch_gps()) {
        printf("GPS fix achieved\r\n");
      	int LatitudeBinary = ((my_gps_data.fix.latitude + 90) / 180.0) * 16777215;
	//printf("lat:%06x\n",LatitudeBinary&0xffffff);
      	int LongitudeBinary = ((my_gps_data.fix.longitude + 180) / 360.0) * 16777215;
	//printf("lon:%06x\n",LongitudeBinary&0xffffff);
      	int AltitudeBinary = my_gps_data.fix.altitude;
	//printf("alt:%04x\n",AltitudeBinary&0xffff);
      	int HdopBinary = my_gps_data.dop.hdop * 10.0;
	//printf("hdop:%02x\n",HdopBinary&0xff);
	//sprintf(&buf[0], "%06x%06x%04x%02x", LatitudeBinary & 0xFFFFFF, LongitudeBinary & 0xFFFFFF, AltitudeBinary & 0xFFFF, HdopBinary & 0xFF);
	buf[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  	buf[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  	buf[2] = LatitudeBinary & 0xFF;
  	buf[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  	buf[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  	buf[5] = LongitudeBinary & 0xFF;
  	buf[6] = ( AltitudeBinary >> 8 ) & 0xFF;
  	buf[7] = AltitudeBinary & 0xFF;
  	buf[8] = HdopBinary & 0xFF;
  	buf[9] = 0x00;
	
	printf("lat: %f, lon: %f, alt: %f, hdop: %f\n", my_gps_data.fix.latitude, my_gps_data.fix.longitude, my_gps_data.fix.altitude, my_gps_data.dop.hdop);
      	for (int jj=0;jj<10;jj++) {
        	mydata[jj]=buf[jj];
		printf("%#x ",buf[jj]);
	//	printf("|%02x |",buf[jj]);
      	}
	printf("\n");
      	LMIC_setTxData2(1, mydata, 9, 0);
    }else
    {
	    printf("No gps fix\n");
    }
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    os_setTimedCallback(j, os_getTime()+sec2osticks(SENT_EVERY_X_MINUTES*60), do_send);
         
}


void setup() {
  // LMIC init
  wiringPiSetup();

  // Initializing  GPS
  printf("Now initializing GPS ...\r\n");
  if (init_gps() == false) {
    logMsg("GPS init failed\r\n");
    return ;
  }
  printf("GPS init finished\r\n");
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (u1_t*)DEVKEY, (u1_t*)ARTKEY);
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  //
  flush_gps_data();

}

void loop() {

do_send(&sendjob);

while(1) {
  os_runloop();
//  os_runloop_once();
  }
}


int main() {
  setup();

  while (1) {
    loop();
  }
  return 0;
}

