/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <GCS_MAVLink.h>

#ifdef membug
#include <MemoryFree.h>
#endif

// Configurations
#include "OSD_Config.h"
#include "ArduCam_Max7456.h"
#include "OSD_Vars.h"
#include "OSD_Func.h"

/* *************************************************/
/* ***************** DEFINITIONS *******************/

//OSD Hardware 
//#define ArduCAM328
#define MinimOSD

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port
#define BOOTTIME         1000   // Time in milliseconds that we show boot loading bar and wait user input

// Objects and Serial definitions
FastSerialPort0(Serial);
OSD osd; //OSD object 

SimpleTimer  mavlinkTimer;


uint8_t c1 = 0;
uint8_t c2 = 0;
uint8_t d1 = 0;
uint8_t d2 = 0;
int loop_counter = 0;

void setup() 
{
#ifdef ArduCAM328
    pinMode(10, OUTPUT); // USB ArduCam Only
#endif
    pinMode(MAX7456_SELECT,  OUTPUT); // OSD CS

    Serial.begin(TELEMETRY_SPEED);
    // setup mavlink port
    mavlink_comm_0_port = &Serial;

#ifdef membug
    Serial.println(freeMem());
#endif

    // Prepare OSD for displaying 
    unplugSlaves();
    osd.init();

    // Start 
    startPanels();
    delay(500);

    // OSD debug for development (Shown at start)
#ifdef membug
    osd.setPanel(1,1);
    osd.openPanel();
    osd.printf("%i",freeMem()); 
    osd.closePanel();
#endif

    // Check EEPROM to for a new version that needs EEPROM reset
    if(readEEPROM(CHK_VERSION) != VER) {
        osd.setPanel(3,9);
        osd.openPanel();
        osd.printf_P(PSTR("EEPROM mapping outdated!|Update with the OSD Tool.")); 
        osd.closePanel();
        // run for ever until EEPROM version is OK 
        for(;;) {}
    }

    // Get correct panel settings from EEPROM
    readSettings();
    for(panel = 0; panel < npanels; panel++) readPanelSettings();
    panel = 0; //set panel to 0 to start in the first navigation screen
    // Show bootloader bar
    loadBar();

    // Startup MAVLink timers  
    mavlinkTimer.Set(&OnMavlinkTimer, 120);

    // House cleaning, clear display and enable timers
    osd.clear();
    mavlinkTimer.Enable();


// Initialize values


  // Turn warnings off:
  osd_fix_type = 2; //  GPS Fix good >= 2
  stall = 0;
  overspeed = 100;
  osd_vbat_A = 99;
  osd_battery_remaining_A = 99;
  motor_armed = 1;
  last_warning = 1;



} // END of setup();



/* ***********************************************/
/* ***************** MAIN LOOP *******************/

void loop() 
{

  

  
  while(Serial.available() >= 8) {
    
    if(loop_counter == 0)
    {
    c1 = Serial.read();
    c2 = Serial.read();
    }
    else
    {
      c1 = c2;
      c2 = Serial.read();
    }
    
    if(c1==B11111110 && c2==B11111110)
    {
          d1 = Serial.read();
          d2 = Serial.read();
          osd_pitch = (((d1 << 8)+d2)/200-90);
          d1 = Serial.read();
          d2 = Serial.read();
          osd_roll = (((d1 << 8)+d2)/50-180);
          d1 = Serial.read();
          d2 = Serial.read();
          osd_heading = (((d1 << 8)+d2)/50);
         
          lastMAVBeat = millis();
          loop_counter = 0;
    }
    else
    {
      loop_counter = 1; 
    }
    
  
  
  }
  
  
  
  

  
    //read_mavlink();
    mavlinkTimer.Run();
}

/* *********************************************** */
/* ******** functions used in main loop() ******** */
void OnMavlinkTimer()
{
    setHeadingPatern();  // generate the heading patern

    //  osd_battery_pic_A = setBatteryPic(osd_battery_remaining_A);     // battery A remmaning picture
    //osd_battery_pic_B = setBatteryPic(osd_battery_remaining_B);     // battery B remmaning picture

    setHomeVars(osd);   // calculate and set Distance from home and Direction to home
    
    writePanels();       // writing enabled panels (check OSD_Panels Tab)
}


void unplugSlaves(){
    //Unplug list of SPI
#ifdef ArduCAM328
    digitalWrite(10,  HIGH); // unplug USB HOST: ArduCam Only
#endif
    digitalWrite(MAX7456_SELECT,  HIGH); // unplug OSD
}
