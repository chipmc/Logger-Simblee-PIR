///
/// @mainpage	Logger-Simblee-PIR
///
/// @details	SImblee side with a PIR sensor
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/10/17 10:03 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Logger_Simblee_PIR.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		3/10/17 10:03 AM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE

// Set parameters
//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 7             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 2               // First word of daily counts
#define HOURLYOFFSET 30             // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4064      // used in modulo calculations - sets the # of hours stored - 256k (4096-14-2)
#define VERSIONADDR 0x0             // Memory Locations By Name not Number
#define PARKOPENSADDR 0x1           // When does the park open
#define PARKCLOSESADDR 0x2          // when does the park close
#define MONTHLYREBOOTCOUNT 0x3      // This is where we store the reboots - indication of system health
#define DAILYPOINTERADDR 0x4        // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
//Second Word - 8 bytes for storing current counts
#define CURRENTHOURLYCOUNTADDR 0x8  // Current Hourly Count
#define CURRENTDAILYCOUNTADDR 0xA   // Current Daily Count
#define CURRENTCOUNTSTIME 0xC       // Time of last count
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYCOUNTOFFSET 2          // Count is a 16-bt value
#define DAILYBATTOFFSET 4           // Where the battery charge is stored
#define HOURLYCOUNTOFFSET 4         // Offsets for the values in the hourly words
#define HOURLYBATTOFFSET 6          // Where the hourly battery charge is stored
// Finally, here are the variables I want to change often and pull them all together here
#define DEVICENAME "Umstead"
#define SERVICENAME "Graylyn"
#define SOFTWARERELEASENUMBER "1.0.0"



// Include application, user and local libraries
#include <Wire.h>               //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "DS3232RTC.h"          //http://github.com/JChristensen/DS3232RTC
#include <Timelib.h>            //http://www.arduino.cc/playground/Code/Time
#include "MAX17043.h"           // Drives the LiPo Fuel Gauge
#include "Adafruit_FRAM_I2C.h"   // Note - had to comment out the Wire.begin() in this library
#include "SimbleeForMobile.h"
#include <stdlib.h>
#include <stdio.h>
#include "FRAMcommon.h"     // Where I put all the common FRAM read and write extensions
#include "SimbleeForMobileClient.h"


// Prototypes
// Prototypes From the included libraries
MAX17043 batteryMonitor;                      // Init the Fuel Gauge
SimbleeForMobileClient client;


// Prototypes for General Functions
void BlinkForever(); // Ends execution if there is an error
int sprintf ( char * str, const char * format, ... );


// Prototypes for Date and Time Functions
time_t findMidnight(time_t unixT); // Need to break at midnight
void toArduinoTime(time_t unixT); // Puts time in format for reporting


// Prototypes for the Simblee
void SimbleeForMobile_onConnect();   // Actions to take once we get connected
void SimbleeForMobile_onDisconnect();    // Can clean up resources once we disconnect
void ui();   // The function that defines the iPhone UI
void ui_event(event_t &event);   // This is where we define the actions to occur on UI events
void createCurrentScreen(); // This is the screen that displays current status information
void updateCurrentScreen(); // Since we have to update this screen three ways: create, menu bar and refresh button
void createDailyScreen(); // This is the screen that displays current status information
void createHourlyScreen(); // This is the screen that displays current status information
void createAdminScreen(); // This is the screen that you use to administer the device
void printEvent(event_t &event);     // Utility method to print information regarding the given event
int wakeUpAlarm(uint32_t dummyButton); // Function to let us know we are waking up
void enable32Khz(uint8_t enable); // Need to turn on the 32k square wave for bus moderation


// Define variables and constants
// Pin Value Variables
const int SCLpin = 13;    // Simblee i2c Clock pin
const int SDApin = 14;    // Simblee i2c Data pin
const int AlarmPin = 20;  // This is the open-drain line for signaling an Alarm
const int resetPin = 30;  // This pin can reset the Arduino
const int intPin = 2;   // This is the pin which wakes the Arduino
const int HeartbeatPin = 4;  // HEartbeat pin to Arduino
const int PowerDownPin = 6;  // This pin can turn off the power to generate a reboot

// Battery monitor
float stateOfCharge = 0;    // Initialize state of charge

// FRAM and Unix time variables
tmElements_t tm;        // Time elements (such as tm.Minues
time_t t;               // UNIX time format (not note 32 bit number unless converted)
volatile boolean alarmInterrupt = false;  // OK, this is the Alarm Interrupt flag
int lastHour = 0;  // For recording the startup values
int lastDate = 0;   // For providing day break counts
byte ParkOpens;             // When does the park open
byte ParkCloses;            // When does the park close
unsigned int hourlyPersonCount = 0;  // hourly counter
unsigned int dailyPersonCount = 0;   //  daily counter

// Interface vairables
unsigned int lastupdate = 0;    // For when we are on the current screen
int updateFrequency = 500;     // How often will we update the current screen
int adminAccessKey = 27617;     // This is the code you need to enter to get to the admin field
int adminAccessInput = 0;       // This is the user's input
bool clearFRAM = false;
const char* releaseNumber = SOFTWARERELEASENUMBER;
bool adminUnlocked = false;  // Start with the Admin tab locked
int resetCount = 0; // How many times has the Simblee reset the Arduino
unsigned long resetDelay = 20000;   // Don't want to keep resetting the Arduino
unsigned long lastReset;

// Variables for Simblee Display
const char *titles[] = { "Current", "Daily", "Hourly", "Admin" };
const char *fields[] = { "Hr","Min","Sec","Yr","Mon","Sec"};
int currentScreen; // The ID of the current screen being displayed
int ui_adminLockIcon;       // Shows whether the admin tab is unlocked
int ui_adminAccessField;    // Where the Admin access code is entered
int ui_StartStopSwitch; // Start stop button ID on Admin Tab
int ui_StartStopStatus; // Text field ID for start stop status on Admin Tab
int ui_EraseMemSwitch; // Erase Memory button ID on Admin Tab
int ui_EraseMemStatus; // Text field ID for Erase Memory status on Admin Tab
int ui_LedOnOffSwitch;      // To turn the lights on and off
int ui_UpdateButton;  // Update button ID on Admin Tab
int ui_UpdateStatus;    // Indicates if an update is pending
int ui_dateTimeField;  // Text field on Current Tab
int ui_hourlyField;    // Hour field ID for Houly Tab
int ui_dailyField;     // Date feild ID for Hourly Tab
int ui_chargeField;    // State of charge field ID on Current Tab
int ui_ParkOpensField;  // When does the park open on Current Tab
int ui_ParkClosesField; // When does the park close on Current Tab
int ui_ParkOpensStepper;    // Update Open Hour on Admin Tab
int ui_ParkClosesStepper;   // Update Close Hour on Admin Tab
int ui_OpensUpdateField;      // For the Admin screen on Admin Tab
int ui_ClosesUpdateField;    // For the Admin Screen on Admin Tab
int ui_menuBar;        // ID for the tabbed Menu Bar
int ui_setYear, ui_setMonth,ui_setDay,ui_setHour,ui_setMinute,ui_setSecond; // Element which displays date and time values on Admin Tab
int ui_hourStepper, ui_minStepper, ui_secStepper, ui_yearStepper, ui_monthStepper, ui_dayStepper;   // Stepper IDs for adjusting on Admin Tab

// Variables for Simblee Cloud
unsigned int userID = 0xe983942d; //Enter your assigned userID here
unsigned int destESN = 0x00001010; // This is the destination from the Simblee Cloud Admin Site
int ui_sendCloudSwitch;


// Variables for the control byte
// Control Register  (8 - 7 Reserved, 6- Simblee Reset Flag, 5-Clear Counts, 4-Simblee Sleep, 3-Start / Stop Test, 2-WarmUp, 1-LEDs)
byte turnLedsOn = B00000001;    // Turn on the LEDs
byte turnLedsOff = B11111110;   // Turn off the LEDs
byte toggleStartStop = B00000100;   // Mask for accessing the start / stop bit
byte signalSimbleeSleep = B00001000;        // Mask for accessing the Simblee Health bit
byte clearSimbleeSleep = B11110111;         // Mask to clear the Sleep bit
byte signalClearCounts = B00010000; // Flag to have the Arduino clear current counts
byte signalSimbleeReset = B00100000;    // Sets the reset flag
byte controlRegisterValue;  // Current value of the control register


// include newlib printf float support (%f used in sprintf below)
asm(".global _printf_float");

// Add setup code
void setup()
{
    Wire.beginOnPins(SCLpin,SDApin);  // Not sure if this talks to the i2c bus or not - just to be safe...
    Serial.begin(9600);     //  Access to the terminal
    fram.begin();           //  Access to the FRAM Chip
    
    // Unlike Arduino Simblee does not pre-define inputs
    pinMode(TalkPin, INPUT);  // Shared Talk line
    pinMode(The32kPin, INPUT);   // Shared 32kHz line from clock
    pinMode(AlarmPin,INPUT);    // Shared DS3231 Alarm Pin
    pinMode(intPin,INPUT);  // Need to watch this pin
    //digitalWrite(intPin,HIGH);
    pinMode(resetPin,OUTPUT);   // If needed, we can reset the Arduino
    attachPinInterrupt(AlarmPin,wakeUpAlarm,LOW);    // this will trigger an alarm that will put Simblee in low power mode until morning
    
    Serial.println(F("Startup delay..."));
    delay(100); // This is to make sure that the Arduino boots first as it initializes the various devices
    
    // Set up the Simblee Mobile App and Simblee Cloud
    Serial.println("");
    SimbleeForMobile.deviceName = DEVICENAME;          // Device name
    SimbleeForMobile.advertisementData = SERVICENAME;  // Name of data service
    SimbleeForMobile.begin();
    
    Serial.println("Ready to go....");
}

// Add loop code
void loop()
{
    if (alarmInterrupt)         // Here is where we manage the Simblee's sleep and wake cycles
    {
        TakeTheBus();
        if (RTC.alarm(ALARM_1))
        {
            alarmInterrupt = true;   // (re)set the Alarm Flag as it is time to go to sleep
        }
        else
        {
            RTC.alarm(ALARM_2);     // Weird I know but this code does not recognize Alarm 2 for waking up - simply clear it here.
            alarmInterrupt = false;  // Clear the Alarm Flag as it is time to wake up
        }
        GiveUpTheBus();
        if (alarmInterrupt)
        {
            Simblee_pinWakeCallback(AlarmPin, LOW, wakeUpAlarm); // configures pin 20 to wake up device on a Low signal
            alarmInterrupt = false;  // Now we can clear the interrupt flag
            Serial.println("... goodnight going to Sleep");
            controlRegisterValue = FRAMread8(CONTROLREGISTER);
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue | signalSimbleeSleep);
            delay(1000);
            Simblee_systemOff();  // Very low power - only comes back with interrupt
        }
        else
        {
            Serial.println("Time to wake up");
            controlRegisterValue = FRAMread8(CONTROLREGISTER);
            FRAMwrite8(CONTROLREGISTER, controlRegisterValue & clearSimbleeSleep);
            
        }
    }
    if (millis() >= lastupdate + updateFrequency)
    {
        SimbleeForMobile.process(); // process must be called in the loop for SimbleeForMobile
        //cloud.process();            // process must be called in the loop for Simblee Cloud
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        if (SimbleeForMobile.connected && SimbleeForMobile.screen == 1)
        {
            if (SimbleeForMobile.updatable) updateCurrentScreen();
        }
        lastupdate = millis();
        if (!digitalRead(intPin)) // If this pin is low, then the Arduino should service the interrupt - unless it is locked up....
        {
            NonBlockingDelay(500);  // Give the Arduino a half-second to recover itself
            if(!digitalRead(intPin) && millis() >> (resetDelay + lastReset)) // OK, I guess the Arudino is Frozen
            {
                lastReset = millis();
                digitalWrite(resetPin,LOW);
                delay(100);      // Bring this line low for 100msec to reset
                digitalWrite(resetPin,HIGH);
                resetCount++;   // Increment the reset count
                Serial.print("Arduino reset count: ");
                Serial.println(resetCount);
            }
        }
    }
}

int wakeUpAlarm(uint32_t dummyButton)  // Function to let us know we are waking up
{
    Simblee_resetPinWake(AlarmPin); // reset state of pin that caused wakeup
    alarmInterrupt = true;  // Set the Alarm flag
}

void SimbleeForMobile_onConnect()   // Actions to take once we get connected.
{
    currentScreen = -1;     // Reset the current screen to non being displayed
    
}


void SimbleeForMobile_onDisconnect()    // Can clean up resources once we disconnect
{
    adminUnlocked = false;      // Clear the Admin Unlock values on disconnect
    adminAccessInput = 0;
    Serial.println("Disconnecting setting the reset flag");
    controlRegisterValue = FRAMread8(CONTROLREGISTER);  // Get the latest control register value
    FRAMwrite8(CONTROLREGISTER, controlRegisterValue | signalSimbleeReset); // This will set the SimbleeReset Flag on disconect
}

void ui()   // The function that defines the iPhone UI
{
    if(SimbleeForMobile.screen == currentScreen) return;    // If we are on the right screen then we are all set
    currentScreen = SimbleeForMobile.screen;    // If not, let's capture the current screen number
    switch(SimbleeForMobile.screen)
    {
        case 1:
            createCurrentScreen();  // Create the current screen
            updateCurrentScreen();
            break;
            
        case 2:
            createDailyScreen(); // This screen is created and populated in one step
            break;
            
        case 3:
            createHourlyScreen(); // This screen is created and populated in one step
            break;
            
        case 4:
            createAdminScreen(); // Create the current screen
            if ((controlRegisterValue & toggleStartStop) >> 2) // Populate the fields with values
            {
                SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else
            {
                SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
            }
            break;
            
        default:
            Serial.print("ui: Uknown screen requested: ");  // Don't think we can get here using the menuBar but just in case
            Serial.println(SimbleeForMobile.screen);
    }
}

void ui_event(event_t &event)   // This is where we define the actions to occur on UI events
{
    printEvent(event);
    currentScreen = SimbleeForMobile.screen;    // As the event ids are specific to each screen, we have to switch on screen
    if (event.id == ui_menuBar)                 // This is the event handler for the menu bar
    {
        switch(event.value)
        {
            case 0: // This is the current tab
                SimbleeForMobile.showScreen(1);
                break;
                
            case 1: // This is the Daily Tab
                SimbleeForMobile.showScreen(2);
                break;
                
            case 2: // This is the Hourly Tab
                SimbleeForMobile.showScreen(3);
                break;
                
            case 3: // This is the Admin Tab
                if (adminUnlocked) SimbleeForMobile.showScreen(4);
                else if (currentScreen == 1) SimbleeForMobile.updateValue(ui_menuBar, 0);
                else SimbleeForMobile.showScreen(1);
                break;
        }
    }
    switch (currentScreen) {
        case 1:// The Current Screen
            if (event.id == ui_adminAccessField)   // Where you enter the Admin code
            {
                adminAccessInput = event.value;
                if (adminAccessInput == adminAccessKey)
                {
                    SimbleeForMobile.updateColor(ui_adminLockIcon,GREEN);
                    adminUnlocked = true;
                }
                else if (adminAccessInput != adminAccessKey)
                {
                    adminAccessInput = 0;
                    SimbleeForMobile.updateValue(ui_adminAccessField,0);
                    SimbleeForMobile.updateColor(ui_adminLockIcon,RED);
                    adminUnlocked = false;
                    
                }
            }
            else if (event.id == ui_LedOnOffSwitch) // Using the LED switch - remember this is on or off and in the LSB
            {
                controlRegisterValue = FRAMread8(CONTROLREGISTER);
                if (event.value)
                {
                    FRAMwrite8(CONTROLREGISTER,controlRegisterValue | turnLedsOn );  // Turn on the LED bit
                }
                else
                {
                    FRAMwrite8(CONTROLREGISTER,controlRegisterValue & turnLedsOff );  // Turn off the LED bit
                }
            }
            break;
        case 2:// The Daily Screen
            Serial.println("No ui events on the Daily Screen");
            break;
        case 3:// The Hourly Screen
            Serial.println("No ui events on the Hourly Screen");
            break;
        case 4:// The Admin Screen
            if (event.id == ui_StartStopSwitch && event.type == EVENT_RELEASE) // Start / Stop Button handler from the Admin screen
            {
                controlRegisterValue = FRAMread8(CONTROLREGISTER);
                FRAMwrite8(CONTROLREGISTER,toggleStartStop ^ controlRegisterValue);  // Toggle the start stop bit
                if ((controlRegisterValue & toggleStartStop) >> 2) {
                    SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
                }
                else SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
            }
            else if (event.id == ui_EraseMemSwitch && event.type == EVENT_RELEASE)  // This button allows us to erase the FRAM from the Admin screen
            {
                clearFRAM = true;
                SimbleeForMobile.updateText(ui_UpdateStatus,"Press \"Update\" to confirm");
            }
            else if (event.id == ui_UpdateButton && event.type == EVENT_RELEASE)  // A bit more complicated - Update botton event on Admin Tab
            {
                if (tm.Year >> 0 && tm.Month >> 0 && tm.Day >> 0) // Will only act on the update if these values are all non-zero
                {
                    t= makeTime(tm);
                    TakeTheBus();  // Clock is an i2c device
                    RTC.set(t);             //use the time_t value to ensure correct weekday is set
                    setTime(t);
                    GiveUpTheBus();
                    tm.Year = tm.Month = tm.Day = tm.Hour  = tm.Minute = tm.Second = 0;
                    Serial.println("Updating time");
                }
                if (clearFRAM)
                {
                    SimbleeForMobile.updateText(ui_EraseMemStatus,"Started");
                    ResetFRAM();
                    SimbleeForMobile.updateText(ui_EraseMemStatus,"Erased");
                    controlRegisterValue = FRAMread8(CONTROLREGISTER);
                    FRAMwrite8(CONTROLREGISTER,signalClearCounts ^ controlRegisterValue);  // Toggle the clear counts bit
                    clearFRAM = false;
                }
                if (FRAMread8(PARKOPENSADDR) != ParkOpens | FRAMread8(PARKCLOSESADDR) != ParkCloses)
                {
                    FRAMwrite8(PARKOPENSADDR,ParkOpens);
                    Serial.println(FRAMread8(PARKOPENSADDR));
                    RTC.setAlarm(ALM2_MATCH_HOURS,00,00,ParkOpens,0); // Set the morning Alarm
                    FRAMwrite8(PARKCLOSESADDR, ParkCloses);
                    RTC.setAlarm(ALM1_MATCH_HOURS,00,00,ParkCloses,0); // Set the evening Alarm
                }
                SimbleeForMobile.updateText(ui_UpdateStatus," ");
            }
            else if (event.id == ui_ParkOpensStepper)    // Used to increment / decrement values for setting clock on the Admin Screen
            {
                ParkOpens = event.value;
                SimbleeForMobile.updateValue(ui_OpensUpdateField, ParkOpens);
                SimbleeForMobile.updateText(ui_UpdateStatus,"Press \"Update\" to confirm");
            }
            else if (event.id == ui_ParkClosesStepper)    // Used to increment / decrement values for setting clock on the Admin Screen
            {
                ParkCloses = event.value;
                SimbleeForMobile.updateValue(ui_ClosesUpdateField, ParkCloses);
                SimbleeForMobile.updateText(ui_UpdateStatus,"Press \"Update\" to confirm");
            }
            else if (event.id == ui_hourStepper)    // Used to increment / decrement values for setting clock on the Admin Screen
            {
                tm.Hour = event.value;
                SimbleeForMobile.updateValue(ui_setHour, tm.Hour);
            }
            else if (event.id == ui_minStepper)
            {
                tm.Minute = event.value;
                SimbleeForMobile.updateValue(ui_setMinute, tm.Minute);
            }
            else if (event.id == ui_secStepper)
            {
                tm.Second = event.value;
                SimbleeForMobile.updateValue(ui_setSecond, tm.Second);
            }
            else if (event.id == ui_yearStepper)
            {
                tm.Year = CalendarYrToTm(event.value);
                SimbleeForMobile.updateValue(ui_setYear, tm.Year+1970);
            }
            else if (event.id == ui_monthStepper)
            {
                tm.Month = event.value;
                SimbleeForMobile.updateValue(ui_setMonth, tm.Month);
            }
            else if (event.id == ui_dayStepper)
            {
                tm.Day = event.value;
                SimbleeForMobile.updateValue(ui_setDay, tm.Day);
            }
            break;
        default:
            break;
    }
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    Serial.print("Control Register Value = ");
    Serial.println(controlRegisterValue);
}


void createCurrentScreen() // This is the screen that displays current status information
{
    char IDBuffer[42];   // Should be enough for 16 chars of service and name, version and text
    char ParkOpensBuffer[5]; // Format Time
    char ParkClosesBuffer[5]; // Format Time
    
    
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    ui_menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(ui_menuBar, 0);
    
    // all we are doing here is laying out the screen - updates are in a separate function
    ui_dateTimeField = SimbleeForMobile.drawText(40, 140, " ");
    SimbleeForMobile.drawText(40, 160, "Hourly Count:");
    ui_hourlyField = SimbleeForMobile.drawText(200,160," ");
    SimbleeForMobile.drawText(40, 180, "Daily Count:");
    ui_dailyField =   SimbleeForMobile.drawText(200,180," ");
    SimbleeForMobile.drawText(40, 200, "State of Charge:");
    ui_chargeField = SimbleeForMobile.drawText(200,200," ");
    SimbleeForMobile.drawText(40, 220, "Counter Status:");
    ui_StartStopStatus = SimbleeForMobile.drawText(200, 220, " ");
    SimbleeForMobile.drawText(40,300,"LEDs on or off: ");
    ui_LedOnOffSwitch = SimbleeForMobile.drawSwitch(200,300);
    SimbleeForMobile.drawText(40,240, "Park Opens: ");
    ParkOpens = FRAMread8(PARKOPENSADDR);
    snprintf(ParkOpensBuffer, 6,"%i:00",ParkOpens);   // Puts :00 next to time
    ui_ParkOpensField = SimbleeForMobile.drawText(200, 240, ParkOpensBuffer);
    SimbleeForMobile.drawText(40, 260, "Park Closes: ");
    ParkCloses = FRAMread8(PARKCLOSESADDR);
    snprintf(ParkClosesBuffer, 6,"%i:00",ParkCloses);// Puts :00 next to time
    ui_ParkClosesField = SimbleeForMobile.drawText(200, 260, ParkClosesBuffer);
    ui_adminLockIcon = SimbleeForMobile.drawText(40,340,"Admin Code:",RED);
    ui_adminAccessField = SimbleeForMobile.drawTextField(132,335,80,adminAccessInput);
    snprintf(IDBuffer, 43,"%s - %s v%s - %i reboots",DEVICENAME,SERVICENAME,SOFTWARERELEASENUMBER,FRAMread8(MONTHLYREBOOTCOUNT));   // Identifies Device on Current screen
    SimbleeForMobile.drawText(10,(SimbleeForMobile.screenHeight-20),IDBuffer);
    SimbleeForMobile.endScreen();
}

void updateCurrentScreen() // Since we have to update this screen three ways: create, menu bar and refresh button
{
    char battBuffer[4];   // Should be enough 3 digits plus % symbol
    char ParkOpensBuffer[5]; // Format Time
    char ParkClosesBuffer[5]; // Format Time
    
    TakeTheBus();
    t = RTC.get();
    stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    toArduinoTime(t);  // Update Time Field on screen
    
    if (stateOfCharge >= 105)   // Update the value and color of the state of charge field
    {
        SimbleeForMobile.drawText(210,200,"Error");
    }
    else if (stateOfCharge > 75) {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(ui_chargeField,battBuffer);
        SimbleeForMobile.updateColor(ui_chargeField,GREEN);
    }
    else if (stateOfCharge > 50) {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(ui_chargeField,battBuffer);
        SimbleeForMobile.updateColor(ui_chargeField,MAGENTA);
    }
    else {
        snprintf(battBuffer, 5,"%1.0f%%",stateOfCharge);   // Puts % next to batt from 0-100
        SimbleeForMobile.updateText(ui_chargeField,battBuffer);
        SimbleeForMobile.updateColor(ui_chargeField,RED);
    }
    
    SimbleeForMobile.updateValue(ui_hourlyField, FRAMread16(CURRENTHOURLYCOUNTADDR)); // Populate the hourly and daily fields with values
    SimbleeForMobile.updateValue(ui_dailyField, FRAMread16(CURRENTDAILYCOUNTADDR));
    
    controlRegisterValue = FRAMread8(CONTROLREGISTER);          // Update the Start and Stop fields with the latest status
    if ((controlRegisterValue & toggleStartStop) >> 2) {
        SimbleeForMobile.updateText(ui_StartStopStatus, "Running");
    }
    else SimbleeForMobile.updateText(ui_StartStopStatus, "Stopped");
    
    if ((controlRegisterValue & turnLedsOn)) {
        SimbleeForMobile.updateValue(ui_LedOnOffSwitch, 1);
    }
    else SimbleeForMobile.updateValue(ui_LedOnOffSwitch,0);
    
    ParkOpens = FRAMread8(PARKOPENSADDR);
    snprintf(ParkOpensBuffer, 6,"%i:00",ParkOpens);   // Puts :00 next to time
    SimbleeForMobile.updateText(ui_ParkOpensField, ParkOpensBuffer);
    ParkCloses = FRAMread8(PARKCLOSESADDR);
    snprintf(ParkClosesBuffer,6,"%i:00",ParkCloses);
    SimbleeForMobile.updateText(ui_ParkClosesField, ParkClosesBuffer);
    
    
    if (adminUnlocked) {
        SimbleeForMobile.updateValue(ui_adminAccessField,adminAccessInput);
        SimbleeForMobile.updateColor(ui_adminLockIcon,GREEN);
    }
}

void createDailyScreen() // This is the screen that displays current status information
{
    int xAxis = 30;
    int yAxis = 110;
    int rowHeight = 15;
    int columnWidth = 5;
    int dailyCount = 0;
    char battBuffer[4];   // Should be enough 3 digits plus % symbol
    char IDBuffer[19];   // Should be enough for 16 chars of service and name with separator
    
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    ui_menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(ui_menuBar, 1);
    
    snprintf(IDBuffer, 20,"%s - %s",DEVICENAME,SERVICENAME);   // Identifies Device on Current screen
    SimbleeForMobile.drawText(110,40,IDBuffer);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Date");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    
    
    for (int i=0; i < DAILYCOUNTNUMBER; i++) {
        int pointer = (DAILYOFFSET + (i+FRAMread8(DAILYPOINTERADDR)) % DAILYCOUNTNUMBER)*WORDSIZE;
        dailyCount = FRAMread16(pointer+DAILYCOUNTOFFSET);
        if (dailyCount > 0) {
            yAxis = yAxis + rowHeight;
            SimbleeForMobile.drawText(xAxis, yAxis,FRAMread8(pointer));
            SimbleeForMobile.drawText(xAxis+3*columnWidth, yAxis,"/");
            SimbleeForMobile.drawText(xAxis+5*columnWidth, yAxis,FRAMread8(pointer+DAILYDATEOFFSET));
            SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
            SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,dailyCount);
            SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
            snprintf(battBuffer, 5,"%u%%",FRAMread8(pointer+DAILYBATTOFFSET));   // Puts % next to batt from 0-100
            SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,battBuffer);
        }
    }
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
}

void createHourlyScreen() // This is the screen that displays today's hourly counts
{
    int xAxis = 30;
    int yAxis = 110;
    int rowHeight = 15;
    int columnWidth = 5;
    int row = 1;
    int hourIndex = FRAMread16(HOURLYPOINTERADDR);
    char battBuffer[4];   // Should be enough 3 digits plus % symbol
    char hourBuffer[5];   // Should be enough for 4 hour and :00
    char IDBuffer[19];   // Should be enough for 16 chars of service and name with separator
    
    TakeTheBus();
    t = RTC.get();                  // First we will establish the time
    GiveUpTheBus();
    time_t mighnightUnixT = findMidnight(t);   // This is the midnight that preceeds today's day
    
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    ui_menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(ui_menuBar, 2);
    
    snprintf(IDBuffer, 20,"%s - %s",DEVICENAME,SERVICENAME);   // Identifies Device on Current screen
    SimbleeForMobile.drawText(110,40,IDBuffer);
    
    SimbleeForMobile.drawText(xAxis, yAxis,"Hour");
    SimbleeForMobile.drawText(xAxis+21*columnWidth, yAxis,"Count");
    SimbleeForMobile.drawText(xAxis+41*columnWidth, yAxis,"Batt %");
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
    
    
    for (int i=1; i <= 24; i++) {           // The most hours we can have in a day is 24 - right?
        hourIndex = (hourIndex ? hourIndex : HOURLYCOUNTNUMBER) -1; // Modeled on i = (i ? i : range) - 1;
        int pointer = (HOURLYOFFSET + hourIndex) * WORDSIZE;
        yAxis = yAxis + rowHeight;
        time_t unixTime = FRAMread32(pointer);
        if (unixTime < mighnightUnixT) {       // Let's make sure we are still on today's date
            Serial.println("That is all for today");
            break;
        }
        snprintf(hourBuffer, 6, "%i:00",hour(unixTime));
        SimbleeForMobile.drawText(xAxis, yAxis,hourBuffer);
        SimbleeForMobile.drawText(xAxis+15*columnWidth, yAxis," - ");
        SimbleeForMobile.drawText(xAxis+22*columnWidth, yAxis,FRAMread16(pointer+HOURLYCOUNTOFFSET));
        SimbleeForMobile.drawText(xAxis+34*columnWidth, yAxis,"  -  ");
        snprintf(battBuffer, 5,"%u%%",FRAMread8(pointer+HOURLYBATTOFFSET));   // Puts % next to batt from 0-100
        SimbleeForMobile.drawText(xAxis+42*columnWidth, yAxis,battBuffer);
    }
    SimbleeForMobile.endScreen();       // So, everything below this is not cached
}


void createAdminScreen() // This is the screen that displays current status information
{
    
    SimbleeForMobile.beginScreen(WHITE, PORTRAIT); // Sets orientation
    ui_menuBar = SimbleeForMobile.drawSegment(20, 70, 280, titles, countof(titles));
    SimbleeForMobile.updateValue(ui_menuBar, 3);
    
    SimbleeForMobile.drawRect(10,310,300,170,YELLOW);
    SimbleeForMobile.drawRect(10,200,300,100, YELLOW);
    
    // The first control will be a switch
    controlRegisterValue = FRAMread8(CONTROLREGISTER);
    
    ui_EraseMemStatus = SimbleeForMobile.drawText(50, 130, " ");
    ui_EraseMemSwitch = SimbleeForMobile.drawButton(20,150,110, "Reset Mem",RED);
    SimbleeForMobile.setEvents(ui_EraseMemSwitch, EVENT_RELEASE);
    
    ui_StartStopStatus = SimbleeForMobile.drawText(200, 130, " ");
    ui_StartStopSwitch = SimbleeForMobile.drawButton(180,150,110, "Start/Stop");
    SimbleeForMobile.setEvents(ui_StartStopSwitch, EVENT_RELEASE);
    
    SimbleeForMobile.drawText(50, 220, "Set Park Open and Closing Hours");
    ui_OpensUpdateField = SimbleeForMobile.drawText(80, 240, "HH");
    ui_ClosesUpdateField = SimbleeForMobile.drawText(220, 240, "HH");
    ui_ParkOpensStepper = SimbleeForMobile.drawStepper(40, 260, 80, 0, 12);
    ui_ParkClosesStepper = SimbleeForMobile.drawStepper(190, 260, 80, 13, 23);
    
    SimbleeForMobile.drawText(100,310,"Set Time and Date");
    ui_setYear = SimbleeForMobile.drawText(50,330,"YYYY");
    ui_setMonth = SimbleeForMobile.drawText(150,330,"MM");
    ui_setDay = SimbleeForMobile.drawText(250,330,"DD");
    ui_setHour = SimbleeForMobile.drawText(60,400,"HH");
    ui_setMinute = SimbleeForMobile.drawText(150,400,"MM");
    ui_setSecond = SimbleeForMobile.drawText(250,400,"SS");
    ui_hourStepper = SimbleeForMobile.drawStepper(25,420,80,0,24);
    ui_minStepper = SimbleeForMobile.drawStepper(120,420,80,0,60);
    ui_secStepper = SimbleeForMobile.drawStepper(215,420,80,0,60);
    ui_yearStepper = SimbleeForMobile.drawStepper(25,350,80,2017,2020);
    ui_monthStepper = SimbleeForMobile.drawStepper(120,350,80,1,12);
    ui_dayStepper = SimbleeForMobile.drawStepper(215,350,80,1,31);
    
    
    ui_UpdateStatus = SimbleeForMobile.drawText(75,535," ");
    ui_UpdateButton = SimbleeForMobile.drawButton(100,500,120, "Update");
    SimbleeForMobile.setEvents(ui_UpdateButton, EVENT_RELEASE);
    
    SimbleeForMobile.endScreen();
}

void printEvent(event_t &event)     // Utility method to print information regarding the given event
{
    switch (currentScreen) {
        case 1: Serial.print("On Current screen "); break;
        case 2: Serial.print("On Daily screen ");   break;
        case 3: Serial.print("On Hourly screen ");  break;
        case 4: Serial.print("On Admin screen ");   break;
        default:                                    break;
    }
    switch (event.id) {
        case 0: Serial.print("the Menu bar");           break;
        case 2: Serial.print("the Mem  Reset Button");  break;
        case 4: Serial.print("the Start/Stop Button");  break;
        case 6: Serial.print("undefined");    break;
        case 8: Serial.print("undefined"); break;
        case 9: Serial.print("the Park Opens Stepper"); break;
        case 10:Serial.print("the Refresh button");     break;
        case 11:Serial.print("the Park Closes Stepper");   break;
        case 12:Serial.print("the LED on-off switch");  break;
        case 17:Serial.print("the Admin Code field");  break;
        case 21:Serial.print("the Update button");      break;
        default:
            Serial.print(" element ");
            Serial.print(event.id);
            break;
    }
    switch (event.type) {
        case 0: Serial.print(" was pressed with a value of ");  Serial.println(event.value);    break;
        case 1: Serial.println(" was pressed"); break;
        case 2: Serial.println(" was released");break;
        default:
            Serial.print(" had an event ");
            Serial.println(event.type);
            break;
    }
}

time_t findMidnight(time_t unixT) // Need to break at midnight
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    unsigned long workingNumber;
    workingNumber = unixT - 3600*timeElement.Hour -60*timeElement.Minute - timeElement.Second;
    return workingNumber;
}


void toArduinoTime(time_t unixT)   // Converts to date time for the UI
{
    char dateTimeArray[18]="mm/dd/yy hh:mm:ss";
    char *dateTimePointer;
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    dateTimePointer = dateTimeArray;
    if(timeElement.Month < 10) {
        dateTimeArray[0] = '0';
        dateTimeArray[1] = timeElement.Month+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[0] = int(timeElement.Month/10)+48;
        dateTimeArray[1] = (timeElement.Month%10)+48;
    }
    if(timeElement.Day < 10) {
        dateTimeArray[3] = '0';
        dateTimeArray[4] = timeElement.Day+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[3] = int(timeElement.Day/10)+48;
        dateTimeArray[4] = (timeElement.Day%10)+48;
    }
    int currentYear = int(timeElement.Year)-30; // Year is displayed as 2016 not 16
    if(currentYear < 10) {
        dateTimeArray[6] = '0';
        dateTimeArray[7] = currentYear+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[6] = int(currentYear/10)+48;
        dateTimeArray[7] = (currentYear%10)+48;
    }
    if(timeElement.Hour < 10) {
        dateTimeArray[9] = '0';
        dateTimeArray[10] = timeElement.Hour+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[9] = int(timeElement.Hour/10)+48;
        dateTimeArray[10] = (timeElement.Hour%10)+48;
    }
    if(timeElement.Minute < 10) {
        dateTimeArray[12] = '0';
        dateTimeArray[13] = timeElement.Minute+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[12] = int(timeElement.Minute/10)+48;
        dateTimeArray[13] = (timeElement.Minute%10)+48;
    }
    if(timeElement.Second < 10) {
        dateTimeArray[15] = '0';
        dateTimeArray[16] = timeElement.Second+48;  // Stupid but the +48 gives the right ASCII code
    }
    else {
        dateTimeArray[15] = int(timeElement.Second/10)+48;
        dateTimeArray[16] = (timeElement.Second%10)+48;
    }
    SimbleeForMobile.updateText(ui_dateTimeField, dateTimePointer);
}



void BlinkForever()
{
    Serial.println(F("Error - Reboot"));
    while(1) { }
}

void enable32Khz(uint8_t enable)  // Need to turn on the 32k square wave for bus moderation
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    // status register
    Wire.requestFrom(0x68, 1);
    
    uint8_t sreg = Wire.read();
    
    sreg &= ~0b00001000; // Set to 0
    if (enable == true)
        sreg |=  0b00001000; // Enable if required.
    
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(sreg);
    Wire.endTransmission();
}
