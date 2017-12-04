// Waldo 3.0 on Arduino Mega2560
// By: Brice Petty, Evan Downard and Johnny Elderton
// Last Edit: 4/24/17 @ 8:36 PM by Brice Petty
//
// Description: Implementing Bluetooth and Accelerometer. Log directional accelerations and send
// bluetooth alerts when device is logging. This will simulate the anti-theft features of GPS
// tracking device on a mobile network for a fraction of the price. General code cleanups and 
// streamlining. Comments added and reorganized. EEPROM and pusbutton implementation. Push button
// once to add current mileage to the running total and view the final total. Push again to reset
// count to 0. New screen upgrades to display directional accelerations on main screen. Wiring
// cleanup and board reorginization. BLE LED status indicator. On for connected, off for no connection.
// blinks for UART received. 
//
// [X] Compiled
// [X] Tested
// [X] Added to Master Build
//
// Download Adafruit's Bluefruit app on Android or iPhone for easy UART communications for testing
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Waldo Wiring
// Re-map Mega's pinout 18 and 19 to Adafruit's logger shield TX and RX pin holes (software serial work-around)
// Megas pin 52 to Bluetooth's SCK
// Megas pin 50 to Bluetooth's MISO
// Megas pin 51 to Bluetooth's MOSI
// Megas pin 49 to Bluetooth's REQ
// Megas pin 48 to Bluetooth's RST
// Megas pin 53 to bleStatus LED
// Megas pin 46 to push button
// Megas/LoggerShield pin 2 to Bluetooth's RDY
// Megas pin 20 and 21 to Accelerometer AND OLED display's SDA and SCL, respectively
// Megas 5V to Bluetooth, OLED Display, and Accelerometer's VIN
// Megas GND to Bluetooth, OLED Display, and Accelerometer's GND

#include <SPI.h>               //bluetooth
#include <SD.h>                //micro SD
#include <SoftwareSerial.h>    //Serial
#include <EEPROM.h>            //EEPROM mem
#include <Wire.h>              //I2C 
#include <math.h>              //math
#include <Adafruit_GPS.h>      //GPS
#include <Adafruit_Sensor.h>   //accelerometer
#include <Adafruit_BLE_UART.h> //bluetooth
#include <Adafruit_SSD1306.h>  //display
#include <Adafruit_MMA8451.h>  //accelerometer

// Configuration:
#define LOGGING_PERIOD_SEC 10 // EDIT HERE to scale delay times between logs.  
#define SD_CS_PIN 10 // Pin for SD card.
#define OLED_RESET 4 // OLED reset
#define XPOS 0
#define YPOS 1
#define buttonPin 46
#define ADAFRUITBLE_REQ 49
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 48
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
#define SSD1306_LCDHEIGHT 64
#define bleLight 53

// Global States:
Adafruit_GPS GPS(&Serial1); // GPS class to interact with receiver.
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
File logfile; // SD card log file.
uint32_t logCounter = 0; // Counter until next location log is recorded.
int error = 0;
int buttonState = 0;
int bleDisCount = 0;
int waldoStatus = 0;
long saveAddress = 8;   //edit here to change the address we keep our mileage total (divisible by 4)
float lastFlat = 0;
float lastFlon = 0;
float totalDistance = 0.0;
float mileage = 0.00;
float oldAccel = 0.0;
float meters2miles = 1609.34; //meters/meters2milesmiles = miles
float mph = 1.15077945; //multiply by knots/hour for mph
bool bleStatus = 0;

// Timer interrupt called every millisecond to check for new data from GPS.
SIGNAL(TIMER0_COMPA_vect) {
  // Check for new GPS data.
  GPS.read();
  
  // Decrease the count since last location log.
  if (logCounter > 0) {
    logCounter--;
  }
}

// Halt function called when an error occurs.  Will print an error and stop execution while
void halt(const __FlashStringHelper *error) {
  Serial.println("error1");
}

// Distance function called to compare two sets of coordinates (degrees) and return the diference in meters
unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2) {
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  diflat=radians(flat2-flat1);
  flat1=radians(flat1);
  flat2=radians(flat2);
  diflon=radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=6371000.0; //Convert to meters
  return dist_calc;
}

// Log the current GPS location with the specified note.
void logLocation(const char* note, float Xaccel, float Yaccel, float Zaccel) {
  logfile.print(GPS.latitudeDegrees, 8);
  logfile.print(',');
  logfile.print(GPS.longitudeDegrees, 8);
  logfile.print(',');
  logfile.print(GPS.month, DEC);
  logfile.print('/');
  logfile.print(GPS.day, DEC);
  logfile.print("/20");
  logfile.print(GPS.year, DEC);
  logfile.print(',');
  logfile.print(GPS.hour, DEC);
  logfile.print(':');
  logfile.print(GPS.minute, DEC);
  logfile.print(':');
  logfile.print(GPS.seconds, DEC);
  logfile.print(',');
  logfile.print((GPS.speed)*mph, 2);
  logfile.print(',');
  logfile.print(Xaccel);
  logfile.print(',');
  logfile.print(Yaccel);
  logfile.print(',');
  logfile.print(Zaccel);
  logfile.print(',');
  logfile.print(GPS.altitude);
  logfile.print(',');
  logfile.print((int)GPS.satellites);
  logfile.print(',');
  logfile.print(note);
  logfile.println();
  logfile.flush();
  
  //Set a delay until we take an additional log. Differentiate depending on speed to save battery/mem
  if ((GPS.speed)*mph >= 1.2 && (GPS.speed)*mph < 20) {
    logCounter = LOGGING_PERIOD_SEC*150;
    }
  if ((GPS.speed)*mph >= 20 && (GPS.speed)*mph < 40) {
    logCounter = LOGGING_PERIOD_SEC*250;
    }
  if ((GPS.speed)*mph >= 40 && (GPS.speed)*mph < 55) {
    logCounter = LOGGING_PERIOD_SEC*350;
    }
  if ((GPS.speed)*mph >= 55 && (GPS.speed)*mph < 150) {
    logCounter = LOGGING_PERIOD_SEC*700;
    }
}

// This displays everything
void displayStuff(int waldoStatus) {
  sensors_event_t event;  
  mma.getEvent(&event);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (waldoStatus == -1) {
    display.print("Status:Error"); 
  }
  else if (waldoStatus == 0) {
    display.print("Status:No-Fix"); 
  }
  else if (waldoStatus == 1) {
    display.print("Status:Idle"); 
  }
  else if (waldoStatus == 2) {
    display.print("Status:Logging"); 
  }
  display.setCursor(91,0);
  display.print("Sat:");
  display.println((int)GPS.satellites);
  display.setCursor(0,41);
  display.print("X:");
  display.print(event.acceleration.x);
  display.setCursor(42,41);
  display.print("Y:");
  display.print(event.acceleration.y);
  display.setCursor(84,41);
  display.print("Z:");
  display.print(event.acceleration.z);
  display.setTextSize(2);
  display.setCursor(0,17);
  display.print("Mi:");
  display.println(mileage, 2);
  if (error == 1) {
    display.setTextSize(1);
    display.setCursor(0,57);
    display.print("!!! Transfer SD Files");
  }
  display.display();
  display.clearDisplay();
}

void displaySave() {
  long Old = EEPROMReadlong(saveAddress);
  float New = round(mileage);
  long New2 = New + Old;
  EEPROMWritelong(saveAddress, New2);
  display.clearDisplay();             
  display.setTextSize(2);                
  display.setTextColor(WHITE);
  display.setCursor(0,15);
  display.println(New2);
  display.setTextSize(1);
  display.setCursor(0,45);
  display.println("Press again to reset");
  display.display();
  display.clearDisplay();
  delay(250);
        
  for (int i = 0; i < 4000; i++) {
    buttonState = digitalRead(buttonPin);
    delay(1);
          
    if (buttonState == HIGH) {
      EEPROMWritelong(saveAddress, 0);
      display.clearDisplay();             
      display.setTextSize(2);                 
      display.setTextColor(WHITE);
      display.setCursor(0,14);
      display.print("   Reset");
      display.setCursor(5,31);
      display.print(" Complete");
      display.display();
      display.clearDisplay();
      delay(2000);
      break;
    }
  }     
}

void bleCheck () 
{
  if (bleStatus == 1){
    digitalWrite(bleLight, HIGH);
    delay(3);
  }
  else {
    digitalWrite(bleLight, LOW);
    delay(3);
    bleDisCount += 1;
    if (bleDisCount == 16) {
      uart.setRXcallback(rxCallback);      
      uart.setACIcallback(aciCallback);
      uart.setDeviceName("Waldo");
      uart.begin();
      bleDisCount = 0;
    }
  }
}

//This interrupt is called and prints important status changes to the serial monitor when one occurs
void aciCallback(aci_evt_opcode_t event)
{
  switch(event)
  {
    case ACI_EVT_DEVICE_STARTED:
      Serial.println(F("Advertising started"));
      break;
    case ACI_EVT_CONNECTED:
      Serial.println(F("Connected!"));
      bleStatus = 1;
      break;
    case ACI_EVT_DISCONNECTED:
      Serial.println(F("Disconnected or advertising timed out"));
      bleStatus = 0;
      bleDisCount = 0;
      break;
    default:
      break;
  }
}

//This is called when bluetooth data is sent to Waldo. It responds to confirm receipt of data
void rxCallback(uint8_t *buffer, uint8_t len)
{
  Serial.print(F("Received "));
  Serial.print(len);
  Serial.print(F(" bytes: "));
  for(int i=0; i<len; i++)
   Serial.print((char)buffer[i]); 

  Serial.print(F(" ["));

  for(int i=0; i<len; i++)
  {
    Serial.print(" 0x"); Serial.print((char)buffer[i], HEX); 
  }
  Serial.println(F(" ]"));

  uart.print("Input received!");
  digitalWrite(bleLight, LOW);
  delay(70);
  digitalWrite(bleLight, HIGH);
  delay(3);
}

void EEPROMWritelong(int address, long value)
      {
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
      }

long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
      }
      
void setup() {
  //Initialize the accelerometer on I2C at the default address
  //Initialize with the I2C addr 0x3D (for 128x64 I2C OLED display)
  Serial.begin(115200);
  mma.begin();                                  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  delay(20);
  display.clearDisplay();             
  display.setTextSize(3); // Display start screen                 
  display.setTextColor(WHITE);
  display.setCursor(22,18);
  display.print("Waldo");
  display.display();
  display.clearDisplay();
  delay(2000);
  

  //Set the SD pin for output
  pinMode(SD_CS_PIN, OUTPUT);
  pinMode(bleLight, OUTPUT);

  //Initialize the SD card
  if (!SD.begin(SD_CS_PIN, 11, 12, 13)) {        
    Serial.println("error2");
    halt(F("Card init. failed!"));
  }

  //Create a new logfile on the SD card
  char filename[15];                            
  strcpy(filename, "Trip#00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    if (i > 95 ) {
      error = 1;
    }
    if (!SD.exists(filename)) {
      Serial.println("error3");
      delay(50);
      break;
    }
  }

  //Open the new logfile
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    halt(F("Failed to open log file!"));
    Serial.println("error4");
  }

  //These are the column headers for the output CSV file
  logfile.println("Latitude,Longitude,Date,Time(GMT),Speed(mph),Xaccel,Yaccel,Zaccel,Altitude(meters),Satellites,Note");
  logfile.flush();

  // Connect to the GPS receiver and configure it to receive location updates once a second.
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // RMC and GGA for altitude/satellite count.
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Once a second update rate.
  GPS.sendCommand(PGCMD_NOANTENNA);              // Turn off antenna status.
 
  // Configure timer0 to compare interrupt to run and parse GPS data every millisecond. SEE SIGNAL
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  uart.setRXcallback(rxCallback);      
  uart.setACIcallback(aciCallback);
  uart.setDeviceName("Waldo");
  uart.begin();

  //EEPROMWritelong(0, 0);   //uncomment this to force reset running total
}

void loop() {
  //poll for new data packets and check the pushbutton
  uart.pollACI();
  buttonState = digitalRead(buttonPin);

  //Check for new NMEA sentence. If we've got one, parse it
  if (GPS.newNMEAreceived()) {  
    GPS.parse(GPS.lastNMEA());
    uart.pollACI();
  }
  
  //If button is pressed, add current mileage to the running total and display the final total. Press again to reset to 0
  if (buttonState == HIGH) {
    displaySave();
    uart.pollACI();
    }

  // Periodically log the location ONLY if we have a fix, we're moving, and there's no miscalculation
  if (logCounter == 0) {
      //Check BLE status and turn on/off the indicator LED. Make a new sensor event get our accelerometer measurements
      bleCheck(); 
      sensors_event_t event;  
      mma.getEvent(&event);
      uart.pollACI();
       
      //Total and compare all of the forces on Waldo between logs. If there's a significant difference we know we've moved
      float newAccel = event.acceleration.x + event.acceleration.y + event.acceleration.z;
      if (oldAccel == 0) { //If we just powered on Waldo.
        oldAccel = newAccel; //Ignore the first forces difference. We just powered on and probably aren't moving anyways
        uart.pollACI();
      }
      float testAccel = (newAccel - oldAccel);
      oldAccel = newAccel;
      uart.pollACI();
        
      //If we've had a change in forces, we have a fix, and we're moving at an appropriate speed
      if ((testAccel >= 0.30 | testAccel <= -0.30) && GPS.fix && (GPS.speed)*mph > 1.2 && (GPS.speed)*mph < 150) {
        //This is our mock anti-theft capability. Expand our alert depending on application demands
        uart.pollACI();
        if (bleStatus ==  1) {
          uart.println("Waldo moved! ");
          uart.pollACI();
        }
             
        //Get our GPS calculations ready. Then log all of our measurements to the microSD
        float newFlat = GPS.latitudeDegrees;
        float newFlon = GPS.longitudeDegrees;
        if (lastFlat == 0 && lastFlon == 0) { //If we just turned on Waldo. 0,0 is the west coast of Africa +7,000 mi 
          lastFlat = GPS.latitudeDegrees;
          lastFlon = GPS.longitudeDegrees;
          uart.pollACI();
        }
        float distance = calc_dist(newFlat, newFlon, lastFlat, lastFlon);
        
        //IF we're in our very first iteration OR we've moved a reasonable distance in a max of 7 seconds
        if (distance == 0 | (distance >= 0.75 && distance <= 300)) {
          waldoStatus = 2;
          logLocation("Location", event.acceleration.x, event.acceleration.y, event.acceleration.z);
          lastFlat = newFlat;
          lastFlon = newFlon;
          totalDistance += distance;
          mileage = totalDistance / meters2miles;
          displayStuff(waldoStatus);
          uart.pollACI();
        }
      }
      
      //If we aren't moving.
      else if (GPS.fix && (GPS.speed)*mph > 0 && (GPS.speed)*mph <= 1.2) {
        uart.pollACI();
        waldoStatus = 1;                                             
        displayStuff(waldoStatus);
        
        logCounter = LOGGING_PERIOD_SEC*150;
        }
        
      //If we don't have a fix at all
      else if (!GPS.fix) {
        uart.pollACI();
        waldoStatus = 0;                                             
        displayStuff(waldoStatus);
        logCounter = LOGGING_PERIOD_SEC*200;
      }

      //If we missed all of our conditionals. Most likely meaning we got a bad NMEA sentence. Shit happens
      else {
        uart.pollACI();
        waldoStatus = -1;                                             
        displayStuff(waldoStatus);                                             
        logCounter = LOGGING_PERIOD_SEC*100;
      }
    }
  }
