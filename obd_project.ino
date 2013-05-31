/*
  CS497 Spring 2011 Project:
  Arduino-based OBD-II interface, display, and data logger
  By: Ryan Miller
  
  ISO9141 Initialization adapted from OBDuino Project
  http://code.google.com/p/opengauge/wiki/OBDuino
*/

#define K_IN    0
#define K_OUT   1
#define DEBOUNCE_DELAY 150
#define ISO_DELAY 55
#define POLL_INTERVAL 100 // time in ms between PID polling
#define NUM_PIDS 6  // Current number of PIDs polled
#define MAX_PIDS 16  // maximum number of PIDs for the device to poll

/* Supported PIDs */
#define ENGINE_RPM 0x0C
#define VEHICLE_SPEED 0x0D
#define ENGINE_COOLANT_TEMP 0x05
#define MAF_AIRFLOW 0x10
#define THROTTLE_POS 0x11

/* Don't work on a 2005 Toyota Matrix */
#define BAROMETRIC_PRESSURE 0x33
#define CONTROL_VOLTAGE 0x42 

// Data logging capabilities
#define ENABLE_LOGGING  0 // turns on/off logging to SD card


#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4,5,6,7,8,9);

volatile byte mode; // Current mode of operation
/*
  Mode 0: Pre-initilization
  Mode 1: Initializing
  Mode 2: ECU connected, polling
*/
volatile long lastDebounceTime = 0;  // Hold value for button interrupt
volatile boolean cancelled = false;  // Interrupted ISO init
volatile int screen = 0;  // which screen is currently selected
int buttonInterrupt = 1;  // Button interrupt value (Arduino pin 3)
int initAttempts = 0;  // Counter for how many attempts it took to initialize with the ECU
boolean isConnected = false;  // Whether or not ECU is connected

// Real-Time clock object
RTC_DS1307 RTC;

// The logging file
File logfile;

DateTime now;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// pin 2 is attached to a red LED for error status
const int errorLED = 2;

// Debugging
int numInvalid = 0;

// Type pidInfo to hold PID information
typedef struct pidInfo
{
  char *name;    // full name
  char *unit;    // result unit
  byte pid;      // pid hex number
  byte length;   // return result length
  double result;   // returned result
  // TOOD: byte encoding method
};

pidInfo pidArray[MAX_PIDS];

void error(char *str)
{
  lcd.clear();
  lcd.print("Error:");
  lcd.setCursor(0,1);
  lcd.print(str);
  
  // red LED indicates error
  digitalWrite(errorLED, HIGH);

  while(1);
}

void setup() {
  // set up K-line
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
  pinMode(errorLED, OUTPUT);
  
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelect, OUTPUT);
  
  numInvalid = 0;
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Attach button interrupt
  attachInterrupt(buttonInterrupt, switchScreen, RISING);
  // Set up debouncing
  lastDebounceTime = millis();
  

  // Add PIDs to poll
  addPID("Engine RPM", "rpm", ENGINE_RPM, 2);
  addPID("Vehicle Speed", "mph", VEHICLE_SPEED, 1);
  addPID("Eng Coolant Temp", "C", ENGINE_COOLANT_TEMP, 1);
  addPID("MPG", "mpg", MAF_AIRFLOW, 2);
  
  //addPID("Throttle Pos", "%", THROTTLE_POS, 1);
  
  //startup();  // Display startup screen

  // Start init mode
  mode = 1;

  // DEBUG
  // skip to mode 2
  //mode = 2;
}

void loop() {
  // Main loop
  
  switch(mode)
  {
    case 0:  // pre-init mode
      // wait for init
      break;
    case 1:  // initialize
      attemptConnect();
      break;
    case 2:  // ECU connected
      pollPIDArray();
      
      drawScreen();
      
#if ENABLE_LOGGING

      // transfer values from PIDArray to SD card     
      logValues();
      
#endif //enable_logging
      
      delay(POLL_INTERVAL);
      
      break;
    default:
      break;
      
  }

}

void startup()
{
  lcd.print("Press Button");
  lcd.setCursor(0,1);
  lcd.print("to init. ISO9141");
}

void attemptConnect()
{
  lcd.clear();
  lcd.print("Initializing...");
  
  while (!isConnected)
  {
    if (cancelled)
    {
      lcd.clear();
      cancelled = false;
      initAttempts = 0;
      startup();
      return;
    }
    lcd.setCursor(0,1);
    lcd.print(initAttempts);
    isConnected = initialize_iso();
  }
  
  // Success
  lcd.clear();    
  mode = 2;
  isConnected = true; 
  
#if ENABLE_LOGGING

  setupLogging(); 
  
#endif //enable_logging

}

boolean addPID(char *name, char *unit, byte pid, byte length)
{
  boolean isAdded = false;
  for (int i = 0; i < NUM_PIDS; i++)
  {
    if (pidArray[i].name == NULL)
    {
      pidArray[i].name = strdup(name);
      pidArray[i].unit = strdup(unit);
      pidArray[i].pid = pid;
      pidArray[i].length = length;
      isAdded = true;
      break;
    }
  }
  return isAdded;
}

void setupLogging()
{
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card init fail");
  }
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("Can't create file");
  }
  
  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    error("RTC failed");
  }
  
  // setup the CSV format
  logfile.print("datetime");
  
  for (int i = 0; i < NUM_PIDS; i++)
  {
    if (pidArray[i].name != NULL)
    {
      logfile.print(",");
      logfile.print(pidArray[i].name);
    }
  }   
  
  logfile.println("");
}

void pollPIDArray()
{
  for (int i = 0; i < NUM_PIDS; i++)
  {
    if (pidArray[i].name != NULL)
    {
      // poll for PID
      requestPID(pidArray[i].pid);
      
      byte buf[pidArray[i].length];   // to receive the result 
      byte returnLength = pidArray[i].length;
      if (iso_read_data(buf, returnLength) != returnLength)
      {
        //lcd.print("ERROR");
        pidArray[i].result = -1;
        numInvalid++;
        
        if (numInvalid = 10)
          error(">10 Invalid");
      }
      else  // TODO: better way to handle the byte encoding
      {
        numInvalid = 0;
        if (!strcmp(pidArray[i].name, "Engine RPM"))
        {
          double rpm;
          rpm = (((double)buf[0] * 256) + (double)buf[1]) / 4.0;
          pidArray[i].result = rpm;
        }
        else if (!strcmp(pidArray[i].name, "Vehicle Speed"))
        {
          double mph;
          mph = (double)buf[0] / 1.609344;  // Convert from km/h to mph
          pidArray[i].result = mph;
        }
        else if (!strcmp(pidArray[i].name, "Eng Coolant Temp"))
        {
          double degree;
          degree = (double)buf[0] - 40.0;
          pidArray[i].result = degree;
        }
        else if (!strcmp(pidArray[i].name, "MPG"))
        {
          // Compute MAF in g of air /second
          double maf;
          maf = (((double)buf[0] * 256.0) + (double)buf[1]) / 100.0;
          
          // convert to MPG (MPG = MPH * 1/GPH)
          // where GPH = MAF * .0889
          
          // CHANGE INDEX TO INDEX OF VEHICLE SPEED PID
          double mpg = pidArray[1].result * (1 / (maf * .0889));
          
          pidArray[i].result = mpg;
        }
        else if (!strcmp(pidArray[i].name, "Throttle Pos"))
        {
          double pos;
          pos = (double)buf[0] * 100.0 / 255.0;
          pidArray[i].result = pos;
        }
        else
        {
          // Don't know how to encode unsupported PID
          pidArray[i].result = -1;
        }
      }
    }
  }  
}

// Assumes 16x2 LCD screen
void drawScreen()
{
  int row = 0;
  int col = 0;
  lcd.clear();
  for (int i = screen * 4; i < ((screen * 4) + 4); i++)
  {
    if (pidArray[i].name != NULL)
    { 
      switch (i % 4)
      {
        case 0:
          lcd.setCursor(0,0);
          //lcd.print(pidArray[i].result, DEC);
          lcdPrintDouble(pidArray[i].result, 1);
          lcd.setCursor(5,0);
          lcd.print(pidArray[i].unit);
          break;
        case 1:
          lcd.setCursor(9,0);
          //lcd.print(pidArray[i].result, DEC);
          lcdPrintDouble(pidArray[i].result, 1);
          lcd.setCursor(13,0);
          lcd.print(pidArray[i].unit);
          break;
        case 2:
          lcd.setCursor(0,1);
          //lcd.print(pidArray[i].result, DEC);
          lcdPrintDouble(pidArray[i].result, 1);
          lcd.setCursor(5,1);
          lcd.print(pidArray[i].unit);
          break;
        case 3:
          lcd.setCursor(9,1);
          //lcd.print(pidArray[i].result, DEC);
          lcdPrintDouble(pidArray[i].result, 1);
          lcd.setCursor(13,1);
          lcd.print(pidArray[i].unit);
          break;
        default:
          break;
      }
    }
  }
  
}

// Takes values from PIDArray and puts them into CSV spreadsheet
void logValues()
{
  // fetch the time
  now = RTC.now();
  
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
  
  for (int i = 0; i < NUM_PIDS; i++)
  {
    if (pidArray[i].name != NULL)
    {
      logfile.print(",");
      if (pidArray[i].result != -1)  // Don't log invalid values
        logfile.print(pidArray[i].result);
    }
  }   
  
  logfile.println("");
}



// Button interrupt handler
void switchScreen()
{
  int maxscreens;
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    // actual button press
    
    switch(mode)
    {
      case 0:  // pre-init mode
        mode = 1;  // put into init mode
        break;
      case 1:  // initializing
        // if button pressed, cancel init
        lcd.clear();
        lcd.print("Init. Cancelled");
        mode = 0;
        cancelled = true;
        break;
      case 2:  // ecu connected, main display
        // if button pressed, switch screen
        maxscreens = NUM_PIDS / 4;
        if (NUM_PIDS % 4 != 0)
          maxscreens++;
        
        screen = (screen + 1) % maxscreens;
        
        break;
      case 3:
        break;
      default:
        break;
        
    }
  }
  
  lastDebounceTime = millis();
}

void lcdPrintDouble( double val, byte precision){
  // prints val on a ver 0012 text lcd with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: printDouble( 3.1415, 2); // prints 3.14 (two decimal places)

  if(val < 0.0){
    lcd.print('-');
    val = -val;
  }

  lcd.print (int(val));  //prints the int part
  if( precision > 0) {
    lcd.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
  mult *=10;

    if(val >= 0)
 frac = (val - int(val)) * mult;
    else
 frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
 padding--;
    while(  padding--)
 lcd.print("0");
    lcd.print(frac,DEC) ;
  }
}

/*********************************************
  ISO-9141 functions
*********************************************/


// read n byte(s) of data (+ header + cmd and crc)
// return the count of bytes of message (includes all data in message)
byte iso_read_data(byte *data, byte len)
{
  byte i;
  byte buf[20];
  byte dataSize = 0;

  // header 3 bytes: [80+datalen] [destination=f1] [source=01]
  // data 1+1+len bytes: [40+cmd0] [cmd1] [result0]
  // checksum 1 bytes: [sum(header)+sum(data)]
  // a total of six extra bytes of data

  for(i=0; i<len+6; i++)
  {
    if (iso_read_byte(buf+i))
    {
      dataSize++;
    }
  }

  // test, skip header comparison
  // ignore failure for the moment (0x7f)
  // ignore crc for the moment

  // we send only one command, so result start at buf[4] Actually, result starts at buf[5], buf[4] is pid requested...
  memcpy(data, buf+5, len);

  delay(ISO_DELAY);    //guarantee 55 ms pause between requests

  return dataSize - 6; // return payload length
}

void requestPID(byte pid)
{
  /* 
    Byte   0: Message Header 1... 0x68
           1: Message header 2... 0x6A for OBDI-II request
           2: Source address ... 0xF1 for off-board tool
           3-9: Data
             with  3: 0x01, get PID
                   4: pid in hex
           Final: checksum
  */
  byte message[6];
  message[0] = 0x68;
  message[1] = 0x6A;
  message[2] = 0xF1;
  message[3] = 0x01;
  message[4] = pid;
  message[5] = iso_checksum(message, 5);
  
  // write message to ECU
  for (int i = 0; i < 6; i++)
  {
    iso_write_byte(message[i]);
  }  
}


// inspired by SternOBDII\code\checksum.c
byte iso_checksum(byte *message, byte index)
{
  byte i;
  byte crc;

  crc=0;
  for(i=0; i<index; i++)
    crc=crc+message[i];

  return crc;
}

void serial_rx_on() {
  Serial.begin(10400);
}

void serial_rx_off() {
  UCSR0B &= ~(_BV(RXEN0));  //disable UART RX
}

void serial_tx_off() {

   UCSR0B &= ~(_BV(TXEN0));  //disable UART TX
   delay(20);                 //allow time for buffers to flush
}

#define READ_ATTEMPTS 125

// User must pass in a pointer to a byte to recieve the data.
// Return value reflects success of the read attempt.
boolean iso_read_byte(byte * b)
{
  int readData;
  boolean success = true;
  byte t=0;
  
  // 125 ms worth of attempts
  for (t = 0; t < READ_ATTEMPTS; t++)
  {
    if (Serial.available())
    {
      readData = Serial.read();
      break; 
    }
    delay(1); 
  }
  
  if (t >= READ_ATTEMPTS) 
  {
    success = false;
  }
  
  if (success)
  {
    *b = (byte) readData;
  }

  return success;
}

void iso_write_byte(byte b)
{
  serial_rx_off();
  Serial.print(b);
  delay(10);		// ISO requires 5-20 ms delay between bytes.
  serial_rx_on();
}

// Adapted from OBDuino Project
boolean initialize_iso()
{
  initAttempts++;
  
  byte b;
  byte kw1, kw2;
  serial_tx_off(); //disable UART so we can "bit-Bang" the slow init.
  serial_rx_off();
  delay(3000); //k line should be free of traffic for at least two secconds.
  // drive K line high for 300ms
  digitalWrite(K_OUT, HIGH);
  delay(300);

  // send 0x33 at 5 bauds
  // start bit
  digitalWrite(K_OUT, LOW);
  delay(200);
  
  // data
  b=0x33;  // 11 00 11 00
  
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      digitalWrite(K_OUT, HIGH); // send 1
    else
      digitalWrite(K_OUT, LOW); // send 0
    delay(200);
  }
  // stop bit
  digitalWrite(K_OUT, HIGH);
  delay(260);

  // switch now to 10400 bauds
  Serial.begin(10400);
  
  //delay(50);

  // wait for 0x55 from the ECU (up to 300ms)
  // since our time out for reading is 125ms, we will try it three times
  byte i=0;
  bool read55 = iso_read_byte(&b);
  while(i < 3 && !read55)
  {
    i++;
    read55 = iso_read_byte(&b);
  }

  if(b!=0x55)
  {
    return false;
  }

  // wait for kw1 and kw2
  iso_read_byte(&kw1);

  iso_read_byte(&kw2);
  
  delay(25);

  // send ~kw2 (invert of last keyword)
  iso_write_byte(~kw2);

  // ECU returns 0xCC (~0x33)
  i = 0;
  bool readCC = iso_read_byte(&b);
  while (i < 3 && !readCC)
  {
    i++;
    readCC = iso_read_byte(&b);
  }
  
  if(b==0xCC)
    return true;
  else
    return false;
}
