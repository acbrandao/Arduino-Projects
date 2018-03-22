/* 
 *  LoRA TTGO Sender: Sends Lora RAdio encoded in JSON  of sensor data. This code is setup
 *  to use a GPS sensor data.
 *  
 *  Hardware: Lora ESP32 TTgo Board
 *  Dependencies: (GPS Sensor) TinyGPS++ https://github.com/mikalhart/TinyGPSPlus
 *  OLED requires alias for `#include "SSD1306Wire.h"
 *  LORA Radio   https://github.com/sandeepmistry/arduino-LoRa
 *   *  Example from Sandeep Mistry with mods from AliExpress/LilyGo docs  TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */
#include <TinyGPS++.h>
// The TinyGPS++ object
TinyGPSPlus gps;

/* note: ESp32 uses hardware serial not Softwareserial.h , 
 *  function as .avaialble() and read() perform the same thing  */

HardwareSerial hSerial(1);
static const int RXPin = 34, TXPin = 35;  //GPS Module TX and RX Pins attached to change as needed
static const uint32_t GPSBaud = 9600;

#include <time.h>
#include <sys/time.h>

/* ArduinoJson  Helps us create a convient JSON, may be too large for LoRA consider reducing
 *  */
 
#include <ArduinoJson.h>  //https://arduinojson.org/doc/encoding/ 
StaticJsonBuffer<255> jsonBuffer;  //Define JSON buffer used by ARduinojSON
char JSON[255];
JsonObject& root = jsonBuffer.createObject();



#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"
//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

SSD1306  display(0x3c, 4, 15);


// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)
#define LORA_SpreadingFactor  10 // ranges from 6-12, default 7 see API docs larger more range less data rate
#define LORA_TX_Power  17  // - TX power in dB, defaults to 17
#define FREQ 915E6  //Define Lora Frequency depens on Regional laws usually 433E6 , 866E8 or 915E6
#define BEACON_INTERVAL 15*1000  //Define how often to sen out the lora signal 

//Esp32 DeepSleep functions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  15        /* Time ESP32 will go to sleep (in seconds) */
#define TIME_TO_SCAN  60        /* Time for the GPS sreach will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();
//uint8_t g_phyFuns;

#ifdef __cplusplus
}
#endif

uint8_t temp_farenheit;
float temp_celsius;
char strftime_buf[64];
time_t now = 0;
struct tm timeinfo;
char buf[256];
char message[128];

#define OFF LOW   // For LED
#define ON HIGH
const int blueLED = 2; 
int counter = 0;


bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

void setup() {
  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
   //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds"); 
  
  hSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.begin( 115200 );
  
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  Serial.begin(115200);
  
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  displayString("LoRa SNDR","Frequency "+String(FREQ) );  
   Serial.println("LoRa Sender");

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  if (!LoRa.begin(FREQ)) {
    Serial.println("Starting LoRa failed!");
    displayString("TTGO LORA", "Failed");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(LORA_SpreadingFactor); // ranges from 6-12, default 7 see API docs

  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(LORA_TX_Power, PA_OUTPUT_RFO_PIN);
  
}

void loop() {

jsonBuffer.clear();
JsonObject& root = jsonBuffer.createObject();

  
  Serial.print("Sending packet: ");
  Serial.println(counter);

 
/*
 *  ESP32 board capabilities Hall sensor and temerature sensor
  
   int measurement = 0;
   measurement = hallRead();
   root["sensor_magnetic"]=String(measurement);

  //packet contents
   root["board_temp_f"]= temprature_sens_read();  //read the board temperature

   root["count"] = counter++;  //just indicate loop is moving
*/
    counter++;
   Serial.println("Message#: "+String(counter) );

    //Get GPS data
    get_GPS_data();

   if (newData)
      root["msg"] ="GPS "+(String)gps.satellites.value()+" x:"+(String)gps.satellites.age() ;
    else
       root["msg"] ="M# "+(String)chars+": Aquiring";

   
    root.printTo(JSON);  //Now create a Serialized minified JSON string  
    
    Serial.print("JSON buffer  size: ");
    Serial.println(jsonBuffer.size());
    
    //  displayString("PACKET","Bulding");
    LoRa.beginPacket();
    LoRa.print(JSON);  //print the entire JSON buffer to send
    LoRa.endPacket();
    
  
    char frequencyband[4];
    ltoa(FREQ,frequencyband,10);
    Serial.println("LoRa sent JSON: "+String(FREQ).substring(0,3)+"Mhz, Packet#:"+(String)counter );
    Serial.println(JSON);

  //After sending a bunch of JSON beacons lets sleep to conserve battery
   if (counter % TIME_TO_SCAN == 0)
   {

     Serial.println("Conserver battery ging to sleep now for "+String(TIME_TO_SLEEP) +  " Seconds");
     root["msg"]="Sleep "+(String)TIME_TO_SLEEP ;
     root.printTo(JSON);  //Now create a Serialized minified JSON string  
      LoRa.beginPacket();
     LoRa.print(JSON);  //print the entire JSON buffer to send
     LoRa.endPacket();
      esp_deep_sleep_start();
    
   }

   
   delay(1000); //now lets pause a bit to save battery 
}



void get_GPS_data()
{
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
   
    while (hSerial.available() )
    {
      // comment lines below out when debugging gps chip complete
      char c = hSerial.read();
     //   Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      // comment lines above when debugging gps chip complete
        
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    printfullGPSdata();  //optional used for debugging and testing
    
    Serial.print(F("Location: "));   //Valid Location data 
    if (gps.location.isValid())
    {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.println();
    
    root["gps_signal"]= true;
    root["gps_fix_age"]=gps.location.age(); //age() method, which returns the number of milliseconds since its last update
    root["gps_sats"]=gps.satellites.value();
    root["gps_mph"]= gps.speed.mph();
    root["gps_course"]= gps.course.deg();
    root["gps_alt_ft"]= gps.altitude.feet();
    root["gps_lat"] =gps.location.lat();
    root["gps_long"] =gps.location.lng();
  
    }
    else
    {
    Serial.print(F("GPS Location NOT AVAILABLE"));
    }

    //valid timestamp data
     
      if (gps.date.isValid())
      {
      char datetime[16];
       sprintf(datetime,"%02d/%02d/%02d",gps.date.month(),gps.date.day(),gps.date.year() );
       root["gps_date"]=(String)datetime;
       Serial.println("GPS DATE: "+String(datetime));
      }
      else
      {
        root["gps_date"]=(String)"Invalid Date";
      Serial.println(F("GPS DATE/TIME INVALID"));
      }

  if (gps.time.isValid())
  {
    char timestamp[16];
   sprintf(timestamp,"%02d:%02d:%02d",gps.time.hour(),gps.time.minute(),gps.time.second() );
   root["gps_time"]=(String)timestamp;
   Serial.println("GPS TIME: "+(String)timestamp );
     }
  else
  {
     root["gps_time"]=(String)"Invalid Time";
    Serial.println(F("GPS TIMESTAMP INVALID"));
  }
    
   
   }
   else
   {
   root["gps_signal"]= false;
   }
  
 
  //Serial.printf(" CHARS=%s  SENTENCES=%s CSUM ERR=%s \n",gps.charsProcessed(),gps.sentencesWithFix(),gps.failedChecksum());
 // root["gps_sentence"]= gps.sentencesWithFix();
  root["gps_chars"]= gps.charsProcessed();
  //root["gps_ksum"]= gps.failedChecksum();
  
 if (gps.charsProcessed() < 10)
      Serial.println("WARNING: No GPS data.  Check wiring.");

 } //end of function

 void  printfullGPSdata()
 {
Serial.print("TinyGPSPlus Version: ");
Serial.println(TinyGPSPlus::libraryVersion()); 
Serial.println("-------------------------------------");
Serial.print("Latitude : ");
Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
Serial.print("Longitude : ");
Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
Serial.print("Latitude  +/-:");
Serial.print(gps.location.rawLat().negative ? "-" : "+");
Serial.print("Raw latitude in whole degrees: ");
Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
// Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
Serial.print("longitude  +/-:");
Serial.print(gps.location.rawLng().negative ? "-" : "+");
Serial.print("Raw longitude in whole degrees: ");
Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
Serial.println("------| TIME   |------------------------------");

//Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
Serial.print("Raw date in DDMMYY format: ");
Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
Serial.print("Month (1-12): ");
Serial.println(gps.date.year()); // Year (2000+) (u16)
Serial.print("");
Serial.println(gps.date.month()); // Month (1-12) (u8)
Serial.print("Day (1-31): ");
Serial.println(gps.date.day()); // Day (1-31) (u8)
Serial.print(" Raw time in HHMMSSCC format : ");
Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.print("Hour (0-23) : ");
Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.print("Minute (0-59) : ");
Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.print("Second (0-59) : ");
Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.print("centisecond:  ");
Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)

Serial.println("------| SPEED/  ALT / COURSE  |------------------------------");
Serial.print(" Raw speed in 100ths of a knot : ");
Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
Serial.print("Speed in knots : ");
Serial.println(gps.speed.knots()); // Speed in knots (double)
Serial.print("Speed in miles per hour : ");
Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
Serial.print("Speed in meters per second : ");
Serial.println(gps.speed.mps()); // Speed in meters per second (double)
Serial.print("Speed in kilometers per hour : ");
Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
Serial.print("Raw course in 100ths of a degree : ");
Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
Serial.print("Course in degrees : ");
Serial.println(gps.course.deg()); // Course in degrees (double)
Serial.print("Raw altitude in centimeters: ");
Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
Serial.print("Altitude in meters : ");
Serial.println(gps.altitude.meters()); // Altitude in meters (double)
Serial.print("Altitude in miles : ");
Serial.println(gps.altitude.miles()); // Altitude in miles (double)
Serial.print("Altitude in kilometers : ");
Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
Serial.print("Altitude in feet: ");
Serial.println(gps.altitude.feet()); // Altitude in feet (double)

Serial.print("SATELLITES Fix Age:" );  //Age of last Sat Fix
Serial.println(gps.satellites.age());
Serial.print("Number of satellites in use: ");
Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
Serial.print("Horizontal Dim. of Precision (100ths) ):");
Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)

 }


  void displayString(String title, String body)
{
  Serial.println("DisplayString...");
  
  display.clear();  // clear the display
  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 5,  title);  
  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 20, body);

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 50, String(millis()/1000)+"s" );

/*  Lora ESP32 board temperature
 *   display.setTextAlignment(TEXT_ALIGN_LEFT);
  temp_farenheit= temprature_sens_read();
  display.drawString(0, 50, "Radio:"+String(temp_farenheit)+"ºF" );
*/  
  // write the buffer to the display
  display.display();
  
  delay(10);
}

