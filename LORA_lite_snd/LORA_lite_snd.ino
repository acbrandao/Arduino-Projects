 /* 
 *  LoRA TTGO Lite Sender: Sends Lora RAdio encoded with minimal sensor data. 
 *  This code is setupto use a GPS sensor data.
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

#include <lora_config.h>  /* saved in /ibraries/loraconfig/lora_config.h and has common lora configuration */

/* note: ESp32 uses hardware serial not Softwareserial.h , 
 *  function as .avaialble() and read() perform the same thing  */

HardwareSerial hSerial(1);
static const int RXPin = 34, TXPin = 35;  //GPS Module TX and RX Pins attached to change as needed
static const uint32_t GPSBaud = 9600;

#include <time.h>
#include <sys/time.h>

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


//Esp32 DeepSleep functions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  15        /* Time s ESP32 will go to sleep (in seconds) */
#define TIME_TO_SEND  60        /* Time s the LoRa radio will transmit GPS signal befre sleeping */

RTC_DATA_ATTR int bootCount = 0;


#define OFF LOW   // For LED
#define ON HIGH

int counter = 0;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;

// Packet Buffer set to 51 bytes for max Spreading factor
// Pipe delimted format
// DeviceID | GPS  GOOD | MSG COUNT| SATS/MSG| Latitude | Longitude | Altitude | Age
//GPS01|1|26|7|04:01:03|40.801503|-74.16623|127.95275|


static const char DEVADDR[] = "GPS01";  //DeviceAddress ID

char buff[10]; // holds temporay float to string or other numeric conversions

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
  
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
 // displayString("LoRa LITE","Frequency "+String(FREQ) );  
  Serial.println("LoRa LITE Sender");

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  if (!LoRa.begin(FREQ)) {
    Serial.println("YIKES! Starting LoRa failed!");
//    displayString("TTGO LITE ", "Failed");
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
 Serial.printf("LoRa Radio ready %XHz Spreading Factor: %d TxPower: %d \n",FREQ,LORA_SpreadingFactor,LORA_TX_Power );
  Serial.print("--| Sending packet: |-----------------------------------");
   counter++;
   Serial.println("Message#: "+String(counter) );

   //Get GPS data

    get_GPS_data();  //read the GPS signal 
   
    if (!newData)  //We didn't get good data so lets send an empty
    {
      sprintf(packet_count,"%d",counter);
      sprintf(gps_signal,"%d",gps.location.isValid() );
      encodeGPSString();  //saves it into the TxBuffer
   }

    Serial.printf("Sending TxBuffer size:%d \n",strlen(TxBuffer));
   
    //Transmit this packet
     Serial.printf("LoRa sending ...");
    LoRa.beginPacket();
    LoRa.print(TxBuffer);  //print the entire tx buffer to send
    LoRa.endPacket();
    Serial.printf("sent! \n");

  //After sending a bunch of messages lets sleep to conserve battery
   if (counter % TIME_TO_SEND == 0)
   {

     Serial.println("Conserver battery ging to sleep now for "+String(TIME_TO_SLEEP) +  " Seconds");
      LoRa.sleep();
      esp_deep_sleep_start();
    
   }

   delay(1000); //now lets pause a bit to save battery 

}  //end of main loop

void get_GPS_data()
{
 char timestamp[10];
  
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
    sprintf(gps_lat,"%9.6f", gps.location.lat() );
    sprintf(gps_long,"%9.6f", gps.location.lng() );
    sprintf(gps_alt,"%9.6f", gps.altitude.feet() );

    Serial.println("--------------------------------------------------------");
    Serial.printf("COORDS: lat:%s  Long:%s   Alt:%s  \n",gps_lat,gps_long,gps_alt);
    Serial.println("--------------------------------------------------------");

     
    /* denote center coords to calculate  distance to HQ points and heading to
    double HQ_LAT=40.801621 ;
    double HQ_LONG=-74.166074;
  //  root["gps_dist"]=distanceBetweenPoints(HQ_LAT,HQ_LONG);
  //  root["gps_heading"]=courseToPoints(HQ_LAT,HQ_LONG);
    */
    } //if valid gps signal
      
    if (gps.date.isValid())
      {
       sprintf(gps_date,"%02d/%02d/%02d",gps.date.month(),gps.date.day(),gps.date.year() );
      Serial.println("GPS DATE: "+(String)gps_date);
      } //valid date

      if (gps.time.isValid())
      {
     
      sprintf(gps_time,"%02d:%02d:%02d",gps.time.hour(),gps.time.minute(),gps.time.second() );
        Serial.println("GPS TIME: "+(String)gps_time );
      }  //valid time

      sprintf(packet_count,"%d",counter);
      sprintf(gps_signal,"%d",gps.location.isValid() );
      sprintf(packet_message,"%d",gps.satellites.value() );
     
       //Now encode the data into a GPS message string Txbuffer
       encodeGPSString();
    }
    else
    {
    Serial.print(F("GPS Location NOT AVAILABLE"));
    }

  //Serial.printf(" CHARS=%s  SENTENCES=%s CSUM ERR=%s \n",gps.charsProcessed(),gps.sentencesWithFix(),gps.failedChecksum());
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

double distanceBetweenPoints(double p2_lat, double p2_long)
{
  double distanceKm=0.0;
  //static method require :: 
 distanceKm = TinyGPSPlus::distanceBetween( gps.location.lat(), gps.location.lng(), p2_lat,  p2_long) / 1000.0;
 
  return distanceKm;
}

double courseToPoints(double p2_lat, double p2_long)
{
double courseTo=0.0;

 courseTo =( !p2_lat ||  !p2_long   ) ?  0.0: TinyGPSPlus::courseTo( gps.location.lat(), gps.location.lng(), p2_lat, p2_long);

Serial.print("Human directions: ");
Serial.println(TinyGPSPlus::cardinal(courseTo));

return courseTo;
}


