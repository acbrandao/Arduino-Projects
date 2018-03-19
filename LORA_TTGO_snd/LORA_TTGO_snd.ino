/* Example from Sandeep Mistry 
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio
 * http://www.semtech.com/apps/product.php?pn=SX1276
 * RMB 29Nov2017
 */
#include <TinyGPS.h>
TinyGPS gps;

// note: ESp32 uses hardware serial not Softwareserial.h , function as .avaialble() and read() are the same
HardwareSerial hSerial(1);
static const int RXPin = 34, TXPin = 35;
static const uint32_t GPSBaud = 9600;

#include <time.h>
#include <sys/time.h>

#include <ArduinoJson.h>  //https://arduinojson.org/doc/encoding/ 
StaticJsonBuffer<200> jsonBuffer;  //Define JSON buffer used by ARduinojSON
char JSON[200];
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
#define LORA_SpreadingFactor  7 // ranges from 6-12, default 7 see API docs larger more range less data rate
#define FREQ 915E6  //Define Lora Frequency depens on Regional laws usually 433E6 , 866E8 or 915E6
#define BEACON_INTERVAL 15*1000  //Define how often to sen out the lora signal 

//Esp32 DeepSleep functions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

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


  Serial.println();
  Serial.println();


  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  displayString("LoRa SNDR","Frequency "+String(FREQ) );  
   Serial.println("LoRa Sender");

  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  //pinMode(blueLED, OUTPUT); // For LED feedback


  if (!LoRa.begin(FREQ)) {
    Serial.println("Starting LoRa failed!");
    displayString("TTGO LORA", "Failed");

    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(LORA_SpreadingFactor); // ranges from 6-12, default 7 see API docs
    displayString("LORA SFactor", (String) LORA_SpreadingFactor );

  // Change the transmit power of the radio
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
  // Most modules have the PA output pin connected to PA_BOOST, gain 2-17
  // TTGO and some modules are connected to RFO_HF, gain 0-14
  // If your receiver RSSI is very weak and little affected by a better antenna, change this!
  LoRa.setTxPower(17, PA_OUTPUT_RFO_PIN);
  
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  digitalWrite(blueLED, ON);  // Turn blue LED on
  

  //Get GPS data
   get_GPS_data();

  //packet contents
   temp_farenheit= temprature_sens_read();  //read the board temperature
   int measurement = 0;
   measurement = hallRead();

    counter++;  //bump up the loop count
   root["sensor_temp"] = String(temp_farenheit)+"ºF" ;
   root["counter"] = counter;
   root["sensor_magnetic"]=String(measurement);

   Serial.println("Message#: "+String(counter) );

   if (newData)
      root["msg"] ="GPS "+(String)gps.satellites() ;
    else
       root["msg"] ="M# "+(String)chars+" ";
    
    
   root.printTo(JSON);  //Now create a Serialized minified JSON string  
  
  //  displayString("PACKET","Bulding");
    LoRa.beginPacket();
    LoRa.print(JSON);  //print the entire JSON buffer to send
    LoRa.endPacket();
  
  digitalWrite(blueLED, OFF); // Turn blue LED off
 
 
  
   char frequencyband[4];
   ltoa(FREQ,frequencyband,10);
    Serial.println("LoRa sent JSON: "+String(FREQ).substring(0,3)+"Mhz, Packet#:"+(String)counter );
      Serial.println(JSON);

  //After sending a bunch of JSON beacons lets sleep to conserver battery
   if (counter % TIME_TO_SLEEP == 0)
   {

     Serial.println("Conserver battery ging to sleep now for "+String(TIME_TO_SLEEP) +  " Seconds");
     root["msg"]="Sleep "+(String)TIME_TO_SLEEP ;
     root.printTo(JSON);  //Now create a Serialized minified JSON string  
 
     LoRa.beginPacket();
     LoRa.print(JSON);  //print the entire JSON buffer to send
     LoRa.endPacket();
     delay(1000); //give radio some time to send it out
     
     esp_deep_sleep_start();
    
   }

   
   delay(1000); //now lets pause a bit to save battery 
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

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  temp_farenheit= temprature_sens_read();
  display.drawString(0, 50, "Radio:"+String(temp_farenheit)+"ºF" );
  
  // write the buffer to the display
  display.display();
  
  delay(10);
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
    unsigned long fix_age, time, date, speed, course;
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());


      
     // returns speed in 100ths of a knot
    speed = gps.speed();
       
      // course in 100ths of a degree
    course = gps.course();
    root["gps_signal"]= true;
    root["gps_age"]=age;
    root["gps_sats"]=gps.satellites();
    root["gps_speed"]= speed;
    root["gps_course"]= course;
    root["gps_lat"] =flat;
    root["gps_long"] =flon;
   }
   else
   {
   root["gps_signal"]= false;
   root["gps_sentence"]= (String)sentences;
   root["gps_checksum"]= failed;
   }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
    
 } //end of function
  
