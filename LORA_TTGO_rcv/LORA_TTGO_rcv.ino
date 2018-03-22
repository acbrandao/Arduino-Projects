/* 
 *  TTGO LoRa Receive Sketch: Listens on FREQUENCY defined in FREQ and then displays received values
 */
#include <time.h>
#include <sys/time.h>

#include <ArduinoJson.h>  //https://arduinojson.org/doc/encoding/ 
DynamicJsonBuffer jsonBuffer;


#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <SSD1306.h>
//#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

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

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();
//uint8_t g_phyFuns;

#ifdef __cplusplus
}
#endif



 String rssi = "";
String packet = "";

uint8_t temp_farenheit;
float temp_celsius;
char strftime_buf[64];
time_t now = 0;
struct tm timeinfo;
char buf[256];

#define OFF LOW   // For LED
#define ON HIGH
const int blueLED = 2; 
int counter = 0;

void setup() {
  
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
  displayString("LoRa RECV","Frequency "+String(FREQ) );  
   Serial.println("LoRa Receiver");
   
 // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         

  displayString("LoRa RCV","waiting...");


  if (!LoRa.begin(FREQ)) {
    Serial.println("Receiving LoRa failed!");
     displayString("RCV LORA", "Failed");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(LORA_SpreadingFactor);  // ranges from 6-12, default 7 see API docs

  
}

void loop() {

static  int errorcount=0;
 // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) 
  {
    // received a packet
    
    Serial.print("Received packet '");
   //displayString("Packet","RCVD "+String(packetSize)+"bytes" );

    // read packet
    packet="";                   // Clear packet
    while (LoRa.available()) {
      packet += (char)LoRa.read(); // Assemble new packet
    }

    //try to convert to JSON object
     JsonObject& root = jsonBuffer.parseObject(packet.c_str());
    //root  = jsonBuffer.parseObject(packet.c_str());


   //too many errors
   if (errorcount==10)
   {
      Serial.println("Restarting too many errors.");
      ESP.restart();
   }
        
   if (!root.success()) {
      // Parsing failed :-(
         Serial.println("** FAILED**  JSON BAD  packet\n '"+String(packet )+" Size:"+(String)packetSize);
          displayString("Bad Packet - offline?","Size: "+(String)packetSize );
          errorcount++;
     }
    else
    {
       rssi = LoRa.packetRssi();

    // Extract JSON Info
    const char* temp= root["temp"];
    const char* msg= root["msg"];
    bool gps_signal=root["gps_signal"];
    int gps_sats=root["gps_sats"];
    long counter = root["counter"];
    float gps_lat=root["gps_lat"];
    float gps_long=root["gps_long"];
    int gps_sentences= root["gps_sentence"];
    int gps_mph=root["gps_mph"];
    int gps_bearing=root["gps_bearing"];

    
    
    /* Available json values
     *  
     *  
   root["sensor_temp"] = String(temp_farenheit)+"ºF" ;
   root["msg"] = "LoRaGPS ";
   root["counter"] = counter;
   root["sensor_magnetic"]=String(measurement);
     *  root["gps_signal"]= true;
    root["gps_age"]=age;
    root["gps_sats"]=gps.satellites();
    root["gps_speed"]= speed;
    root["gps_course"]= course;
    root["gps_lat"] =flat;
    root["gps_long"] =flon;
      root["gps_signal"]= false;
   root["gps_sentence"]= (String)sentences;
   root["gps_checksum"]= failed;
     * 
     */


    Serial.println("LoRa Receiver");
    Serial.println("Received packet:");
    Serial.println("    '" + packet + "'");
    Serial.println("RSSI " + rssi);
    
    Serial.println(packet + "' with RSSI " + rssi);     
    char frequencyband[4];
     ltoa(FREQ,frequencyband,10);

   if ( gps_signal )
    {
     //convert lat long to dispalyable string9
    String flat =(String)gps_lat;
    String flong =(String)gps_long;
    String LatLong= flat.substring(0,4)+" "+flong.substring(0,4); 
    displayString("GPS Sat:"+(String)gps_sats+" "+(String)counter,(String)gps_mph+ "MPH H" +(String)gps_bearing );
    }
   else
   displayString("GPS MSG: "+(String)gps_sentences, (String)msg+ (String)counter );
  
    } //success JSON parse
  }
}

void displayString(String title, String body)
{
 
  display.clear();  // clear the display
  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 5,  title);  
  
   display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawStringMaxWidth(0, 20, 128,body );

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 50, String(TimeToString(millis()/1000) ) );


 display.setTextAlignment(TEXT_ALIGN_LEFT);
  temp_farenheit= temprature_sens_read();
  display.drawString(0, 50, "RSSI :"+String(  LoRa.packetRssi() ) );
  
  // write the buffer to the display
  display.display();
  
  delay(10);
}

// t is time in seconds = millis()/1000;
char * TimeToString(unsigned long t)
{
 static char str[12];
 
 long h = t / 3600;
 long d = h / 24;
 t = t % 3600;
 int m = t / 60;
 int s = t % 60;
 sprintf(str, "%02ld:%02d:%02ds",  h, m, s);
 return str;
}
