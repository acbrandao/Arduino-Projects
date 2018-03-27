/*
 LoRa_gps_sndrcv:A sketch for TTGO-Lora-OLED boards that makes use of a GPS moule to transmit data 
     every few minutes
     Sketch used IFDEF HAS_GPS_CHIP and IS_SENDER to determine type of node
   Sends a message every half second, and uses callback
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.  based on LoRa Duplex communication wth callback   created 28 April  author  by Tom Igoe

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

 
*/

#define HAS_GPS_CHIP 1        //define for use with  GPS library or not
#define SENDER_NODE  1          //define or comment out toggle  sender and receiver nodes
#define HAS_OLED                //Define if this has an sdd1306 OLED

#include <SPI.h>              // include libraries
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <LoRa.h>
#define LORA_TX_Power  17  // - TX power in dB, defaults to 17 laws usually 433E6 , 866E6 or 915E6
#define LORA_SpreadingFactor  11 // MUST MATCH Sender: ranges from 6-12, default 7 see API docs larger more range less data rate
#define FREQ 866E6  //MUST MATCH Sender: Lora Frequency depends region: 433E6  (Asia), 866E8 (Eur) or 915E6 (US)
#define BEACON_INTERVAL 2*1000  //Define how often to sen out the lora signal 

// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 -  SX1276 MOSI
#define LORA_CS 18     // GPIO18 -   SX1276 CS
#define LORA_RST 14   // GPIO14 -    SX1276 RST
#define LORA_IRQ 26  // GPIO26 -     SX1276 IRQ (interrupt request)

#ifdef HAS_GPS_CHIP

#include <TinyGPS++.h>
TinyGPSPlus gps;   // The TinyGPS++ object  ig GPS sensor attached

static const int RXPin = 34, TXPin = 35;  //GPS Module TX and RX Pins attached to these pins
HardwareSerial hSerial(1);
static const uint32_t GPSBaud = 9600;  //standard read data from GPS radio
#endif

#ifdef SENDER_NODE
byte localAddress = 0xBB;     // SENDER: address of this device
byte destination = 0xFF;      // RECEIVER: destination to send to
#define ANNOUNCE_MESSAGE  "LoRa SENDER"
#else
byte localAddress = 0xFF;     // RECEIVER: address of this device
byte destination = 0xBB;      // SENDER ADDRESS: destination to send to
#define ANNOUNCE_MESSAGE  "LoRa RECEIVER"
#endif

#ifdef HAS_OLED
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
#endif


//Esp32 DeepSleep functions
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  15        /* Time s ESP32 will go to sleep (in seconds) */
#define TIME_TO_SEND  60        /* Time s the LoRa radio will transmit GPS signal befre sleeping */

RTC_DATA_ATTR int bootCount = 0;


String outgoing;              // outgoing message
byte msgCount = 0;            // count of outgoing messages
long lastSendTime = 0;        // last send time
int interval = BEACON_INTERVAL;          // interval between sends

 /*  Defines GPS specific fields used for both sender and reciever*/
char gps_sats[3]="00";
char gps_signal[2]="0";
char gps_time[10]="00:00:00";
char gps_date[10]="00-00-00";
char gps_lat[16]="00.000";
char gps_long[16]="00.000";
char gps_message[16]="***";  //used to hold no  signal messages
char gps_alt[12]="0.0";
char gps_age[10]="00"; 
char timestamp[10];

/*  setup() : Progrma on startup mde*/
void setup() {
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

 #ifdef HAS_GPS_CHIP
  hSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);   //setup the Serial reaad form GPS chip
 #endif

 #ifdef HAS_OLED
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
 #endif

   //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("------------------------------------------------------------");
  Serial.println(String(ANNOUNCE_MESSAGE)+"  "+String(FREQ).substring(0,3)+"Mhz SF:"+String(LORA_SpreadingFactor));  //addounce the type of sender
  Serial.println("------------------------------------------------------------");
   displayOLED(String(ANNOUNCE_MESSAGE)+" "+String(FREQ).substring(0,3)+"Mhz  SF:"+String(LORA_SpreadingFactor),"Ready...");

   
  // Very important for SPI pin configuration!
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS); 
  // Very important for LoRa Radio pin configuration! 
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); 


  if (!LoRa.begin(FREQ)) {
    Serial.println("YIKES! Starting LoRa failed!");
//    displayString("TTGO LITE ", "Failed");
    while (1);
  }
  
  //larger the spreading factor = greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(LORA_SpreadingFactor); // ranges from 6-12, default 7 see API docs

  #ifdef SENDER_NODE  
  // Default is LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);
   LoRa.setTxPower(LORA_TX_Power, PA_OUTPUT_RFO_PIN);
  #endif 

 

  //LoRa.onReceive(onReceive);  //register the Callback
  LoRa.receive();
  Serial.println("LoRa init succeeded.");

 
} //end of setup()

/*  loop() : Main Program loop */
void loop() {

 
  if (millis() - lastSendTime > interval) {
    #ifdef SENDER_NODE  
    String message = "HeLo "+String( millis()/1000)  ;   // send a message

    #ifdef HAS_GPS_CHIP
    message=get_GPS_data();   //lets grab GPS data if we have a chip
    #else
    message="LoRa "+String(msgCount);
    #endif
    
    sendMessage(message);   // SEnd out message through radio
    
    lastSendTime = millis();            // timestamp the message
    interval = random(BEACON_INTERVAL) + 1000;     // 2-3 seconds  
    LoRa.receive();                     // go back into receive mode
    #else
      // RECEIVER try to parse packet
      int packetSize = LoRa.parsePacket();
      onReceived(packetSize);
      //Serial.println("Rx "+String(FREQ).substring(0,3)+"Mhz SF:"+String(LORA_SpreadingFactor) );
    //delay(500);    
    #endif
  }


}

void sendMessage(String outgoing) {

#ifdef SENDER_NODE  
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);    // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID

 Serial.println("#"+String(msgCount)+" To: "+String(destination,HEX)+ "\n msg:"+String(outgoing) );
#endif
}

void onReceived(int packetSize) {
   
  if (packetSize == 0) 
   {
    //displayOLED("Waiting for Message","Empty Packet");
    return;          // if there's no packet, return
   }
  Serial.println("Received packet size:"+(String)packetSize );
  
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
      displayOLED("Received from: 0x" + String(sender, HEX) ,"Corrupt Packet ");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));

  //is this a GPS message
   if ( validGPS_message(incoming) )       //Is this a valid GPS packet?
    {
       Serial.println("GPS SIGNAL: " + incoming);
       displayOLED("GPS "+String(gps_time)+" "+(String)packetSize+"b" ,String(gps_lat).substring(0,5)+"/"+String(gps_long).substring(0,6) ); 
    }
  else
  {
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  displayOLED("Sent:" + String(sender, HEX)+" "+(String)packetSize+"b",String( incoming).substring(0,9) ); 
  }
  
}

void displayOLED(String title, String body)
{
#ifdef HAS_OLED
 // Serial.println("displayOLED...");
  
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

  //bottom left rssi
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 50, "RSSI :"+ String(LoRa.packetRssi()) +" SnR:"+String(LoRa.packetSnr()) ); 

  // write the buffer to the display
  display.display();
  
#endif
}

bool validGPS_message(String gpsdata)
{
int valid=false;
char msg[64];  //assign a c string to gps data

  strcpy(msg,gpsdata.c_str() );
int n = sscanf(msg,"%[^|]|%[^|]|%[^|]|%[^|]|%s",gps_signal ,gps_lat,gps_long,gps_alt,gps_time);

Serial.printf("--SSCANF RESULT ---------------------------------------------------------------- \n" );
Serial.printf("%s|%s|%s|%s|%s \n",gps_signal ,gps_lat,gps_long,gps_alt,gps_time);
Serial.printf("--ORIGINAL MESSAGE ---------------------------------------------------------------- \n" );
Serial.println(gpsdata);
Serial.printf("------------------------------------------------------------------------ \n" );

valid=atoi(gps_signal);  //convert the GPS signal

if (valid==false)
  strcpy(gps_message,gps_lat);  // copy the message when not available
  

return valid;  
}

#ifdef HAS_GPS_CHIP
String get_GPS_data()
{
bool newData=false;
String gps_message="0|no gps signal";  //default message 0=no signal
  
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
    if (gps.location.isValid())
    {
    sprintf(gps_lat,"%9.6f", gps.location.lat() );
    sprintf(gps_long,"%9.6f", gps.location.lng() );
    sprintf(gps_alt,"%9.6f", gps.altitude.feet() );

    Serial.println("--------------------------------------------------------");
    Serial.printf("COORDS: lat:%s  Long:%s   Alt:%s  \n",gps_lat,gps_long,gps_alt);
    Serial.println("--------------------------------------------------------");
   } //if valid gps signal
      
    if (gps.date.isValid())
      {
       sprintf(gps_date,"%02d/%02d/%02d",gps.date.month(),gps.date.day(),gps.date.year() );
      Serial.println("VALID GPS DATE: "+(String)gps_date);
      } //valid date

      if (gps.time.isValid())
      {
     
      sprintf(gps_time,"%02d:%02d:%02d",gps.time.hour(),gps.time.minute(),gps.time.second() );
        Serial.println("VALID GPS TIME: "+(String)gps_time );
      }  //valid time

     
      sprintf(gps_signal,"%d",gps.location.isValid() );
      sprintf(gps_sats,"%d",gps.satellites.value() );
     
       //Now encode the data into a GPS messagePipe delimited format
      // GPS SIGNAL|  Latitude | Longitude | Altitude | Time
       gps_message=String(gps_signal)+"|"+String(gps_lat)+"|"+String(gps_long)+"|"+String(gps_alt)+"|"+(String)gps_time;
      return gps_message;      
       
    }
    else
    {
    Serial.println("GPS Location NOT AVAILABLE");
    }

  //Serial.printf(" CHARS=%s  SENTENCES=%s CSUM ERR=%s \n",gps.charsProcessed(),gps.sentencesWithFix(),gps.failedChecksum());
 if (gps.charsProcessed() < 10)
      Serial.println("WARNING: No GPS data.  Check wiring.");
 
 return gps_message;
 } //end of function

#endif
