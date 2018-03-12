
extern "C" {
#include "user_interface.h"  //used to acccess RTC variable storage
}

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Time.h>
#include "wifi_config.h"  // Include file wifi_config.h contains WiFi credentials same as below 

// replace the  // with #define and update the credentials to match your environment
// --------- cut here and save as  wifi_config.h ---------
//define wifi_ssid "YOUR_AP_SSID"
//define wifi_password "YOUR_AP_PASSWORD"
// --------- cut here and save as  wifi_config.h ---------


#define mqtt_server "broker.hivemq.com"  //cloud MQTT server
// define mqtt_server "192.168.1.190"  // local MQTT Server

#define mqtt_user ""
#define mqtt_password ""
#define mqtt_retry_delay 5000 // in milliseconds wait till re-connecting to broker
#define mqtt_retry_attempts 2 //after this many attemps go to sleep

WiFiClient espClient;
PubSubClient client(espClient);

// RTC data must be i 4-byte chunks /buckets, be sure to pad as needed

 struct {
  uint32_t crc32;                     // 4 bytes checksum to make sure data stored is valids
  boolean rtc_running = false;        //1 byte
  boolean rtc_lastrun_state= false;   //1 byte
  uint16_t padding=0;              //2 bytes
  uint32_t rtc_vibration_count=0; //4 bytes  
  uint16_t rtc_bootcount=0;          //2 bytes counter 
  uint16_t rtc_bootcount_last=0;   //2bytes counter 
  } rtcData,  *pRTC;                         //total 8 bytes for structure

const unsigned long SLEEP_INTERVAL = 15 * 1000 * 1000; // 5 sec
const unsigned long  MEASURE_INTERVAL = 15 ; // how many tries times to measure vibration before going to sleep
int RUN_COUNT_THRESHOLD= MEASURE_INTERVAL/2;  //Arbitrary generally half the measure interval 

int SENSOR_PIN_VIBRATION = 5;    // select the input pin for the vibration
int PHOTO_SENSOR_PIN=A0 ; //photo resistor sensor pin - Analog reading


int//Global Constants
long counter=0;
boolean running = false;  //is the machine running (aka vibrating)
boolean last_running_state=running;  //keeps track fo state changes
int  continuos_vibration_count=0;  //how many times did we count the machine vibrating

long start_time =0; //set on start of script

int idle_message_timeout=15;  //number of seconds to reset the done message 
int last_millis=0; //keeps track of last milliseconds to idle timeout count

bool setup_wifi() {
  int wifi_retries=20;
  int count=0;
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    count++;

    if (count > wifi_retries)
    {
    Serial.println("Unable to Connect to WiFi .. check AP");
     return false;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}

void reconnect() {
  int tries=0;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.printf("Attempting MQTT connection to: %s ...",mqtt_server);
  //  if (client.connect("")) {
   if (client.connect("ESP8266123")){ 
      Serial.println("connected");
    } else {
      tries++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.printf(" try again in %d seconds",mqtt_retry_delay);
    
      delay(mqtt_retry_delay);    // Wait n seconds before retrying

      if ( tries > mqtt_retry_attempts)
      {
         Serial.println("Too Many MQTT retries, aborting...");
         return ;
      }
        
    }
  }
}

//Connectes to WiFi and MQTT broker
void  connectMQTT()
{
setup_wifi();  //connect to wifi and send data
client.setServer(mqtt_server, 1883);
 if (!client.connected()) {
  reconnect();
 }
   client.loop();
}

// start of setup routine
void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_PIN_VIBRATION, INPUT);
  pinMode(PHOTO_SENSOR_PIN, INPUT);

  start_time = millis();  

  boolean rtcValid=readRTC();

  if (rtcValid==true) 
  {
    //assign local variables to rtc values -- refactor may not need local variables 
   running=  rtcData.rtc_running; 
   last_running_state= rtcData.rtc_lastrun_state;
   continuos_vibration_count= rtcData.rtc_vibration_count;
   rtcData.rtc_bootcount++;
  }
  
  Serial.printf("\nREAD RTC Values:  running:%d  RTCLastState:%d  RTCVirbCount:%d   boots: %d bootcntlast:%d \n", 
       rtcData.rtc_running ,
      rtcData.rtc_lastrun_state, 
      rtcData.rtc_vibration_count,
      rtcData.rtc_bootcount,
      rtcData.rtc_bootcount_last
      );
  

  
}  //end of setup

void loop() 
{
  long now = millis();
 
  // vibration_measurement
  long vibration_measurement =TP_init(); //Read the vibration sensor
  int vibration = digitalRead(SENSOR_PIN_VIBRATION); // Read the state of the switch
  
  //  analogRead() function to measure the voltage coming from the photoresistor
  // resistor pair. This number can range between 0 (0 Volts) and
  // 1023 (5 Volts), but this circuit will have a smaller range
  // between dark and light.
  long lightLevel = analogRead(PHOTO_SENSOR_PIN);
        
  counter++;
   
  if(vibration_measurement > 100 )
  {
     continuos_vibration_count++;
   }
  
  //No virbation then reduce the vibration activity counter if> 1
  if (vibration_measurement == 0)
   {
    continuos_vibration_count= (continuos_vibration_count > 1) ? --continuos_vibration_count: 0;
    }
   
  //We exceeded vibration count threshold assume machine is on.. send a message  
  if (  continuos_vibration_count > RUN_COUNT_THRESHOLD)
  {
    running=true;
    continuos_vibration_count = RUN_COUNT_THRESHOLD;  //set the counter to max so as not to keep counting
    
   if (last_running_state==false)
    {
      connectMQTT();  //Open WiFi and connect to MQTT broker
      
     Serial.print("Machine was STARTED!");  //send done/off message to MQTT
     last_running_state=running;
     client.publish("/sensor/machine_running", String(running).c_str(), false); // machine state
     client.publish("/sensor/light_level", String(lightLevel).c_str(), false);
     client.publish("/sensor/vibration", String(vibration_measurement).c_str(), false);
     client.publish("/sensor/wash_done", String(false).c_str(), false); // reset wash state when machine starts
    } 

  }

  // No continous vibrations occured lets  send a message if we WERE running
  if ( continuos_vibration_count == 0 )
  {
   running=false;
   if (last_running_state==true)  //was there a state change?
    {
     connectMQTT();  //Open WiFi and connect to MQTT broker
      
     Serial.println("Machine TURNED OFF...");  //send done/off message to MQTT
     last_running_state=running;
     client.publish("/sensor/machine_running", String(running).c_str(), false); // machine state
     client.publish("/sensor/vibration", String(vibration_measurement).c_str(), false); //stopped running
     client.publish("/sensor/wash_done", String(true).c_str(), false); // state change from running means wash done
     client.publish("/sensor/light_level", String(lightLevel).c_str(), false);
     client.publish("/sensor/bootcount", String(rtcData.rtc_bootcount).c_str(), false);
     rtcData.rtc_bootcount_last=rtcData.rtc_bootcount;

     last_millis= millis();
     
    }
  }

  
  

  unsigned long elapsed_Secs=(millis() - start_time ) / 1000;
  Serial.printf("#:%d [ %d s < %d s] Light:%d VibrMeas:%d VibrateDig:%d RunCount:%d Running:%d \n", counter ,elapsed_Secs, MEASURE_INTERVAL,lightLevel , vibration_measurement,vibration, continuos_vibration_count, running);

   if ( counter % MEASURE_INTERVAL==0) //After taking some readings above threshold... 
  {
    //timeout the wash done alert so as not to keep it on
    // if after 3 reboots and still not running assume the wash_done message can be turned off
     if (running==false && (rtcData.rtc_bootcount - rtcData.rtc_bootcount_last) ==3  )
      {
       
       connectMQTT();  //Open WiFi and connect to MQTT broker
       client.publish("/sensor/wash_done", String(false).c_str(), false);  //wash done timeout, assume alert was seen
       Serial.println("Sending TURN OFF wash_done sensor");  
       delay(100);
       }

    //store the RTC variables belfore going to sleep
    writeRTC(running,last_running_state, continuos_vibration_count);
 
    Serial.printf("Going into deep sleep for %d seconds\n",SLEEP_INTERVAL);
   ESP.deepSleep(SLEEP_INTERVAL); // sleep interval in microseconds
  }
}  //end of main loop

boolean readRTC()
{
// Try to read WiFi settings from RTC memory
bool rtcValid = false;

if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
  // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
  uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
  if( crc == rtcData.crc32 ) {
    rtcValid = true;
  }
 }

return rtcValid;
} //end of function

boolean writeRTC(boolean isrun,boolean lastrunstate, int virb_count)
{


  

//store the realtime clock data into structure
rtcData.rtc_running=isrun;
rtcData.rtc_lastrun_state=lastrunstate;
rtcData.rtc_vibration_count=virb_count;

rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );

Serial.printf("\nWRITING RTC Values:  running:%d  RTCLastState:%d  RTCVirbCount:%d   boots: %d bootcntlast:%d \n", 
       rtcData.rtc_running ,
      rtcData.rtc_lastrun_state, 
      rtcData.rtc_vibration_count,
      rtcData.rtc_bootcount,
      rtcData.rtc_bootcount_last
      );


return true;
}

long TP_init(){
  //delay(10);
  //waits one second if no Pulse High on Pin
  long vibration_measurement=pulseIn(SENSOR_PIN_VIBRATION, HIGH);  //wait for the pin to get HIGH and returns vibration_measurement
  return vibration_measurement;
}

//CRC calculation for make sure data stored in RTC is valid
uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      boolean bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}

