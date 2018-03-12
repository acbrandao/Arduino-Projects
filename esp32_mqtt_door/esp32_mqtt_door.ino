/*
  ESP32 MQTT Door sensor
  Uses a ESP32 with Interrupt IO pins to wake the ESP32 get the door sensor and send a payload to the MQTT broker
  This example code is in the public domain.

  http://www.abrandao.com/arduino_esp32_door
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Time.h>
#include "wifi_info.h" //defines with senstivie data (use .gitignore ) to keep off github public repos

WiFiClient espClient;

#define LED_BUILTIN 5


gpio_num_t  reedSensor = GPIO_NUM_33;  // Reed PIN:
gpio_num_t GPIO_INPUT_IO_TRIGGER = reedSensor;  //alias name
int GPIO_DOOR_CLOSED_STATE= HIGH;   //Default state when the reed and magnet are next to each other (depends on reed switch)
int GPIO_DOOR_OPEN_STATE = !GPIO_DOOR_CLOSED_STATE;  //Open state is oposite f closed

int doorState=0;
int start_time=0;  //timing for when esp32 is active (not sleeping)
char runtime_str[21];  //timing holds string variation of it.
uint64_t chipid;  //ChipID is MAC address

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR long sleepmillis = 0;
RTC_DATA_ATTR long last_state=LOW;  // will store last door state to prevent spurious
RTC_DATA_ATTR long running_time_millis=0;  //Running time in millisecods

long lastMsg = 0;
float temp = 0.0;
float mailbox = 0.0;
float diff = 1.0;
long msgcount=0;
char door_state_val[8];
char state_val[8];
 int mqtt_door_state=0;

long rssi =0;
 
/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t  wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}


PubSubClient client(espClient);

void setup_wifi() {
  int count=0;  
  start_time=millis();
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  
  WiFi.begin(wifi_ssid, wifi_password);

 /*  
   if (WiFi.status()== WL_NO_SSID_AVAIL)   //Fix for hardware bug when ESP returns code=1 WL_NO_SSID_AVAIL
      {
        digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
       Serial.print("Restarting the Board WIFI Connection bug.");
      ESP.restart();  //kludge reset the board
      }
*/

  while (WiFi.status() != WL_CONNECTED) 
  {
    count++;
     doorState = digitalRead(reedSensor);
   
    
   if ( count % 40==0 )
    Serial.print(String(millis()-start_time)+"\n");
    else
    Serial.print(doorState);
    delay(25);
 
     //while waiting for WIFI connection check if door state is closed if so then just dont bother to connect and sleep
   
    
   if (doorState==GPIO_DOOR_CLOSED_STATE)  //going to sleep
       {
         Serial.println("door now closed - WiFi Connection ended. ");
         Serial.println("Going to sleep now");
            esp_deep_sleep_start();
        }

 /* Case where WIFI is still looking for SSID related to 50% re-connect issue with esp32 bug */
    if ( WiFi.status() == WL_NO_SSID_AVAIL )  // WL_NO_SSID_AVAIL,= status 6
       {
         Serial.println("Wifi Failed to connect Restarting BOARD ESp32 bug ");
         esp_restart();
        }
    
    //wifi not responding go to sleep and wait for next interrupt
     if (millis()- start_time > wifi_timeout)
          {
             Serial.println( (String)(millis()- start_time ) + "ms Error connecting to :"+wifi_ssid +" Check AP ");
            //Go to sleep now
             Serial.println("Wifi Timedout Going to sleep now");
            esp_deep_sleep_start();
          }
  } 
  
  rssi= WiFi.RSSI();
  Serial.println( "\n WiFi CONNECT! to  AP:"+(String)wifi_ssid+ " Rssi:"+(String)rssi );
  Serial.println("\n IP address: ");
  Serial.println(WiFi.localIP());
  
}


// MQTT Reconnect
void reconnect() {
  int failcount=0;  //too many mqtt fails go to sleep
  int max_fails=2;  
  
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection to:"+String(mqtt_server)+" ...");
   
    if (client.connect("ESPxxClient")) {
      Serial.println("connected to "+String(mqtt_server) );
    } else {
       failcount++;
      if (  failcount >=  max_fails )
          {
            //Go to sleep now
             Serial.println("MQTT Server NOT available Timedout");
             Serial.println("Going to sleep now");
            esp_deep_sleep_start();
          }
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      delay(3000);
    }
  }
}

   
void SendMQTT_message(const char* mqtt_topic,const char* mqtt_value)
{
  Serial.println("MQTT publish:" + (String)"Topic " + mqtt_topic + ": " + mqtt_value);
  client.publish(mqtt_topic, mqtt_value, true);
  
}


void runtime(unsigned long ms, char runtime_str[])
{

    unsigned long runMillis=ms;
    unsigned long allSeconds=runMillis/1000;
    int runHours= allSeconds/3600;
    int secsRemaining=allSeconds%3600;
    int runMinutes=secsRemaining/60;
    int runSeconds=secsRemaining%60;
    sprintf(runtime_str,"Runtime%02d:%02d:%02d",runHours,runMinutes,runSeconds);
 
}


// Start of the main setup  routine /////////////////////
void setup() {
  Serial.begin(115200);

   //Print the wakeup reason for ESP32
  print_wakeup_reason();
 

//assign the reedSensor an Input Pinmode. 
   pinMode(reedSensor, INPUT_PULLUP);
   
   //prepare for deepsleep setup interrupt on pin
 //!< Keep power domain enabled in deep sleep, if it is needed by one of the wakeup options. Otherwise power it down.
 //  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
  gpio_pulldown_en(GPIO_INPUT_IO_TRIGGER);    // use pulldown on NO door state
  
 // gpio_pullup_dis(GPIO_INPUT_IO_TRIGGER);       // not use pullup on GPIO

  //Wake up when it goes low
  esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER,GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)
  
// initialize digital pin LED_BUILTIN as an output.
 // pinMode(LED_BUILTIN, OUTPUT);

   
  //Increment boot number and print it every reboot
  long currentMillis = millis();
 

  ++bootCount;
  Serial.println("Boot number  : " + String(bootCount) );
  Serial.println("SleepMissis :"+String(sleepmillis) );

  //What state is door in
   doorState = digitalRead(reedSensor);
   Serial.println("Door State:"+String(doorState) );
   
 if(doorState != GPIO_DOOR_CLOSED_STATE) //is the door opened?
  {
  
  int count=0;
  
   setup_wifi();  //connect to WIFI
   
   client.setServer(mqtt_server, 1883);
  
   //Now do the normal LoopCode here
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
   
   //Now send the MQTT message  
   String ds=String(!doorState);  //invert state because we're using a Normally open closed switch
   SendMQTT_message("sensor/door/state",ds.c_str() );

   
   String boot=String(bootCount);
   SendMQTT_message("sensor/door/bootcount",boot.c_str() );

   String nm=String("Wemos LoLin32");
   SendMQTT_message("sensor/door/name",nm.c_str() );
  
   char rssi_val[10];
   ltoa(rssi,rssi_val,10);
    SendMQTT_message("sensor/door/rssi",rssi_val );

    int currentdoorState=doorState;
    int n=0;
    Serial.println("Door State now:");
     Serial.println(doorState);

   unsigned long timer1= millis();
    
    while (doorState==GPIO_DOOR_OPEN_STATE)  //while where open
    {
      n++;
    doorState = digitalRead(reedSensor);
    last_state=doorState;
    
    Serial.print(doorState);
    
    if (n%50==0)
    Serial.println( (String) (millis()-timer1 ) + "ms \n" );
    
    delay(25);        // delay in between reads for stability
    client.loop();    //loop here to keepalive and tell the broker we're still here
    }

  

    runtime( running_time_millis + millis(),runtime_str ) ; 
    SendMQTT_message("sensor/door/runtime",runtime_str);  
    Serial.println("Running Time:  "+(String)(runtime_str)); //send the runtime to the sensor


    //Now send the MQTT message  
    ds=String(!doorState);  //invert state because we're using a Normally open closed switch
   SendMQTT_message("sensor/door/state",ds.c_str() );


   delay(100); //give MQTT broker a chance to get message 
 //  WiFi.disconnect();
  
  }
  else
  Serial.println("Door State Unchanged not connecting to wifi");


  //Go to sleep now
   Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
 
}  //end of setup

void loop() 
{
ESP.restart();
}  //end of main loop
