/*
  ESP32 MQTT Door sensor

  Uses a ESP32 with Interrupt IO pins to wake the ESP32 get the door sensor and send a payload to the MQTT broker
  This example code is in the public domain.

  http://www.abrandao.com/arduino_esp32_door
*/


#include <WiFi.h>
#include <PubSubClient.h>
#include <Time.h>
#include <wifi_info.h>  //defines with senstivie data (use .gitignore ) to keep off github public repos

WiFiClient espClient;

/* Sample wifi_info.h file  cut and save belw as your own wifi_info.h file
 * 
 * 
 * #define wifi_ssid "WIFI-N"  //replace with actual WIFI SSID
 * #define wifi_password "your_password_here"  //replace with your actual password
 * #define wifi_timeout 20*1000   //Milliseconds to go to sleep if not connected
 * #define mqtt_server "192.168.1.199"  //MQTT broker address, recommend you make it static
 * #define mqtt_user ""
 * #define mqtt_password ""

 * 
 */

#define MQTT_KEEPALIVE 45;  //default keep alive is 15S this overrides that

#define LED_BUILTIN 2
#define topic_door "sensor/door"


gpio_num_t  reedSensor = GPIO_NUM_33;  // Reed PIN:
gpio_num_t GPIO_INPUT_IO_TRIGGER = reedSensor;
int GPIO_DOOR_CLOSED_STATE= HIGH;   //Default state when the reed and magent are next to each other (depends on reed switch)
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
  
  start_time=millis();
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  
  WiFi.begin(wifi_ssid, wifi_password);

 int count=0;
 String ssid=String(wifi_ssid);
 
  while (WiFi.status() != WL_CONNECTED) {
    
    count++;
   if ( count % 40==0 )
    Serial.print("\n");
    else
   Serial.print(".");

    delay(25);
    
     //while waiting for WIFI connection check if door state is closed if so then just dont bother to connect and sleep
    doorState = digitalRead(reedSensor);
    if (doorState==GPIO_DOOR_CLOSED_STATE)  //going to cleep
       {
         Serial.println("Wifi Failed to connect BUT door now closed .. Going to sleep now");
            esp_deep_sleep_start();
        }
    //wifi not responding go to sleep and wait for next interrupt
     if (millis()- start_time > wifi_timeout)
          {
             Serial.println( (String)(millis()- start_time ) + "ms Error connecting to :"+ssid +" Check AP ");
            //Go to sleep now
             Serial.println("Wifi Timedout Going to sleep now");
            esp_deep_sleep_start();
          }
  }
  
  rssi= WiFi.RSSI();
  Serial.println( "\n WiFi connected in " +(String)(millis()- start_time ) + "ms to AP:"+ssid+ " Rssi:"+(String)rssi );
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // for password
//    if (client.connect("ESPxxClient", mqtt_user, mqtt_password)) {
    if (client.connect("ESPxxClient")) {
      Serial.println("connected to "+String(mqtt_server) );
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

   
void SendMQTT_message(const char* mqtt_topic,const char* mqtt_value)
{
  Serial.println((String)"Topic " + mqtt_topic + ": " + mqtt_value);
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

// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  
  //Now prepare the sleep to wake on reed sensor change
   //If you were to use ext1, you would use it like
  //esp_deep_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);
  
//!< Keep power domain enabled in deep sleep, if it is needed by one of the wakeup options. Otherwise power it down.
//  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
  gpio_pulldown_en(GPIO_INPUT_IO_TRIGGER);    // use pulldown on NO door state
  gpio_pullup_dis(GPIO_INPUT_IO_TRIGGER);       // not use pullup on GPIO

  //Wake up when it goes low
  esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER,GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)
   
  //Increment boot number and print it every reboot
  long currentMillis = millis();
 
  //Now wait for the state to change back to close.. 
   pinMode(reedSensor, INPUT);

 //Print the wakeup reason for ESP32
  print_wakeup_reason();


  ++bootCount;
  Serial.println("Boot number  : " + String(bootCount) );
  Serial.println("SleepMissis :"+String(sleepmillis) );

  //What state is door in
  doorState = digitalRead(reedSensor);
 
 if(doorState != GPIO_DOOR_CLOSED_STATE) //is the door still closed?
  {
   //Now Setup WIFI and MQTT connections - 
  //ONLY IF DOOR IS OPENED otehrwise ignore
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  setup_wifi();
  client.setServer(mqtt_server, 1883);

 digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
 
  //Now do the normal LoopCode here
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
   
   //Now send the MQTT message  
  
   mqtt_door_state=!doorState;
   itoa(mqtt_door_state, door_state_val, 10);
   SendMQTT_message(topic_door,door_state_val );

  String boot=String(bootCount);
   SendMQTT_message("sensor/bootcount",boot.c_str() );

   chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
 Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

     String nm=String("Wemos LoLin32");
   SendMQTT_message("sensor/name",nm.c_str() );

     char rssi_val[10];
     ltoa(rssi,rssi_val,10);
   SendMQTT_message("sensor/rssi",rssi_val );
  
    int currentdoorState=doorState;
    int n=0;
    Serial.println("Door State now:");
     Serial.println(currentdoorState);
   
    while (doorState==GPIO_DOOR_OPEN_STATE)  //while where open
    {
      n++;
        doorState = digitalRead(reedSensor);
          last_state=doorState;
     // print out the state of the button:
      Serial.print(doorState);
      if (n%40==0)
       Serial.println( (String) (millis()-start_time ) + "ms \n" );
      delay(50);        // delay in between reads for stability
    }


      runtime( running_time_millis + millis(),runtime_str ) ;  //Savves the output in runtime str
      Serial.println(runtime_str);
      SendMQTT_message("sensor/running_time",runtime_str);


  //Now send the changed doorstate MQTT message 
   mqtt_door_state= 0;
   itoa(mqtt_door_state, door_state_val, 10);
   SendMQTT_message(topic_door,door_state_val);
    Serial.println((String)"Pausing to give MQTT time to get message: Result of Publish: ");
   
   delay(500); //give MQTT broker a chance to get message 
   WiFi.disconnect();
  
  }
  else
  Serial.println("Door State Unchanged not triggering wifi");

  //Go to sleep now
   Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
 
}  //end of setup

void loop() 
{

}  //end of main loop
