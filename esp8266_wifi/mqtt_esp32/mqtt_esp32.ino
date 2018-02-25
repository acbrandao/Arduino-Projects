#include <WiFi.h>
#include <PubSubClient.h>
#include <Time.h>

#define wifi_ssid "WIFI-N"
#define wifi_password "ff00000000"
#define mqtt_server "192.168.1.190"
#define mqtt_user ""
#define mqtt_password ""

#define topic_door "sensor/door"

gpio_num_t  reedSensor = GPIO_NUM_33;  // Reed PIN:

WiFiClient espClient;

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR long sleepmillis = 0;
RTC_DATA_ATTR long previousMillis=0;  // will store last time deep sleep awake was updated
 
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
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
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
//    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

long lastMsg = 0;
float temp = 0.0;
float mailbox = 0.0;
float diff = 1.0;
long msgcount=0;
char door_state_val[8];
char state_val[8];

   
void SendMQTT_message(const char* mqtt_topic,const char* mqtt_value)
{
  Serial.println((String)"Topic " + mqtt_topic + ": " + mqtt_value);
  client.publish(mqtt_topic, mqtt_value, true);
}

void setup() {
  Serial.begin(115200);
  delay(250); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  long currentMillis = millis();
 
  //Now wait for the state to change back to close.. 
   pinMode(reedSensor, INPUT);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);

 //Now do the normal LoopCode here
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
  //If you were to use ext1, you would use it like
  //esp_deep_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);

 
  if(currentMillis > previousMillis   ) {
    
    sleepmillis=( currentMillis - previousMillis   )+sleepmillis;
  // save the last time we woke upLED 
    previousMillis = currentMillis;
  
  }
  
  ++bootCount;
  Serial.println("Boot number  : " + String(bootCount) );
  Serial.println("PrevMillis   :"+String(previousMillis) );
  Serial.println("CurrentMillis:"+String(currentMillis) );
  Serial.println("SleepMissis :"+String(sleepmillis) );

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
 
    int doorState = digitalRead(reedSensor);
    
   //Now send the MQTT message  

   itoa(doorState, door_state_val, 10);
   SendMQTT_message(topic_door,door_state_val );

  String boot=String(bootCount);
   SendMQTT_message("sensor/bootcount",boot.c_str() );


    String ms=String(sleepmillis);
   SendMQTT_message("sensor/millis",ms.c_str() );

    int currentdoorState=doorState;
    int n=0;
    Serial.println("Door State now:");
     Serial.println(currentdoorState);
   
    while (currentdoorState==doorState)
    {
      n++;
        doorState = digitalRead(reedSensor);
     // print out the state of the button:
      Serial.print(doorState);
      if (n%40==0)
       Serial.println();
      delay(50);        // delay in between reads for stability
    }

    //Now send the changed doorstate MQTT message  
   itoa(doorState, door_state_val, 10);
   SendMQTT_message(topic_door,door_state_val );
   
    delay(250); //give wifi a chance to get message

  //Now prepare the sleep to wake on reed sensor change
   esp_sleep_enable_ext0_wakeup(reedSensor,1); //1 = High, 0 = Low
   
  //Go to sleep now
   Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
 
}  //end of setup

void loop() 
{

}  //end of main loop
