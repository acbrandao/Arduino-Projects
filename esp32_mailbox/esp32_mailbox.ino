/*
 * ESP32 Arduino Door/Mailbox reed sensor to MQTT server
 *
 *  This sketch makes use  of a ESP32 with its WiFi and  Interrupt & Timer GPIO pins to wake the ESP32 
 *  get the door sensor and send a payload to the MQTT broker. The script is designed to run in  a low-power
 *  state utilizing ESP32 ppower saving deep sleep modes and detecting stuck open door. 
 *  
 *  I used this for my mailbox, but it can be adapted to any door-like environment where you can attach reed/magentic
 *  or  other sensors to detect open/close states.
 * 
 * This example code is in the public domain.
 *
 *For full code explanation visit:
  * http://www.abrandao.com/arduino_esp32_wifi_door_mailbox_sensor
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <Time.h>

/*
 * wifi_info.h includs wifi ssid, passwords, mqtt broker info, customize included wifi_info_sample.h and
 * save as wifi_info.h in this same folder.
 */
#include "wifi_info.h"  // contains ssid and access credentials,sms, mqtt broker configs sperate file for GitHub
#define sms_enabled false //enable this if you have setup an sms service like tiwlio or nexmo etc..

#define MAX_OPENDOOR_TIME 30000   //default 30s in milliseconds how long to wait while door is open to consider it stuck open 
#define MAX_STUCK_BOOT_COUNT 5  // If the door is stuck for more than x times let's switch to timer interrupt to save battery
#define TIMER_SLEEP_MICROSECS 1800 * 1000000  //when on timer interrupt how long to sleep in seconds * microseconds

// Define the the MQTT and WiFI client variables
WiFiClient espClient;
PubSubClient client(espClient);
HTTPClient http;  //for posting 

//Define the door Sensor PIN and initial state (Depends on your reed sensor N/C  or N/O )
gpio_num_t  doorSensorPIN = GPIO_NUM_12;  // Reed GPIO PIN:
gpio_num_t GPIO_INPUT_IO_TRIGGER = doorSensorPIN;
int GPIO_DOOR_CLOSED_STATE= LOW ;  //Default state when the reed and magent are next to each other (depends on reed switch)
int GPIO_DOOR_OPEN_STATE = !GPIO_DOOR_CLOSED_STATE;  //Open state is oposite f closed

//RTC Memory attribute is retained across resets
RTC_DATA_ATTR long bootCount = 0;
RTC_DATA_ATTR long stuckbootCount = 0;  //if the door is stuck open increment this counter
RTC_DATA_ATTR long last_doorState=-99;  // will store last door state to detect stuck open 
RTC_DATA_ATTR long time_awake_millis=0;  //total awake time in milliseconds - useful for tracking how long battery lasts

long currentMillis;  //timer start of awake
int doorState=LOW;   //maintains  the door sensor state usually LOW= 0 or 1 
int start_time=0;  //timing for when esp32 is active (not sleeping)
char total_time_awake[21];  //timing holds string h:m:s for time door is opened.
long rssi =0;
 
/* Method to print the reason by which ESP32 has been awaken from sleep */
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

/* Connect to Wifi but test for sensor closed or timeouts and issue sleep to conserver battery */
void setup_wifi() {
   start_time=millis();
  
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.disconnect();

  Serial.printf("Trying to connec to WiFi: %s, p/w: %s ... \n",wifi_ssid,wifi_password);
  WiFi.begin(wifi_ssid, wifi_password);

 int count=0;

  while (WiFi.status() != WL_CONNECTED) {
    
    count++;
   if ( count % 40==0 )
    Serial.print("\n");
    else
   Serial.print(".");

    delay(25);
    
    if (millis()- start_time > wifi_timeout )  //did wifi fail to connect then time out
    {
         Serial.println("Wifi Failed to connect ..timeout..");
            esp32_sleep();
        }
  
  }
  
  rssi= WiFi.RSSI();
  Serial.println( "\n WiFi connected in " +(String)(millis()- start_time ) + "ms to AP:"+wifi_ssid+ " Rssi:"+(String)rssi );
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


/* MQTT Broker connection  */
void reconnect() {
  // Loop until we're reconnected
   int retry =0;
   
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // for password use: if (client.connect("ESPxxClient", mqtt_user, mqtt_password)) {
    
    if (client.connect(mqtt_clientid)) {
      Serial.println("connected to "+String(mqtt_server) );
    } else {
      retry++;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
       delay(3000);

      if (retry > mqtt_max_retries)
        return;  //end stop trying to connect 
    }
  }
}

/* Sends a SMS if you have a service like twiliio, or nexmo defined */
int send_SMS(String message)
{
  if (!sms_enabled) 
        return false; //no sms send feature is available
    
      if(WiFi.status()== WL_CONNECTED) //Check WiFi connection status
      { 
        
        
        http.begin(sms_post_url, "‎0f80611c823161d52f28e78d4638b42ce1c6d9e2");
        http.addHeader("Content-Type", "application/json");

        /* SMS CODE PoST example is for the nexmo specific service ONLY, 
         *  if you use another popular SMS service  ones like  Twilio , Plivo, Sinch 
         *  be sure too change the POST request according. Refer to their cURL examples
         */
        /*
        curl -X "POST" "https://rest.nexmo.com/sms/json" \
        -d "from=NEXMO Number" \
        -d "text=A text message sent using the Nexmo SMS API" \
        -d "to=TO_NUMBER" \
        -d "api_key=NEXMO_API_KEY" \
        -d "api_secret=NEXMO_API_SECRET"
        */

        
        String postMessage = String(" {'from' : '" +(String)sms_from +"', 'text' : '" + (String)message +"', 'to' : '"+(String) sms_to_number+"', 'api_key' : '"+ (String)sms_api_key+"', 'api_secret' : '"+ (String)sms_api_secret+"' \}");
        
        int httpCode = http.POST(postMessage);
        
        Serial.printf("\nhttp result: %d",httpCode);
        http.writeToStream(&Serial);
     
        String payload = http.getString();
        
        http.end();
        
        return httpCode;

        } 
      else
      {
       Serial.print("Error in Wifi connection");
        return false;
      }
        
}  //end send sms

/* Formatting function to convert millis into h:m:s */
void runtime(unsigned long ms )
{
    unsigned long runMillis=ms;
    unsigned long allSeconds=runMillis/1000;
    int runHours= allSeconds/3600;
    int secsRemaining=allSeconds%3600;
    int runMinutes=secsRemaining/60;
    int runSeconds=secsRemaining%60;
    sprintf(total_time_awake,"%02d:%02d:%02d",runHours,runMinutes,runSeconds);

 
}

/* Esp32 dep sleep function call and save state variables */
void esp32_sleep()
{
  //Go to sleep now
   time_awake_millis=time_awake_millis+(millis()-currentMillis  );
   last_doorState=digitalRead(doorSensorPIN) ;  //store the last door state generally should be closed
  Serial.printf("\n DoorState %d Last Door State  %d  Awake ms: %ld",doorState,last_doorState,time_awake_millis);
    Serial.println("\nGoing to sleep now");  
   esp_deep_sleep_start(); //Enter deep sleep
   Serial.println("This will never be printed");
}



/*  Start of the main setup  routine  */
void setup() {
  Serial.begin(115200);
  
  //start a timer to see how long we're awake
  currentMillis = millis();
  
// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_INPUT_IO_TRIGGER,INPUT_PULLUP);

 
//!< Keep power domain enabled in deep sleep, if it is needed by one of the wakeup options. Otherwise power it down.
//  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
  gpio_pullup_en(GPIO_INPUT_IO_TRIGGER);    // use pulldown on NO door state
  gpio_pulldown_dis(GPIO_INPUT_IO_TRIGGER);       // not use pullup on GPIO

 //What state is door in
  doorState = digitalRead(doorSensorPIN);

  ++bootCount;
  Serial.println("Boot counter  : " + String(bootCount) );
  Serial.println("Boot Stuck cnt: " + String(stuckbootCount) );
  Serial.println("Last Door    :"+String(last_doorState) );
  Serial.println("DoorState    :"+String(doorState) );
  
  if (last_doorState == doorState )  // is the door stuck open triggering Interrupt
    {
      stuckbootCount++;

    if (stuckbootCount > MAX_STUCK_BOOT_COUNT)  //door is still stuck open we need to sleep on a timer
      {
        Serial.println("Max stuck open count door reached ");
        Serial.printf("\n Putting ESP into a TIMER WAKEUP of %d secs. \n",(TIMER_SLEEP_MICROSECS/1000000) );
        esp_sleep_enable_timer_wakeup(TIMER_SLEEP_MICROSECS); //lets go to TIMER Interrupt sleep instead of GPIO wakeup
        esp32_sleep();
      }
     else 
     {
      delay(5000);  //pause maybe stuck lid will close down    
      //Wake up when it goes high - may be inverted for your reed door sensors
      esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER,GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)
      esp32_sleep();
     }
    }
  
   stuckbootCount=0; //reset the stuckboot counter each time we get a clean GPIO wakeup

  //Wake up when it goes high - may be inverted for your reed door sensors
  esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER,GPIO_DOOR_OPEN_STATE); //1 = High, 0 = Low  wake door OPEN (magnet away sensor)
 
  
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
 
 if(doorState != GPIO_DOOR_CLOSED_STATE) //is the door NOT closed?
  {
   //Now Setup WIFI and MQTT connections -   //ONLY IF DOOR IS OPENED otehrwise ignore
   digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  //Now do the normal LoopCode here
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
   
   //Now send the MQTT message  
   client.publish("/sensor/door", String(doorState).c_str(), false);  //false means don't retain messages
   client.publish("/sensor/door_status","open", false);  //text version of door
   client.publish("/sensor/bootcount",String(bootCount).c_str(), false); 
   client.publish("/sensor/name", mqtt_clientid , false);  //client id
   client.publish("/sensor/rssi",String(rssi).c_str() , false);   // wifi rssi = signal strenght 0 - 100 , <-80 is poor
   
   send_SMS("You've got mail"); //optional: send sms message - requires SMS service
    
    long  n=0; //loop counter while door is opened
    while (doorState==GPIO_DOOR_OPEN_STATE)  //while where open
    {
      n++;
  
      doorState = digitalRead(doorSensorPIN); //Keep reading the door state
      
      // print out the state of the button:
      Serial.print(doorState);
      
      long elapsed_time=(millis()-start_time); 
      
      if (n%40==0)      //new line so it doens't scroll of the screen
       Serial.printf(  "%ld ms \n",elapsed_time );  //print the time in every 40 cyclels 
      delay(50);        // delay in between reads for stability


      //If the door is still OPEN  after MAX_OPENDOOR_TIME time, assume its stuck open and just send a message
      if (elapsed_time > MAX_OPENDOOR_TIME)
        {
          Serial.printf("\n Door STUCK OPEN for %ld ms > %d ms .. ending loop. ", elapsed_time,MAX_OPENDOOR_TIME );
          client.publish("/sensor/door_status","stuck open?",false);
          //send_SMS("Mailbox stuffed- lid is open.");  //optional send message when mailbox is stuck open
          esp32_sleep();  
          //code here not executed
        }
    }

      //calculate total_time_awake  how long the esp32 board was running
      runtime(millis()-currentMillis +time_awake_millis ) ; 
      Serial.println("ESp32 total awaketime ::"+(String)total_time_awake);     
       client.publish("/sensor/door_status","closed",false);  //text version of /sensor/door
       client.publish("/sensor/total_time_awake",total_time_awake,false);
      int result= client.publish("/sensor/door", String(doorState).c_str(), false);  //door status
    
      delay(500); //give MQTT broker a chance to get message -  before disconnecting wifi
     
      if (result==true)  //did we successfully publish the last message
        {
          client.disconnect();   //disconnect Mqtt broker
          WiFi.disconnect();    //disconnect WiFi
          Serial.println("\n Wifi Disconnected from : "+(String)wifi_ssid);
          }
       else
         Serial.printf("\n  Error in Publishing last message PubSub Error code: %d",result);
       
        
   
    digitalWrite(LED_BUILTIN, LOW); // :: Turn the LED oFF   
  }
  else
  Serial.println("\n Door State Unchanged not triggering wifi");

  //Go to sleep now
  esp32_sleep();
  Serial.println("This will never be printed");
 
}  //end of setup

/* Typical Arduino main loop not used */
void loop() 
{
}  //end of main loop


