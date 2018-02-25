#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Time.h>
#include "DHTesp.h"

#define wifi_ssid "WIFI-N"
#define wifi_password "ff00000000"

#define mqtt_server "192.168.1.190"

#define mqtt_user ""
#define mqtt_password ""

#define topic_mail "sensor/mailbox"

#define topic_door  "sensor/door"
#define topic_timestamp  "sensor/timestamp"
#define topic_devicename  "sensor/name"
#define topic_light  "sensor/light"
#define topic_temp "sensor/temp"
#define topic_humidity "sensor/humidity"


DHTesp dht;
WiFiClient espClient;
PubSubClient client(espClient);


// int humtemp_sensorPin = D4;    // select the input pin for the humidity /temperature
// int sensorPin = A0;    // select the input pin for the light photoresistor
// int sensorValue = 0;  // variable to store the value coming from the sensor

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


//Global Constants
const int REED_PIN = 4;   // Pin connected to reed switch
const int LED_PIN = 5; // LED pin - active-high

long lastMsg = 0;
float temp = 0.0;
float mailbox = 0.0;
float diff = 1.0;
long msgcount=0;


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
 
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Since the other end of the reed switch is connected to ground, we need
  // to pull-up the reed switch pin internally.
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
     
}  //end of setup

void loop() 
{
   long now = millis();
  int proximity = digitalRead(REED_PIN); // Read the state of the switch
  if (proximity == LOW) // If the pin reads low, the switch is closed.
  {
    Serial.println("Switch closed");
    digitalWrite(LED_PIN, HIGH); // Turn the LED on
 
  }
  else
  {
    Serial.print(".");
    digitalWrite(LED_PIN, LOW); // Turn the LED off
  
  }

    // read the value from the sensor:
    long door= proximity;
    long temp= random(0,100);
    long rssi =random(0,100);
   long systime =random(0,100);
   msgcount++;


  Serial.print("Door Sensor:");
  Serial.println(String(door).c_str());
  client.publish(topic_door, String(door).c_str(), true);
 
  Serial.print("message count:");
  Serial.println(String(msgcount).c_str());
  client.publish(topic_timestamp, String(msgcount).c_str(), true);

  char  device_name[]="AdFruit Hazzah";
  Serial.print("Device NAme:");
  Serial.println(String(device_name).c_str());
  client.publish(topic_devicename, String(device_name).c_str(), true);
   //pause before flood mqtt with messasges
  delay(1000);

   printf("millis(): %ld\tcounter: %ld \n", millis(), msgcount );
  //Serial.println("Going into deep sleep for 20 seconds");
 // ESP.deepSleep(20e6); // 20e6 is 20 microseconds
  
}  //end of main loop
