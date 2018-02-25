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


// int humtemp_sensorPin = D4;    // select the input pin for the humidity /temperature
int sensorPin = A0;    // select the input pin for the light photoresistor
int sensorValue = 0;  // variable to store the value coming from the sensor

PubSubClient client(espClient);

const int LED_PIN_RED=5;  //PIN Connected to RED LED
const int REED_PIN =4; // Pin connected to reed switch

void setup() {
  
  Serial.begin(115200);
  
 //   dht.setup(humtemp_sensorPin); // data pin 
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  
  // Since the other end of the reed switch is connected to ground, we need
  // to pull-up the reed switch pin internally.
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN_RED, OUTPUT);
}

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



void loop() 
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();

  int proximity = digitalRead(REED_PIN); // Read the state of the switch
  Serial.print("Reading REED state... ");
  Serial.println(proximity);
  
   if (proximity == LOW) // If the pin reads low, the switch is closed.
  {
    Serial.println("Switch closed ");
    digitalWrite(LED_PIN_RED, HIGH); // Turn the LED on
  }
  else
  {
    Serial.println("Switch OPEN");
    digitalWrite(LED_PIN_RED, LOW); // Turn the LED off
  }

    long mailbox= random(0,2);
    long temp= random(0,100);
    long rssi =random(0,100);
   long systime =random(0,100);
   msgcount++;

 //Enviro Sensors 
 /*
 delay(dht.getMinimumSamplingPeriod()); //must delay for sampple

float temperature = dht.getTemperature();
float temp_f=dht.toFahrenheit(temperature); 
float humidity = dht.getHumidity();
  // Check if any reads failed and exit early (to try again).
 
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
   temp_f=0.0;
   humidity=0.0;
  }

  
  Serial.print(dht.getStatusString());
  
  Serial.print("Temperature Sensor :");
  Serial.println(String(temp_f).c_str());
  client.publish(topic_temp, String(temp_f).c_str(), true);

  Serial.print("Humidity Sensor :");
  Serial.println(String(humidity).c_str());
  client.publish(topic_humidity, String(humidity).c_str(), true);
*/
  
  Serial.print("Light Sensor :");
  Serial.println(String(sensorValue).c_str());
  client.publish(topic_light, String(sensorValue).c_str(), true);
  
  Serial.print("message count:");
  Serial.println(String(msgcount).c_str());
  client.publish(topic_timestamp, String(msgcount).c_str(), true);

  char  device_name[]="ESP8266 huzzah";
  Serial.print("Device NAme:");
  Serial.println(String(device_name).c_str());
  client.publish(topic_devicename, String(device_name).c_str(), true);

  Serial.print("Door:");
  Serial.println(String(proximity).c_str());
  client.publish(topic_door, String(proximity).c_str(), true);

delay(1000);
}  //end of main loop
