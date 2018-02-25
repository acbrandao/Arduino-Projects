#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Time.h>


#define wifi_ssid "WIFI-N"
#define wifi_password "ff00000000"

#define mqtt_server "192.168.1.190"

#define mqtt_user ""
#define mqtt_password ""

#define topic_mail "sensor/mailbox"
#define topic_temp "sensor/temp"
#define topic_dbm  "sensor/dbm"
#define topic_timestamp  "sensor/timestamp"
#define topic_devicename  "sensor/name"
#define topic_light  "sensor/light"
#define topic_hum01  "sensor/temphumidity"

WiFiClient espClient;
DHTesp dht;

void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 17;



int sensorPin = A0;    // select the input pin for the light photoresistor
int sensorValue = 0;  // variable to store the value coming from the sensor

PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);


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
  if (now - lastMsg > 1000) {
    lastMsg = now;

    // read the value from the sensor:
  sensorValue = analogRead(sensorPin);

    long mailbox= random(0,2);
    long temp= random(0,100);
    long rssi =random(0,100);
   long systime =random(0,100);
   msgcount++;

  Serial.print("Light Sensor :");
  Serial.println(String(sensorValue).c_str());
  client.publish(topic_light, String(sensorValue).c_str(), true);
  
  Serial.print("Topi message:");
  Serial.println(String(msgcount).c_str());
  client.publish(topic_timestamp, String(msgcount).c_str(), true);

  char  device_name[]="Weo=mods D1";
  Serial.print("Device NAme:");
  Serial.println(String(device_name).c_str());
  client.publish(topic_devicename, String(device_name).c_str(), true);


  Serial.print("New temperature:");
  Serial.println(String(temp).c_str());
  client.publish(topic_temp, String(temp).c_str(), true);

  Serial.print("New mailbox:");
  Serial.println(String(mailbox).c_str());
  client.publish(topic_mail, String(mailbox).c_str(), true);

  Serial.print("New dbm:");
  Serial.println(String(rssi).c_str());
  client.publish(topic_dbm, String(rssi).c_str(), true);


 
  } //end of wait loop
}  //end of main loop
