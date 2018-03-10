/* Rename this  "wifi_info_sample.h" file to "wifi_info.h"  
 *  customize below for your environment and save as your own wifi_info.h file in this same folder
 *  Inluded in a seperate file becasue of github upload
 * 
*/

#define wifi_ssid "YOUR_AP_NAME"  //replace with actual WIFI SSID
#define wifi_password "your_password"  //replace with your actual password
#define wifi_timeout 30*1000   //Milliseconds to go to sleep if not connected

#define mqtt_server "broker.hivemq.com"  //MQTT broker address, local or cloud address
#define mqtt_user ""    //leave empty if none
#define mqtt_password ""   //leave empty if none
#define mqtt_clientid "ESP32 LoLin"  //Client id string for Mqtt broker
#define mqtt_max_retries 10 //maximum number of times to retry mqtt server before aborting

