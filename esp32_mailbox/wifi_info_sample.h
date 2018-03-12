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


/* OPTIONAL:  but you can define a messaging service SMS such as twilio, nexmo, plivio etc.
* TO USE THIS FEATURe you NEED TO SiGNUP TO A SMS Service  then complete the section below
* refer to their curl examplles
*/

#define sms_enabled false //enable this if you have setup an sms service like tiwlio or nexmo etc..

#define sms_post_url   "https://rest.nexmo.com/sms/json"
#define sms_to_number  "SMS TO NUMBER"   // must be in the list of approved numbers - service whitelist
#define sms_from       "VALID NEXMO NUMBER"   //some services require from their number like Twilio
#define sms_api_key    "API_KEY"
#define sms_api_secret "API_SECRET" 
