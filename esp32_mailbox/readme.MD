 
# ESP32 Mailbox Sensor IoT script

How I took my rusty old mailbox (literally)  and made it into smart mailbox (an IoT device) and in the process  how I  learned about Arduino and particularly  the ESP32 chip which lets us relatively easily  setup with minimal costs and complexity for a battery powered mailbox  notification system (and many other IoT projects) .

This Sketch is for use with an Arduno ESP32 board, and a door sensor and it will communcate the door status and changes via a MQTT broker


## Getting Started

To get started and how to setup the hardware see my blog post here :
his project only requires a few items:

  * An ESP32 development board (I used LoLine32 Lite) ,
  * a door /window magnetic  (A reed switch), detects open/close state
  * a li-poly battery( or really any battery that can power thee ESP32)  and a few connecting wires with
  * a project box, to keep everything enclosed
  *  plus a  tiny bit of soldering, some tape, adhesive putty   and that’s about it
  * a [ MQTT broker | https://www.hivemq.com/blog/mqtt-essentials-part-3-client-broker-connection-establishment ] such as Mosquitto (local) or HiveMQ cloud based , this is what receives the messsages ..

### Installing

Within Arduino IDE First things first make sure you have the right board selected in the board manager and installed all the compulsory libraries  First make sure you get all the required libraries so you can you can build the sketch they include the following: Fire up the ARduino IDE and…


For  your  [ESP32 board Supporting libraries | https://github.com/espressif/arduino-esp32 ], steps to get these vary depending on what ESP board you have, Complete and detailed instructions here on GitHub ESP32  Arduino Core but for most boards the steps can be found here.

in Arduino IDE – Include library PubSub : PubSub Client libraries, need for MQTT support, this can easily be installed in Arduino by going to Sketch | Include Library | “Search PubSub” and install the matching library
 
## Customize you wifi_info.h file 

Take the attached wifi_info_sample file and edit its settings to match your environment then save it as *wifi_info.h*

```
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
```
### Configuring the mqtt_mailbox sketch 

Optionally you can tweak these settings to better suit your needs:

```
#define LED_ENABLED false  //do we want to turn on the on-board LED when connected to WiFi? 
#define MAX_OPENDOOR_TIME 30000   //default 30s in milliseconds how long to wait while door is open to consider it stuck open 
#define MAX_STUCK_BOOT_COUNT 5  // If the door is stuck for more than x times let's switch to timer interrupt to save battery
#define TIMER_SLEEP_MICROSECS 1800 * 1000000  //when on timer interrupt how long to sleep in seconds * microseconds

// /sensor/door_status messages for open closed and stuck
#define  message_door_open "Mailbox Opened!"
#define  message_door_closed ""
#define  message_door_stuck "Mailbox_STUCK_OPEN!"


// Define the the MQTT and WiFI client variables
WiFiClient espClient;
PubSubClient client(espClient);
HTTPClient http;  //for posting 

//Define the door Sensor PIN and initial state (Depends on your reed sensor N/C  or N/O )
gpio_num_t  doorSensorPIN = GPIO_NUM_12;  // Reed GPIO PIN:
gpio_num_t GPIO_INPUT_IO_TRIGGER = doorSensorPIN;
int GPIO_DOOR_CLOSED_STATE= LOW ;  //Default state when the reed and magent are next to each other (depends on reed switch)
int GPIO_DOOR_OPEN_STATE = !GPIO_DOOR_CLOSED_STATE;  //Open state is oposite f closed

```

Now save the changes and run the script


## Installing and Testing 

  *   Attach your ESP32 board with your USB cable, choose the COM n link in your Arduino IDE
   *   Press Control+U to compile and  upload the sketch to your board
   *  Use Contorl+Shift + M to open the serial monitor and make sure its works as expeted.


## Contributing

If you're interested in updating this script or have some ideas feel free to gt in touch.

## Versioning


## Authors


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc
