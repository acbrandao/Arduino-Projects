# Arduino  Mouse Jiggler

Arduino / ESP / SAMD21  based hardware mouse emulator, preventing computer screen-saver from kicking in
and locking desktop

## How it works

After installation on hardware device (arduino) and plugging it to computer USB port it
registers itself as regular HID mouse. Every few seconds it makes tiny cursor movements,
emulating "user activity" and preventing screen-saver from locking up desktop.

## Requirements (what you will need)

  * Arduino board with USB support (Arduino Leonardo/Nano/Micro/Micro Pro or simillar - those could be bought from different retailers for less than $5)
  * USB Micro-B (male) to USB Type-A (male) connector - becaue Arduino comes with micro USB port (those are commonly used to transfer data between smartphone and computer)
  * Arduino IDE (https://www.arduino.cc/en/Main/Software) - program Arduino device

## Installation

  - Download and install Arduino IDE from their website (https://www.arduino.cc/en/Main/Software) or from your package repository: ```apt-get install arduino```.
  - Connect your Arduino board (via onboard micro USB port) to your computer USB port.
  - Clone this folder (includes other Arduino projects you can ignore) `git clone https://github.com/acbrandao/Arduino-Projects.git` 
  - Open Arduino IDE and open project ```jiggler/jiggler.ino``` file from this repository.
  - In Arduino IDE choose device type (```TOOLS > Board > <selelect your board>``` or similar) and port where programmer is connected to (```TOOLS > Port > ...```).
  - Upload your program to Arduino device (right pointing arrow, next to "tick" mark).

## Considerations

  * When plugged in, your Arduino device will be recognized as keyboard & mouse. You can have multiple keyboards and mice attached to your system, no need to remove any of them
  * Mouse cursor should "jiggle" at random patterns and random intevals all this is customizable via the code

### Reliability

Once you have flashed your arduino device it shoudl reboot and you shoudl hear the `bing - bong ` USB connection tone ,
if your device has an LED you can monitor it for when its running mouse activity.                                    

### Forensic footprint

Device acts as normal HID device (usb mouse) and according system journals are updated with entry of inserting USB device. No other files are created/modified except using already installed USB HID drivers.
