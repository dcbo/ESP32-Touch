# ESP32 AZ-Delivery Touch 
Connect Display using SPI

# Basic-Functionality
* Wifi Connection  
* MQTT Connection 
* Self Monitoring connectivity and reconnect on connection loss
* Command Parser accepts commends over MQTT
* MQTT Status Topic, retained, with LastWill
* CRON System which sends different MQTT Topics every 10s, 30s and 60s
* Automatic Versioning System
  * Version Number is incremented after Upload to Production Target
# Device Specific-Functionality
* Write Text on Display
* Scroll Text when Display is full
* Print text received via MQTT on Display

# Available MQTT-Commands 
* Commands must be published to topic `[PREFIX]/cmd`
* Responses are published to `[PREFIX]/result`

## Print Text on Display
### `display "String"`
Display `String` on the LCD, scroll if needed
 Example:
 * command: `display "Hello World"` 

## System related Commands
### `reset`
Reboot ESP32

Example:
 * command: `reset` 
 
