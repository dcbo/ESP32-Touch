/************************************************************
 * ESP32 Hello World
 ************************************************************
 * (c) by dario carluccio esp32.helloworld@carluccio.de
 ************************************************************
 * Hardware:
 * - ESP 32
 ************************************************************
 * Fuctionality:
 * - Wifi Connect (Provide SSID+Pass in platformio.ini)
 * - MQTT Connect (Provide TOPIC in platformio.ini)
 * - OTA Update (needs a UDP connection from ESP to IDE-PC)
 * - Monitor Wfi & MQTT and reconnect on error
 * - Send different States every 10s, 30s or 60s
 * - Accept and Parse commands over MQTT
 * - Automatic increment Version 
 *   - incrementafter upload to Production target
 *   - copy binary to release Folder
 ***********************************************************/

/************************************************************
 * Includes
 ************************************************************/ 
// Extrernal Libraries
#include <WiFi.h>                // Wifi
#include <WiFiUdp.h>             // Wifi / OTA
#include <WiFiClient.h>          // Wifi 
#include <PubSubClient.h>        // MQTT  
#include <ESPmDNS.h>             // for OTA-Update
#include <ArduinoOTA.h>          // for OTA-Update
#include <CommandParser.h>       // To Parse MQTT Commands
#include <SimpleTime.h>          // Time Conversions 
// Own Project Files
#include <prototypes.h>          // Prototypes 
#include <myHWconfig.h>          // Hardware Wireing
#include <Version.h>             // Automatic Version Incrementing (triggered by Upload to Production)
#include <debugOptions.h>        // Debugging [my be improved]
// Project Libraries
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <XPT2046_Touchscreen.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "usergraphics.h"


/************************************************************
 * Compile Target
 ************************************************************/ 
#ifndef TARGET
  #define TARGET "UNKNOWN"
#endif

/************************************************************
 * WIFI Settings
 ************************************************************/ 
// WiFi-SSID 
// should be defined in plattformio.ini e.g.: build_flags = '-DWIFI_SSID="myhomerouter"'
#ifndef WIFI_SSID
  #define WIFI_SSID "myssid"
#endif
// WiFi-Pass 
// should be defined in plattformio.ini e.g.: build_flags = '-DWIFI_PSK="mysecretpassword"'
#ifndef WIFI_PSK
  #define WIFI_PSK  "mypassword"
#endif

/************************************************************
 * MQTT-Settings
 ************************************************************/ 
// Prefix for MQTT Topics should be defined in plattformio.ini
// e.g.: build_flags = '-DMQTT_PREFIX="homectrl/tisch"'
#ifndef  MQTT_SERVER    
  #define MQTT_SERVER "mqtt.example.de"
#endif
#ifndef  MQTT_PORT
  #define MQTT_PORT 1883
#endif
#ifndef  MQTT_USER
  #define MQTT_USER ""
#endif
#ifndef  MQTT_PASS
  #define MQTT_PASS ""
#endif
#ifndef MQTT_PREFIX
  #define MQTT_PREFIX "esp32/default"
#endif

// MQTT-Connection Settings
#define MQTT_BUFSIZE   2048                       // MQTT-Buffersize (may be augmented, when Scan returns many BLE-Devices
// Topic used to subscribe, MQTT_PREFIX will be added
#define T_CMD          "cmd"                      // Topic for Commands (subscribe) (MQTT_PREFIX will be added)
// Topics used to publish, MQTT_PREFIX will be added
#define T_CPU          "cpu"                      // Topic for CPU Status
#define T_LOG          "log"                      // Topic for Logging
#define T_NETWORK      "network"                  // Topic for Network Status 
#define T_RESULT       "result"                   // Topic for Commands Responses
#define T_SKETCH       "sketch"                   // Topic for Sketch Status 
#define T_STATUS       "status"                   // Topic for Online-Status 'ONLINE/OFFLINE' (published at birth and lastwill) (MQTT_PREFIX will be added)
#define STATUS_MSG_ON  "ONLINE"                   // Online Message
#define STATUS_MSG_OFF "OFFLINE"                  // Last Will Message

/************************************************************
 * Debug LED
 ************************************************************/ 
#define DBG_LED 2

/************************************************************
 * Timings
 ************************************************************/ 
#define T_HEARTBEAT_1S         1000  // cron every 1 second
#define T_HEARTBEAT_10S       10000  // cron every 10 seconds
#define T_HEARTBEAT_30S       30000  // cron every 30 seconds
#define T_HEARTBEAT_60S       60000  // cron every 60 seconds
#define T_STATE_LONG          60000  // Print detailed Status every 1 minute
#define T_STATE_SHORT          1000  // Print Status every 1 second
#define T_MQTT_RECONNECT       5000  // How often check MQTT: 5 seconds
#define T_NET_MONITORING      10000  // How often check Wifi: 10 seconds 
#define T_WIFI_MAX_TRIES         10  // Howoften retry to reconnect Wifi: 10 (repeated later)
#define T_REBOOT_TIMEOUT       5000  // ms until Reboot is triggered when g_rebootActive = true

/************************************************************
 * Touch Callibration
 ************************************************************/ 
#define MINPRESSURE 10      // minimum required force for touch event
#define TS_MINX 370
#define TS_MINY 470
#define TS_MAXX 3700
#define TS_MAXY 3600


/************************************************************
 * Objects
 ************************************************************/ 
// WIFI Client
WiFiClient myWiFiClient;

// MQTT Client
PubSubClient mqtt(MQTT_SERVER, MQTT_PORT, myWiFiClient);

// IRQ Handling
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// CommandParser
#define PARSER_NUM_COMMANDS   4   // limit number of commands 
#define PARSER_NUM_ARGS       3   // limit number of arguments
#define PARSER_CMD_LENGTH     10  // limit length of command names [characters]
#define PARSER_ARG_SIZE       64  // limit size of all arguments [bytes]
#define PARSER_RESPONSE_SIZE  64  // limit size of response strings to 64 bytes
typedef CommandParser<PARSER_NUM_COMMANDS, PARSER_NUM_ARGS, PARSER_CMD_LENGTH, PARSER_ARG_SIZE, PARSER_RESPONSE_SIZE> MyCommandParser;
MyCommandParser parser;
// Command Handler Prototypes
void cmd_hello(MyCommandParser::Argument *args, char *response);      // "hello",  ""
void cmd_helloadd(MyCommandParser::Argument *args, char *response);   // "helloadd", "uu"
void cmd_display(MyCommandParser::Argument *args, char *response);    // "display", "s"
void cmd_reset(MyCommandParser::Argument *args, char *response);      // "reset", ""
  
// Display
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Touchscreen
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);


/************************************************************
 * Global Vars
 ************************************************************/ 
boolean     g_Firstrun; 
// Monitoring
uint32_t    g_LastHeartbeat_1s; 
uint32_t    g_LastHeartbeat_10s;
uint32_t    g_LastHeartbeat_30s;
uint32_t    g_LastHeartbeat_60s;
uint32_t    g_LastMqttReconnectAttempt;
uint32_t    g_LastStateLong;
uint32_t    g_LastStateShort;
uint32_t    g_LastNetMonitoring;
uint32_t    g_LastRollerMonitoring;
uint8_t     g_LedState;
// MQTT
uint32_t    g_MqttReconnectCount;
// Wifi
boolean     g_wificonnected;
const char* g_wifissid = WIFI_SSID;
const char* g_wifipass = WIFI_PSK;
const char* g_otahash = OTA_HASH;
// IRQ
volatile boolean g_IrqFlag;
boolean     g_LastIRQ;
// Reboot Timer
boolean     g_rebootActive;                // if true trigger reeboot 5s after g_reboot_triggered
uint32_t    g_rebootTriggered;             // millis() when reboot was started
boolean     g_lastDebug;


// Scrolling Text on Display
#define txtBoxIndent  5   // Indent Text [pixel]
#define txtBoxStart  45   // Y-Pos of 1st Line [pixel]
#define txtBoxLines  10   // No. of Lines ind Buffer  
#define txtBoxVSpace 20   // Y-Increment for each Line [pixel] 
String txtBuffer[txtBoxLines];

//String txtBuffer[15] = {
//  "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"
//};
uint8_t txtBufferPtr = 0;  

/************************************************************
 * IRQ Handler
 * - called whenever GPIO goes low
 ************************************************************/ 
void IRAM_ATTR irqHandler(void){
  portENTER_CRITICAL(&mux);
  g_IrqFlag = true;
  portEXIT_CRITICAL(&mux);    
}


/************************************************************
 * Command "hello"
 * - Return: `world` 
 ************************************************************/ 
void cmd_hello(MyCommandParser::Argument *args, char *response) {  
  String msgStr;  
  msgStr = "world";  
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);
}


/************************************************************
 * Command "helloadd SUM1 SUM2"
 * - Return: `The Answer is: [SUM1+SUM2]` 
 ************************************************************/ 
void cmd_helloadd(MyCommandParser::Argument *args, char *response) {  
  uint32_t sum1;
  uint32_t sum2;
  String msgStr;  
  sum1 = (uint32_t) args[0].asUInt64;
  sum2 = (uint32_t) args[1].asUInt64;
  msgStr = "The Answer is: ";  
  msgStr.concat(sum1 + sum2);
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);
}

/************************************************************
 * Command "helloecho STRING"
 * - Return: `[STRING]` 
 ************************************************************/ 
void cmd_display(MyCommandParser::Argument *args, char *response) {      
  String msgStr;  
  msgStr = args[0].asString;
  displayJournal(String(msgStr));
}


/************************************************************
 * Command "reset"
 * - Reboot ESP32
 ************************************************************/ 
void cmd_reset(MyCommandParser::Argument *args, char *response) {
  String msgStr;  
  msgStr = "Rebooting in 5 seconds ... [please standby]: ";
  g_rebootActive = true;
  g_rebootTriggered = millis();
  msgStr.toCharArray(response, MyCommandParser::MAX_RESPONSE_SIZE);    
}


/************************************************************
 * Compose ClientID
 * - clientId = "esp32_"+ MAC 
 ************************************************************/ 
String composeClientID(void) {
  String myClientId;
  uint8_t myMac[6];
  WiFi.macAddress(myMac);  
  myClientId = ("esp32_");  
  for (int i=3; i<6; ++i) {
    myClientId.concat(String(myMac[i], 16));    
    if (i < 5)
      myClientId.concat('-');
  }  
  return myClientId;
}


/************************************************************
 * Debug Print String
 * - to Serial Console 
 * - MQTT-Message to TOPIC_LOG
 * @param[in] mes Message to be send
 ************************************************************/ 
void dbgout(String msg){  
  mqttPub (T_LOG, msg, false);
}


/************************************************************
 * Monitor Connections
 * - Check Wifi (and reconnect)
 * - Check MQTT (and reconnect)
 * TODO: reconnect on wifi error without rebooting!
 ************************************************************/ 
void monitorConnections(void) {  
  // Monitor WIFI- and MQTT-Connection   
  if (millis() - g_LastNetMonitoring > T_NET_MONITORING) {    
    // Monitor WIFI-Connection   
    g_LastNetMonitoring = millis();
    DBG_MONITOR.print("!!! WiFi localIP: ");
    DBG_MONITOR.println(WiFi.localIP());    
    if ((WiFi.status() != WL_CONNECTED) || (WiFi.localIP()[0] == 0)) {      
      DBG_ERROR.println("WiFi CONNECTION LOST");
      DBG_ERROR.println("reconnecting ...");
      WiFi.disconnect();
      WiFi.reconnect();
      if (WiFi.status() != WL_CONNECTED) {      
        DBG_ERROR.println("WiFi RECONNECTION FAILED, TRYING AGAIN LATER");  
        DBG_MONITOR.println("Not Monitoring MQTT because WiFi OFFLINE");     
        g_wificonnected = false;
      } else {        
        DBG_ERROR.println("WiFi CONNECTION RESTORED");
        g_wificonnected = true;
      }
    } else {        
        DBG_MONITOR.println("Monitoring WiFi... ONLINE");
        g_wificonnected = true;
    }
    // Monitor MQTT-Connection
    if (g_wificonnected){
      DBG_MONITOR.print("!!! MQTT: ");      
      if (!mqtt.connected()) {        
        if (millis() - g_LastMqttReconnectAttempt > T_MQTT_RECONNECT) {
          g_LastMqttReconnectAttempt = millis();
          g_MqttReconnectCount++;
          DBG_ERROR.print("MQTT Connection lost! - Error:");
          DBG_ERROR.println("mqtt.state()");
          DBG_ERROR.print(" - trying to reconnect [");
          DBG_ERROR.print(g_MqttReconnectCount);
          DBG_ERROR.println("]... ");      
          // Attempt to reconnect
          String myClientID;
          myClientID = composeClientID();
          if (mqtt.connect(myClientID.c_str(), MQTT_USER, MQTT_PASS, MQTT_PREFIX "/" T_STATUS, 1, true, STATUS_MSG_OFF, true))  { 
            // connected: publish Status ONLINE
            mqtt.publish(MQTT_PREFIX "/" T_STATUS, STATUS_MSG_ON, true);
            // resubscribe
            mqtt.subscribe(MQTT_PREFIX "/" T_CMD);          
            g_LastMqttReconnectAttempt = 0;
            g_MqttReconnectCount = 0;
            DBG_ERROR.println("MQTT SUCCESSFULLY RECONNECTED");
          } else {
            DBG_ERROR.println("MQTT RECONNECTION FAILED");
          } 
        } 
      } else {
        DBG_MONITOR.println("... ONLINE");             
      }
    }
  } 
}


/************************************************************
 * MQTT Message Received
 * - Callback function started when MQTT Message received
 * - convert Payload to lower case
 * - 
 * @param[in] topic Topic received
 * @param[in] topic Message received
 * @param[in] length Length of the Message received
 ************************************************************/ 
void mqttCallback(char* topic, byte* payload, unsigned int length) {  
  String msg;  
  char* myBuf = (char*)malloc(length + 1);    
  char response[MyCommandParser::MAX_RESPONSE_SIZE];
  // copy Buffer to String
  //   payload[length] = '\0';  // ensure that buffer is null-terminated
  //   msg = String((char*)payload);
  // Copy payload to new buffer and add '0x00' to the end
  for (int i=0; i<length; i++) {
    myBuf [i] = payload [i] ;
  }
  myBuf[length] = '\0';
  // Convert Buffer to String
  msg = String((char*)myBuf);    
  // convert String to Lower-Case
  // msg.toLowerCase();
  // Echo String
  dbgout("received MQTT-Message: \"" + msg + "\"");
  // Parse Command    
  msg.toCharArray(myBuf, msg.length() + 1);    
  parser.processCommand(myBuf, response);            
  // Publish Result;
  mqttPub (T_RESULT, String(response), false);
  // free(msgBuf);
  free(myBuf);
}


/************************************************************
 * Publish & Print Message
 * - to Serial Console
 *   - if mqttOnly is false
 * - Publish MQTT-Message to topic MQTT_PREFIX/TOPIC_LOG
 *   - MQTT_PREFIX is added by this function
 * @param[in] topic MQTT-SubTopic (MQTT_PREFIX will be added)
 * @param[in] msg Message to be send
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void mqttPub(String subtopic, String msg, boolean mqttOnly){  
  String myTopic;
  // Serial
  if (!mqttOnly) {
    DBG.println(msg);    
  }  
  // MQTT Topic
  myTopic = MQTT_PREFIX;
  myTopic.concat("/" + subtopic);
  char* topicBuf = (char*)malloc(myTopic.length() + 1);  // allocate memory
  myTopic.toCharArray(topicBuf, myTopic.length() + 1);  
  // MQTT Message
  char* msgBuf = (char*)malloc(msg.length() + 1);  // allocate memory
  msg.toCharArray(msgBuf, msg.length() + 1);  
  if (mqtt.connected()) {
    mqtt.publish(topicBuf, msgBuf);
  } else {
    DBG_ERROR.println("ERROR: MQTT-Connection lost");
  }
  free(msgBuf);
  free(topicBuf);
}


/************************************************************
 * cronjob
 * - execute things periodicaly
 * 
 ************************************************************/ 
void cronjob(void) {
  // once on Startup
  if (g_Firstrun) {
      g_LastHeartbeat_1s = millis();          
      g_LastHeartbeat_10s = g_LastHeartbeat_1s;    
      g_LastHeartbeat_30s = g_LastHeartbeat_1s;    
      g_LastHeartbeat_60s = g_LastHeartbeat_1s;    
      oncePerSecond();        
      oncePerTenSeconds();
      oncePerThirtySeconds();
      oncePerMinute();          
  } else {
    // once a second  
    if ((millis() - g_LastHeartbeat_1s) > T_HEARTBEAT_1S) {
      g_LastHeartbeat_1s = millis();    
      oncePerSecond();        
      if ((millis() - g_LastHeartbeat_10s) > T_HEARTBEAT_10S) {
        g_LastHeartbeat_10s = g_LastHeartbeat_1s;    
        oncePerTenSeconds();
        if ((millis() - g_LastHeartbeat_30s) > T_HEARTBEAT_30S) {
          g_LastHeartbeat_30s = g_LastHeartbeat_1s;    
          oncePerThirtySeconds();
          if ((millis() - g_LastHeartbeat_60s) > T_HEARTBEAT_60S) {
            g_LastHeartbeat_60s = g_LastHeartbeat_1s;    
            oncePerMinute();
          }
        }
      }
    }
  }
}


/************************************************************
 * Once per Second
 * - execute things once every second
 ************************************************************/ 
void oncePerSecond(void) {
  // Insert here Actions, which should occure every Second
}


/************************************************************
 * Once per 10 Seconds
 * - execute things once every 10 seconds
 ************************************************************/ 
void oncePerTenSeconds(void) {
  // Insert here Actions, which should occure every 10 Seconds
  sendCPUState(true);
  String msgStr;  
  // Write Millis to Display
  msgStr = '[' + (String(millis()) + "] ");  
  // printScreen(msgStr);
  // displayJournal(msgStr);
}

/************************************************************
 * Once per 30 Seconds
 * - execute things once every 30 seconds
 ************************************************************/ 
void oncePerThirtySeconds(void) {
  // Insert here Actions, which should occure every 10 Seconds
  sendNetworkState(true);
}

/************************************************************
 * Once per 60 Seconds
 * - execute things once every minute
 ************************************************************/ 
void oncePerMinute(void) {
  // Insert here Actions, which should occure every 10 Seconds
  sendSketchState(true);
}

/************************************************************
 * Reset Handler
 * - Reboot ESP32 if 
 *   - g_rebootActive 
 *   - g_rebootTriggered was T_REBOOT_TIMEOUT ms ago (default 5000)
 ************************************************************/ 
void resetHandler() {  
  if (g_rebootActive) {
    if (millis() - g_rebootTriggered > T_REBOOT_TIMEOUT) {
      g_rebootActive = false;       
      delay(1000);      
      ESP.restart();    
    }
  }
}


/************************************************************
 * Send CPU State
 * this will send Status of CPU as JSON Message:
 ************************************************************
 * {"Heap Size":349264,"FreeHeap":260632,"Minimum Free Heap":253140,
 *  "Max Free Heap":113792,"Chip Model":"ESP32-D0WDQ5",
 *  "Chip Revision":1,"Millis":5220121,"Cycle Count":3019255534
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendCPUState(boolean mqttOnly) {    
  String msgStr;
  msgStr = "{";    
  msgStr.concat("\"Heap Size\":" + String(ESP.getHeapSize()) + ",");
  msgStr.concat("\"FreeHeap\":" + String(ESP.getFreeHeap()) + ",");
  msgStr.concat("\"Minimum Free Heap\":" + String(ESP.getMinFreeHeap()) + ",");
  msgStr.concat("\"Max Free Heap\":" + String(ESP.getMaxAllocHeap()) + ",");
  msgStr.concat("\"Chip Model\":\"" + String(ESP.getChipModel()) + "\",");
  msgStr.concat("\"Chip Revision\":" + String(ESP.getChipRevision()) + ",");
  msgStr.concat("\"Millis\":" + String(millis()) + ",");
  msgStr.concat("\"Cycle Count\":" + String(ESP.getCycleCount()) + "");
  msgStr.concat("}");
  mqttPub(T_CPU, msgStr, mqttOnly);  
}


/************************************************************
 * Send Network State
 * this will send State of Network  as JSON Message:
 ************************************************************
 * {"IP-Address":"192.168.1.42",
 *  "MQTT-ClientID":"esp32_00_00_00"  
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendNetworkState(boolean mqttOnly) {    
  String msgStr;  
  // Write IP to Display
  // msgStr = '[' + (String(millis()) + "] ");
  // msgStr.concat ("IP: " + WiFi.localIP().toString());
  // printScreen(msgStr);
  // displayJournal(msgStr);
  // tft.println(msgStr);  
  // Publish MQTT
  msgStr = '{';  
  msgStr.concat("\"IP-Address\":\"" + WiFi.localIP().toString() + "\",");
  msgStr.concat("\"MQTT-ClientID\":\"" + composeClientID() + "\"");     
  msgStr.concat("}");    
  mqttPub(T_NETWORK, msgStr, mqttOnly);
}


/************************************************************
 * Send Sketch State
 * this will send Status of Sketch as JSON Message:
 ************************************************************
 * {"Compiletime":"1.11.2022, 20:9:4",
 *  "Sdk Version":"v3.3.5-1-g85c43024c","CpuFreq":240,
 *  "SketchSize":790608,"Free SketchSpace":1310720,
 *  "Sketch MD5":"fc84355a94fd722e55310621cf3645da",
 *  "Flash ChipSize":4194304,"Flash Chip Speed":40000000
 * }
 ************************************************************
 * @param[in] mqttOnly if false, then also Serial Output is generated
 ************************************************************/ 
void sendSketchState(boolean mqttOnly) {    
  String msgStr;    
  msgStr = '{';
  msgStr.concat("\"Project version\":\"" + String(VERSION) + "\",");
  msgStr.concat("\"Target\":\"" + String(TARGET) + "\",");
  msgStr.concat("\"Build timestamp\":\"" + String(BUILD_TIMESTAMP)+ "\",");  
  msgStr.concat("\"Sdk Version\":\"" + String(ESP.getSdkVersion()) + "\",");     
  msgStr.concat("\"CpuFreq\":" + String(ESP.getCpuFreqMHz()) + ",");
  msgStr.concat("\"SketchSize\":" + String(ESP.getSketchSize()) + ",");
  msgStr.concat("\"Free SketchSpace\":" + String(ESP.getFreeSketchSpace()) + ",");
  msgStr.concat("\"Sketch MD5\":\"" + String(ESP.getSketchMD5()) + "\",");
  msgStr.concat("\"Flash ChipSize\":" + String(ESP.getFlashChipSize()) + ",");
  msgStr.concat("\"Flash Chip Speed\":" + String(ESP.getFlashChipSpeed()));  
  msgStr.concat("}");  
  mqttPub(T_SKETCH, msgStr, true);  
}
                  

/************************************************************
 * Init Global Vars
 ************************************************************/ 
void setupGlobalVars(void){
  DBG_SETUP.print("- Global Vars ... ");    
  g_Firstrun = true;
  g_LastHeartbeat_1s = millis();       
  g_LastHeartbeat_10s = millis();       
  g_LastHeartbeat_30s = millis();       
  g_LastHeartbeat_60s = millis();       
  g_LastMqttReconnectAttempt = millis();     
  g_LastNetMonitoring = millis();          // Timer for Monitoring Network 
  g_LedState = 0;    
  g_MqttReconnectCount = 0;  
  g_wificonnected = false;
  g_IrqFlag = false;
  g_LastIRQ = true;  
  g_rebootActive = false;                  // if true trigger reeboot 5s after g_reboot_triggered
  g_rebootTriggered = millis();            // millis() when reboot was started  
  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}


/************************************************************
 * Init GPIO-Ports
 ************************************************************/ 
void setupGPIO(void) {  
  DBG_SETUP.print("- Init GPIO-Port... ");  
  pinMode(DBG_LED, OUTPUT);  
  pinMode(TFT_LED, OUTPUT); 
  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}

/************************************************************
 * Init Display
 ************************************************************/ 
void setupDisplay(void) {  
  DBG_SETUP.print("- Init TFT and Touch...");  

  tft.begin();
  touch.begin();
  tft.setRotation(1);
  tft.setTextWrap(false);  
  DBG_SETUP.print("tftx ="); 
  DBG_SETUP.print(tft.width()); 
  DBG_SETUP.print(" tfty ="); 
  DBG_SETUP.println(tft.height());  
  displayHeader();    
  displayJournal("Boot");
  displayJournal("Setup started...");  
  digitalWrite(TFT_LED, LOW);    // LOW to turn backlight on; 
  delay(1000);  
  delay(DEBUG_SETUP_DELAY);  
}

/************************************************************
 * displayHeader
 * @brief     Clear Display and print Header
 * @param[in] None
 * @return    None
 *********************************************************************/
void displayHeader(void) {
  // Clear Screen
  tft.fillScreen(ILI9341_WHITE);    
  // Set Font
  tft.setFont(&FreeSansBold9pt7b);      
  tft.setTextSize(0);  
  // Header
  tft.fillRect(0, 0, tft.width(), 25, ILI9341_BLACK);  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(90, 20);
  tft.println("ArduiTouch ESP");  
  tft.setTextColor(ILI9341_BLACK);    
  tft.setCursor(5, 45);    
}

/************************************************************
 * displayJournal
 * @brief     Print one Line on the Display 
 *            Scroll Text Verically if Display is full             
 * @param[in] addThisLine String to be added
 * @return    None
 *********************************************************************/
void displayJournal(String addThisLine) {
  uint8_t i;  
  uint16_t posY;    
  if (txtBufferPtr < txtBoxLines){
    // Only Print Element    
    tft.setCursor(txtBoxIndent, txtBoxStart + (txtBufferPtr * txtBoxVSpace));    
    txtBuffer[txtBufferPtr] = addThisLine;    
    tft.println(addThisLine); 
    txtBufferPtr++;
  } else {
    // Clear Area
    tft.fillRect(0, 25, tft.width(), tft.height(), ILI9341_WHITE);    
    // Locate first row  
    tft.setCursor(txtBoxIndent, txtBoxStart);
    for (i = 0; i < (txtBoxLines - 1) ; i++) {    
      txtBuffer[i] = txtBuffer[i+1];            
      tft.println(txtBuffer[i]);
      tft.setCursor(txtBoxIndent, txtBoxStart + ((i+1) * txtBoxVSpace));    
    }
    txtBuffer[txtBoxLines - 1] = addThisLine;    
    tft.println(addThisLine);
  }
}


/************************************************************
 * MQTT Init
 * - Set LastWill to 
 *   - topic:   TOPIC_STATUS 
 *   - message: STATUS_MSG_OFF
 *   - retain:  yes
 * - subscribe to MQTT_PREFIX "/" T_CMD
 * - publish 
 *   - topic:   TOPIC_STATUS 
 *   - message: STATUS_MSG_ON
 *   - retain:  yes
 ************************************************************/ 
void setupMQTT(void) {    
  String myClientID;
  myClientID = composeClientID();
  DBG_SETUP.println("Connecting to MQTT-Server ... ");
  displayJournal("Connecting to MQTT-Server ... ");
  DBG_SETUP.print("  - ClientID: ");
  DBG_SETUP.println(myClientID);  
  if (mqtt.connect(myClientID.c_str(), MQTT_USER, MQTT_PASS, MQTT_PREFIX "/" T_STATUS, 1, true, STATUS_MSG_OFF, true))  { 
    DBG_SETUP.println("  - Register Callback");
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(MQTT_BUFSIZE);
    DBG_SETUP.println("  - Publish State ONLINE");
    mqtt.publish(MQTT_PREFIX "/" T_STATUS, STATUS_MSG_ON, true);
    DBG_SETUP.print("  - Subscribe to ");
    DBG_SETUP.println(MQTT_PREFIX "/" T_CMD);
    mqtt.subscribe(MQTT_PREFIX "/" T_CMD);
    DBG_SETUP.println("  connected.");
    displayJournal("   done.");    
  } else {      
      DBG_SETUP.println("Connection failed - trying later...");
      displayJournal("  Connection failed - trying later...");
  }   
  mqtt.loop();
}


/************************************************************
 * Init Over-The-Air Update Handler
 * - set OTA-Password with ArduinoOTA.setPasswordHash("[MD5(Pass)]");
 * MD5 ("mysecretOtaPassword") = 5386fe58bd9627e6a22aee5f1726c868 
 * ATTENTION: setPasswordHash MUST NOT CONTAIN CAPS Letters A-F
 *   NO:  5386FE58BD9627E6A22AEE5F1726C868
 *   YES: 5386fe58bd9627e6a22aee5f1726c868
 ************************************************************/ 
void setupOTA(void) {  
  DBG_SETUP.print("- Init OTA... ");
  // Set Port 3232
  ArduinoOTA.setPort(3232);
  
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("homectrl32");
  
  // authenticate using password      
  // ArduinoOTA.setPasswordHash("45159b2115f99bf96aa00fa2b9da0cb9");
  ArduinoOTA.setPasswordHash(g_otahash);

  // OTA Callback: onStart
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    // DBG_SETUP.println("Start updating " + type);
    dbgout("Update Started: " + type);
  });  

  // OTA Callback: onEnd
  ArduinoOTA.onEnd([]() {
    dbgout("Update finished");
  });  

  // OTA Callback: onProgress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Progress output disabled
    // DBG_SETUP.printf("Progress: %u%%\r", (progress / (total / 100));
  });  

  // OTA Callback: onError
  ArduinoOTA.onError([](ota_error_t error) {
    DBG_SETUP.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      DBG_SETUP.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DBG_SETUP.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DBG_SETUP.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DBG_SETUP.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DBG_SETUP.println("End Failed");
    }
  });  

  // OTA Init
  ArduinoOTA.begin();

  DBG_SETUP.println("done.");
  delay(DEBUG_SETUP_DELAY);  
}


/************************************************************
 * Init Wifi 
 * - SSID: WIFI_SSID 
 * - PSK:  WIFI_PSK
 ************************************************************/ 
void setupWIFI(void) {      
  int cnt = 0;  
  DBG_SETUP.println("- Init WiFi... ");
  DBG_SETUP.print("  - connecting to '");    
  DBG_SETUP.print(g_wifissid);    
  DBG_SETUP.println("'");    
  displayJournal("Init WiFi..."); 
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_wifissid, g_wifipass);
  delay(5000);
  while ((WiFi.status() != WL_CONNECTED) && (cnt < T_WIFI_MAX_TRIES)){
    cnt++;
    DBG_SETUP.print("  - Connection failed! - Retrying [");      
    DBG_SETUP.print(cnt);      
    DBG_SETUP.println("]...");
    displayJournal("   - Connection failed!"); 
    delay(2000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    g_wificonnected = false;
    DBG_SETUP.println("  - Connection failed! - trying later...");
    displayJournal("  - Connection failed! - trying later..."); 
  } else {
    g_wificonnected = true;
    DBG_SETUP.println("  - Successfully connected");    
    DBG_SETUP.print("  IP address: ");
    displayJournal("   done. - IP: " + WiFi.localIP().toString());
    DBG_SETUP.println(WiFi.localIP());
  }  
  delay(DEBUG_SETUP_DELAY);
}


/************************************************************
 * Main Setup
 * - setupGPIO
 * - setupWIFI
 * - setupOTA
 * - setupGlobalVars 
 ************************************************************/ 
void setup(void) {  
  // Serial Port
  Serial.begin(115200);  
  DBG.println("");
  DBG.println("################################");
  DBG.println("### Darios ESP32 Hello-World ###");
  DBG.println("################################");  
  DBG.println("Version: " + String(VERSION));
  DBG.println("Target: " + String(TARGET));
  DBG.println("Build timestamp: " + String(BUILD_TIMESTAMP));

  DBG_SETUP.println("\nInit ...");
  delay(DEBUG_SETUP_DELAY);  

  // Global Vars
  setupGlobalVars();  

  // GPIO-Ports
  setupGPIO(); 

  // Display and Touch
  setupDisplay();
  
  // WiFi
  setupWIFI();

  // OTA-Update-Handler  
  setupOTA();    

  // MQTT
  setupMQTT();
   
  // MQTT Command Parser
  // "command", Params, Callback-Function 
  // s: String, d:Double, u:Unsigned Int , i:Signed Integer  
  parser.registerCommand("hello",  "", &cmd_hello);                 // hello
  parser.registerCommand("helloadd", "uu", &cmd_helloadd);          // helloadd [SUM1] [SUM2]
  parser.registerCommand("display", "s", &cmd_display);             // Display Text [STRING]
  parser.registerCommand("reset", "", &cmd_reset);                  // reset
  
  // Setup finished  
  dbgout("Init complete, starting Main-Loop");
  displayJournal("Setup complete.");  
  displayJournal("Starting Main-Loop");
  displayJournal("---------------------------------------------");
  DBG_SETUP.println("##########################################");
  delay(DEBUG_SETUP_DELAY);
}


/************************************************************
 * Main Loop
 * - OTA handler
 * - HeartBeat handler
 ************************************************************/ 
void loop(void) {
  // Main Handler
  resetHandler();                  // reset ESP if triggered (see: g_rebootActive and g_rebootTriggered)
  monitorConnections();            // Monitor (and restore) Wifi & MQTT Connection
  mqtt.loop();                     // handle MQTT Messaging  
  ArduinoOTA.handle();             // handle OTA  
  cronjob();                       // Cronjob-Handler  
  // APP Handler
  
  // First Loop completed
  g_Firstrun = false;              
}


