//------------------------------------------------------------------------------------------------------------------
//
// DOOR BELL - Doorbell with Home Assistant
// NodeMCU 0.9 (ESP-12 module)
//⠀⠀⠀            ⠀⣀⡀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⡛⢻⣧
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⣿⣟⢷⣄
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⣿⣿⣿⣿⣆⠹⣆
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⣿⣿⣿⣿⣿⣿⡄⢹⡄
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⡇⠈⣷
//⠀       ⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⣇⠀⢻⣇
//⠀      ⠀⢀⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠻⣧⡀
//⠀      ⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⠀⠀⠹⣷
//⠀      ⠀⠙⠛⠿⠿⢿⣿⣿⣿⣿⣿⣿⡿⠷⠶⠚⠋
//
//------------------------------------------------------------------------------------------------------------------
// source code: https://github.com/fortalbrz/aquarioino/etc/
//
// Materials:
// - 1 NodeMCU 0.9 (ESP-12 module)
// - 1 relay module with 2 channels
// - 1 power source 5v (1 A)
// - 1 tactile push buttom keys and 1 x resistor 10k olhms (resistor "A")
// - 1 buzzer (5V),1 resistor 1k olhms (resistor "B") and 1 transistor BC548 [1]
// - 1 led and resistor 10k olhms (resistor "C") (optional, indicates "power on")
// - 1 electrolytic capacitor 100 uF (optional)
// - flexible cab (22 agw)
//
//                     (door bell)
//   -------------    [push button]---Gnc   
//  |             |        |
//  |   NodeMCU   D0 ------x-------[10k olhms]--Vcc
//  |             D1 --------------------------- Relay "Door Open" (ch1)
//  |             D2 --------------------------- Relay "External Light" (ch2)
//  |             D3 ---[1k olhm]--x
//  |             |                | BC548
//  |             |              -----
//   -------------             E /   \ B 
//                            (gnd)   x------ [Buzzer]---Vcc
//
// Circuit Wiring Instruction:
//  - NodeMCU "Vin" --> +5 V power source (VCC)
//  - NodeMCU "Gnd" --> -5 V power source (GND)
//  - NodeMCU "D0" --> resistor 10k olhms "A" terminal 1
//  - resistor 10k olhms "A" terminal 2 --> +5 V power source (VCC) 
//  - Arduino Nano "D0" --> tactile push buttom "DOORBELL" terminal 1 (NC)
//  - tactile push buttom "DOORBELL" terminal 2 (NC) --> -5 V power source (GND)
//  - Relay 2 ch "VCC" --> +5 V power source (VCC)
//  - Relay 2 ch "GND" --> -5 V power source (GND)
//  - NodeMCU "D1" --> Relay 2 ch "IN 1" (Door Locker)
//  - NodeMCU "D2" --> Relay 2 ch "IN 2" (Front Light)
//  - NodeMCU "D3" --> resistor 1k olhms "B" terminal 1
//  - resistor 1k olhms "B" terminal 2 --> transistor BC548 base (pin 2 - middle)
//  - transistor BC548 emitter (pin 3 - right) --> -5 V power source (GND)
//  - transistor BC548 emitter (pin 1 - left) --> buzzer terminal 1
//  - buzzer terminal 2 --> +5 V power source (VCC)
//  - Led terminal 1 (positive) --> +5 V power source (VCC) (optional, "power on led")
//  - Led terminal 2 (negative/bevel) --> resistor 10k olhms "C" terminal 1 (optional, "power on led")
//  - resistor 10k olhms "C" terminal 2 --> -5 V power source (GND) (optional, "power on led")
//  - capacitor 100uF (positive) --> +5 V power source (VCC) (optional)
//  - capacitor 100uF (negative/"minus sign") --> resistor 10k olhms "C" terminal 2 (optional)
//
// Flashing the code:
//   - the NodeMUC may use the USB/serial IC CH340G, so it's necessary to install the Windows driver: 
//       - CH340G driver: http://bit.ly/44WdzVF (windows 11 compatible)
//       - driver installation instructions (pt-BR): http://bit.ly/3ZqIqc0
//   - download Arduino IDE (ver 2.0): https://www.arduino.cc/en/software
//   - install WifiEsp library:
//       - On arduino IDE select: "File" -> "Preferences" -> "Additional boards manager URL":
//          - add the URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
//       - On arduino IDE select: "Tools" -> "Manage Libraries":
//          - search and install: "WifiEsp" (by bportaluri)
//          - search and install: "PubSubClient" (by Nick)
//          - search and install: "base64_encode" (by dojyorin)
//   - uses a micro USB cable to connect to the Arduino
//   - select the "NodeMCU 0.9 (ESP-12 module)" board in the arduino IDE: "Tools" -> "Board".
//   - select "Sketch" -> "Upload"
//
// Serial Monitor:
//  sets the macro "DEBUG_MODE" as true in order to debug on serial monitor (testing only)
//
// Setup:
//   WIFI_SSID                              Wi-fi SSID
//   WIFI_PASSWORD                          Wi-fi password
//   MQTT_BROKER_ADDRESS                    MQTT broker server ip
//   MQTT_BROKER_PORT                       MQTT broker port
//   MQTT_USERNAME                          MQTT username, can be omitted if not needed
//   MQTT_PASSWORD                          MQTT password, can be omitted if not needed
//
// References:
// [1] https://medium.com/@arthurmehl/como-usar-um-buzzer-no-nodemcu-40b46973e99b
// [2] https://www.makerhero.com/blog/controle-monitoramento-iot-nodemcu-e-mqtt/
// [3] https://projecthub.arduino.cc/tmekinyan/playing-popular-songs-with-arduino-and-a-buzzer-546f4a
//
// 2024 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
// https://jorgealbuquerque.com
//
#define DEBUG_MODE false                              // enables/disables serial debugging messages

#define WIFI_SSID "wifi-ssid"                         // Wi-fi SSID
#define WIFI_PASSWORD "wifi-password"                 // Wi-fi password

#define MQTT_BROKER_ADDRESS "192.168.68.93"           // MQTT broker server ip
#define MQTT_BROKER_PORT 1883                         // MQTT broker port
#define MQTT_USERNAME "mqtt-user"                     // can be omitted if not needed
#define MQTT_PASSWORD "mqtt-password"                 // can be omitted if not needed
#define MQTT_COMMAND_TOPIC "doorbell/cmd"             // MQTT topic for send door commands (e.g., open from door)
#define MQTT_STATUS_TOPIC "doorbell/status"           // MQTT topic for doorbell status (e.g., ringing)
#define MQTT_AVAILABILITY_TOPIC "doorbell/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_AVAILABILITY_TIME 60000                  // elapsed time to send MQTT availability, in miliseconds (default: 1 min)
#define MQTT_DEVICE_ID  "doorbell_12fmo43iowerwe"     // MQTT session identifier
#define MQTT_COMMAND_RING "ring"                  
#define MQTT_COMMAND_OPEN_DOOR "open_door"

#define BUTTON_SINGLE_PUSH_TIME 2500                  // time to avoid double push button press (better usability)
#define DOOR_RELAY_POWER_ON_TIME 800                  // Font door relay power on time
#define BUZZER_POWER_ON_TIME 800                      // buzzen power on time when the doorbell button is pressed

// NodeMCU pinouts
#define D0    16  
#define D1    5   
#define D2    4   
#define D3    0   
#define D4    2
#define D5    14
#define D6    12
#define D7    13
#define D8    15
#define D9    3
#define D10   1

// pinouts
#define DOORBELL_BUTTON_PIN D0
#define DOOR_RELAY_PIN D1
#define LIGHT_RELAY_PIN D2
#define BUZZER_PIN D3

// librarys (see doc above)
#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
#include <arduino_base64.hpp>
 
WiFiClient espClient;
PubSubClient MQTT(espClient);

bool _blink = false;  // builtin led state
unsigned long _lastAvailabilityTime = millis();          
unsigned long _lastButtonPress = millis();


// prototypes
void connectWiFi(); 
void connectMQTT();
void onDoorBell();
void onMessage(char* topic, byte* payload, unsigned int length);


void setup() {
  //
  // initialization
  //
  pinMode(LED_BUILTIN, OUTPUT); // blinking led 
  
  #if (DEBUG_MODE == true)
    // serial monitor (at 115200 bauds)
    Serial.begin(115200);
  #endif

  // ring button pin
  // attachInterrupt(digitalPinToInterrupt(DOORBELL_BUTTON_PIN), onDoorBell, FALLING);

  // door locker relay pin
  pinMode(DOOR_RELAY_PIN, OUTPUT);
  digitalWrite(DOOR_RELAY_PIN, HIGH);

  // external light locker relay pin
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  digitalWrite(LIGHT_RELAY_PIN, HIGH);

  // buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // wifi connect    
  #if (DEBUG_MODE == true)
    Serial.print(F("Connecting to wifi network: "));
    Serial.println(WIFI_SSID);     
  #endif

  delay(50);
  connectWiFi();

  // MQTT broker setup
  MQTT.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  MQTT.setCallback(onMessage);
}


void loop() {
  //
  // main loop
  //
  digitalWrite(LED_BUILTIN, (_blink ? HIGH: LOW)); // blinking led
  _blink = !_blink;

  //
  // keeps MQTT connection alive    
  //
  if (!MQTT.connected()) 
      connectMQTT();

  if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
    // sends MQTT "availability" message
    MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
    _lastAvailabilityTime = millis();
  }

  MQTT.loop();

  delay(2000);

  // keeps wifi connected
  connectWiFi();
}

void onDoorBell() {
  //
  // On doorbell button pressed: publish MQTT status (ringing!)
  // 
  if ((millis() - _lastButtonPress) > BUTTON_SINGLE_PUSH_TIME) {
    #if (DEBUG_MODE == true)
      Serial.println(F("Doorbell button pressed"));
    #endif
    
    if (MQTT.connected()) {        
      digitalWrite(BUZZER_PIN, HIGH);

      MQTT.publish(MQTT_STATUS_TOPIC, MQTT_COMMAND_RING);

      delay(BUZZER_POWER_ON_TIME);
      digitalWrite(BUZZER_PIN, LOW);
    }

    _lastButtonPress = millis();
  }
}

void onMessage(char* topic, byte* payload, unsigned int length) {
  //
  // On MQTT message
  //
  char msg[length];
  base64::encode(payload, length, msg);

  #if (DEBUG_MODE == true)
    Serial.print(F(" - mqtt command: ["));
    //for (unsigned int i = 0; i < length; i++) 
    //  Serial.print((char)payload[i]);
    Serial.print(msg);
    Serial.print(F("] ("));
    Serial.print(topic);
    Serial.println(F(")"));
  #endif
  
  
  if (strcmp(msg, MQTT_COMMAND_OPEN_DOOR) == 0){
    // opens the front door
    digitalWrite(DOOR_RELAY_PIN, LOW);
    delay(DOOR_RELAY_POWER_ON_TIME);
    digitalWrite(DOOR_RELAY_PIN, HIGH);
  }
}

void connectMQTT() {
  //
  // Connects to MQTT broker
  //
  while (!MQTT.connected()) {
    #if (DEBUG_MODE == true)
      Serial.print(F("connecting to MQTT broker: "));
      Serial.println(MQTT_BROKER_ADDRESS);
    #endif        

    connectWiFi();

    if (MQTT.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        #if (DEBUG_MODE == true)
          Serial.println(F("MQTT broker connected!"));
        #endif
        MQTT.subscribe(MQTT_COMMAND_TOPIC); 
    } else {
        #if (DEBUG_MODE == true)
          Serial.println(F("Fail connecting to MQTT broker (retry in 2 secs)."));
        #endif
        delay(2000);
    }
  }
}

void connectWiFi() {
  //
  // connects to WiFi
  //
  if (WiFi.status() == WL_CONNECTED)
    return;

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(100);
    #if (DEBUG_MODE == true)
      Serial.print(F("."));
    #endif
  }

  #if (DEBUG_MODE == true)
    Serial.println();
    Serial.print(F("WiFi connected: "));
    Serial.println(WIFI_SSID);
    Serial.print(F(" - IP address: "));
    Serial.println(WiFi.localIP());
  #endif
}