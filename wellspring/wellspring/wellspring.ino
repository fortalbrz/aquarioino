//------------------------------------------------------------------------------------------------------------------
//
//
// wellspring.INO - water fountain with Home Assistant and NodeMCU
//   - Optimized for NodeMCU 1.0 (ESP-12E Module)
//
//------------------------------------------------------------------------------------------------------------------
// source code: https://github.com/fortalbrz/aquarioino/wellspring/
//
// Overview:
//
// Home Assistant is free and open-source software used for home automation (https://www.home-assistant.io). It serves as an integration platform and smart home hub,
// allowing users to control smart home devices. This project objective is to control and integrate a small water fountain with Home Assistant using the MQTT protocol
// (https://www.home-assistant.io/integrations/mqtt/). The MQTT (aka MQ Telemetry Transport) is a machine-to-machine or “Internet of Things” connectivity protocol
// on top of TCP/IP. It allows extremely lightweight publish/subscribe messaging transport.
//
//   -----------                 -------------               ------------  
//  |           |               |             |             |            | 
//  |  Home     |  <=[addon]=>  |  mosquitto  | <=[MQTT]=>  |  NodeMCU   | ---> relays
//  | Assistant |               |   broker    |             |            | ---> sensors
//  |           |               |             |             |            | 
//   -----------                 -------------               ------------   
//  (integrator)                 (MQTT broker)                 (Wi-Fi)     
//                                                          [wellspring.ino]  
//
// Features:
// - fully integrated with Home Assistant (MQTT)
// - two relays: water pump and led lights
// - push buttons: 
//    - water pump on/off button
//    - led lights turn on/off button
// - home assistant switches:
//    - water pump on/off switch
//    - led lights turn on/off switch
// - buzzer for error notifications (and play star wars tunes!)
//
// Materials:
//   - 1x NodeMCU (ESP 8266-12e) [25 BRL](https://produto.mercadolivre.com.br/MLB-1211973212-modulo-wifi-esp8266-nodemcu-esp-12e-_JM)
//   - 1x relay module 5v 2-channels [12 BRL](https://produto.mercadolivre.com.br/MLB-1758894951-modulo-rele-rele-5v-4-canais-para-arduino-pic-raspberry-pi-_JM)
//   - 1x active buzzer 5v [2.30 BRL](https://www.a2robotics.com.br/buzzer-ativo-5v)
//   - 2x NPN BC548 transistor [1.20  BRL](https://produto.mercadolivre.com.br/MLB-1712833525-transistor-bc548-npn-para-projetos-10-pecas-_JM )
//   - 2x tactile push button [0.20 BRL](https://www.a2robotics.com.br/chave-tactil-6x6x5mm-4-terminais)
//   - 2x resistor 1k ohms (1/8w) 
//   - 20x led and resistor 480 ohms (1/8w) 
//   - 1x power supply 5vdc (1A) [14 BRL](https://produto.mercadolivre.com.br/MLB-3445635491-fonte-alimentaco-5v-1a-bivolt-roteador-wireles-modem-d-link-_JM)
//   - 1x on/off switch 
//   - 1x led and resistor 10k ohms (optional, indicates "power on")
//   - 1x electrolytic capacitor 100 uF (optional)
//   - 1x aquarium pump (3w, 200L/h)
//   - 1x small buddha statue
//   - 1x ceramic Plate (base) 
//   - 4x small clay Pots
//   - 1x white stones (1kg) 
//   - flexible cab (22 agw)
//   - silicone Glue (50g)
//   - "durepox" epoxy resin (100g) 
//
// Circuit Wiring Instruction:
//  - power supply connections:
//    - NodeMCU (GND) -> power supply 5vdc (negative/Gnd)
//    - NodeMCU (Vin) -> power supply 5vdc (positive/Vcc)  
// - relay module (2-channels - sets "NUMBER_OF_RELAYS" as "1" for use a single relay module)):
//    - Relay 2 ch (VCC) -> power supply 5vdc (negative/Gnd)
//    - Relay 2 ch (GND) -> power supply 5vdc (positive/Vcc)
//    - Relay 2 ch (In 1) [water pump] -> NodeMCU (D4)
//    - Relay 2 ch (In 2) [extra outlet] -> NodeMCU (D5)  
// - push buttons (optional - set "USE_PUSH_BUTTONS" as false to ignore):
//    - NodeMCU "D1" -> resistor 1k ohms "A" terminal 1 
//    - resistor 1k ohms "A" terminal 2 -> NodeMCU (3.3V)
//    - NodeMCU "D2" -> resistor 1k ohms "B" terminal 1
//    - resistor 1k ohms "B" terminal 2 -> NodeMCU (3.3V)
//    - NodeMCU "D1" -> tactile push button "water pump on/off" terminal 1 (NC)
//    - tactile push button "water pump on/off" terminal 2 (NC) -> -5 V power source (GND)
//    - NodeMCU "D2" -> tactile push button "light on/off" terminal 1 (NC)
//    - tactile push button "light on/off" terminal 2 (NC) -> -5 V power source (GND)
// - leds (up to 20 leds):
//    - NodeMCU "D3" -> resistor 1k ohms terminal 1
//    - resistor 1k ohms terminal 2 -> BC548 transistor "A" base (pin 2 - leg at middle)
//    - BC548 transistor "A" emitter (pin 3 - right leg) -> -5 V power source (GND)
//    - BC548 transistor "A" collector (pin 1 - left leg) -> 480 ohms resistor "led" terminal 1 (connects 20 up to leds in parallel at this point)
//    - 480 ohms resistor "led" terminal 1 -> Led terminal 2 (negative/bevel) (connects up to 20 leds in parallel)
//    - Led terminal 1 (positive) -> +5 V power source (Vcc) (connects 20 up to leds in parallel at this point)    
// - buzzer: (optional - set "PLAY_TUNES" as false to ignore)
//    - NodeMCU "D6" -> resistor 1k ohms terminal 1
//    - resistor 1k ohms terminal 2 -> BC548 transistor "B" base (pin 2 - leg at middle)
//    - BC548 transistor "B" collector (pin 1 - left leg) -> buzzer 5v negative terminal (-)
//    - buzzer 5v positive terminal (+) -> +5 V power source (Vcc)
//    - BC548 transistor "B" emitter (pin 3 - right leg) -> -5 V power source (GND)
// - power on indicator led (optional)
//    - Led terminal 1 (positive) -> +5 V power source (VCC) (optional, "power on led")
//    - Led terminal 2 (negative/bevel) -> resistor 10k ohms "D" terminal 1 (optional, "power on led")
//    - resistor 10k ohms "D" terminal 2 -> -5 V power source (GND) (optional, "power on led")
// - filter capacitor (optional)
//    - capacitor 100uF (positive) -> +5 V power source (VCC) (optional)
//    - capacitor 100uF (negative/"minus sign") -> resistor 10k ohms "D" terminal 2 (optional)
//
//   Snubber Filter:
//
//   The suppressor filter (aka "snubber"), is a device that serves to limit voltage spikes. On this project
//   is recommend the snubber filter in parallel with the water pump (eletric motor) outlets.
//
//      ---------------[relay]----
//      o         |               |
//     pump    [R=10R]            |
//    outlet      |              Vac              
//      o     [C=470nF]           |
//      |         |               |
//      --------------------------
//
//   The snubber is a RC filter, with recommended values of R=10 olhm (1/8 w) and C=470nf (600v)
//   REMARK: Do not use the snubber filter within a LED light outlet, or a "ghost light" effect 
//   may happen when the light is off.
//
//   Flashing the code:
//
//   Drivers (CH340g) for NodeMCU:
//    - CH340g USB/Serial driver (windows 11 compatible driver): https://bit.ly/44WdzVF 
//    - driver install instructions (pt): https://bit.ly/3ZqIqc0
//   
//   The NodeMCU module should be programed with the sketch with the [Arduino IDE](https://www.arduino.cc/en/software) 
//    - go to File > Preferences
//    - on "Additonal boards manager", set the value "http://arduino.esp8266.com/stable/package_esp8266com_index.json"
//    - go to Tools > Board > Board Manager
//    - search for “ESP8266”
//    - install the ESP8266 Community package ("esp8266" by ESP8266 Community)//   
//    - select board "NodeMCU 1.0 (ESP-12E Module)" and coonected COM port (checks at Windows "device manager")
//    - select "Sketch" > "Upload"
//
// Serial Monitor:
//  sets the macro "DEBUG_MODE" as true in order to debug on serial monitor (testing only) - ensure the baud rate setup!
//
// Configuration flags:
//   - WIFI_SSID: Wi-fi SSID (required)
//   - WIFI_PASSWORD: Wi-fi password (required)
//   - MQTT_BROKER_ADDRESS MQTT: broker server ip address (required)
//   - MQTT_BROKER_PORT: MQTT broker port (required) (default: 1883)
//   - MQTT_USERNAME: mqtt broker username (required)
//   - MQTT_PASSWORD: mqtt broker password (required)
//   - MQTT_DEVICE_ID: MQTT session identifier (changes for more then one gardeino on the same MQTT broker)
//   - DEBUG_MODE: enables/disables serial monitor debugging messages
//   - PLAY_TUNES: enables play music themes
//
//  MQTT topics:
//    - wellspring/available: sensors availability ["online"/"offline"]
//    - wellspring/cmd: pushes commands to NodeMCU [home assistant -> bettaino]:
//         "pump on/off": turns on/off the water pump (i.e. relay #1)
//         "light on/off": turns on/off the led lights (i.e. relay #2)
//         "beep": plays a beep sound [for any notification]
//         "play [sw/dv/tetris/mario/got/gf/nokia/notice]": plays an tune by name [for any notification] (none for random)
//         "refresh": update MQTT state [debug only]
//    - wellspring/state: retrieves NodeMCU states as json [bettaino -> home assistant]
//         {
//            "light": "on",                // "buddha" led lights state (led lights): [on/off]
//            "pump": "on",                 // relay #1 state (water pump): [on/off]
//            "relay2": "on",               // relay #2 state (extra): [on/off]
//            "rssi": -68,                  // wifi signal power 
//            "ip": "192.168.0.58",         // ip address
//            "mac": "E3:3A:21:34:FE:21",   // mac address
//         } 
//------------------------------------------------------------------------------------------------------------------
//
// Jorge Albuquerque (2024) - https://linkedin.com/in/jorgealbuquerque
// https://www.jorgealbuquerque.com
//
#define DEBUG_MODE false                             // enables/disables serial debugging messages
#define PLAY_TUNES true                             // enables play music
#define USE_PUSH_BUTTONS true                       // enables push buttons
#define NUMBER_OF_RELAYS 2                          // nummber of relay channes: 1 or 2 (default: 2)
//------------------------------------------------------------------------------------------------------------------
//
// Configuration flags (enables or disables features in order to "skip" unwanted hardware)
//
//------------------------------------------------------------------------------------------------------------------
Wi-fi setup
#define WIFI_SSID "wifi-ssid"                         // Wi-fi SSID (required)
#define WIFI_PASSWORD "wifi-password"                 // Wi-fi password (required)
// MQTT setup
#define MQTT_BROKER_ADDRESS "192.168.68.10"           // MQTT broker server IP (required)
#define MQTT_BROKER_PORT 1883                         // MQTT broker port (required)
#define MQTT_USERNAME "mqtt-user"                     // MQTT broker username (required)
#define MQTT_PASSWORD "mqtt-password"                 // MQTT broker password (required) 
// MQTT topics
#define MQTT_COMMAND_TOPIC "wellspring/cmd"             // MQTT topic for send door commands (e.g., open from door)
#define MQTT_STATUS_TOPIC "wellspring/status"           // MQTT topic for doorbell status
#define MQTT_AVAILABILITY_TOPIC "wellspring/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_DEVICE_ID "wellspring_12fmo43iowerwe2"     // MQTT session identifier
// others
#define MQTT_STATUS_UPDATE_TIME 300000                  // maximum time to send the MQTT state update, in miliseconds (default: 5 min)
#define MQTT_AVAILABILITY_TIME 60000                    // elapsed time to send MQTT availability, in miliseconds (default: 1 min)
#define BUZZER_POWER_ON_TIME 800                        // default buzzen power on time, in miliseconds (beep)
#define BUTTON_PRESSING_TIME 300                        // default buzzen power on time, in miliseconds (beep)
#define SERIAL_BAUDRATE 9600                            // serial monitor baud rate (only for debuging)
#define LOOP_TIME 200                                   // processor loop timer, in miliseconds
//
// pins definitions (ModeMCU)
//
#define PUSH_BUTTON_WATER_PUMP_PIN D1                   // D1: pull-up (high) - tactile push button "water pump"
#define PUSH_BUTTON_BUDDHA_LIGHT_PIN D2                 // D2: pull-up (high) - tactile push button "buddha lights"
#define RELAY_BUDDHA_LIGHT_PIN D3                       // D3: pull-up (high) (remark: boot fails on low) - "buddha" led lights (npn transistor)
#define RELAY_WATER_PUMP_PIN D4                         // D4: pull-up (high) - relay #1: feeder push button [connected to build-in LED]
#define RELAY_EXTRA_PIN D5                              // D5: pull-up (high) - relay #2: general purpose (normally open)
#define BUZZER_PIN D6                                   // D6: pull-up (high) (remark: boot fails on low) - buzzer
//
// protocol commands
//
#define MQTT_COMMAND_BUDDHA_LIGHT_ON "light on"
#define MQTT_COMMAND_BUDDHA_LIGHT_OFF "light off"
#define MQTT_COMMAND_WATER_PUMP_ON "pump on"
#define MQTT_COMMAND_WATER_PUMP_OFF "pump off"
#define MQTT_COMMAND_RELAY2_ON "relay2 on"
#define MQTT_COMMAND_RELAY2_OFF "relay2 off"
#define MQTT_COMMAND_DEBUG_REFRESH "refresh"
#define MQTT_COMMAND_DEBUG_BEEP "beep"
#define MQTT_COMMAND_DEBUG_PLAY "play"
#define MQTT_COMMAND_DEBUG_PLAY_STAR_WARS "play sw"
#define MQTT_COMMAND_DEBUG_PLAY_DARTH_VADER "play dv"
#define MQTT_COMMAND_DEBUG_PLAY_TETRIS "play tetris"
#define MQTT_COMMAND_DEBUG_PLAY_MARIO "play mario"
#define MQTT_COMMAND_DEBUG_PLAY_GOT "play got"
#define MQTT_COMMAND_DEBUG_PLAY_GODFATHER "play gf"
#define MQTT_COMMAND_DEBUG_PLAY_BRAHMS_LULLABY "play brahms"
#define MQTT_COMMAND_DEBUG_PLAY_PACHELBEL_CANNON_IN_D "play cannon"


//------------------------------------------------------------------------------------------------------------------

// librarys (see doc above)
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


WiFiClient espClient;
PubSubClient MQTT(espClient);
//
// internal states (globals)
//
bool _lightOn = false;
bool _pumpOn = false;
bool _relay2On = false;
unsigned long _lastAvailabilityTime = 0;
unsigned long _lastStatusUpdateTime = 0;
unsigned long _lastButtonPressTime = 0;


#if (PLAY_TUNES == true)
  /* ----------------------------------------------------------------------
    Songs available at https://github.com/robsoncouto/arduino-songs
                                                Robson Couto, 2019
    ----------------------------------------------------------------------*/
  #define NOTE_B0 31
  #define NOTE_C1 33
  #define NOTE_CS1 35
  #define NOTE_D1 37
  #define NOTE_DS1 39
  #define NOTE_E1 41
  #define NOTE_F1 44
  #define NOTE_FS1 46
  #define NOTE_G1 49
  #define NOTE_GS1 52
  #define NOTE_A1 55
  #define NOTE_AS1 58
  #define NOTE_B1 62
  #define NOTE_C2 65
  #define NOTE_CS2 69
  #define NOTE_D2 73
  #define NOTE_DS2 78
  #define NOTE_E2 82
  #define NOTE_F2 87
  #define NOTE_FS2 93
  #define NOTE_G2 98
  #define NOTE_GS2 104
  #define NOTE_A2 110
  #define NOTE_AS2 117
  #define NOTE_B2 123
  #define NOTE_C3 131
  #define NOTE_CS3 139
  #define NOTE_D3 147
  #define NOTE_DS3 156
  #define NOTE_E3 165
  #define NOTE_F3 175
  #define NOTE_FS3 185
  #define NOTE_G3 196
  #define NOTE_GS3 208
  #define NOTE_A3 220
  #define NOTE_AS3 233
  #define NOTE_B3 247
  #define NOTE_C4 262
  #define NOTE_CS4 277
  #define NOTE_D4 294
  #define NOTE_DS4 311
  #define NOTE_E4 330
  #define NOTE_F4 349
  #define NOTE_FS4 370
  #define NOTE_G4 392
  #define NOTE_GS4 415
  #define NOTE_A4 440
  #define NOTE_AS4 466
  #define NOTE_B4 494
  #define NOTE_C5 523
  #define NOTE_CS5 554
  #define NOTE_D5 587
  #define NOTE_DS5 622
  #define NOTE_E5 659
  #define NOTE_F5 698
  #define NOTE_FS5 740
  #define NOTE_G5 784
  #define NOTE_GS5 831
  #define NOTE_A5 880
  #define NOTE_AS5 932
  #define NOTE_B5 988
  #define NOTE_C6 1047
  #define NOTE_CS6 1109
  #define NOTE_D6 1175
  #define NOTE_DS6 1245
  #define NOTE_E6 1319
  #define NOTE_F6 1397
  #define NOTE_FS6 1480
  #define NOTE_G6 1568
  #define NOTE_GS6 1661
  #define NOTE_A6 1760
  #define NOTE_AS6 1865
  #define NOTE_B6 1976
  #define NOTE_C7 2093
  #define NOTE_CS7 2217
  #define NOTE_D7 2349
  #define NOTE_DS7 2489
  #define NOTE_E7 2637
  #define NOTE_F7 2794
  #define NOTE_FS7 2960
  #define NOTE_G7 3136
  #define NOTE_GS7 3322
  #define NOTE_A7 3520
  #define NOTE_AS7 3729
  #define NOTE_B7 3951
  #define NOTE_C8 4186
  #define NOTE_CS8 4435
  #define NOTE_D8 4699
  #define NOTE_DS8 4978
  #define REST 0

  // change this to whichever pin you want to use
  int buzzer = BUZZER_PIN;

  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
  
  // star wars theme
  int melody_star_wars[] = {
    NOTE_F5, 2, NOTE_C6, 2,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8, NOTE_G5, 2
  };

  // darth vader imperial march
  int melody_darth_vader[] = {  
    NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,
    NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,//4
    NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
    NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,    
    NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4, NOTE_GS5,-8, NOTE_G5,16, //7 
    NOTE_DS5,16, NOTE_D5,16, NOTE_DS5,8, REST,8, NOTE_A4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,
    NOTE_C5,16, NOTE_B4,16, NOTE_C5,16, REST,8, NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,-16,//9
    NOTE_C5,4, NOTE_A4,-8, NOTE_C5,16, NOTE_E5,2
  };

  // tetris
  int melody_tetris[] = {
    NOTE_E5, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_C5,8,  NOTE_B4,8,
    NOTE_A4, 4,  NOTE_A4,8,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
    NOTE_B4, -4,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
    NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,8,  NOTE_A4,4,  NOTE_B4,8,  NOTE_C5,8,
    NOTE_D5, -4,  NOTE_F5,8,  NOTE_A5,4,  NOTE_G5,8,  NOTE_F5,8,
    NOTE_E5, -4,  NOTE_C5,8,  NOTE_E5,4,  NOTE_D5,8,  NOTE_C5,8,
    NOTE_B4, 4,  NOTE_B4,8,  NOTE_C5,8,  NOTE_D5,4,  NOTE_E5,4,
    NOTE_C5, 4,  NOTE_A4,4,  NOTE_A4,4, REST, 4
  };

  // super mario bros
  int melody_mario[] = {
    NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, NOTE_E5,8, //1
    NOTE_G5,4, REST,4, NOTE_G4,8, REST,4, 
    NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4, // 3
    NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
    NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
    REST,8, NOTE_E5,4,NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
    NOTE_C5,-4, NOTE_G4,8, REST,4, NOTE_E4,-4, // repeats from 3
    NOTE_A4,4, NOTE_B4,4, NOTE_AS4,8, NOTE_A4,4,
    NOTE_G4,-8, NOTE_E5,-8, NOTE_G5,-8, NOTE_A5,4, NOTE_F5,8, NOTE_G5,8,
    REST,8, NOTE_E5,4,NOTE_C5,8, NOTE_D5,8, NOTE_B4,-4,
    
    REST,4, NOTE_G5,8, NOTE_FS5,8, NOTE_F5,8, NOTE_DS5,4, NOTE_E5,8,//7
    REST,8, NOTE_GS4,8, NOTE_A4,8, NOTE_C4,8, REST,8, NOTE_A4,8, NOTE_C5,8, NOTE_D5,8,
    REST,4, NOTE_DS5,4, REST,8, NOTE_D5,-4,
    NOTE_C5,2, REST,2,

    REST,4, NOTE_G5,8, NOTE_FS5,8, NOTE_F5,8, NOTE_DS5,4, NOTE_E5,8,//repeats from 7
    REST,8, NOTE_GS4,8, NOTE_A4,8, NOTE_C4,8, REST,8, NOTE_A4,8, NOTE_C5,8, NOTE_D5,8,
    REST,4, NOTE_DS5,4, REST,8, NOTE_D5,-4,
    NOTE_C5,2, REST,2
  };

  // game of thrones theme
  int melody_game_of_thrones[] = {
    NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, //1
    NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_DS4,16, NOTE_F4,16,
    NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
    NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16, NOTE_G4,8, NOTE_C4,8, NOTE_E4,16, NOTE_F4,16,
    NOTE_G4,-4, NOTE_C4,-4,//5

    NOTE_DS4,16, NOTE_F4,16, NOTE_G4,4, NOTE_C4,4, NOTE_DS4,16, NOTE_F4,16, //6
    NOTE_D4,-1, //7 and 8
    NOTE_F4,-4, NOTE_AS3,-4,
    NOTE_DS4,16, NOTE_D4,16, NOTE_F4,4, NOTE_AS3,-4,
    NOTE_DS4,16, NOTE_D4,16, NOTE_C4,-1, //11 and 12
  };

  // godfather theme
  int melody_godfather[] = {
    REST, 4, REST, 8, REST, 8, REST, 8, NOTE_E4, 8, NOTE_A4, 8, NOTE_C5, 8, //1
    NOTE_B4, 8, NOTE_A4, 8, NOTE_C5, 8, NOTE_A4, 8, NOTE_B4, 8, NOTE_A4, 8, NOTE_F4, 8, NOTE_G4, 8,
    NOTE_E4, 2, NOTE_E4, 8, NOTE_A4, 8, NOTE_C5, 8,
    NOTE_B4, 8, NOTE_A4, 8, NOTE_C5, 8, NOTE_A4, 8, NOTE_C5, 8, NOTE_A4, 8, NOTE_E4, 8, NOTE_DS4, 8,
    
    NOTE_D4, 2, NOTE_D4, 8, NOTE_F4, 8, NOTE_GS4, 8, //5
    NOTE_B4, 2, NOTE_D4, 8, NOTE_F4, 8, NOTE_GS4, 8,
    NOTE_A4, 2, NOTE_C4, 8, NOTE_C4, 8, NOTE_G4, 8, 
    NOTE_F4, 8, NOTE_E4, 8, NOTE_G4, 8, NOTE_F4, 8, NOTE_F4, 8, NOTE_E4, 8, NOTE_E4, 8, NOTE_GS4, 8,
    NOTE_A4, 2, 
  };

  // Wiegenlied (Brahms' Lullaby)
  int melody_brahms[] = {
    NOTE_G4, 4, NOTE_G4, 4, //1
    NOTE_AS4, -4, NOTE_G4, 8, NOTE_G4, 4,
    NOTE_AS4, 4, REST, 4, NOTE_G4, 8, NOTE_AS4, 8,
    NOTE_DS5, 4, NOTE_D5, -4, NOTE_C5, 8,
    NOTE_C5, 4, NOTE_AS4, 4, NOTE_F4, 8, NOTE_G4, 8,
    NOTE_GS4, 4, NOTE_F4, 4, NOTE_F4, 8, NOTE_G4, 8,
    NOTE_GS4, 4, REST, 4, NOTE_F4, 8, NOTE_GS4, 8,
    NOTE_D5, 8, NOTE_C5, 8, NOTE_AS4, 4, NOTE_D5, 4,

    NOTE_DS5, 4, REST, 4, NOTE_DS4, 8, NOTE_DS4, 8, //8
    NOTE_DS5, 2, NOTE_C5, 8, NOTE_GS4, 8,
    NOTE_AS4, 2, NOTE_G4, 8, NOTE_DS4, 8,
    NOTE_GS4, 4, NOTE_AS4, 4, NOTE_C5, 4,
    NOTE_AS4, 2, NOTE_DS4, 8, NOTE_DS4, 8,
    NOTE_DS5, 2, NOTE_C5, 8, NOTE_GS4, 8,
    NOTE_AS4, 2, NOTE_G4, 8, NOTE_DS4, 8,
    NOTE_AS4, 4, NOTE_G4, 4, NOTE_DS4, 4,
    NOTE_DS4, 2
  };

  // Pachelbel's Canon
  int melody_cannon_in_D[] = {
    NOTE_FS4,2, NOTE_E4,2, NOTE_D4,2, NOTE_CS4,2,
    NOTE_B3,2, NOTE_A3,2, NOTE_B3,2, NOTE_CS4,2,
    NOTE_FS4,2, NOTE_E4,2, NOTE_D4,2, NOTE_CS4,2,
    NOTE_B3,2, NOTE_A3,2, NOTE_B3,2, NOTE_CS4,2,
    NOTE_D4,2, NOTE_CS4,2, NOTE_B3,2, NOTE_A3,2,
    NOTE_G3,2, NOTE_FS3,2, NOTE_G3,2, NOTE_A3,2,

    NOTE_D4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8, 
    NOTE_A4,4, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, 
    NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,4, NOTE_FS3,8, NOTE_G3,8,
    NOTE_A3,8, NOTE_G3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,2,
    NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_G3,4, NOTE_FS3,8, NOTE_E3,8, 
    NOTE_FS3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8, NOTE_A3,8, NOTE_B3,8,

    NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4, NOTE_CS4,8, NOTE_D4,8,
    NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2,
    NOTE_A4,4, NOTE_FS4,8, NOTE_G4,8, NOTE_A4,4,
    NOTE_FS4,8, NOTE_G4,8, NOTE_A4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8,
    NOTE_D4,8, NOTE_E4,8, NOTE_FS4,8, NOTE_G4,8, NOTE_FS4,4, NOTE_D4,8, NOTE_E4,8,
    NOTE_FS4,8, NOTE_CS4,8, NOTE_A3,8, NOTE_A3,8,

    NOTE_CS4,4, NOTE_B3,4, NOTE_D4,8, NOTE_CS4,8, NOTE_B3,4,
    NOTE_A3,8, NOTE_G3,8, NOTE_A3,4, NOTE_D3,8, NOTE_E3,8, NOTE_FS3,8, NOTE_G3,8,
    NOTE_A3,8, NOTE_B3,4, NOTE_G3,4, NOTE_B3,8, NOTE_A3,8, NOTE_B3,4,
    NOTE_CS4,8, NOTE_D4,8, NOTE_A3,8, NOTE_B3,8, NOTE_CS4,8, NOTE_D4,8, NOTE_E4,8,
    NOTE_FS4,8, NOTE_G4,8, NOTE_A4,2  
  };

  // available melodies
  int *pmelodies[]=
  {
    melody_star_wars,
    melody_darth_vader,
    melody_tetris,
    melody_mario,
    melody_game_of_thrones,
    melody_godfather,
    melody_brahms,
    melody_cannon_in_D
  };

  // change this to make the song slower or faster
  int tempo[] = 
  {
    108, 
    120, 
    144, 
    200,
    85,
    80,
    76,
    100
  };

  int notes[] = 
  {
    sizeof(melody_star_wars) / sizeof(melody_star_wars[0]) / 2,
    sizeof(melody_darth_vader) / sizeof(melody_darth_vader[0]) / 2,
    sizeof(melody_tetris) / sizeof(melody_tetris[0]) / 2,
    sizeof(melody_mario) / sizeof(melody_mario[0]) / 2,
    sizeof(melody_game_of_thrones) / sizeof(melody_game_of_thrones[0]) / 2,
    sizeof(melody_godfather) / sizeof(melody_godfather[0]) / 2,
    sizeof(melody_brahms) / sizeof(melody_brahms[0]) / 2,
    sizeof(melody_cannon_in_D) / sizeof(melody_cannon_in_D[0]) / 2    
  };

#endif



//------------------------------------------------------------------------------------------------------------------
//
// prototypes
//
//------------------------------------------------------------------------------------------------------------------
void connectWiFi();
void connectMQTT();
void onMessage(char* topic, byte* payload, unsigned int length);
void updateStates();
#if (USE_PUSH_BUTTONS == true)
  ICACHE_RAM_ATTR void onPumpPushButton();
  ICACHE_RAM_ATTR void onLightPushButton();
#endif
void playTune(unsigned int song);
void setRelay(const unsigned int& relayPin, bool state);
void beep(const uint8_t& n, const unsigned long& time);
String getMacAddress();
String toStr(const bool& value);


//--------------------------------------------------------------------------------------------------
//
// main code
//
//--------------------------------------------------------------------------------------------------

void setup() {
  //
  // initialization
  //
  #if (DEBUG_MODE == true)
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println(F(" "));
    Serial.println(F("-- Debug Mode --"));
  #endif

  // digital ouputs
  pinMode(RELAY_BUDDHA_LIGHT_PIN, OUTPUT);
  digitalWrite(RELAY_BUDDHA_LIGHT_PIN, LOW);
  // relays
  #if (NUMBER_OF_RELAYS > 0)
    pinMode(RELAY_WATER_PUMP_PIN, OUTPUT);  
    digitalWrite(RELAY_WATER_PUMP_PIN, HIGH);
  #endif
  #if (NUMBER_OF_RELAYS > 1)
    pinMode(RELAY_EXTRA_PIN, OUTPUT);
    digitalWrite(RELAY_EXTRA_PIN, HIGH);
  #endif
  
  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  #if (USE_PUSH_BUTTONS == true)
    // push buttons
    pinMode(PUSH_BUTTON_BUDDHA_LIGHT_PIN, INPUT_PULLUP);    
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_BUDDHA_LIGHT_PIN), onLightPushButton, FALLING);

    pinMode(PUSH_BUTTON_WATER_PUMP_PIN, INPUT_PULLUP);    
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_WATER_PUMP_PIN), onPumpPushButton, FALLING);
  #endif

  // starting beep
  beep(3, 100);

  // wifi connect
  #if (DEBUG_MODE == true)
    Serial.print(F("Connecting to wifi network: "));
    Serial.println(WIFI_SSID);
  #endif

  delay(50);
  // Wi-fi connection
  connectWiFi();

  // MQTT broker setup
  MQTT.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  MQTT.setCallback(onMessage);
  delay(150);
  updateStates();
  delay(50);

  // default: water pump on
  #if (DEBUG_MODE == false)
    setRelay(RELAY_WATER_PUMP_PIN, true);
  #else
    Serial.println(F("Setup finished"));    
  #endif
}


void loop() {
  //
  // main loop
  //  
  if (!MQTT.connected())
    connectMQTT();

  // sends MQTT "availability" message
  if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
    MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
    _lastAvailabilityTime = millis();
  }
    
  // status update (with max timespan for 5 minutes)
  if ((millis() - _lastStatusUpdateTime) > MQTT_STATUS_UPDATE_TIME) {    
    updateStates();    
  }

  // wait some
  delay(LOOP_TIME);

  // MQTT loop
  MQTT.loop();
  
  // keeps wifi connected
  connectWiFi();  
}



//--------------------------------------------------------------------------------------------------
//
// push buttons
//
//--------------------------------------------------------------------------------------------------

#if (USE_PUSH_BUTTONS == true)

  void onLightPushButton() {
    //
    // "buddha" lightening button handler: invertd light state (on/off)
    //  
    //setRelay(RELAY_BUDDHA_LIGHT_PIN, !_lightOn);
    if ((millis() - _lastButtonPressTime) > BUTTON_PRESSING_TIME) {
      _lightOn = !_lightOn;
      digitalWrite(RELAY_BUDDHA_LIGHT_PIN, (_lightOn ? HIGH : LOW));      
      _lastButtonPressTime = millis();      
    }
  }

  void onPumpPushButton() {
    //
    // water pump button handler: invertd pump state (on/off)
    //  
    //setRelay(RELAY_WATER_PUMP_PIN, !_pumpOn);
    if ((millis() - _lastButtonPressTime) > BUTTON_PRESSING_TIME) {
      _pumpOn = !_pumpOn;
      digitalWrite(RELAY_WATER_PUMP_PIN, (_pumpOn ? LOW : HIGH));
      _lastButtonPressTime = millis();      
    }
  }

#endif
//--------------------------------------------------------------------------------------------------
//
// MQTT
//
//--------------------------------------------------------------------------------------------------
void onMessage(char* topic, byte* payload, unsigned int length) {
  //
  // On MQTT message
  //
  char msg[length + 1];
  memcpy(msg, payload, length);
  msg[length] = '\0';  // NULL;

  #if (DEBUG_MODE == true)
    Serial.print(F(" - mqtt command: ["));
    Serial.print(msg);
    Serial.println(F("]"));
  #endif

  //
  // decode Home Assistant command (MQTT)
  //
  if (strcmp(msg, MQTT_COMMAND_BUDDHA_LIGHT_ON) == 0) {
    // turns on the "buddha" led lights
    setRelay(RELAY_BUDDHA_LIGHT_PIN, true);

  } else if (strcmp(msg, MQTT_COMMAND_BUDDHA_LIGHT_OFF) == 0) {
    // turns off the "buddha" led lights
    setRelay(RELAY_BUDDHA_LIGHT_PIN, false);

  } else if (strcmp(msg, MQTT_COMMAND_WATER_PUMP_ON) == 0) {
    // turn on the water pump (relay #1)
    setRelay(RELAY_WATER_PUMP_PIN, true);

  } else if (strcmp(msg, MQTT_COMMAND_WATER_PUMP_OFF) == 0) {
    // turn off the water pump (relay #1)
    setRelay(RELAY_WATER_PUMP_PIN, false);    

  } else if (strcmp(msg, MQTT_COMMAND_RELAY2_ON) == 0) {
    // turn on the led lights (relay #2)
    setRelay(RELAY_EXTRA_PIN, true);

  } else if (strcmp(msg, MQTT_COMMAND_RELAY2_OFF) == 0) {
    // turn off the led lights (relay #2)
    setRelay(RELAY_EXTRA_PIN, false);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_BEEP) == 0) {
    // plays a "beep" sound
    beep(3, 100);
  
  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY) == 0) {
    // plays random tune
    int song = random(0, 7);
    playTune(song);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_STAR_WARS) == 0) {
    // playd the star wars theme
    playTune(0);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_DARTH_VADER) == 0) {
    // playd the darth vader theme
    playTune(1);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_TETRIS) == 0) {
    // plays the tetris theme
    playTune(2);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_MARIO) == 0) {
    // plays the super mario bros theme
    playTune(3);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_GOT) == 0) {
    // plays the super mario bros theme
    playTune(4);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_GODFATHER) == 0) {
    // plays the super mario bros theme
    playTune(5);    

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_BRAHMS_LULLABY) == 0) {
    // plays the super mario bros theme
    playTune(6);    

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_PACHELBEL_CANNON_IN_D) == 0) {
    // plays the super mario bros theme
    playTune(7);    

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_REFRESH) == 0) {
    // updated mqtt states (debug)
    updateStates();
  }  
}


void updateStates() {
  //
  // MQTT publish states update
  //
  if (MQTT.connected()) {    

    StaticJsonDocument<300> json;
    json["light"] = toStr(_lightOn);
    json["pump"] = toStr(_pumpOn);
    json["relay2"] = toStr(_relay2On);
    json["rssi"] = WiFi.RSSI();
    json["ip"] = WiFi.localIP().toString();
    json["mac"] = getMacAddress();
        
    //unsigned int n = measureJson(json) + 1;
    char message[200];
    serializeJson(json, message);

    #if (DEBUG_MODE == true)
        Serial.print(F("mqtt update: "));
        Serial.println(message);
    #endif
    yield();

    MQTT.publish(MQTT_STATUS_TOPIC, message);
    yield();

    _lastStatusUpdateTime = millis();
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

    // retry until connection
    connectWiFi();

    if (MQTT.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      #if (DEBUG_MODE == true)
            Serial.println(F("MQTT broker connected!"));
      #endif
      // subscribe to command topic on connection
      MQTT.subscribe(MQTT_COMMAND_TOPIC);
      delay(50);
      MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
      delay(50);
      updateStates();
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

  // retry until connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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

    Serial.print(F(" - MAC: "));
    Serial.println(getMacAddress());
    
    Serial.print(F(" - RSSI [db]: "));
    Serial.println(WiFi.RSSI());
  #endif
}

//--------------------------------------------------------------------------------------------------

void setRelay(const unsigned int& relayPin, bool state) {
  //
  // sets the relay states (on/off)
  //
  // parameters:
  //     relayPin       relay pin number
  //     state          true for turn on, false for turn off
  //
  // handles relays with special behaviour
  switch (relayPin) {
    //
    // "buddha" lights relay
    //
    case RELAY_BUDDHA_LIGHT_PIN:      
      if (_lightOn == state)        
        return;        
      _lightOn = state;
      #if (DEBUG_MODE == true)
        Serial.print(F("light: "));
        Serial.println(toStr(state));
      #endif
      // led light is not a relay: 
      // it's activates a npn transistor
      // therefore, it has a inverted logic
      state = !state;
      break;    
    //
    // water pump relay (relay #1)
    //
    case RELAY_WATER_PUMP_PIN:      
      if (_pumpOn == state)        
        return;
      // set state
      _pumpOn = state;
      #if (DEBUG_MODE == true)
        Serial.print(F("relay pump: "));
        Serial.println(toStr(state));
      #endif
      break;
    //
    // relay #2
    //
    case RELAY_EXTRA_PIN:      
      if (_relay2On == state)        
        return;        
      _relay2On = state;
      #if (DEBUG_MODE == true)
        Serial.print(F("relay #2: "));
        Serial.println(toStr(state));
      #endif
      break;
  }

  // relays are active LOW
  digitalWrite(relayPin, (state ? LOW : HIGH));

  // update MQTT states
  updateStates();
}


//--------------------------------------------------------------------------------------------------

void beep(const uint8_t& n = 1, const unsigned long& time = BUZZER_POWER_ON_TIME) {
  //
  // plays a beep sound
  //
  // parameters:
  //     n       number of beeps
  //     time    beep duration (in miliseconds)
  //
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(time);
    digitalWrite(BUZZER_PIN, LOW);
    if (n > 1) {
      delay(time);
    }
  }
}

void playTune(unsigned int song = 0) {
  //
  // Plays the Dart Vader theme (Imperial March) on buzzer
  //
  #if (PLAY_TUNES == false)
    // simple beep
    beep(200, 4);
    delay(1000);
  #else
    //
    // plays some music!
    //  
    // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
    // there are two values per note (pitch and duration), so for each note there are four bytes

    // this calculates the duration of a whole note in ms
    const int wholenote = (60000 * 4) / tempo[song];
    int divider = 0, noteDuration = 0;
       
    // iterate over the notes of the melody.
    // Remember, the array is twice the number of notes (notes + durations)
    for (int currentNote = 0; currentNote < notes[song] * 2; currentNote = currentNote + 2) {
      
      divider = pmelodies[song][currentNote + 1];

      if (divider > 0) {
        // regular note, just proceed
        noteDuration = (wholenote) / divider;
      } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration = (wholenote) / abs(divider);
        noteDuration *= 1.5;  // increases the duration in half for dotted notes
      }

      // we only play the note for 90% of the duration, leaving 10% as a pause
      digitalWrite(BUZZER_PIN, HIGH);
      tone(buzzer, pmelodies[song][currentNote], noteDuration * 0.9);
            
      // Wait for the specief duration before playing the next note.
      delay(noteDuration);

      // stop the waveform generation before the next note.
      noTone(buzzer);
      digitalWrite(BUZZER_PIN, LOW);
    }
  #endif
}

//--------------------------------------------------------------------------------------------------

String getMacAddress() {
  //
  // Gets current MAC address
  //
  byte mac[6];
  WiFi.macAddress(mac);
  
  return String(mac[5], HEX) + String(":") + String(mac[4], HEX) + String(":") + String(mac[3], HEX) + 
    String(":") + String(mac[2], HEX) + String(":") + String(mac[1], HEX) + String(":") + String(mac[0], HEX);
}

String toStr(const bool& value) {
  //
  // Encodes a boolean value as "on"/"off" string
  //
  return String(value? F("on") : F("off"));
}
