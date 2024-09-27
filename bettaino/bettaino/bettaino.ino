//------------------------------------------------------------------------------------------------------------------
//
//
// BETTA.INO - Aquarium Automation with Home Assistant and NodeMCU
//   - Optimized for NodeMCU 1.0 (ESP-12E Module)
//
// light version of Aquario.ino (https://github.com/fortalbrz/aquarioino/)
//
//           /`-._
//          /_,.._`:-
//      ,.-'  ,   `-:..-')
//     : o ):';      _  {
//      `-._ `'__,.-'\`-.)
//          `\\  \,.-'`
//
//------------------------------------------------------------------------------------------------------------------
// source code: https://github.com/fortalbrz/aquarioino/betaino/
//
// Overview:
//
// Home Assistant is free and open-source software used for home automation (https://www.home-assistant.io). It serves as an integration platform and smart home hub,
// allowing users to control smart home devices. This project objective is to control and integrate a small fisk tank with Home Assistant using the MQTT protocol
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
//                                                          [betaino.ino]  
//
// Features:
// - fully integrated with Home Assistant (MQTT)
// - automation of water replenishment by evaporation (optional, see ENABLE_WATER_REPOSITION configuration flag)
// - four relays: feeder, lights, sump pump, water reposition pump, etc
// - feeding 
// - push buttons: 
//    - lightning turn on/off button
//    - feeding button
// - home assistant switches:
//   - feeding button
//   - lightening switch
//   - "sump" and "water reposition" pumps automation switches
// - buzzer for error notifications (and play star wars tunes!)
//
// Water Replenishment (by evaporation loses):
//  this routine uses a water level sensor to refill the fisk tank with clean water (from an external container)
//  when the water level is low.
//
//    |~~~~~~~~~~~~~~~~~~~~~~|        --------------------
//    |[water level sensor]  |       |   water container  |
//    |      fish tank       | <=====|["repo pump"]       |
//     ----------------------         --------------------
//
// Materials:
//   - 1 x NodeMCU (ESP 8266-12e) [25 BRL](https://produto.mercadolivre.com.br/MLB-1211973212-modulo-wifi-esp8266-nodemcu-esp-12e-_JM)
//   - 1 x relay module 5v 4-channels [24 BRL](https://produto.mercadolivre.com.br/MLB-1758894951-modulo-rele-rele-5v-4-canais-para-arduino-pic-raspberry-pi-_JM)
//   - 1 x water level sensor [R$ 14]https://produto.mercadolivre.com.br/MLB-3164716754-sensor-de-nivel-de-agua-interruptor-de-boia-e-mini-boia-c-_JM) (optional, see ENABLE_WATER_REPOSITION configuration flag)
//   - 1 x active buzzer 5v [2.30 BRL](https://www.a2robotics.com.br/buzzer-ativo-5v)
//   - 1 x NPN BC548 transistor [1.20  BRL](https://produto.mercadolivre.com.br/MLB-1712833525-transistor-bc548-npn-para-projetos-10-pecas-_JM )
//   - 2 x tactile push button [0.20 BRL](https://www.a2robotics.com.br/chave-tactil-6x6x5mm-4-terminais)
//   - 3 x resistor 10k ohms (1/8w) 
//   - 1 x resistor 1k ohms (1/8w) 
//   - 1 x power supply 5vdc (1A) [14 BRL](https://produto.mercadolivre.com.br/MLB-3445635491-fonte-alimentaco-5v-1a-bivolt-roteador-wireles-modem-d-link-_JM)
//   - 1 x led and resistor 10k ohms (optional, indicates "power on")
//   - 1 x electrolytic capacitor 100 uF (optional)
//   - 1 x large plastic water container (reused, optional as "water reposition container")
//   - plastic hoses (connect bumps)
//   - flexible cab (22 agw)
//
// Circuit Wiring Instruction:
//   - NodeMCU (GND) --> power supply 5vdc (negative/Gnd)
//   - NodeMCU (Vin) --> power supply 5vdc (positive/Vcc)
//   - Relay 4 ch (VCC) --> power supply 5vdc (negative/Gnd)
//   - Relay 4 ch (GND) --> power supply 5vdc (positive/Vcc)
//   - Relay 4 ch (In 1) --> NodeMCU (D4)
//   - Relay 4 ch (In 2) --> NodeMCU (D5)
//   - Relay 4 ch (In 3) --> NodeMCU (D6)
//   - Relay 4 ch (In 4) --> NodeMCU (D7)
//   - NodeMCU "D1" --> resistor 10k ohms "A" terminal 1 
//   - resistor 10k ohms "A" terminal 2 --> NodeMCU (3.3V)
//   - NodeMCU "D2" --> resistor 10k ohms "B" terminal 1
//   - resistor 10k ohms "B" terminal 2 --> NodeMCU (3.3V)
//   - NodeMCU "D1" --> tactile push button "FEED" terminal 1 (NC)
//   - tactile push button "FEED" terminal 2 (NC) --> -5 V power source (GND)
//   - NodeMCU "D2" --> tactile push button "LIGHT" terminal 1 (NC)
//   - tactile push button "LIGHT" terminal 2 (NC) --> -5 V power source (GND)
//   - NodeMCU "A0" --> water level sensor terminal 1 (optional, see ENABLE_WATER_REPOSITION configuration flag)
//   - water level sensor terminal 2 --> NodeMCU (3.3V)  (optional, see ENABLE_WATER_REPOSITION configuration flag)
//   - NodeMCU "A0" --> resistor 10k ohms "C" terminal 1 (optional, see ENABLE_WATER_REPOSITION configuration flag)
//   - resistor 10k ohms "C" terminal 2 --> NodeMCU "GND" (optional, see ENABLE_WATER_REPOSITION configuration flag)
//   - NodeMCU "D3" --> resistor 1k ohms terminal 1
//   - resistor 1k ohms terminal 2 --> BC548 transistor base (pin 2 - leg at middle)
//   - BC548 transistor collector (pin 1 - left leg) --> buzzer 5v negative terminal (-)
//   - buzzer 5v positive terminal (+) --> +5 V power source (Vcc)
//   - BC548 transistor emitter (pin 3 - right leg) --> -5 V power source (GND)
//   - Led terminal 1 (positive) --> +5 V power source (VCC) (optional, "power on led")
//   - Led terminal 2 (negative/bevel) --> resistor 10k ohms "D" terminal 1 (optional, "power on led")
//   - resistor 10k ohms "D" terminal 2 --> -5 V power source (GND) (optional, "power on led")
//   - capacitor 100uF (positive) --> +5 V power source (VCC) (optional)
//   - capacitor 100uF (negative/"minus sign") --> resistor 10k ohms "D" terminal 2 (optional)
//
//   Snubber Filter:
//
//   The suppressor filter (aka "snubber"), is a device that serves to limit voltage spikes. On this project
//   is recommend the snubber filter in parallel with the pumps (eletric motors) outlets.
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
//
// Error Codes (buzzer)
//   - 2 fast bips: Wifi error
//   - 3 fast bips: MQTT broker error
//   - 5 bips: low water level
//
// Wiring Testing:
//  sets the macro "WIRING_TEST_MODE" as true in order to check buttons, relays and water sensor connections (testing only)
//
// Serial Monitor:
//  sets the macro "DEBUG_MODE" as true in order to debug on serial monitor (testing only) - ensure the baud rate setup!
//
// Configuration flags:
//   - WIFI_SSID: Wi-fi SSID
//   - WIFI_PASSWORD: Wi-fi password
//   - MQTT_BROKER_ADDRESS MQTT: broker server ip address
//   - MQTT_BROKER_PORT: MQTT broker port (default: 1883)
//   - MQTT_USERNAME: mqtt broker username
//   - MQTT_PASSWORD: mqtt broker password
//   - MQTT_DEVICE_ID: MQTT session identifier (changes for more then one gardeino on the same MQTT broker)
//   - ENABLE_WATER_REPOSITION: enables/disables water level sensors (disable it to not use the water level sensors)
//   - DEBUG_MODE: enables/disables serial monitor debugging messages
//   - WIRING_TEST_MODE: enables/disables a wiring test mode
//   - PLAY_TUNE: enables play music theme
//
//  MQTT topics:
//    - bettaino/available: sensors availability ["online"/"offline"]
//    - bettaino/cmd: pushes commands to NodeMCU [home assistant -> bettaino]:
//         "feed": feeding routine (simulate a use button press on relay #1 - connect to feeder button)
//         "light on/off": turns on/off the lights (i.e. relay #2)
//         "sump enable/disable": enables/disables sump pump automation routine
//         "repo enable/disable": enables/disables water reposition pump automation routine
//         "alarm on/off": turns on/off a low water level alarm sound
//         "beep": plays a beep sound [for any notification]
//         "play [sw/dv/tetris/mario/got/gf/nokia/notice]": plays an tune by name [for any notification] (none for random)
//         "emergency on/off": emergency alarm [for any notification]
//         "sensor on/off": enables/disables the water level sensor to block the sump pump [debug only]
//         "relays on/off": turn on/off all relays [debug only]
//         "refresh": update MQTT state [debug only]
//    - bettaino/state: retrieves NodeMCU states as json [bettaino -> home assistant]
//         {
//            "light": "on",          // relay #2 state (lights): [on/off]
//            "sump": "on",           // relay #3 state (sump pump): [on/off]
//            "repo": "off",          // relay #4 state (water reposition pump): [on/off]
//            "sump_enabled": "on",   // sump pump automation routine enabled: [on/off]
//            "repo_enabled": "off",  // water reposition pump automation routine enabled: [on/off]
//            "alarm": "on",          // play a alarm sound on low water level: [on/off]
//            "sensor": "on",         // water level sensor enabled to block the sump pump: [on/off]
//            "water_low": "off"      // low water level: [on/off]
//         } 
//------------------------------------------------------------------------------------------------------------------
//
// Jorge Albuquerque (2024) - https://linkedin.com/in/jorgealbuquerque
// https://www.jorgealbuquerque.com
//
#define DEBUG_MODE false                            // enables/disables serial debugging messages
#define WIRING_TEST_MODE false                      // enables/disables testing mode
#define ENABLE_WATER_REPOSITION true                // true for enable water reposition (reposition pump), false otherwise
#define USE_INTERRUPTIONS false                     // use interruptions on push buttons
#define PLAY_TUNE true                              // enables play music
//------------------------------------------------------------------------------------------------------------------
//
// Configuration flags (enables or disables features in order to "skip" unwanted hardware)
//
//------------------------------------------------------------------------------------------------------------------
// Wi-fi setup
#define WIFI_SSID "jorge cps"                       // Wi-fi SSID
#define WIFI_PASSWORD "casa1976bonita1980"          // Wi-fi password
// MQTT setup
#define MQTT_BROKER_ADDRESS "192.168.68.10"         // MQTT broker server IP
#define MQTT_BROKER_PORT 1883                       // MQTT broker port
#define MQTT_USERNAME "mqtt-user"                   // can be omitted if not needed
#define MQTT_PASSWORD "mqtt"                        // can be omitted if not needed
// MQTT topics
#define MQTT_COMMAND_TOPIC "bettaino/cmd"             // MQTT topic for send door commands (e.g., open from door)
#define MQTT_STATUS_TOPIC "bettaino/status"           // MQTT topic for doorbell status
#define MQTT_AVAILABILITY_TOPIC "bettaino/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_DEVICE_ID "bettaino_12fmo43iowerwe2"     // MQTT session identifier
// others
#define MQTT_STATUS_UPDATE_TIME 120000               // time for send and status update (default: 2 min)
#define MQTT_AVAILABILITY_TIME 60000                 // elapsed time to send MQTT availability, in miliseconds (default: 1 min)
#define BUTTON_SINGLE_PUSH_TIME 300                  // time to avoid double push button press (better usability)
#define BUZZER_POWER_ON_TIME 800                     // default buzzen power on time (beep)
#define WATER_SENSOR_LOW_COUNTER 15                   // counter for smooth transition to "low water state": avoids false positives
#define WATER_SENSOR_HIGH_COUNTER 25                  // counter for smooth transition to "water state refiled": avoids false negatives
#define SERIAL_BAUDRATE 9600                         // serial monitor baud rate (only for debuging)
#define EEPROM_ADDRESS 0
#define RELAY_SIZE 4
//
// pins definitions (ModeMCU)
//
#define WATER_LOW_LEVEL_SENSOR_PIN A0                // A0: water level sensor (should be located at desired aquarium level: open when water is low)
#define PUSH_BUTTON_FEEDING_PIN D1                   // D1: pull-up (high) - tactile push button "feed"
#define PUSH_BUTTON_LIGHT_PIN D2                     // D2: pull-up (high) - tactile push button "lights"
#define BUZZER_PIN D3                                // D3: pull-up (high) (remark: boot fails on low) - buzzer
#define RELAY_FEEDER_PIN D4                          // D4: pull-up (high) - relay #1: feeder push button [connected to build-in LED]
#define RELAY_LIGHT_PIN D5                           // D5: pull-up (high) - relay #2: light (normally open)
#define RELAY_SUMP_PUMP_PIN D6                       // D6: pull-up (high) - relay #3: sump pump (normally open, common on 110vac)
#define RELAY_WATER_REPOSITION_PUMP_PIN D7           // D7: pull-up (high) - relay #4: water reposition pump (normally open, common on 110vac)
//
// protocol commands
//
#define MQTT_COMMAND_FEED "feed"
#define MQTT_COMMAND_LIGHT_ON "light on"
#define MQTT_COMMAND_LIGHT_OFF "light off"
#define MQTT_COMMAND_SUMP_PUMP_ON "sump enable"
#define MQTT_COMMAND_SUMP_PUMP_OFF "sump disable"
#define MQTT_COMMAND_REPO_PUMP_ON "repo enable"
#define MQTT_COMMAND_REPO_PUMP_OFF "repo disable"
#define MQTT_COMMAND_DEBUG_SENSOR_ON "sensor on"
#define MQTT_COMMAND_DEBUG_SENSOR_OFF "sensor off"
#define MQTT_COMMAND_DEBUG_RELAYS_ON "relays on"
#define MQTT_COMMAND_DEBUG_RELAYS_OFF "relays off"
#define MQTT_COMMAND_DEBUG_REFRESH "refresh"
#define MQTT_COMMAND_DEBUG_BEEP "beep"
#define MQTT_COMMAND_DEBUG_PLAY "play"
#define MQTT_COMMAND_DEBUG_PLAY_NOTICE "play notice"
#define MQTT_COMMAND_DEBUG_PLAY_STAR_WARS "play sw"
#define MQTT_COMMAND_DEBUG_PLAY_DARTH_VADER "play dv"
#define MQTT_COMMAND_DEBUG_PLAY_TETRIS "play tetris"
#define MQTT_COMMAND_DEBUG_PLAY_MARIO "play mario"
#define MQTT_COMMAND_DEBUG_PLAY_GOT "play got"
#define MQTT_COMMAND_DEBUG_PLAY_GODFATHER "play gf"
#define MQTT_COMMAND_DEBUG_PLAY_NOKIA "play nokia"
#define MQTT_COMMAND_DEBUG_LOW_LEVEL_ALARM_ON "alarm on"
#define MQTT_COMMAND_DEBUG_LOW_LEVEL_ALARM_OFF "alarm off"
#define MQTT_COMMAND_DEBUG_EMERGENCY_ALARM_ON "emergency on"
#define MQTT_COMMAND_DEBUG_EMERGENCY_ALARM_OFF "emergency off"


//------------------------------------------------------------------------------------------------------------------

// librarys (see doc above)
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

WiFiClient espClient;
PubSubClient MQTT(espClient);

//
// internal states (globals)
//
bool _lightOn = false;
bool _sumpOn = false;
bool _repoOn = false;
bool _sensorEnabled = true;
bool _sumpEnabled = true;
bool _repoEnabled = true;
bool _lowWaterLevel = false;
bool _alarmEnabled = true;
byte _state = 0x00; 

unsigned int _counter;
unsigned long _lastAvailabilityTime = 0;
unsigned long _lastStatusUpdateTime = 0;
unsigned long _lastLowWaterLevelWarningTime = 0;
unsigned long _lastButtonPress = 0;


byte RELAY_PINS[] = {RELAY_FEEDER_PIN, RELAY_LIGHT_PIN, RELAY_SUMP_PUMP_PIN, RELAY_WATER_REPOSITION_PUMP_PIN};

#if (PLAY_TUNE == true)
  /* ----------------------------------------------------------------------
    Star Wars theme  
    Connect a piezo buzzer or speaker to pin 11 or select a new pin.
    More songs available at https://github.com/robsoncouto/arduino-songs                                                                                          
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

  // nokia ringtone 
  int melody_nokia[] = {
    NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
    NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
    NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
    NOTE_A4, 2, 
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
    melody_nokia
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
    180
  };

  int notes[] = 
  {
    sizeof(melody_star_wars) / sizeof(melody_star_wars[0]) / 2,
    sizeof(melody_darth_vader) / sizeof(melody_darth_vader[0]) / 2,
    sizeof(melody_tetris) / sizeof(melody_tetris[0]) / 2,
    sizeof(melody_mario) / sizeof(melody_mario[0]) / 2,
    sizeof(melody_game_of_thrones) / sizeof(melody_game_of_thrones[0]) / 2,
    sizeof(melody_godfather) / sizeof(melody_godfather[0]) / 2,
    sizeof(melody_nokia) / sizeof(melody_nokia[0]) / 2
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
void alarm();
void playTune(unsigned int song);
void setRelay(const unsigned int& relayPin, bool state);
void loadConfig();
void saveConfig();
void beep(const uint8_t& n, const unsigned long& time);
void wiringTest();
String toStr(const char* label, const bool& state, const bool& end);


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

  // relays
  for(unsigned int i = 0; i < RELAY_SIZE; i++) {    
    pinMode(RELAY_PINS[i], OUTPUT);
    // relays off (security)
    digitalWrite(RELAY_PINS[i], HIGH);
  }  

  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  // push buttons
  pinMode(PUSH_BUTTON_FEEDING_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_LIGHT_PIN, INPUT_PULLUP);
  #if (USE_INTERRUPTIONS == true)
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_FEEDING_PIN), onFeedPushButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_LIGHT_PIN), onLightPushButton, FALLING);
  #endif

  // starting beep
  beep(1, 100);

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
  #if (WIRING_TEST_MODE == false)
    MQTT.setCallback(onMessage);
    
    loadConfig();
    delay(250);
    updateStates();
  #endif

  setRelay(RELAY_SUMP_PUMP_PIN, true);

  #if (DEBUG_MODE == true)
    Serial.println(F("Setup finished"));  
  #endif
}


void loop() {
  //
  // main loop
  //
  #if (WIRING_TEST_MODE == true)
    // wiring test routine
    wiringTest();
    delay(2000);
    return;
  #endif

  //
  // keeps MQTT connection alive
  //
  if (!MQTT.connected())
    connectMQTT();

  // sends MQTT "availability" message
  if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
    MQTT.publish(MQTT_AVAILABILITY_TOPIC, "online");
    _lastAvailabilityTime = millis();
  }

  bool sumpOn = true;
  if (_sensorEnabled) {    
    // checks sump tank water level
    bool lowWaterLevel = analogRead(WATER_LOW_LEVEL_SENSOR_PIN) == 0;    
    
    //
    // state machine (finite automata):
    //  water level ok        wait some    low water level       wait some 
    //       (0) -[water low]-> (1) -----------> (2) -[water ok]-> (3)
    //         <---------------------------------------------------- 
    //    
    switch (_state) {
      case 0x00:
        // state: water level ok (sump on)
        sumpOn = true;
        if (lowWaterLevel){
          // got to waiting state
          _state = 0x01;
          _counter = 0;
        }
        break;
      case 0x01:
        // state: waiting to confirm low water level!
        sumpOn = true;
        if (!lowWaterLevel)
          // false alarm
          _state = 0x00;
        _counter++;
        if (_counter > WATER_SENSOR_LOW_COUNTER) {
          // confirmed: go to low water level!
          _state = 0x02;
        }        
        break;
      case 0x02:
        // state: low water level (sump off)
        sumpOn = false;
        if (!lowWaterLevel){
          // got to waiting state
          _state = 0x03;
          _counter = 0;
        }
        break;
      case 0x03:
        // state: waiting to confirm water level is ok again!
        sumpOn = false;
        if (lowWaterLevel)
          // false alarm
          _state = 0x02;
        _counter++;
        if (_counter > WATER_SENSOR_HIGH_COUNTER) {
          // confirmed: go to water level ok!
          _state = 0x00;
        }        
        break;
    }
  }
  else
  {
    // default
    _state = 0x00;
  }

  // stops sump pump if the water level is low (protects the sump pump)  
  setRelay(RELAY_SUMP_PUMP_PIN, sumpOn);
  
  // refills the sump tank with fresh water (if enabled)
  #if (ENABLE_WATER_REPOSITION == true)
    if (_repoEnabled) {
      // water reposition pump is turn on on low water level (confirmed)
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, !sumpOn);
    } else {
      // disable water reposition pump and play an alarm
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
      alarm();
    }
  #else
    // disable water reposition pump and play an alarm
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
    alarm();
  #endif
  
  // update states (mqtt), if required 
  // ("_lowWaterLevel" flag is expected to have value "!sumpOn")
  if (_lowWaterLevel == sumpOn) {
      _lowWaterLevel = !sumpOn;
      updateStates();
  }
  

  MQTT.loop();

  #if (USE_INTERRUPTIONS == false)
    if (digitalRead(PUSH_BUTTON_LIGHT_PIN) == LOW)
      onLightPushButton();
    else if (digitalRead(PUSH_BUTTON_FEEDING_PIN) == LOW)
      onFeedPushButton();
  #endif

  if ((millis() - _lastStatusUpdateTime) > MQTT_STATUS_UPDATE_TIME) {
    // status update (with max timespan for 5 minutes)
    updateStates();    
  }

  // keeps wifi connected
  connectWiFi();

  delay(200);
}



//--------------------------------------------------------------------------------------------------
//
// push buttons
//
//--------------------------------------------------------------------------------------------------

void onLightPushButton() {
  //
  // lightening button handler: invertd light state
  //
  if ((millis() - _lastButtonPress) > BUTTON_SINGLE_PUSH_TIME) {    
    _lastButtonPress = millis();
    // inverts light state (on/off)
    setRelay(RELAY_LIGHT_PIN, !_lightOn);    
    updateStates();
  }
}

void onFeedPushButton() {
  //
  // feeding button handler: feed!
  //
  if ((millis() - _lastButtonPress) > BUTTON_SINGLE_PUSH_TIME) {
    _lastButtonPress = millis();

    // simulates a push button press
    setRelay(RELAY_FEEDER_PIN, true);
    delay(BUTTON_SINGLE_PUSH_TIME);
    setRelay(RELAY_FEEDER_PIN, false);    

    updateStates();
    playTune(0);
  }
}

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

  // decode Home Assistant command (MQTT)
  if (strcmp(msg, MQTT_COMMAND_FEED) == 0) {
    // feed the fishes
    onFeedPushButton();

  } else if (strcmp(msg, MQTT_COMMAND_LIGHT_ON) == 0) {
    // turn on the light (relay #2)
    setRelay(RELAY_LIGHT_PIN, true);
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_LIGHT_OFF) == 0) {
    // turn off the light (relay #2)
    setRelay(RELAY_LIGHT_PIN, false);
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_SUMP_PUMP_ON) == 0) {
    // enables sump pump
    _sumpEnabled = true;
    saveConfig();
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_SUMP_PUMP_OFF) == 0) {
    // disables sump pump
    _sumpEnabled = false;
    saveConfig();
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_REPO_PUMP_ON) == 0) {
    // enables repo pump
    _repoEnabled = true;
    saveConfig();
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_REPO_PUMP_OFF) == 0) {
    // disables repo pump
    _repoEnabled = false;
    saveConfig();
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_BEEP) == 0) {
    // plays a "beep" sound
    beep(3, 100);
  
  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY) == 0) {
    // plays random tune
    int song = random(0, 6);
    playTune(song);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_NOTICE) == 0) {
    // plays random tune
    int notice = random(1, 5);
    playTune(notice);

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

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_PLAY_NOKIA) == 0) {
    // plays the super mario bros theme
    playTune(6);

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_REFRESH) == 0) {
    // updated mqtt states (debug)
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_SENSOR_ON) == 0) {
    // enables water level sensor
    _sensorEnabled = true;
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_SENSOR_OFF) == 0) {
    // disables water level sensor
    _sensorEnabled = false;
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_RELAYS_ON) == 0) {
    // turns on all relays (debug), and set sensor off (keeps relays on)
    _sensorEnabled = false;
    for(unsigned int i = 0; i < RELAY_SIZE; i++)     
      setRelay(RELAY_PINS[i], true);
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_RELAYS_OFF) == 0) {
    // turs off all relays (debug), and set sensor off (keeps relays off)
    _sensorEnabled = false;
    for(unsigned int i = 0; i < RELAY_SIZE; i++)     
      setRelay(RELAY_PINS[i], false);  
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_LOW_LEVEL_ALARM_ON) == 0) {
    // enables low water level alarm
    _alarmEnabled = true;
    saveConfig();
    updateStates();

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_LOW_LEVEL_ALARM_OFF) == 0) {
    // disables low water level alarm
    _alarmEnabled = false;
    saveConfig();
    updateStates();       
  
  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_EMERGENCY_ALARM_ON) == 0) {
    // play loud sound (non stop)
    digitalWrite(BUZZER_PIN, HIGH);   

  } else if (strcmp(msg, MQTT_COMMAND_DEBUG_EMERGENCY_ALARM_OFF) == 0) {
    // turn buzzer off
    digitalWrite(BUZZER_PIN, LOW);    
  }
}


void updateStates() {
  //
  // MQTT publish states update
  //
  if (MQTT.connected()) {

    String json = String(F("{"));
    json += toStr("light", _lightOn, false);
    json += toStr("sump", _sumpOn, false);
    json += toStr("repo", _repoOn, false);
    json += toStr("sump_enabled", _sumpEnabled, false);
    json += toStr("repo_enabled", _repoEnabled, false);
    json += toStr("alarm", _alarmEnabled, false);
    json += toStr("sensor", _sensorEnabled, false);    
    json += toStr("water_low", _lowWaterLevel, true);

    unsigned int n = json.length() + 1;
    char message[n];
    json.toCharArray(message, n);

    #if (DEBUG_MODE == true)
        Serial.print(F("mqtt update: "));
        Serial.println(message);
    #endif

    MQTT.publish(MQTT_STATUS_TOPIC, message);

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
      delay(50);
    } else {
      #if (DEBUG_MODE == true)
            Serial.println(F("Fail connecting to MQTT broker (retry in 2 secs)."));
      #endif
      beep(3, 100);
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
    beep(2, 100);
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

//--------------------------------------------------------------------------------------------------

void setRelay(const unsigned int& relayPin, bool state) {
  //
  // sets the relay states (on/off)
  //
  // parameters:
  //     relayPin       relay pin number
  //     state          true for turn on, false for turn off
  //
  delay(50);

  // handles relays with special behaviour
  switch (relayPin) {
    //
    // lights relay
    //
    case RELAY_LIGHT_PIN:      
      if (_lightOn == state)
        // current state: do nothing!
        return;
        // set state
      _lightOn = state;
      break;
    //
    // sump pump relay
    //
    case RELAY_SUMP_PUMP_PIN:      
      if (!_sumpEnabled)
        // turns off sump pump, if disabled      
        state = false;      
      if (_sumpOn == state)
        // current state: do nothing!
        return;
      // set state
      _sumpOn = state;
      break;
    //
    // water reposition pump relay
    //
    case RELAY_WATER_REPOSITION_PUMP_PIN:
      if (!_repoEnabled) {
        // turns off sump pump, if disabled      
        state = false;
      }      
      if (_repoOn == state)
        // current state: do nothing!
        return;
      // set state
      _repoOn = state;
      break;
  }

  // relays are active LOW
  digitalWrite(relayPin, (state ? LOW : HIGH));
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

void alarm() {
  //
  // alarm sound (i.e. low water level alarm)
  //
  if (_alarmEnabled && (millis() - _lastLowWaterLevelWarningTime) > 5000) {
    _lastLowWaterLevelWarningTime = millis();
    beep(5, 100);  
  }
}

void playTune(unsigned int song = 0) {
//
// Plays the Dart Vader theme (Imperial March) on buzzer
//
  #if (PLAY_TUNE == false)
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

void loadConfig() {
  //
  // loads configurations from EEPROM
  //
  byte states = EEPROM.read(EEPROM_ADDRESS);
  // "crc": expected 101
  bool crc = (bitRead(states, 0) == 1 && bitRead(states, 1) == 0 && bitRead(states, 2) == 1);
  if (crc) {
    _sumpEnabled = (bitRead(states, 3) == 1);
    _repoEnabled = (bitRead(states, 4) == 1);
    _alarmEnabled = (bitRead(states, 5) == 1);
  } else {
    // previous data not found: set default values
    _sumpEnabled = true;
    _repoEnabled = true;
    _alarmEnabled = true;
    saveConfig();
    delay(50);
  }

  #if (DEBUG_MODE == true)
    Serial.println(F(" - configuration loaded: "));
    Serial.println(toStr("sump", _sumpEnabled, false));
    Serial.println(toStr("repo", _repoEnabled, false));
    Serial.println(toStr("alarm", _alarmEnabled, false));
  #endif
}

void saveConfig() {
  //
  // saves configurations to EEPROM
  //
  byte states = 0x00;
  // "crc": expected 101
  bitSet(states, 0);
  bitSet(states, 2);
  // enables sump pump automation
  if (_sumpEnabled)
    bitSet(states, 3);
  // enables repo pump automation
  if (_repoEnabled)
    bitSet(states, 4);
  if (_alarmEnabled)
    bitSet(states, 5);

  // emulates EEPROM.update() 
  byte previous_states = EEPROM.read(EEPROM_ADDRESS);
  if (previous_states != states)
    EEPROM.write(EEPROM_ADDRESS, states);  

  #if (DEBUG_MODE == true)
    Serial.print(F(" - configuration saved: "));
    Serial.println(states, BIN);
  #endif
}

//--------------------------------------------------------------------------------------------------

void wiringTest() {
//
// Testing routine
//
  #if (TESTING_MODE == true)

    bool testing = true;
    while (testing) {
      Serial.println(F("TESTING MODE"));
      Serial.println(F("1) Water Level Sensor"));
      Serial.println(F("2) Relay 1 - Feeder"));
      Serial.println(F("3) Relay 2 - Light"));
      Serial.println(F("4) Relay 3 - Sump Pump"));
      Serial.println(F("5) Relay 4 - Water Repo Pump"));
      Serial.println(F("6) Buzzer"));
      Serial.println(F("7) Exit"));
      Serial.println(F("Select one option [1-7]:"));
      Serial.println();
      while (Serial.available() < 2) {
        delay(300);        
      }

      // can be 0 if read error
      int input = Serial.parseInt();

      Serial.print(F("option: "));
      Serial.println(input);

      switch (input) {
        case 1:
          // water level sensor testing
          while (Serial.available() > 0) {
            Serial.read();
          }
          while (Serial.available() == 0) {
            if (digitalRead(WATER_LOW_LEVEL_SENSOR_PIN))
              Serial.println(F("Water Level: ok"));
            else
              Serial.println(F("Water Level: LOW"));
            Serial.println(F("press any key to exit..."));
            delay(1000);            
          }
          Serial.read();
          break;

        case 2:
          // relay 1 testing
          onFeedPushButton();
          break;

        case 3:
          // relay 2 testing
          onLightPushButton();
          break;

        case 4:
          // relay 3 testing
          _sumpEnabled = true;
          setRelay(RELAY_SUMP_PUMP_PIN, true);
          delay(1000);
          setRelay(RELAY_SUMP_PUMP_PIN, false);
          delay(1000);
          break;
        case 5:
          // relay 4 testing
          _repoEnabled = true;
          setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, true);
          delay(1000);
          setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
          delay(1000);
          break;
        case 6:
          // buzzer
          beep(100, 5);
          break;
        case 7:
          // Exit
          testing = false;
          return;
        default:
          Serial.println(F("Invalid option: [1-7]."));
          break;
      }
     
    }
  #endif
}

//--------------------------------------------------------------------------------------------------

String toStr(const char* label, const bool& state, const bool& end = false) {
  //
  // writes a string line with the format:
  //     "label": "on/off",
  //
  String text = end ? String(F("\"[LABEL]\": \"[VALUE]\" }")) : String(F("\"[LABEL]\": \"[VALUE]\", "));
  text.replace(F("[LABEL]"), label);
  text.replace(F("[VALUE]"), (state ? F("on") : F("off")));
  return text;
}

//--------------------------------------------------------------------------------------------------