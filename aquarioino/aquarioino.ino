//------------------------------------------------------------------------------------------------------------------
//
//
// AQUARIO.INO - Aquarium Automation with Home Assistant
//
//                                 _.'.__
//                              _.'      .
//        ':'.               .''   __ __  .
//          '.:._          ./  _ ''     "-'.__
//        .'''-: """-._    | .                "-"._
//         '.     .    "._.'                       "
//            '.   "-.___ .        .'          .  :o'.
//              |   .----  .      .           .'     (
//               '|  ----. '   ,.._                _-'
//                .' .---  |.""  .-:;.. _____.----'
//                |   .-""""    |      '
//              .'  _'         .'    _'
//             |_.-'            '-.'
//
//------------------------------------------------------------------------------------------------------------------
// source code: https://github.com/fortalbrz/aquarioino
// see readme: https://github.com/fortalbrz/aquarioino#readme
//
// making off: https://youtu.be/3CjU-o5LTEI (version 1.0)
// overview: https://youtu.be/8RszBkeuUlk (version 1.0)
//
// optimed fpr Arduino Mega 2560 pro mini (CI CH340G)
// - 70 digital I/O, 16 analog inputs, 14 PWM, 4 UART
//
// Change log:
//   - version 2.0: IoT 
//     - home assistant integration (MQTT)
//     - 200 programmable timers
//     - refactored code for enabling/disabling most of the features (adaptation for use needs)
//     - stepper motor feeder
//   - version 1.0: standalone controller 
//     - water temperature control (cooler/heater)
//     - pH measurements
//     - relays timers (lightning, UV filter)
//     - automated partial water change (PWC) routine     
//     - automated water replenishment (due evaporation losses)
//     - feeding control (timers)
//     - water levels, water temperature and pH alarms
//     - weather station (air temperature, humidity and pressure sensors, local / standalone forecasting)
//     - LCD display
//
// Features:
// - fully integrated with home assistant (MQTT) or standalone (optional - see configuration flags)
// - automation of the partial water change (PWC / TPA) routine (optional - see configuration flags)
// - automation of water replenishment by evaporation (optional - see configuration flags)
// - lightning and UV filter turn on/off timers (optional - see configuration flags)
// - water temperature control (heating/cooler fan) (optional - see configuration flags)
// - feeding timer (optional - see configuration flags)
// - pH measurement (optional - see configuration flags)
// - up to 200 standalone timers (remotely programmable with MQTT):
//   - lightning, UV filter, heater, cooler, pumps (i.e., all relays) turn on/off timers  
//   - feeding timers
// - alarms (optional - see configuration flags):
//   - temperature (max/min)
//   - pH (max/min)
//   - water levels (aquarium low water level, sump pump low water level)
// - home assistant sensors:
//   - water temperature
//   - water pH
//   - water levels
//   - air temperature
//   - air humidity
//   - air pressure
//   - weather station forecasting
// - home assistant switches:
//   - lightning switch
//   - feeding button
//   - UV filter switch
//   - heater switch
//   - cooler fan switch
//   - sump, "drain" and "water reposition" pumps switches
//   - enables/disables timers and feeding switches
//   - start partial water change (PWC) routine button 
//   - turn off / restart / "manual cleaning" / save configurations buttons
// - weather station (standalone) (optional - see configuration flags)
// - displays information on LCD 16 x 2 (optional - see configuration flags)
// - push buttons: (optional - see configuration flags) 
//  - lightning turn on/off button
//  - partial water change (PWC) routine button
//  - manual cleaning button
//  - turn off button
//  - restart button
// - protects sump pump of running out of water
// - local time synchronization with home assistant (optional - see configuration flags)
//
// Materials:
// - 1 arduino mega 2560 pro (mini) - https://produto.mercadolivre.com.br/MLB-2012119523-arduino-mega-pro-mini-lacrado-_JM
// - 1 DS18B20 temperature sensor (waterproof) - https://produto.mercadolivre.com.br/MLB-1659212606-sensor-de-temperatura-ds18b20-prova-dagua-arduino-_JM
// - 1 DHT11 temperature and humidity sensor- https://produto.mercadolivre.com.br/MLB-688214170-sensor-de-umidade-e-temperatura-dht11-com-pci-pic-arduino-_JM
// - 1 BMP180 barometer - https://produto.mercadolivre.com.br/MLB-1335729819-barmetro-bmp180-sensor-de-presso-e-temperatura-arduino-_JM
// - 1 pH sensor PH4502C - https://produto.mercadolivre.com.br/MLB-1894057619-modulo-sensor-de-ph-ph4502c-com-eletrodo-sonda-bnc-arduino-_JM
// - 1 relay module with 8 channels (5 V) - https://produto.mercadolivre.com.br/MLB-1758954385-modulo-rele-rele-5v-8-canais-para-arduino-pic-raspberry-pi- _JM
// - 2 float water level sensors and 2 x 220 olhm resistors - https://produto.mercadolivre.com.br/MLB-1540418150-sensor-de-nivel-de-agua-boia-para-arduino-esp8266-esp32-_JM
// - 1 stepper motor with driver (optinal stepper motor feeder)
// - 1 power source 5v 1A (any voltage between 6 V and 9 V)
// - 1 trimpot 10k
// - 6 tactile push buttom keys and 6 x 1k resistors - https://produto.mercadolivre.com.br/MLB-1858468268-kit-10x-chave-tactil-push-button-6x6x5mm-arduino-eletrnica-_JM
// - 6 female recessed sockets - https://produto.mercadolivre.com.br/MLB-1844503844-6-tomada-embutir-fmea-preta-3-pinos-10a-painel-aparelho-_JM
// - 1 led and resistor 10k olhm (optional: indicates "power on")
// - 1 power source 12v, 1 fan 12v (cooler), 1 water pump 12v (reposition pump)
// - plastic hoses (connect bumps)
// - flexible cab (22 agw)
// - 1 box (reused, wooden)
// - 1 large plastic water container (reused, optional as "water reposition container")
// 
// remark: assuming that sump tank, sump pump, sump water buoy, etc are already in place...
//
// Circuit Wiring Instruction (step by step):
// - see readme: https://github.com/fortalbrz/aquarioino#readme
// - see https://www.circuito.io/static/reply/index.html?solutionId=65010bbd91d445002e8974a5&solutionPath=storage.circuito.io
//  
// Flashing the code
//   - the Arduino Mega Pro Mini may use the USB/serial IC CH340G, so it's necessary to install the Windows driver: 
//       - CH340G driver: http://bit.ly/44WdzVF (windows 11 compatible)
//       - driver installation instructions (pt-BR): http://bit.ly/3ZqIqc0
//   - Download Arduino IDE (ver 2.0): https://www.arduino.cc/en/software
//   - uses a micro USB cable to connect to the Arduino and select the "Arduino Mega or Mega 2560" board in the IDE ("Tools" -> "Board").
//   - in the library manager, select and install:
//       - "OneWire" by Jim Studt, Tom Pollard, Robin James, etc
//       - "LiquidCrystal" by Arduino, Adafuit
//       - "DallasTemperature" by Milles Burtton, etc
//       - "Adafruit BMP085 Library" by Adafruit
//       - "DHT sensor library" by Adafruit
//       - "RTClib" by Adafruit
//       - "home-assistant-integration" by David Chyrzynsky
//
// Assembly 
//   The water levels sensors are used for:
//   - protects the sump pump (preventing it from running without water)
//   - automation of water replenishment due evaporation losses
//   - automation of the partial water change (PWC) routine
//   The water levels sensors are optional, in the sense that their use can be disabled using the configuration flags. 
//   However, the features above will be suppressed. Tthe sensors are expected to be mounted as *closed* for full water level and *opened* for low water level. 
//
//   The figure bellow, shows the main solution components with focus on water pumps and water level sensors:
//
//                                [water input]
//   |                       |     |                                                |     ------------------------|   |
//   | -- [water level 1]    |     |      |                                         |     |                           |
//   |                       |     |      |         |                               |     |          water            |
//   |        [fish]         |     |      |         |                               |     |        reposition         |
//   |                       |     |      | filters | -- [water level 2]            |     |        reservatory        |
//   |                       |     |      |   ...   |                               |     |                           |
//   | [drain pump]=>waste   |     |      |         |        [sump pump]=> aquarium |     | ["water reposition" pump] |=> aquarium
//   -------------------------     --------------------------------------------------     -----------------------------    
//       T A N K / Aquarium                            S U M P                                adds water to aquarium
//
//  [water level 1] - "drain pump" water level sensor: detects that the aquarium water level is low 
//                    (activates the "water reposition" pump)
//  [water level 2] - "sump pump" water level sensor: protects the sump pump
//  [sump pump]     - pushes water from sump back to aquarium (as usual)
//  [drain pump]    - (cheap/low flow pump) removes dirt water from aquarium into waste/sink 
//                    (used on first step of the automated partial water change / PWC routine)
//  ["water reposition" pump] - (cheap/low flow pump) pushes water from an external water container
//                              (with conditioned water) into aquarium (compensates evaporation losses
//                              and second step of automated partial water change / PWC routine)
//
//
// - "drain" pump and "drain" water level sensor are mounted at aquarium/fisk tank as show (with the "drain" water level sensors at maximum aquarium level)
// - "sump" pump and "sump" water level sensor are mounted at sump tank as show (with the "sump" water level sensors at minimum level for sump pump safety)
// - "water reposition" pump is mounted a external water container ("water reposition tank)
//
// The "drain" water level sensor and "water reposition" pump work together in the automation of water replenishment due evaporation losses (optional, see config flags). 
// Whenever the water level drops (at water level sensor), the water reposition pump pushes more water into the fisk tank (keeping it stable).
// This feature is optional and can be disabled using the[configuration flags. 
// In that case, "water reposition container", "water reposition pump" and the water level sensor can be dismissed. 
// Notice that the flow performance demand over the "water reposition pump" is very low, and a really cheap / low flow pump (~ 3w) is strongly recommended.  
//
// As expected, the "sump" pump pushed water from sump back into fisk tank. The "sump" water sensor level 
// turn of the sump pump case water level is to low (preventing it from running without water).
//
// More over, "sump" pump, "sumo" water level sensor, "drain" pump and "drain" water level sensor 
// work together in the automation of the partial water change (PWC) routine (optional, see config flags). 
// Using a ingenious teamwork, the solution perform 2 main steps:
//   - step 1: removes dirt water from aquarium / fisk tank into waste/sink
//   - step *: pushes water from the external water container (with conditioned water) into aquarium / fisk tank, until the original water level.
//
// This feature is also optional and can be disabled using the configuration flags. 
// In that case, "water reposition container", "water reposition pump", "drain" pump and "drain" water level sensor can be dismissed. 
// Notice that the flow performance demand over the "drain pump" is also very low (a slower PWC is better for the fishes), and a really cheap / low flow pump (~ 3w) is also strongly recommended.  
//
// Furthermore, the code was designed to use a minimum amount of water level sensors. However, you can improve the setup adding independent water level sensors for maximum aquarium level and PWC stop level.
// You can also include a water level sensor at external water container ("water reposition tank") in order to alarm and/or protect the "water reposition" pump.
//
// Feeder
//
// Using feeder feature (ENABLE_FEEDING true) requires an fish food container that can dispense it to the fisk tank. There are two possible options:
//
// - integrates an external feeder (e.g., Boyu feeder) (FEEDING_AS_PUSH_BUTTON true)
// - making a feeder from scratch using a stepper motor (FEEDING_AS_PUSH_BUTTON false)
//
// Integrating external feeder
//
// This configuration (FEEDING_AS_PUSH_BUTTON true) assumes to control an external feeder (e.g., Boyu - that I already have!), 
// and requires disassembly it and a little wiring. Connects (parallel) the "feed" push button of your feeder (both terminals) 
// to the "normally open" pins of your relay pins (RELAY_FEEDER_PIN). 
// Therefore, the relay can simulate a user button press (and the feeder push button still working independently, just as before...).
//
// Stepper motor feeder
//
// This configuration (FEEDING_AS_PUSH_BUTTON false) assumes making a feeder from scratch using a stepper motor (and driver).  
// On that step, the stepper motor will the specified number of turns in order to feed the fishes.
// The simpler way of do the "mechanical" parts of the feeder is to fix and small plastic bottle on stepper motor axis.
// Make one hole (transversal) in the bottle in order to pass the food as the bottle rotates, like:
//   - https://www.instructables.com/Simplest-Automatic-Fish-Feeder/
//   - https://www.instructables.com/Arduino-Fish-Feeder/
// Alternatively, you can use any kind of drill or wire do drive the food:
//   - https://hackaday.com/2014/10/14/diy-auto-fish-feeder-feeds-fish-automatically/) 
//   - https://flourishingplants.com/how-to-make-an-automatic-fish-feeder-with-simple-materials/)
//
// Remark: in this step up, the relay pin (RELAY_FEEDER_PIN) is free to be used as an extra relay for any application 
// (the same if feeding routine is disabled at all - ENABLE_FEEDING false).
//
// TODO:
//  - use push buttons by interuption
//
//------------------------------------------------------------------------------------------------------------------
//
// 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
//
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <Ethernet.h>
#include <ArduinoHA.h>
//
// Configuration flags (enables or disables features in order to "skip" unwanted hardware)
//
#define USE_LCD_DISPLAY true                       // enables/disables LCD display (disable it to not use the LCD display)
#define USE_HOME_ASSISTANT true                    // enables/disables home assistant integration (disable it to not use the LAN ethernet module)
#define USE_WATER_PH_SENSOR true                   // enables/disables water pH sensor (disable it to not use the Ph4502c sensor)
#define USE_WATER_TEMPERATURE_SENSOR true          // enables/disables water temperature sensor (disable it to not use the DS18B20 temperature sensor)
#define USE_AIR_TEMPERATURE_SENSOR true            // enables/disables air temperature and humidity sensor (disable it to not use the DHT11 sensor)
#define USE_AIR_PRESSURE_SENSOR true               // enables/disables air pressure sensor (disable it to not use the BMP085 sensor)
#define USE_WATER_LEVEL_SENSORS true               // enables/disables water level sensors (disable it to not use the water level sensors)
#define USE_RTC_CLOCK true                         // enables/disables clock (disable it to not use the RTC module)
#define USE_RELAYS true                            // enables/disables relays (disable it to not use the relay module)
#define USE_PUSH_BUTTONS true                      // enables/disables push buttons (disable it to not use the push buttons)
#define USE_STANDALONE_TIMERS true                 // enables/disables timers
#define USE_EEPROM false                           // enables/disables EEPROM 
#define ENABLE_LIGHTS_TIMER true                   // enables/disables lights timers
#define ENABLE_UV_FILTER_TIMER true                // enables/disables UV filter timers
#define ENABLE_FEEDING true                        // true for uses "feeding" relay as feeder, false for uses it as regular extra relay
#define ENABLE_FEEDING_TIMER true                  // enables/disables feeding timers
#define FEEDING_AS_PUSH_BUTTON true                // true for feeding using a external feeder push button or false for use a step motor as feeder (see docs)
#define ENABLE_SENSOR_ALARMS true                  // enables/disables sensor alarming
#define ENABLE_WEATHER_STATION true                // enables/disables standalone weather station (requires air pressure sensor BMP085)
#define MAX_NUMBER_OF_TIMERS 50                    // maximum number of programmable timers (default: 50)
#define FEEDER_STEPPER_MOTOR_PULSER_PER_TURN 200   // number of pulses per turn on stepper motor feeder
#define STEPPER_MOTOR_DELAY 700                    // time delay (in microseconds) between the motor steps (use it for change the rotation speed)
#define FEEDER_TURNS 1                             // default number of feeder turns (comple cycles)
#define LAMBDA_EWMA 0.96                           // exponential weighted mean average lambda

//
// MQTT configuration
//
#define MQTT_BROKER_ADDRESS "192.168.68.93"        // MQTT broker server ip
#define MQTT_BROKER_PORT 1883                      // MQTT broker port
#define MQTT_USERNAME  "mqtt-user"                 // can be omitted if not needed
#define MQTT_PASSWORD  "mqtt-pass"                 // can be omitted if not needed
#define MQTT_DISCOVERY_TOPIC "homeassistant"       // MQTT discovery topic prefix 
#define MQTT_STATES_TOPIC "aquarioino/state"       // MQTT state topic prefix 
#define MQTT_COMMAND_TOPIC "aquarioino/cmd"        // MQTT topic for custom text commands
#define MQTT_TIME_SYNC_TOPIC "ha/datetime"         // [optional] home assistant MQTT topic that broadcast current date/time (see the docs)
#define MQTT_AVAILABILITY_TIME 60000               // time to send MQTT availability (default: 1 min)
//
// debug & testing flags
//
#define DEBUG_MODE true                            // enable/disables serial debugging messages
// testing routines: enable flags to test hardware / wiring 
#define DEBUG_LCD_ROUTINE false                    // test routine for LCD display (shows test message)
#define DEBUG_RTC_ROUTINE false                    // test routine for RTC (shows curent time)
#define DEBUG_RELAY_ROUTINE false                  // test routine for Relay (activate relay channels in sequence)
#define DEBUG_PUSH_BUTTONS_ROUTINE false           // test routine for push buttons
#define DEBUG_WATER_LEVEL_SENSORS_ROUTINE false    // test routine for water level sensors
#define DEBUG_WATER_REPOSITION_PUMP_ROUTINE false  // test routine for repostion pump
#define DEBUG_COOLER_FAN_ROUTINE false             // test routine for cooler fan
//
//
// pins definitions (Arduino Mega 2560 pro mini)
//
// LCD
#define LCD_PIN_RS 12                              // LCD pin 4   
#define LCD_PIN_ENABLED 11                         // LCD pin 6   
#define LCD_PIN_D4 5                               // LCD pin 11
#define LCD_PIN_D5 4                               // LCD pin 12
#define LCD_PIN_D6 3                               // LCD pin 13
#define LCD_PIN_D7 2                               // LCD pin 14
// relays
#define RELAY_SIZE 8                               // do not change: 8 channels relay recommended!
#define RELAY_HEATER_PIN 20                        // relay heater (normally open, common on 110vac)
#define RELAY_LIGHTS_PIN 19                        // relay lightening (normally open, common on 110vac)
#define RELAY_FILTER_UV_PIN 18                     // relay filter UV (normally open, common on 110vac)
#define RELAY_SUMP_PUMP_PIN 17                     // relay sump pump (normally open, common on 110vac)
#define RELAY_DRAIN_PUMP_PIN 16                    // relay drain pump (normally open, common on 110vac)
#define RELAY_WATER_REPOSITION_PUMP_PIN 13         // relay water reposition pump (normally open, common on 110vac)
#define RELAY_COOLER_FAN_PIN 14                    // relay cooler fan (normally open, common on 5v/12vdc)
#define RELAY_FEEDER_PIN 15                        // relay feeder (normally open, common on short circuit at my old boyu feeder "feed button")
// water level sensors
#define DRAIN_WATER_LOW_LEVEL_SENSOR_PIN 23        // this water level sensor should be located at maximum aquarium level (see diagram above)
#define SUMP_WATER_LOW_LEVEL_SENSOR_PIN 24         // this water level sensor should be located at minimum level of the sump pump (see diagram above)
// LAN ethernet module
#define ETHERNETMODULE_PIN_CS 10                   // Ethernet ECN28J60 (CS)
#define ETHERNETMODULE_PIN_INT 3                   // Ethernet ECN28J60 (INT)
// analog sensors
#define WATER_TEMPERATURE_SENSOR_DS18B20_PIN 7     // sensor DS18B20 (analog)  
#define AIR_TEMPERATURE_SENSOR_DHT11_PIN 8         // sensor DHT11   (analog)
#define AIR_PRESSURE_SENSOR_BMP180_PIN_SDA 20      // sensor BMP180  (analog) - Arduino Mega 2560
#define AIR_PRESSURE_SENSOR_BMP180_PIN_SCL 21      // sensor BMP180  (analog) - Arduino Mega 2560
#define WATER_PH_SENSOR_PH450C_PIN 6               // sensor Ph4502c (analog)
// buttons and leds
#define PUSH_BUTTON_TURN_OFF_PIN 25
#define PUSH_BUTTON_RESTART_PIN 26
#define PUSH_BUTTON_LIGHTS_PIN 27
#define PUSH_BUTTON_PWC_ROUTINE_PIN 28
#define PUSH_BUTTON_MANUAL_CLEANING_PIN 39
#define PUSH_BUTTON_FEEDING_PIN 30 
// steper motor
#define STEPPER_MOTOR_STEP_PIN 31
#define STEPPER_MOTOR_DIRECTION_PIN 32
//
// LCD text messages (MAX 16 characters) - [pt-BR]
//
#define TEXT_CLOCK_NOT_FOUND F("Clock not found!")
#define TEXT_GOOD_MORNING F("Bom dia!")
#define TEXT_GOOD_AFTERNOON F("Boa tarde!")
#define TEXT_GOOD_NIGHT F("Boa noite!")
#define TEXT_AIR_TEMPERATURE F("Temp Ar: ")
#define TEXT_AIR_HUMIDITY F("Umid Ar: ")
#define TEXT_AIR_PRESSURE F("Pres Atm: ")
#define TEXT_ALTITUDE F("Altitude: ")
#define TEXT_WATER_TEMPERATURE F("Temp agua: ")
#define TEXT_WATER_PH F("pH agua: ")
#define TEXT_MIN F("Min: ")
#define TEXT_MAX F("Max: ")
#define TEXT_AVG F("Med: ")
#define TEXT_LIGHTS F("Luz: ")
#define TEXT_ON F("ON")
#define TEXT_OFF F("OFF")
#define TEXT_TIMER F("TIMER")
#define TEXT_FEED F("Feed: ")
#define TEXT_ALARM_LOW_WATER_LEVEL F("Nivel agua baixo")
#define TEXT_ALARM_HIGH_TEMPERATURE F("Temp alta: ")
#define TEXT_ALARM_LOW_TEMPERATURE F("Temp baixa: ")
#define TEXT_ALARM_HIGH_PH F("Ph alto: ")
#define TEXT_ALARM_LOW_PH F("Ph baixo: ")
#define TEXT_PWC_STARTING F("T P A")
#define TEXT_DRAINING_WATER F("Drenando agua...")
#define TEXT_REPLACING_WATER F("Repondo agua... ")
#define TEXT_CHECKING F("Verificando...  ")
#define TEXT_PWC_FINISHED F("TPA encerrado...")
#define TEXT_MANUAL_CLEANING F("Limpeza manual")
#define TEXT_TURN_STARTING F("Iniciando...")
#define TEXT_TURN_OFF F("Desligando")
#define TEXT_LOCAL_FORECAST F("Forecast local:")
#define TEXT_WEATHER_NO_FORECAST F("Sem previsao")
#define TEXT_WEATHER_GOOD F("Tempo BOM")
#define TEXT_WEATHER_DRY F("Tempo seco")
#define TEXT_WEATHER_NO_CHANGES F("Sem alteracao")
#define TEXT_WEATHER_LIGHT_RAIN F("Chuva fraca")
#define TEXT_WEATHER_RAINING F("Chuva forte")
#define TEXT_WEATHER_HEAVY_RAIN F("Chuva torrencial")
#define TEXT_WEATHER_TUNDERSTORM F("TEMPESTADE")
#define TEXT_RELAY_LIGHTS F("light")
#define TEXT_RELAY_HEATER F("heater")
#define TEXT_RELAY_FILTER F("filter")
#define TEXT_RELAY_COOLER F("cooler")
#define TEXT_RELAY_SUMP_PUMP F("sump")
#define TEXT_RELAY_REPOSITION_PUMP F("repo")
#define TEXT_RELAY_DRAIN_PUMP F("drain")
#define TEXT_RELAY_FEED F("feed")
#define TEXT_UNKNOW F("UNKNOW")
//
// EEPROM
//
#define EEPROM_START_ADDRESS 0
#define EEPROM_FLAGS_ADDRESS 1
#define EEPROM_FLAG_TIMERS_ENABLED_BIT 0
#define EEPROM_FLAG_FEEDER_TIMERS_ENABLED_BIT 1
#define EEPROM_ALARM_BOUNDS_ADDRESS 2
#define EEPROM_TIMERS_ADDRESS (EEPROM_ALARM_BOUNDS_ADDRESS + 4 * sizeof(float))
#define EEPROM_START_CHECK 0x24
//
// status codes (internal)
//
#define STATUS_CODE_SHOW_DATETIME 0
#define STATUS_CODE_SHOW_WATER_TEMPERATURE_01 1
#define STATUS_CODE_SHOW_WATER_TEMPERATURE_02 2
#define STATUS_CODE_SHOW_WATER_TEMPERATURE_03 3
#define STATUS_CODE_SHOW_WATER_PH 4
#define STATUS_CODE_SHOW_AIR_TEMPERATURE_01 5
#define STATUS_CODE_SHOW_AIR_TEMPERATURE_02 6
#define STATUS_CODE_SHOW_WEATHER_FORECAST_LOCAL 7
#define STATUS_CODE_SHOW_FLAGS 8
#define STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE 10
#define STATUS_CODE_MANUAL_CLEANING 11
#define STATUS_CODE_TURNED_OFF 12
#define WEATHER_CODE_NO_FORECAST 0
#define WEATHER_CODE_GOOD 1
#define WEATHER_CODE_DRY 2
#define WEATHER_CODE_NO_CHANGES 3
#define WEATHER_CODE_LIGHT_RAIN 4
#define WEATHER_CODE_RAINING 5
#define WEATHER_CODE_HEAVY_RAIN 6
#define WEATHER_CODE_TUNDERSTORM 7 
//
// reference states
//
#define STATUS_CYCLE_START STATUS_CODE_SHOW_DATETIME                         // Cycle start
#define STATUS_CYCLE_END STATUS_CODE_SHOW_FLAGS                              // Cycle end
#define STATUS_CYCLE_OFF STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE  // first non cycle state
#define STATUS_INVALID 100                                                   // invalid state
#define ARDUINOHA_DEBUG DEBUG_MODE

#define DEFAULT_ALARM_BOUND_TEMP_MAX 29.1
#define DEFAULT_ALARM_BOUND_TEMP_MIN 26.0
#define DEFAULT_ALARM_BOUND_PH_MAX 8.9
#define DEFAULT_ALARM_BOUND_PH_MIN 7.5
// timers start/end hours
#define DEFAULT_TIMER_LIGHT_ON 6;
#define DEFAULT_TIMER_LIGHT_OFF 23;
#define DEFAULT_TIMER_UV_ON 8;
#define DEFAULT_TIMER_UV_OFF 15;
#define DEFAULT_TIMER_FEED_1 7;
#define DEFAULT_TIMER_FEED_2 12;
#define DEFAULT_TIMER_FEED_3 18;

//
// timer
//
struct timer{
   bool enabled = false; 
   bool active = false;       
   byte relayIndex = 0x00;      // relay index: [0, RELAY_SIZE - 1]
   byte turnOnHour = 0x00;      // hh [0 - 23]
   byte turnOnMinute = 0x00;    // mm [0 - 59]
   byte turnOffHour = 0x00;     // hh [0 - 23]
   byte turnOffMinute = 0x00;   // mm [0 - 59]
};
//
// measured value
//
struct measure {
   float value = 0;
   float average = 0;
   float max = 0;
   float min = 100000000000;
};

#define is_valid_timer(t) ((t.turnOnHour != t.turnOffHour || t.turnOnMinute != t.turnOffMinute))

#if (USE_LCD_DISPLAY == true)
  //
  // LCD Display (16 x 2)
  //
  // Pin1 --> Gnd
  // Pin2 --> 5v
  // Pin3 --> 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd)
  // Pin4 --> Arduino Pin12
  // Pin5 --> Gnd
  // Pin6 --> Arduino Pin11
  // Pin7 --> no connection
  // Pin8 --> no connection
  // Pin9 --> no connection
  // Pin10 --> no connection
  // Pin11 --> Arduino Pin5
  // Pin12 --> Arduino Pin4
  // Pin13 --> Arduino Pin3
  // Pin14 --> Arduino Pin2
  // Pin15 --> 5v
  // Pin16 --> Tactile button (other side of tack button goes to Gnd)
  LiquidCrystal _lcd(LCD_PIN_RS, LCD_PIN_ENABLED, LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7);
#endif

#if (USE_RTC_CLOCK == true)
  //
  // Real time clock (RTC DS3231)
  //
  // Pin1 (SCL) --> Pin20
  // Pin2 (SDA) --> Pin21
  // Pin3 (vcc) --> 5v
  // Pin4 (gnd) --> Gnd
  RTC_DS3231 _clock;
#endif

#if (USE_WATER_TEMPERATURE_SENSOR == true)
  //
  // Water temperature sensor (DS18B20)
  //
  // VCC  --> Arduino 5v, plus a 4.7K resistor going from VCC to Data
  // Data --> Arduino Pin7
  // GND  --> Arduino Gnd
  OneWire oneWire(WATER_TEMPERATURE_SENSOR_DS18B20_PIN);
  DallasTemperature _sensorWaterTemperature(&oneWire);
#endif

#if (USE_AIR_TEMPERATURE_SENSOR == true)
  //
  // Air temperature sensor (DHT11)
  //
  // Pin1 --> 5v and 10k ohm resistor
  // Pin2 --> Arduino Pin8 and 10k ohm resistor
  // Pin3 --> no connection
  // Pin4 --> Gnd
  DHT _sensorAirTemperature(AIR_TEMPERATURE_SENSOR_DHT11_PIN, DHT11);
#endif

#if (USE_AIR_PRESSURE_SENSOR == true)
  //
  // Air pressure sensor (BMP180)
  //
  // Pin1 (SDA) --> Pin20
  // Pin2 (SCL) --> Pin21
  // Pin3 (gnd) --> Gnd
  // Pin4 (vcc) --> 5v
  Adafruit_BMP085 _sensorAirPresure; // (I2C)
#endif 


#if (USE_WATER_PH_SENSOR == true)
  //
  // pH pressure sensor ()
  //
#endif

#if (USE_HOME_ASSISTANT == true)
  //
  // Ethernet module (ECN28J60)
  //
  // Pin1 (CLK) --> no connection
  // Pin2 (INT) --> Arduino Nano Pin6 (D3)
  // Pin3 (WOL) --> no connection
  // Pin4 (SO) --> Arduino Nano Pin15 (D12)
  // Pin5 (S1) --> Arduino Nano Pin14 (D11)
  // Pin6 (SCK) --> Arduino Nano Pin16 (D13)
  // Pin7 (CS) --> Arduino Nano Pin13 (D10)
  // Pin8 (RST) --> no connection
  // Pin9 (VCC) --> 3.3v (Arduino Nano Pin17 - recommend to use a voltage regulator 3.3v: LD11173v3)
  // Pin10 (GND) --> Gnd (Arduino Nano Pin29)
  EthernetClient client;
  //
  // MQTT strings labels
  //
  const char str_device_name[] = "aquarioino"; // MQTT device name
  const char str_device_version[] =  "1.0.0"; // MQTT device version
  const char str_device_manufacturer[] =  "Jorge"; // MQTT device manufacturer
  const char str_icon[] = "mdi:water"; // MQTT default cins
  const char str_unit_temperature[] = "C"; // MQTT temperature unit
  const char str_state_sensor_uid[] = "aquarioino_state_sensor";
  const char str_weather_forecast_sensor_uid[] = "aquarioino_weather_forecast_sensor";
  const char str_water_temperature_sensor_uid[] = "aquarioino_water_temperature_sensor";
  const char str_water_ph_sensor_uid[] = "aquarioino_water_ph_sensor";
  const char str_air_temperature_sensor_uid[] = "aquarioino_air_temperature_sensor";
  const char str_air_humidity_sensor_uid[] = "aquarioino_air_humidity_sensor";
  const char str_air_pressure_sensor_uid[] = "aquarioino_air_pressure_sensor";
  const char str_light_switch_uid[] = "aquarioino_lights";
  const char str_heater_switch_uid[] = "aquarioino_heater_switch";
  const char str_cooler_switch_uid[] = "aquarioino_cooler_switch";
  const char str_uv_filter_switch_uid[] = "aquarioino_uv_filter_switch";
  const char str_timers_switch_uid[] = "aquarioino_timers_switch";
  const char str_feeder_timers_switch_uid[] = "aquarioino_feeder_timers_switch";
  const char str_turn_off_button_uid[] = "aquarioino_turn_off_button";
  const char str_restart_button_uid[] = "aquarioino_restart_button";
  const char str_manual_cleaning_button_uid[] = "aquarioino_manual_cleaning_button";
  const char str_pwc_button_uid[] = "aquarioino_pwc_button";
  const char str_feed_button_uid[] = "aquarioino_feed_button";
  const char str_save_button_uid[] = "aquarioino_save_button";
  const char str_temperature_max_uid[] = "aquarioino_temperature_max";
  const char str_temperature_min_uid[] = "aquarioino_temperature_min";
  const char str_ph_max_uid[] = "aquarioino_ph_max";
  const char str_ph_min_uid[] = "aquarioino_ph_min";
  const char str_feeder_turns_uid[] = "aquarioino_feeder_turns";
  const char TEXT_MQTT_NAME_STATE[] = "Tank State";
  const char TEXT_MQTT_NAME_LIGHT[] = "Tank Light";
  const char TEXT_MQTT_NAME_HEATER[] = "Tank Heater";
  const char TEXT_MQTT_NAME_COOLER[] = "Tank Cooler";
  const char TEXT_MQTT_NAME_TIMERS[] = "Tank Timers";
  const char TEXT_MQTT_NAME_FEED_TIMERS[] = "Tank Feeder";
  const char TEXT_MQTT_NAME_WEATHER_FORECAST[] = "Weather Forecast";
  const char TEXT_MQTT_NAME_UV_FILTER[] = "Tank UV Filter";
  const char TEXT_MQTT_NAME_TURN_OFF[] = "Tank Turn off";
  const char TEXT_MQTT_NAME_RESTART[] = "Tank Restart";
  const char TEXT_MQTT_NAME_MANUAL_CLEANING[] = "Tank Manual Cleaning";
  const char TEXT_MQTT_NAME_PWC[] = "Tank PWC";
  const char TEXT_MQTT_NAME_FEED[] = "Tank Feed";
  const char TEXT_MQTT_NAME_RUNNING[] = "running";
  const char TEXT_MQTT_NAME_WATER_TEMPERATURE[] = "Tank Water Temperature";
  const char TEXT_MQTT_NAME_WATER_PH[] = "Tank pH";
  const char TEXT_MQTT_NAME_AIR_TEMPERATURE[] = "Air Temperature";
  const char TEXT_MQTT_NAME_AIR_HUMIDITY[] = "Humidity";
  const char TEXT_MQTT_NAME_AIR_PRESSURE[] = "Atmospheric Pressure";
  const char TEXT_MQTT_NAME_SAVE[] = "Salve Tank Setup";
  const char TEXT_MQTT_FEEDER_TURNS[] = "Tank Feeder Turns";
  #define TEMP_MIN 10
  #define TEMP_MAX 45
  #define TEMP_STEP 0.1
  #define PH_MIN 3
  #define PH_MAX 12
  //
  // Home Assistant (MQTT)
  //
  byte _mac[] = {0xF0, 0x24, 0xAF, 0xE6, 0x32, 0xA4};
  HADevice device(_mac, sizeof(_mac));
  HAMqtt mqtt(client, device);
  
  // MQTT general state sensor
  HASensor sensorState(str_state_sensor_uid);
  HASensor sensorWeatherForecast(str_weather_forecast_sensor_uid);

  // MQTT temperature, humidity and pressure sensors
  HASensorNumber sensorWaterTemp(str_water_temperature_sensor_uid, HASensorNumber::PrecisionP1);
  HASensorNumber sensorWaterPh(str_water_ph_sensor_uid, HASensorNumber::PrecisionP1);
  HASensorNumber sensorAirTemp(str_air_temperature_sensor_uid, HASensorNumber::PrecisionP1);
  HASensorNumber sensorAirHumidity(str_air_humidity_sensor_uid, HASensorNumber::PrecisionP0);
  HASensorNumber sensorAirPressure(str_air_pressure_sensor_uid, HASensorNumber::PrecisionP0);

  // MQTT light switches
  HALight chkLightSwitch(str_light_switch_uid);
  HASwitch chkFilterSwitch(str_uv_filter_switch_uid);
  HASwitch chkHeaterSwitch(str_heater_switch_uid);
  HASwitch chkCoolerSwitch(str_cooler_switch_uid);
  HASwitch chkTimersSwitch(str_timers_switch_uid);
  HASwitch chkFeederTimersSwitch(str_feeder_timers_switch_uid);

  // MQTT buttons
  HAButton btnTurnOff(str_turn_off_button_uid);
  HAButton btnRestart(str_restart_button_uid);
  HAButton btnManualCleaning(str_manual_cleaning_button_uid);
  HAButton btnStartPWC(str_pwc_button_uid);
  HAButton btnFeed(str_feed_button_uid);
  HAButton btnSave(str_save_button_uid);

  // MQTT parameters
  HANumber lblTemperatureMax(str_temperature_max_uid, HASensorNumber::PrecisionP1);
  HANumber lblTemperatureMin(str_temperature_min_uid, HASensorNumber::PrecisionP1);
  HANumber lblPhMax(str_ph_max_uid, HASensorNumber::PrecisionP1);
  HANumber lblPhMin(str_ph_min_uid, HASensorNumber::PrecisionP1);
  HANumber lblFeederTurns(str_feeder_turns_uid, HASensorNumber::PrecisionP0);

#endif

//
// Globals: states and measured values
//
DateTime _now;
int _status = STATUS_CYCLE_START;                       // aguarium state code
bool _isTimersEnabled = USE_STANDALONE_TIMERS;          // enables/disables timers
bool _isFeedingTimersEnabled = ENABLE_FEEDING_TIMER;    // enables/disables feeding timers
bool _statusFeeded = false;                             // checks that feed routine was executed (feeding semaphore): prevents feed several times
bool _waterDrainLevelLow = false;                       // "drain" water level state (see diagram above)
bool _waterSumpLevelLow = false;                        // "sump" water level state (see diagram above)
measure _measuredAirTemperature;                        // air temperature (C)
measure _measuredAirHumidity;                           // air humidity (%)
measure _measuredAirPressure;                           // atmosferic pressure (mPa)
measure _measuredAirAltitude;                           // altirude (m)
measure _measuredAirSealevelPressure;                   // sea level pressure (mPa)  
measure _measuredWaterTemperature;                      // water temperature (C)
measure _measuredWaterPh;                               // water pH 
unsigned long _feederTurns = FEEDER_TURNS;              // number of feeder turns
bool _blink = false;                                    // builtin led state
unsigned long _lastAvailabilityTime = millis();

// required for "setAllRelays()" routine
const unsigned int RELAY_PINS[] = {RELAY_LIGHTS_PIN, RELAY_HEATER_PIN, RELAY_FILTER_UV_PIN, 
  RELAY_COOLER_FAN_PIN, RELAY_SUMP_PUMP_PIN, RELAY_WATER_REPOSITION_PUMP_PIN, RELAY_DRAIN_PUMP_PIN, RELAY_FEEDER_PIN};
// relay states cache
bool RELAY_STATES[RELAY_SIZE];
// force relay state to ON (turned on by user button)
bool RELAY_FORCED_TURN_ON[RELAY_SIZE];

// timers
#if (USE_STANDALONE_TIMERS == true)
  #define TIMERS_SIZE MAX_NUMBER_OF_TIMERS
#else
  // minimum for store default timers (5) on EEPROM
  #define TIMERS_SIZE 5
#endif
timer RELAY_TIMERS[TIMERS_SIZE];

//
// reference alarm levels/bounds (with default values)
//
float _alarmBoundWaterTemperatureMax = DEFAULT_ALARM_BOUND_TEMP_MAX;
float _alarmBoundWaterTemperatureMin = DEFAULT_ALARM_BOUND_TEMP_MIN;
float _alarmBoundWaterPhMax = DEFAULT_ALARM_BOUND_PH_MAX;
float _alarmBoundWaterPhMin = DEFAULT_ALARM_BOUND_PH_MIN;

//--------------------------------------------------------------------------------------------------
//
// setup
//
//--------------------------------------------------------------------------------------------------

void setup() {
  //
  // initialization
  //
  pinMode(LED_BUILTIN, OUTPUT); // blinking led 

  #if (DEBUG_MODE == true)
    // serial output only in debug mode!
    Serial.begin(9600);    
    while (!Serial) {
      // wait for serial port to connect
      delay(100);
    }     
    Serial.println(F("starting..."));
  #endif

  #if (USE_RELAYS == true)
    //
    // initializes relays
    //
    pinMode(RELAY_HEATER_PIN, OUTPUT);
    pinMode(RELAY_LIGHTS_PIN, OUTPUT);
    pinMode(RELAY_FILTER_UV_PIN, OUTPUT);
    pinMode(RELAY_SUMP_PUMP_PIN, OUTPUT);
    pinMode(RELAY_DRAIN_PUMP_PIN, OUTPUT);
    pinMode(RELAY_FEEDER_PIN, OUTPUT);
    pinMode(RELAY_COOLER_FAN_PIN, OUTPUT);
    pinMode(RELAY_WATER_REPOSITION_PUMP_PIN, OUTPUT);

    // initializes relays states (HIGH = turns off)
    digitalWrite(RELAY_HEATER_PIN, HIGH);
    digitalWrite(RELAY_LIGHTS_PIN, HIGH);
    digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
    digitalWrite(RELAY_FILTER_UV_PIN, HIGH);
    digitalWrite(RELAY_DRAIN_PUMP_PIN, HIGH);
    digitalWrite(RELAY_FEEDER_PIN, HIGH);  
    digitalWrite(RELAY_COOLER_FAN_PIN, HIGH);
    digitalWrite(RELAY_WATER_REPOSITION_PUMP_PIN, HIGH);  
  #endif

  #if (USE_LCD_DISPLAY == true)
    //
    // intialize LCD 16 x 2
    //
    _lcd.begin(16, 2);
    #if (DEBUG_LCD_ROUTINE == true)
      #if (DEBUG_MODE == true)
      Serial.println(F("LCD debug mode!"));
      #endif
      // case testing LCD, does not intializes anything more... :)
      return;
    #endif
  #endif

  #if (USE_RTC_CLOCK)
    //
    // initialize real time clock (RTC)
    //  
    if (! _clock.begin()) {
      // clock not found!
      #if (DEBUG_MODE == true)      
      Serial.println(F("clock RTC DS3231 not found! Stop!"));
      #endif

      #if (USE_LCD_DISPLAY)
      _lcd.setCursor(0, 0);
      _lcd.print(TEXT_CLOCK_NOT_FOUND);
      #endif

      // Clock NOT FOUND: STOP!
      while (1);
    }

    if (_clock.lostPower()) {
      #if (DEBUG_MODE == true)
        Serial.println(F("settting clock RTC DS3231 date and time!"));
      #endif

      // sets sketch compilation date and time
      _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));  
    }

    #if (DEBUG_MODE == true)
      Serial.println(F("clock RTC DS3231: Ok!"));
    #endif

    #if (DEBUG_RTC_ROUTINE == true)
      // RTC testing routine
      #if (DEBUG_MODE == true)
        Serial.println(F("real time clock debug mode!)");
      #endif

      _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));

      // case testing RTC, does not intializes anything more...
      return;
    #endif
  #endif

  //
  // initialize sensors
  //
  #if (USE_WATER_TEMPERATURE_SENSOR == true)
      #if (DEBUG_MODE == true)
        Serial.println(F("starting water temperature sensor"));
      #endif

    _sensorWaterTemperature.begin();    
  #endif
  
  #if (USE_AIR_TEMPERATURE_SENSOR == true)
      #if (DEBUG_MODE == true)
        Serial.println(F("starting air temperature sensor"));
      #endif

    _sensorAirTemperature.begin();
  #endif
  
  #if (USE_AIR_PRESSURE_SENSOR == true)
    if (!_sensorAirPresure.begin()) {
      // sensor not found
      #if (DEBUG_MODE == true)
        Serial.println(F("ERROR: Air pressure sensor BMP180 not found!"));
      #endif
    }
    else {
      // initialize pressure measurements
      updateMeasure(_measuredAirPressure, _sensorAirPresure.readPressure());
    }
  #endif

  #if (USE_WATER_LEVEL_SENSORS == true)
    //
    // water level sensors
    //
    pinMode(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN, INPUT);
    pinMode(SUMP_WATER_LOW_LEVEL_SENSOR_PIN, INPUT);
  #endif

  #if (USE_PUSH_BUTTONS == true)
    //
    // push buttons
    //
    pinMode(PUSH_BUTTON_TURN_OFF_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_RESTART_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_LIGHTS_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_PWC_ROUTINE_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_MANUAL_CLEANING_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_FEEDING_PIN, INPUT_PULLUP);
  #endif
  delay(100);

  #if (ENABLE_FEEDING == true && FEEDING_AS_PUSH_BUTTON == false) 
    //
    // Initialized stepper motor feeder (not use "push button"-like feeder)
    //
    pinMode(STEPPER_MOTOR_STEP_PIN, OUTPUT); 
    pinMode(STEPPER_MOTOR_DIRECTION_PIN, OUTPUT);
  #endif

  #if (USE_HOME_ASSISTANT == true)
    //
    // MQTT initialization
    //
    device.setName(str_device_name);
    device.setSoftwareVersion(str_device_version);
    device.setManufacturer(str_device_manufacturer);    
    device.enableSharedAvailability();
    device.setAvailability(false);
    device.enableLastWill();
    

    mqtt.setDiscoveryPrefix(MQTT_DISCOVERY_TOPIC);
    mqtt.setDataPrefix(MQTT_STATES_TOPIC);
    mqtt.onMessage(onMessage);
    mqtt.onConnected(onConnected);

    sensorWaterTemp.setName(TEXT_MQTT_NAME_WATER_TEMPERATURE);    
    sensorAirTemp.setName(TEXT_MQTT_NAME_AIR_TEMPERATURE);
    sensorAirHumidity.setName(TEXT_MQTT_NAME_AIR_HUMIDITY);
    sensorAirPressure.setName(TEXT_MQTT_NAME_AIR_PRESSURE);
    
    sensorState.setName(TEXT_MQTT_NAME_WEATHER_FORECAST);

    chkLightSwitch.setName(TEXT_MQTT_NAME_LIGHT);
    chkLightSwitch.setRetain(true);
    chkLightSwitch.onStateCommand(onLightStateCommand);

    chkFilterSwitch.setName(TEXT_MQTT_NAME_UV_FILTER);
    chkFilterSwitch.setRetain(true);
    chkFilterSwitch.onCommand(onSwitchCommand);

    chkHeaterSwitch.setName(TEXT_MQTT_NAME_HEATER);
    chkHeaterSwitch.setRetain(true);
    chkHeaterSwitch.onCommand(onSwitchCommand);

    chkCoolerSwitch.setName(TEXT_MQTT_NAME_COOLER);
    chkCoolerSwitch.setRetain(true);
    chkCoolerSwitch.onCommand(onSwitchCommand);

    chkTimersSwitch.setName(TEXT_MQTT_NAME_TIMERS);
    chkTimersSwitch.setRetain(true);
    chkTimersSwitch.onCommand(onSwitchCommand);

    chkFeederTimersSwitch.setName(TEXT_MQTT_NAME_FEED_TIMERS);
    chkFeederTimersSwitch.setRetain(true);
    chkFeederTimersSwitch.onCommand(onSwitchCommand);
    
    btnTurnOff.setName(TEXT_MQTT_NAME_TURN_OFF);
    btnTurnOff.onCommand(onButtonCommand);

    btnRestart.setName(TEXT_MQTT_NAME_RESTART);
    btnRestart.onCommand(onButtonCommand);

    btnManualCleaning.setName(TEXT_MQTT_NAME_MANUAL_CLEANING);
    btnManualCleaning.onCommand(onButtonCommand);

    btnStartPWC.setName(TEXT_MQTT_NAME_PWC);
    btnStartPWC.onCommand(onButtonCommand);

    btnFeed.setName(TEXT_MQTT_NAME_FEED);
    btnFeed.onCommand(onButtonCommand);

    btnSave.setName(TEXT_MQTT_NAME_SAVE);
    btnFeed.onCommand(onButtonCommand);

    lblTemperatureMax.setMax(TEMP_MAX);
    lblTemperatureMax.setMin(TEMP_MIN);    
    lblTemperatureMax.setStep(TEMP_STEP);
    lblTemperatureMax.onCommand(onNumberChange);

    lblTemperatureMin.setMax(TEMP_MAX);
    lblTemperatureMin.setMin(TEMP_MIN);    
    lblTemperatureMin.setStep(TEMP_STEP);
    lblTemperatureMin.onCommand(onNumberChange);
    
    lblPhMax.setMax(PH_MAX);
    lblPhMax.setMin(PH_MIN);
    lblPhMax.setStep(TEMP_STEP);
    lblPhMax.onCommand(onNumberChange);
        
    lblPhMin.setMax(PH_MAX);
    lblPhMin.setMin(PH_MIN);
    lblPhMin.setStep(TEMP_STEP);
    lblPhMin.onCommand(onNumberChange);

    lblFeederTurns.setMax(10);
    lblFeederTurns.setMin(1);
    lblFeederTurns.setStep(1);
    lblFeederTurns.onCommand(onNumberChange);
    
    // start MQTT
    mqtt.begin(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_PASSWORD);
  #endif

  // loads system configurations
  loadConfigurations();
  
  //
  // set defaults
  //
  restart();
}


//--------------------------------------------------------------------------------------------------
//
// loop
//
//--------------------------------------------------------------------------------------------------

void loop() {
  //
  // main loop
  //
  digitalWrite(LED_BUILTIN, (_blink ? HIGH: LOW)); // blinking led
  _blink = !_blink;

  #if (USE_HOME_ASSISTANT)
    //
    // keeps MQTT connection alive
    //
    Ethernet.maintain();
    mqtt.loop();
  #endif

  #if (USE_LCD_DISPLAY)
    // clears LCD
    _lcd.clear();
  #endif

  //
  // execute testing routines (if required)
  //
  if (testComponents())    
    // test routines was executed! do not continue!!!
    return;
  
  //
  // reads sensors data
  //
  readData();

  if (_status == STATUS_CODE_TURNED_OFF) {
    //
    // turned off: nothing to do!
    //
    turnOff();
    return;
  }

  #if (USE_LCD_DISPLAY == true)  
    // sets 1st LCD line: mensages
    _lcd.setCursor(0, 0);
  #endif
  //
  // shows rotating messages on LCD display / serial
  //    
  switch (_status) {
    case STATUS_CODE_SHOW_AIR_TEMPERATURE_01:
      showAirTemperature1();
      break;
    case STATUS_CODE_SHOW_AIR_TEMPERATURE_02:
      showAirTemperature2();
      break;
    case STATUS_CODE_SHOW_WEATHER_FORECAST_LOCAL:
      showLocalWeatherForecast();
      break;  
    case STATUS_CODE_SHOW_WATER_TEMPERATURE_01:
      showWaterTemperature(1);
      break;
    case STATUS_CODE_SHOW_WATER_TEMPERATURE_02:
      showWaterTemperature(2);
      break;
    case STATUS_CODE_SHOW_WATER_TEMPERATURE_03:
      showWaterTemperature(3);
      break;
    case STATUS_CODE_SHOW_WATER_PH:
      showWaterPh();
      break;
    case STATUS_CODE_SHOW_FLAGS:
      showSystemFlags();
      break;    
    case STATUS_CODE_MANUAL_CLEANING:
      runManualCleaning();
      break;
    case STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE:
      runWaterParcialChangeRoutine();
      break;    
    case STATUS_CODE_TURNED_OFF:
      return;
    default:
      // shows date and time
      showDateTime();
      break;
  }
  

  #if (USE_LCD_DISPLAY == true)  
    // sets 2nd LCD line: alarms!
    _lcd.setCursor(0, 1);
  #endif

  #if (ENABLE_SENSOR_ALARMS == true)
    showAlarms();
  #endif
  
  // handles actions (5 seconds)
  for (int i = 0; i <= 10 && !readButtons(); i++) {
    
    #if (USE_STANDALONE_TIMERS == true)
      runTimers();
    #endif

    #if (ENABLE_SENSOR_ALARMS == true)
      handleAlarms();
    #endif

    delay(500);
  }

  // increments states
  nextAquariumState();
}

//--------------------------------------------------------------------------------------------------
//
// feeding
//
//--------------------------------------------------------------------------------------------------

void feed() {  
  //
  // Feeding procedure
  //
  #if (USE_RELAYS == true && ENABLE_FEEDING == true)  

    #if (FEEDING_AS_PUSH_BUTTON == true)
      //
      // this configuration assumes to control an external feeder (Boyu - that I already have!), 
      // connect the "feed" push button (both terminals) of the external feeder to the 
      // "normally open" pins of the relay RELAY_FEEDER_PIN. Therefore, the relay can 
      // simulate an user button press (still working independently).
      //

      // ensures that relay have the transition states: off -> on -> off
      // when relay is on: hort circuit the Boyu feeder push button, simulating user pressing!
      // REMARK: set pin=LOW to turn on relay 
      digitalWrite(RELAY_FEEDER_PIN, HIGH);
      delay(100);
            
      digitalWrite(RELAY_FEEDER_PIN, LOW);
      delay(300);

      digitalWrite(RELAY_FEEDER_PIN, HIGH);
      delay(300);    

    #else
      //
      // TODO: Uses a step motor as feeder (new feature)
      //
      digitalWrite(STEPPER_MOTOR_DIRECTION_PIN, HIGH); 

      for (unsigned int i = 0 ; i < _feederTurns; i++){
        for(unsigned int j = 0 ; j < FEEDER_STEPPER_MOTOR_PULSER_PER_TURN; j++){ 
          // by changing this time delay between the steps we can change the rotation speed
          digitalWrite(STEPPER_MOTOR_STEP_PIN, HIGH); 
          delayMicroseconds(STEPPER_MOTOR_DELAY);
          digitalWrite(STEPPER_MOTOR_STEP_PIN, LOW); 
          delayMicroseconds(STEPPER_MOTOR_DELAY); 
        }
    #endif

  #endif
}

//--------------------------------------------------------------------------------------------------
//
// Auxiliar functions
//
//--------------------------------------------------------------------------------------------------

void updateMeasure(measure& state, const float& value) {
  //
  // updates new value at measure struct
  //

  // updates value
  state.value = value;

  // exponential weighted mean avarage (EWMA) values
  if (state.average == 0)
      state.average = state.value;
  state.average = (1 - LAMBDA_EWMA) * value + LAMBDA_EWMA * state.average; 
  
  // maximum
  if (value > state.max) 
    state.max = value;

  // minimum
  if (value < state.min) 
    state.min = value;
}

void setRelay(unsigned int relayPin, const bool& state, 
  const bool& useIndex = false, const bool& force = false) {
  //
  // sets relay state by relay pin number (useIndex = false): 
  // set "true" to activates the relay, "false" to deactivates it.
  //
  // alternativelly, set relay state by its index ([0-7]) (useIndex = true)
  //
  #if (USE_RELAYS == true)

    // gets relay index by pin number
    unsigned int index = (useIndex ? relayPin : getRelayIndexByPin(relayPin));
  
    // ensure valid relay pin number
    if (index >= RELAY_SIZE){
      // invalid relay pin! 
      #if (DEBUG_MODE == true) 
        Serial.print(F("Warning: invalid relay pin: "));
        Serial.println(relayPin);    
      #endif
      return;
    }
  
    if (useIndex)
      // gets relay pin from relay index
      relayPin = RELAY_PINS[index];

    //
    // at this point:
    //   relayPin = relay pin number
    //   index = relay index at RELAY_PIN vector
    // as it should be!
    //
    
    #if (ENABLE_FEEDING == true)
      //
      // "feeder" relay works different in "ENABLE_FEEDING" mode!
      //      
      if (relayPin == RELAY_FEEDER_PIN) {        
        // default states
        RELAY_STATES[index] = false;
        RELAY_FORCED_TURN_ON[index] = false;        
        if (state)
          feed();
        return;
      }
    #endif

    if (force) {
      // sets "forced" state: on/off
      RELAY_FORCED_TURN_ON[index] = state;
    }
    else if (!state && RELAY_FORCED_TURN_ON[index]){
      // if "forced turned on": do not turn off!
      return;
    }

    if (RELAY_STATES[index] == state)
      // nothing to do!
      return;

    //
    // [IMPORTANT] relays are active LOW!
    //
    digitalWrite(relayPin, (state ? LOW : HIGH));
    
    // keeps relay state on cache
    RELAY_STATES[index] = state;

    #if (DEBUG_MODE == true) 
      Serial.print(F(" - relay "));
      Serial.print(index + 1);
      Serial.print(F(" (pin "));
      Serial.print(relayPin);
      Serial.print(F("): "));
      Serial.println(state ? F("on") : F("off"));    
    #endif
    
    #if (USE_HOME_ASSISTANT == true) 
      //
      // updates states on home assistant MQTT switches
      //
      switch (relayPin) {
        case RELAY_LIGHTS_PIN:
          chkLightSwitch.setState(state);
          break;
        case RELAY_HEATER_PIN:
          chkHeaterSwitch.setState(state);
          break;
        case RELAY_COOLER_FAN_PIN:
          chkCoolerSwitch.setState(state);
          break;
        case RELAY_FILTER_UV_PIN:
          chkFilterSwitch.setState(state);
          break;
      }
    #endif
  #endif
}

bool setAquariumState(unsigned int state) {
  //
  //
  //
  if (_status == state)
    // nothing to do!
    return false;

  if (state < STATUS_CYCLE_START || state > STATUS_CODE_TURNED_OFF)
    // invalid state
    return false;

  // stores state
  _status = state;

  // removes forced turn on at manual cleaning, pwc or turn off
  if (state == STATUS_CODE_MANUAL_CLEANING 
    || state == STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE 
    || state == STATUS_CODE_TURNED_OFF) {
    for(unsigned int i = 0; i < RELAY_SIZE; i++)
        RELAY_FORCED_TURN_ON[i] = false;
  }
  
  #if (DEBUG_MODE == true) 
    //
    // prints current state at serial
    //
    switch (state) {
      case STATUS_CODE_TURNED_OFF:
        Serial.println(F("STATUS: Aquarium turned off"));
        break;
      case STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE:
        Serial.println(F("STATUS: Water Partial Change"));
        break;
      case STATUS_CODE_MANUAL_CLEANING:
        Serial.println(F("STATUS: Manual Cleaning"));
        break;
      default:
        Serial.println(F("STATUS: Running"));
        break;
    }
  #endif  

  #if (USE_HOME_ASSISTANT == true) 
    //
    // updates MQTT state sensor
    //
    switch (state) {
      case STATUS_CODE_TURNED_OFF:
        // turned off status
        sensorState.setValue(TEXT_MQTT_NAME_TURN_OFF);
        break;
      case STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE:
        // PWC status
        sensorState.setValue(TEXT_MQTT_NAME_PWC);
        break;
      case STATUS_CODE_MANUAL_CLEANING:
        // manual cleaning status
        sensorState.setValue(TEXT_MQTT_NAME_MANUAL_CLEANING);
        break;
      default:
        // otherwise, status "running"
        sensorState.setValue(TEXT_MQTT_NAME_RUNNING);
        break;
    }
  #endif  

  return true;
}


void nextAquariumState(){
  //
  // increments states (not for unusual ones: off/cleaning/PWC)
  //  
  if (_status < STATUS_CYCLE_OFF) {
    // increment state
    _status++;

    // circular state transition
    if (_status > STATUS_CYCLE_END)
      _status = STATUS_CYCLE_START;
  }
}

void setAllRelays(const bool& state, const unsigned int& exception = RELAY_SIZE, const bool& force = false) {
  //
  // sets all relays at specified state (with an inverted exception, if required)
  //
  // parmeters:
  //   state        new relay states
  //   exception    [optional] the specified relay pin will have inverted state
  //   force        [optional] ativate the relay "forced" state: 
  //                           - state=true, force=true => relay is turned on and only can be turned off with the combination below   
  //                           - state=false, force=true => relay is turned off (even with a previous forced turn on)
  //                           - state=true, force=false => relay is turned on
  //                           - state=false, force=false => relay is turned off CASE "forced turn on" state is not set, otherwise, do nothing.
  //
  for (unsigned int index = 0; index < RELAY_SIZE; index++) {
    
    #if (ENABLE_FEEDING == true)
      // on "ENABLE_FEEDING" mode, the feeding relay cannot be turned on/off
      if (RELAY_PINS[index] == RELAY_FEEDER_PIN)
        continue;      
    #endif

    if (exception < RELAY_SIZE && index == exception)
      setRelay(index, !state, true, force);
    else
      setRelay(index, state, true, force);
  }
}

unsigned int getRelayIndexByPin(const unsigned int& relayPin){
  //
  // gets relay index (at RELAY_PINS vector) by specified relay pin number
  //
  #if (USE_RELAYS == true)
  for (unsigned int index = 0; index < RELAY_SIZE; index++)
    if (RELAY_PINS[index] == relayPin)
      return index;
  #endif

  // not found!
  return RELAY_SIZE;
}

String getRelayNameByPin(const unsigned int& relayPin){
  //
  // gets relay encoded string name by specified relay pin number
  //
  switch (relayPin) {

    case RELAY_LIGHTS_PIN:
      return String(TEXT_RELAY_LIGHTS);

    case RELAY_HEATER_PIN:
      return String(TEXT_RELAY_HEATER);

    case RELAY_COOLER_FAN_PIN:
      return String(TEXT_RELAY_COOLER);

    case RELAY_FILTER_UV_PIN:
      return String(TEXT_RELAY_FILTER);
    
    case RELAY_SUMP_PUMP_PIN:
      return String(TEXT_RELAY_SUMP_PUMP);

    case RELAY_DRAIN_PUMP_PIN:
      return String(TEXT_RELAY_DRAIN_PUMP);

    case RELAY_WATER_REPOSITION_PUMP_PIN:
      return String(TEXT_RELAY_REPOSITION_PUMP);

    case RELAY_FEEDER_PIN:
      return String(TEXT_RELAY_FEED);    
  }  

  // not found!
  return String(TEXT_UNKNOW);
}

unsigned int getRelayPinByName(const String& relayName) {
  //
  // gets relay pin number by specified encoded string name
  //
  if (relayName.endsWith(TEXT_RELAY_LIGHTS))
      return RELAY_LIGHTS_PIN;

  if (relayName.endsWith(TEXT_RELAY_HEATER))
      return RELAY_HEATER_PIN;

  if (relayName.endsWith(TEXT_RELAY_COOLER))
      return RELAY_COOLER_FAN_PIN;

  if (relayName.endsWith(TEXT_RELAY_FILTER))
      return RELAY_FILTER_UV_PIN;

  if (relayName.endsWith(TEXT_RELAY_SUMP_PUMP))
      return RELAY_SUMP_PUMP_PIN;

  if (relayName.endsWith(TEXT_RELAY_DRAIN_PUMP))
      return RELAY_DRAIN_PUMP_PIN;

  if (relayName.endsWith(TEXT_RELAY_REPOSITION_PUMP))
      return RELAY_WATER_REPOSITION_PUMP_PIN;

  if (relayName.endsWith(TEXT_RELAY_FEED))
      return RELAY_FEEDER_PIN;

  // not found!
  return RELAY_SIZE;
}



int availableMemory() {
  //
  // gets free available memory
  //
  int size = 8192;
  byte *buf;
  while ((buf = (byte *) malloc(--size)) == NULL);
  free(buf);
  return size;
}


#if (DEBUG_MODE == true)
  
  void printMeasure(const measure& measure) {
    //
    // print measure values (debug only)
    //
    Serial.print(" - value: ");
    Serial.print(measure.value);
    Serial.print(", avg: ");
    Serial.print(measure.average);
    Serial.print(", max: ");
    Serial.print(measure.max);
    Serial.print(", min: ");
    Serial.println(measure.min);
  }

#endif

//--------------------------------------------------------------------------------------------------
//
// EEPROM
//
//--------------------------------------------------------------------------------------------------

void saveConfigurationFlag(const unsigned int& bit, const boolean& value) {
  //
  // save configuration flag on EEPROM
  //
  #if (USE_EEPROM == true)
    byte flags = EEPROM.read(EEPROM_FLAGS_ADDRESS);
    bitWrite(flags,  bit, value);
    EEPROM.update(EEPROM_FLAGS_ADDRESS, flags);
  #endif
}


void setTimersEnabled(const boolean& value) {
  //
  // enables/disables timers execution
  //
  if (_isTimersEnabled != value) {
    _isTimersEnabled = value;
    #if (USE_EEPROM == true)
      saveConfigurationFlag(EEPROM_FLAG_TIMERS_ENABLED_BIT, value);
    #endif
  }
}

void setFeederTimersEnabled(const boolean& value) {
  //
  // enables/disables feeder timers execution
  //
  if (_isFeedingTimersEnabled != value) {
    _isFeedingTimersEnabled = value;
    #if (USE_EEPROM == true)
      saveConfigurationFlag(EEPROM_FLAG_FEEDER_TIMERS_ENABLED_BIT, value);
    #endif
  }
}

void loadConfigurations(){
  //
  // loads configuration from EEPROM
  //
  unsigned int index;
  #if (USE_EEPROM == true)

    #if (DEBUG_MODE == true)
      Serial.println(F("loading configurations from EEPROM"));
    #endif

    byte check = EEPROM.read(EEPROM_START_ADDRESS);
    if (check != EEPROM_START_CHECK) {
      // EEPROM checker not found: loads default configurations
      #if (DEBUG_MODE == true)
        Serial.println(F("no previous data found on EEPROM! loading default configurations!"));
      #endif
      // loads default configuration
      loadDefaultConfigurations();      
      // saves default
      saveConfigurations();
      return;
    }    
    
    // loads states flags
    byte states = EEPROM.read(EEPROM_FLAGS_ADDRESS);
    _isTimersEnabled = bitRead(states, EEPROM_FLAG_TIMERS_ENABLED_BIT) == 1;
    _isFeedingTimersEnabled = bitRead(states, EEPROM_FLAG_FEEDER_TIMERS_ENABLED_BIT) == 1;
        
    #if (DEBUG_MODE == true)
      Serial.print(F(" - timers enabled: "));
      Serial.print(toStr(_isTimersEnabled));
      Serial.print(F(" - feeding timers enabled: "));
      Serial.print(toStr(_isFeedingTimersEnabled));
    #endif
    
    // alarms bounds (min, max)
    unsigned int address = EEPROM_ALARM_BOUNDS_ADDRESS;
    EEPROM.get(address, _alarmBoundWaterTemperatureMax);
    address+= sizeof(float);
    EEPROM.get(address, _alarmBoundWaterTemperatureMin);
    address+= sizeof(float);
    EEPROM.get(address, _alarmBoundWaterPhMax);
    address+= sizeof(float);
    EEPROM.get(address, _alarmBoundWaterPhMin);
    address+= sizeof(float);
    
    #if (DEBUG_MODE == true)
      Serial.print(F(" - temp max: "));
      Serial.println(_alarmBoundWaterTemperatureMax);
      Serial.print(F(" - temp min: "));
      Serial.println(_alarmBoundWaterTemperatureMin);
      Serial.print(F(" - ph max: "));
      Serial.println(_alarmBoundWaterPhMax);
      Serial.print(F(" - ph min: "));
      Serial.println(_alarmBoundWaterPhMin);
    #endif

    loadTimers();    

  #else    
    // loads default configurations
    loadDefaultConfigurations();    
  #endif
}

void loadTimers() {
  //
  // loads timers from EEPROM
  //
  #if (USE_STANDALONE_TIMERS == true && USE_EEPROM == true)
    byte check = EEPROM.read(EEPROM_START_ADDRESS);
    if (check != EEPROM_START_CHECK) {      
      // EEPROM checker not found: no timers!
      #if (DEBUG_MODE == true)
        Serial.println(F("Warning: no previous configuration found on EEPROM!"));
      #endif

      cleanTimers();
      return;    
    }

    // reads timers
    address= EEPROM_TIMERS_ADDRESS;
    unsigned int n = EEPROM.read(address);
    #if (DEBUG_MODE == true)
      Serial.print(F("Loading "));
      Serial.print(n);
      Serial.println(F("timers from EEPROM:"));
    #endif

    for (unsigned int i = 0; i < n; i++){
      // relay "bit 7" is the "enabled" state
      byte relayIndex = EEPROM.read(address++);
      RELAY_TIMERS[i].enabled = bitRead(relayIndex, 7);
      bitWrite(relayIndex, 7, 0);      
      RELAY_TIMERS[i].relayIndex = relayIndex; 
      RELAY_TIMERS[i].turnOnHour = EEPROM.read(address++);;
      RELAY_TIMERS[i].turnOnMinute = EEPROM.read(address++);;
      RELAY_TIMERS[i].turnOffHour = EEPROM.read(address++);;
      RELAY_TIMERS[i].turnOffMinute = EEPROM.read(address++);;
      RELAY_TIMERS[i].active = true;

      #if (DEBUG_MODE == true)
        printTimer(RELAY_TIMERS[i], i);
      #endif
    }
  #endif
}

void saveConfigurations(const bool savingTimers = true) {
  //
  // saves configuration on EEPROM
  //
  #if (USE_EEPROM == true)
    
    #if (DEBUG_MODE == true)
      Serial.println(F("saving current configurations at EEPROM"));
    #endif

    // EEPROM value checker
    EEPROM.update(EEPROM_START_ADDRESS, EEPROM_START_CHECK);
    
    // general state flags
    byte flags = EEPROM.read(EEPROM_FLAGS_ADDRESS);
    bitWrite(flags, EEPROM_FLAG_TIMERS_ENABLED_BIT, _isTimersEnabled);
    bitWrite(flags, EEPROM_FLAG_FEEDER_TIMERS_ENABLED_BIT, _isTimersEnabled);
    EEPROM.update(EEPROM_FLAGS_ADDRESS, flags);

    float value;
    // alarms bounds (min, max)    
    unsigned int address = EEPROM_ALARM_BOUNDS_ADDRESS;
    value = EEPROM.get(address, value);
    if (value != _alarmBoundWaterTemperatureMax)
      EEPROM.put(address, _alarmBoundWaterTemperatureMax);
    address+= sizeof(float);

    value = EEPROM.get(address, value);
    if (value != _alarmBoundWaterTemperatureMin)
      EEPROM.put(address, _alarmBoundWaterTemperatureMin);
    address+= sizeof(float);

    value = EEPROM.get(address, value);
    if (value != _alarmBoundWaterPhMax)
      EEPROM.put(address, _alarmBoundWaterPhMax);
    address+= sizeof(float);

    value = EEPROM.get(address, value);
    if (value != _alarmBoundWaterPhMin)
      EEPROM.put(address, _alarmBoundWaterPhMin);
    address+= sizeof(float);
    
    // // saves pressure reference
    // value = EEPROM.get(address, value);
    // if (value != _measuredAirPressure.average)
    //   EEPROM.put(address, _measuredAirPressure.average);
    
    if (savingTimers)
      saveTimers();

  #endif
}


void saveTimers() {
  //
  // saves timers configurations on EEPROM
  //
  #if (USE_EEPROM == true)      
    unsigned int n_timers = getTimersCount();
    #if (DEBUG_MODE == true)
      Serial.print(F("Saving "));
      Serial.print(n_timers);
      Serial.println(F(" timers at EEPROM"));
    #endif
    
    unsigned int address = EEPROM_TIMERS_ADDRESS;
    EEPROM.update(address++, n_timers);

    for (unsigned int i = 0; i < TIMERS_SIZE; i++) {     
      // for each timer, check if its active (and valid)
      if (RELAY_TIMERS[i].active && is_valid_timer(RELAY_TIMERS[i])) {
        byte relayIndex = RELAY_TIMERS[i].relay;
        if (RELAY_TIMERS[i].enabled)
          bitWrite(relayIndex, 7, 1);
        EEPROM.update(address++, relayIndex);
        EEPROM.update(address++, RELAY_TIMERS[i].turnOnHour);
        EEPROM.update(address++, RELAY_TIMERS[i].turnOnMinute);
        EEPROM.update(address++, RELAY_TIMERS[i].turnOffHour);
        EEPROM.update(address++, RELAY_TIMERS[i].turnOffMinute);
      }
    }

    #if (DEBUG_MODE == true)
      Serial.println(F("saved timers:"));
      printTimers();
    #endif

  #endif
}

void loadDefaultConfigurations() {
  //
  // loads default configurations
  //
  #if (DEBUG_MODE == true)
    Serial.println(F("Loading default configurations"));
  #endif

  _isTimersEnabled = USE_STANDALONE_TIMERS;
  _isFeedingTimersEnabled = ENABLE_FEEDING_TIMER;

  // default alarms bounds
  _alarmBoundWaterTemperatureMax = DEFAULT_ALARM_BOUND_TEMP_MAX;
  _alarmBoundWaterTemperatureMin = DEFAULT_ALARM_BOUND_TEMP_MIN;
  _alarmBoundWaterPhMax = DEFAULT_ALARM_BOUND_PH_MAX;
  _alarmBoundWaterPhMin = DEFAULT_ALARM_BOUND_PH_MIN;
  
  // default timers
  loadDefaultTimers();
}

void loadDefaultTimers(){
  //
  // loads default timers configurations
  //
  cleanTimers();

  unsigned int index = getRelayIndexByPin(RELAY_LIGHTS_PIN);
  RELAY_TIMERS[0].enabled = true; // LIGHTS: 06:00-23:00h
  RELAY_TIMERS[0].relayIndex = index; 
  RELAY_TIMERS[0].turnOnHour = DEFAULT_TIMER_LIGHT_ON;
  RELAY_TIMERS[0].turnOnMinute = 0;
  RELAY_TIMERS[0].turnOffHour = DEFAULT_TIMER_LIGHT_OFF;
  RELAY_TIMERS[0].turnOffMinute = 0;
  RELAY_TIMERS[0].active = true;

  index = getRelayIndexByPin(RELAY_FILTER_UV_PIN);
  RELAY_TIMERS[1].enabled = true; // UV FILTER: 08:00-15:00h
  RELAY_TIMERS[1].relayIndex = index;
  RELAY_TIMERS[1].turnOnHour = DEFAULT_TIMER_UV_ON;
  RELAY_TIMERS[1].turnOnMinute = 0;
  RELAY_TIMERS[1].turnOffHour = DEFAULT_TIMER_UV_OFF;
  RELAY_TIMERS[1].turnOffMinute = 0;
  RELAY_TIMERS[1].active = true;
  
  index = getRelayIndexByPin(RELAY_FEEDER_PIN);
  RELAY_TIMERS[2].relayIndex = index;  //FEED 07h
  RELAY_TIMERS[2].turnOnHour = DEFAULT_TIMER_FEED_1;
  RELAY_TIMERS[2].turnOnMinute = 00;
  RELAY_TIMERS[2].turnOffHour = 00;
  RELAY_TIMERS[2].turnOffMinute = 00;
  RELAY_TIMERS[2].active = true;
  
  RELAY_TIMERS[3].relayIndex = index; //FEED 12h
  RELAY_TIMERS[3].turnOnHour = DEFAULT_TIMER_FEED_2;
  RELAY_TIMERS[3].turnOnMinute = 00;
  RELAY_TIMERS[3].turnOffHour = 00;
  RELAY_TIMERS[3].turnOffMinute = 00;
  RELAY_TIMERS[3].active = true;
  
  RELAY_TIMERS[4].relayIndex = index; //FEED 18h
  RELAY_TIMERS[4].turnOnHour = DEFAULT_TIMER_FEED_3;
  RELAY_TIMERS[4].turnOnMinute = 00;
  RELAY_TIMERS[4].turnOffHour = 00;
  RELAY_TIMERS[4].turnOffMinute = 00;
  RELAY_TIMERS[4].active = true;

  #if (DEBUG_MODE == true)
    Serial.println(F("default timers set (factory reset):"));
    printTimers();
  #endif
}
//--------------------------------------------------------------------------------------------------
//
// reading sensors
//
//--------------------------------------------------------------------------------------------------

void readData() {
  //
  // read sensors data
  //
  #if (USE_STANDALONE_TIMERS == true)
    // read real time clock
    _now = _clock.now();
  #else
    // testing value
    _now = DateTime(2000, 1, 1, 12, 30, 0);
  #endif

  //
  // pH sensor
  //
  #if (USE_WATER_PH_SENSOR == true)
    updateMeasure(_measuredWaterPh, 7.8);
  #else
    // testing value
    updateMeasure(_measuredWaterPh, 7.8);
  #endif

  //
  // water level sensors
  //
  #if (USE_WATER_LEVEL_SENSORS == true)
    // level sensor open (false) => LevelLow alarm = true 
    _waterDrainLevelLow = !digitalRead(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN);
    _waterSumpLevelLow = !digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN);
  #else
    // testing values
    _waterDrainLevelLow = false;
    _waterSumpLevelLow = false;
  #endif

  //
  // water temperature sensor
  //
  #if (USE_WATER_TEMPERATURE_SENSOR == true)
    _sensorWaterTemperature.requestTemperatures();
    updateMeasure(_measuredWaterTemperature, _sensorWaterTemperature.getTempCByIndex(0));        
    
    #if (USE_HOME_ASSISTANT == true)       
      sensorWaterTemp.setValue(_measuredWaterTemperature.average);
    #endif

  #else
    // testing value
    updateMeasure(_measuredWaterTemperature, 27.2);
  #endif

  // 
  // DTH11 temperature & humidity sensor
  //
  #if (USE_AIR_TEMPERATURE_SENSOR == true)
    updateMeasure(_measuredAirTemperature, _sensorAirTemperature.readTemperature());
    updateMeasure(_measuredAirHumidity, _sensorAirTemperature.readHumidity());
        
    #if (USE_HOME_ASSISTANT == true) 
      sensorAirTemp.setValue(_measuredAirTemperature.average);
      sensorAirHumidity.setValue(_measuredAirHumidity.average);
    #endif

  #else
    // testing values
    updateMeasure(_measuredAirTemperature, 27.2);
    updateMeasure(_measuredAirHumidity, 60.3);
  #endif

  // 
  // pressure sensor
  //
  #if (USE_AIR_PRESSURE_SENSOR == true)
    // air pressure
    updateMeasure(_measuredAirPressure, _sensorAirPresure.readPressure());
    updateMeasure(_measuredAirAltitude, _sensorAirPresure.readAltitude());
    updateMeasure(_measuredAirSealevelPressure, _sensorAirPresure.readSealevelPressure());
    
    #if (USE_HOME_ASSISTANT == true) 
      sensorAirPressure.setValue(_measuredAirPressure.average);
    #endif

  #else
    // testing values
    updateMeasure(_measuredAirPressure, 1000);
    updateMeasure(_measuredAirAltitude, 10);
    updateMeasure(_measuredAirSealevelPressure, 1000);
  #endif

}

bool readButtons() {
  //
  // reads buttons
  //
  #if (USE_PUSH_BUTTONS == true)
    
    if (digitalRead(PUSH_BUTTON_TURN_OFF_PIN) == LOW) {
      turnOff();
      return true;
    }
    if (digitalRead(PUSH_BUTTON_RESTART_PIN) == LOW) {
      restart();
      return true;
    }
    if (digitalRead(PUSH_BUTTON_LIGHTS_PIN) == LOW) {
      setRelay(RELAY_LIGHTS_PIN, true, false, true);
      return true;
    }
    if (digitalRead(PUSH_BUTTON_PWC_ROUTINE_PIN) == LOW) {
      runWaterParcialChangeRoutine();
      return true;
    }
    if (digitalRead(PUSH_BUTTON_MANUAL_CLEANING_PIN) == LOW) {
      runManualCleaning();
      return true;
    }
    if (digitalRead(PUSH_BUTTON_FEEDING_PIN) == LOW) {
      feed();
      return true;
    }

  #endif

  // no button pressed
  return false;
}

//--------------------------------------------------------------------------------------------------
//
// display
//
//--------------------------------------------------------------------------------------------------

void showDateTime() {
  //
  // Shows current date and time
  //
  #if (USE_RTC_CLOCK == true)
    DateTime now = _clock.now();  
    unsigned int hour = now.hour();

    #if (DEBUG_MODE == true)
      // shows date time at serial (debug)
      Serial.print(now.day(), DEC);
      Serial.print("/");
      Serial.print(now.month(), DEC);
      Serial.print("/");
      Serial.print(now.year(), DEC);
      Serial.print(" ");
      Serial.print(now.hour(), DEC);
      Serial.print(":");
      Serial.println(now.minute(), DEC);
      // shows greetings message
      if (hour > 6 && hour < 12) 
        Serial.println(TEXT_GOOD_MORNING);       
      else if (hour >= 12 && hour < 18) 
        Serial.println(TEXT_GOOD_AFTERNOON);      
      else 
        Serial.println(TEXT_GOOD_NIGHT);      
    #endif

    #if (USE_LCD_DISPLAY == true)
      // shows date time at LCD display
      _lcd.setCursor(0, 0);
      _lcd.print(now.day(), DEC);
      _lcd.print("/");
      _lcd.print(now.month(), DEC);
      _lcd.print("/");
      _lcd.print(now.year(), DEC);
      _lcd.print(" ");
      _lcd.print(now.hour(), DEC);
      _lcd.print(":");
      _lcd.print(now.minute(), DEC);
      _lcd.setCursor(0, 1);
      // shows greetings message
      if (hour > 6 && hour < 12) 
        _lcd.print(TEXT_GOOD_MORNING);       
      else if (hour >= 12 && hour < 18) 
        _lcd.print(TEXT_GOOD_AFTERNOON);      
      else 
        _lcd.print(TEXT_GOOD_NIGHT);      
    #endif

  #endif
}

void showAirTemperature1() {
  //
  // Shows atmosferic data (1):
  // - air temperature
  // - humidity
  //
  #if (DEBUG_MODE == true)    
    Serial.print(TEXT_AIR_TEMPERATURE);
    printMeasure(_measuredAirTemperature);

    Serial.print(TEXT_AIR_HUMIDITY);
    printMeasure(_measuredAirHumidity);
  #endif

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_AIR_TEMPERATURE);
    _lcd.print(_measuredAirTemperature.value);
    _lcd.print(str_unit_temperature);
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_AIR_HUMIDITY);
    _lcd.print(_measuredAirHumidity.value);
    _lcd.print(F("%"));
  #endif
}

void showAirTemperature2() {
  //
  // Shows atmosferic data (2):
  // - atmosferic pressure
  // - altitude
  //
  #if (DEBUG_MODE == true)    
    Serial.print(TEXT_AIR_PRESSURE);
    printMeasure(_measuredAirPressure);

    Serial.print(TEXT_ALTITUDE);
    printMeasure(_measuredAirAltitude);
  #endif

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_AIR_PRESSURE);
    _lcd.print(_measuredAirPressure.value / 100, 0);
    _lcd.print(F("mPa"));
    
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_ALTITUDE);
    _lcd.print(_measuredAirAltitude.average, 0);
    _lcd.print(F("m"));
  #endif
}

void showLocalWeatherForecast() {
  //
  // shows local weater forecast
  //
  #if (ENABLE_WEATHER_STATION == true)
    unsigned int forecast = getStandaloneWeatherForecast();

    #if (DEBUG_MODE == true)
      //
      // debug
      //
      Serial.print(TEXT_LOCAL_FORECAST);
      switch (forecast) {
        case WEATHER_CODE_GOOD:
          Serial.println(TEXT_WEATHER_GOOD);
          break;
        case WEATHER_CODE_DRY:
          Serial.println(TEXT_WEATHER_DRY);
          break;
        case WEATHER_CODE_LIGHT_RAIN:
          Serial.println(TEXT_WEATHER_LIGHT_RAIN);  
          break;
        case WEATHER_CODE_RAINING:
          Serial.println(TEXT_WEATHER_RAINING);  
          break;
        case WEATHER_CODE_HEAVY_RAIN:
          Serial.println(TEXT_WEATHER_HEAVY_RAIN);  
          break;
        case WEATHER_CODE_TUNDERSTORM:
          Serial.println(TEXT_WEATHER_TUNDERSTORM);  
          break;
        case WEATHER_CODE_NO_CHANGES:
          Serial.println(TEXT_WEATHER_NO_CHANGES);
          break;
        default:
          Serial.println(TEXT_WEATHER_NO_FORECAST);
          break;
      }  
    #endif

    #if (USE_LCD_DISPLAY == true)
      //
      // LCD display
      //
      _lcd.setCursor(0, 0);
      _lcd.print(TEXT_LOCAL_FORECAST);
      _lcd.setCursor(0, 1);
      switch (forecast) {
        case WEATHER_CODE_GOOD:
          _lcd.print(TEXT_WEATHER_GOOD);
          break;
        case WEATHER_CODE_DRY:
          _lcd.print(TEXT_WEATHER_DRY);
          break;
        case WEATHER_CODE_LIGHT_RAIN:
          _lcd.print(TEXT_WEATHER_LIGHT_RAIN);  
          break;
        case WEATHER_CODE_RAINING:
          _lcd.print(TEXT_WEATHER_RAINING);  
          break;
        case WEATHER_CODE_HEAVY_RAIN:
          _lcd.print(TEXT_WEATHER_HEAVY_RAIN);  
          break;
        case WEATHER_CODE_TUNDERSTORM:
          _lcd.print(TEXT_WEATHER_TUNDERSTORM);  
          break;
        case WEATHER_CODE_NO_CHANGES:
          _lcd.print(TEXT_WEATHER_NO_CHANGES);
          break;
        default:
          _lcd.print(TEXT_WEATHER_NO_FORECAST);
          break;
      }  
    #endif

  #endif
}

void showWaterTemperature(int idx) {
  //
  // Shows temperature and specified statistic
  // 
  // parameters:
  //    idx: 0 = shows min, 1 = shows max, 2 = shows average
  //  
  #if (DEBUG_MODE == true)
    Serial.print(TEXT_WATER_TEMPERATURE);
    printMeasure(_measuredWaterTemperature);
  #endif

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_WATER_TEMPERATURE);
    _lcd.print(_measuredWaterTemperature.value);
    _lcd.print(F("C"));
    _lcd.setCursor(0, 1);
    switch (idx) {
      case 1:
        _lcd.print(TEXT_MIN);
        _lcd.print(_measuredWaterTemperature.min);
        break;
      case 2:
        _lcd.print(TEXT_MAX);
        _lcd.print(_measuredWaterTemperature.max);
        break;
      case 3:
        _lcd.print(TEXT_AVG);
        _lcd.print(_measuredWaterTemperature.average);
        break;
    }
    _lcd.print(F("C"));
  #endif
}

void showWaterPh() {
  //
  // Shows Ph
  //
  #if (DEBUG_MODE == true)
    Serial.print(TEXT_WATER_PH);
    printMeasure(_measuredWaterPh);
  #endif

  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_WATER_PH);
    _lcd.print(_measuredWaterPh.value);
    _lcd.setCursor(0, 1);
    _lcd.print(F("Alcalin: 120 ppm"));
  #endif  
}

void showSystemFlags() {
  //
  // Shows system flags
  //
  unsigned int index = getRelayIndexByPin(RELAY_LIGHTS_PIN);
  bool lightsForcedOn = RELAY_FORCED_TURN_ON[index];
  
  #if (DEBUG_MODE == true)
    Serial.print(TEXT_LIGHTS);
    Serial.println((lightsForcedOn ? TEXT_ON : TEXT_TIMER));
        
    Serial.print(TEXT_FEED);
    // feeder timer is forced off: only feed with buttons
    Serial.println((_isFeedingTimersEnabled ? TEXT_OFF: TEXT_FEED)); 
  #endif 

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_LIGHTS);
    _lcd.print((lightsForcedOn ? TEXT_ON : TEXT_TIMER));
    
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_FEED);
    // feeder timer is forced off: only feed with buttons
    _lcd.print((_isFeedingTimersEnabled ? TEXT_OFF : TEXT_TIMER)); 
  #endif  
}

//--------------------------------------------------------------------------------------------------
//
// weather forecasting
//
//--------------------------------------------------------------------------------------------------

unsigned int getStandaloneWeatherForecast(){
  //
  // gets standalone weather forecast
  //
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 600)  
    return WEATHER_CODE_NO_FORECAST;

  if (_measuredAirPressure.max - _measuredAirPressure.average >= 550 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 599) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 500 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 549) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 450 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 499) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 400 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 449) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 350 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 399) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 300 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 349) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.max - _measuredAirPressure.average >= 200 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 299) 
    return WEATHER_CODE_DRY;

  if (_measuredAirPressure.max - _measuredAirPressure.average >= 100 
    && _measuredAirPressure.max - _measuredAirPressure.average <= 199) 
    return WEATHER_CODE_NO_CHANGES;

  if (_measuredAirPressure.max - _measuredAirPressure.min >= 200 
    && _measuredAirPressure.average - _measuredAirPressure.min <= 299) 
    return WEATHER_CODE_LIGHT_RAIN;

  if (_measuredAirPressure.max - _measuredAirPressure.min >= 300 
    && _measuredAirPressure.average - _measuredAirPressure.min <= 399) 
    return WEATHER_CODE_RAINING;
  
  if (_measuredAirPressure.max - _measuredAirPressure.min >= 400 
    && _measuredAirPressure.average - _measuredAirPressure.min <= 499) 
    return WEATHER_CODE_HEAVY_RAIN;
  
  if (_measuredAirPressure.max - _measuredAirPressure.min >= 500 
    && _measuredAirPressure.average - _measuredAirPressure.min <= 599) 
    return WEATHER_CODE_TUNDERSTORM;
  
  if (_measuredAirPressure.max - _measuredAirPressure.min >= 600)  
    return WEATHER_CODE_NO_FORECAST;
  
  if (_measuredAirHumidity.average < 30)
    return WEATHER_CODE_DRY;

  return WEATHER_CODE_NO_FORECAST;
}

//--------------------------------------------------------------------------------------------------
//
// timers
//
//--------------------------------------------------------------------------------------------------
#if (USE_STANDALONE_TIMERS == true)

  bool checkInRange(const DateTime& now, const timer& t, const bool& checkTurnOnOnly = false) {
    //
    // returns True if the specified time ("now") is between timer "on" window
    //
    // "same day" timer, eg:
    //   - current time:  05:00 am (outside range)
    //   - turn on:  07:00 am
    //   - current time:  08:00 am (inside range)
    //   - turn off: 10:00 am
    //   - current time:  11:00 am (outside range)
    //
    // "next day" timer, eg:
    //   - current time:  19:00 pm (outside range)
    //   - turn on:  20:00 pm
    //   - current time:  23:00 pm (inside range)
    //   - current time:  03:00 am (inside range)
    //   - turn off: 05:00 am (next day)
    //   - current time:  08:00 am (outside range)
    //
    unsigned int hour = now.hour(); 
    unsigned int minute = now.minute();    
    
    
    if (checkTurnOnOnly) {
      // only checks for start hour:minute
      return hour == t.turnOnHour && minute == t.turnOnMinute;
    }

    if (hour < t.turnOnHour || hour > t.turnOffHour)
      // out of "turn on" hours
      return false;
        
    // checks if before initial "turn on" minute
    if (hour == t.turnOnHour && minute < t.turnOnMinute)
      return false;

    // checks if after final "turn on" minute
    if (hour == t.turnOffHour && minute >= t.turnOffMinute)
      return false;

    return true;
  }

  void runTimers() {
    //
    // execute the standalone timers
    //  
    if (!_isTimersEnabled)
      // nothing to do!
      return;

    if (_status >= STATUS_CYCLE_OFF) {
      // non usual mode: just ensures UV filter is turned off
      setRelay(RELAY_FILTER_UV_PIN, false);
      return;
    }

    #if (USE_RTC_CLOCK == true)
      if (_clock.lostPower()) {
        // could not check timers without clock time!!!
        #if (DEBUG_MODE == true) 
          Serial.println(F("WARNING: unreliable RTC (lost power status) checking timers"));
        #endif
        return;
      }
      // gets current date & time (happy day)
      DateTime now = _clock.now();
    #else
      #if (DEBUG_MODE == true) 
        // DEBUG ONLY: testing timers without RTC connected
        Serial.println(F(" - checking timers - "));
        DateTime now = DateTime(2023, 9, 1, 6, 1, 9);  
      #else
        // no RTC / no debug: nothing to do!
        return;
      #endif
    #endif


    bool shouldFeed = false;
    for (unsigned int i = 0; i < TIMERS_SIZE; i++) {    
      // for each timer, check if its enabled, active and valid
      if (RELAY_TIMERS[i].enabled 
        && RELAY_TIMERS[i].active 
        && is_valid_timer(RELAY_TIMERS[i])) {

        // retrieves relay index and pin
        unsigned int relayIndex = RELAY_TIMERS[i].relayIndex;
        unsigned int relayPin = RELAY_PINS[relayIndex];

        //
        // handles special cases first!
        //
        #if (ENABLE_LIGHTS_TIMER == false)
          if (relayPin == RELAY_LIGHTS_PIN)
            // disabled
            continue;
        #endif
        #if (ENABLE_UV_FILTER_TIMER == false)
          if (relayPin == RELAY_FILTER_UV_PIN)
            // disabled
            continue;
        #endif
        #if (ENABLE_FEEDING_TIMER == false)
          if (relayPin == RELAY_FEEDER_PIN)
            // disabled
            continue;
        #else          
          #if (ENABLE_FEEDING == true)
            // feeder relay behaves different on ENABLE_FEEDING mode
            if (relayPin == RELAY_FEEDER_PIN) {
              shouldFeed = shouldFeed || checkInRange(now, RELAY_TIMERS[i], true);
              continue;
            }
          #endif
        #endif

        // checks if the relay should be on (happy day)
        bool shouldBeOn = checkInRange(now, RELAY_TIMERS[i]) || RELAY_FORCED_TURN_ON[relayIndex];
        
        if (!RELAY_STATES[relayIndex] && shouldBeOn){
          // turns on relay (should be on and is currently off)
          setRelay(relayIndex, true, true);
          //updateStates();

        } else if (RELAY_STATES[relayIndex] && !shouldBeOn){
          // turns off relay (should be off and is currently on)
          setRelay(relayIndex, false, true);
          //updateStates();
        }
      }
    }

    #if (ENABLE_FEEDING == true)
      //
      // ensures that feeding routine will be called ONE time
      // per timer using the "_statusFeeded" semaphore!
      //
      if (shouldFeed){        
        if (!_statusFeeded)
          feed();
      }
      else
        _statusFeeded = false;
    #endif
  }

  
  void cleanTimers() {
    //
    // reset all timers! 
    //
    for (unsigned int i = 0; i < TIMERS_SIZE; i++) {
      RELAY_TIMERS[i].enabled = false;
      RELAY_TIMERS[i].relayIndex = 0xFF;
      RELAY_TIMERS[i].turnOnHour = 0x00;
      RELAY_TIMERS[i].turnOnMinute = 0x00;
      RELAY_TIMERS[i].turnOffHour = 0x00;
      RELAY_TIMERS[i].turnOffMinute = 0x00;
      RELAY_TIMERS[i].active = false;
    }

    #if (DEBUG_MODE == true) 
      Serial.println(F("timers cleaned"));
      printTimers();
    #endif
  }

  unsigned int getTimersCount() {
    //
    // returns the number of active timers
    //    
    unsigned int count = 0;
    for (unsigned int i = 0; i < TIMERS_SIZE; i++) {     
      // for each timer, check if its active (and valid)
      if (RELAY_TIMERS[i].active && is_valid_timer(RELAY_TIMERS[i]))
        count++;
    }

    return count;
  }

  void enableTimer(unsigned int index) {
    //
    // enables timer
    //
    if (index >= TIMERS_SIZE || (RELAY_TIMERS[index].enabled && RELAY_TIMERS[index].active))
      // invalid or already enabled    
      return;

    #if (DEBUG_MODE == true) 
      Serial.print(F(" - Enabling timer: "));
      Serial.println(index);    
    #endif

    RELAY_TIMERS[index].enabled = true;
    RELAY_TIMERS[index].active = true;
    saveTimers();
  }


void disableTimer(unsigned int index) {
  //
  // enables timer new timer
  //
  if (index >= TIMERS_SIZE || !RELAY_TIMERS[index].enabled)
    // invalid or already disabled
    return;

  #if (DEBUG_MODE == true) 
    Serial.print(F(" - Disabling timer: "));
    Serial.println(index);
  #endif

  RELAY_TIMERS[index].enabled = false;
  RELAY_TIMERS[index].active = true;
  saveTimers();
}


void deleteTimer(unsigned int index) {
  //
  // enables timer new timer
  //
  if (index >= TIMERS_SIZE || !RELAY_TIMERS[index].active)
    return;

  #if (DEBUG_MODE == true) 
    Serial.print(F(" - Deleting timer: "));
    Serial.println(index);    
  #endif

  RELAY_TIMERS[index].active = false;
  saveTimers();
  loadTimers();
}


bool addTimer(unsigned int relayPin, const byte& hourOn, const byte& minuteOn, 
  const byte& hourOff, const byte& minuteOff) {
  //
  // adds new timer
  //
  unsigned int relayIndex = getRelayIndexByPin(relayPin);
  if ((hourOn == hourOff && minuteOn == minuteOff) || relayIndex >= RELAY_SIZE) {
    #if (DEBUG_MODE == true) 
      Serial.print(F("ERROR creating timer: invalid parameters!"));
    #endif
    return false;
  }

  for (unsigned int i = 0; i < TIMERS_SIZE; i++) {
    if (!RELAY_TIMERS[i].active){
      // adds on first free position
      RELAY_TIMERS[i].enabled = true;
      RELAY_TIMERS[i].relayIndex = (byte)relayIndex;
      RELAY_TIMERS[i].turnOnHour = hourOn;
      RELAY_TIMERS[i].turnOnMinute = minuteOn;
      RELAY_TIMERS[i].turnOffHour = hourOff;
      RELAY_TIMERS[i].turnOffMinute = minuteOff;
      RELAY_TIMERS[i].active = true;
      
      // save changes
      saveTimers();

      #if (DEBUG_MODE == true) 
        // debug only
        Serial.print(F("new timer created "));
        printTimer(RELAY_TIMERS[i], i);
      #endif
      return true;
    }
  }  

  #if (DEBUG_MODE == true) 
    Serial.print(F("could not save timer: no free space!"));
  #endif
  return false;
}


bool updateTimer(unsigned int timerId, const byte& hourOn, const byte& minuteOn, 
  const byte& hourOff, const byte& minuteOff) {
  //
  // updates the specified timer
  //
  if (timerId >= TIMERS_SIZE || (hourOn == hourOff && minuteOn == minuteOff)){
    #if (DEBUG_MODE == true) 
      Serial.print(F("ERROR updating timer: invalid parameters!"));
    #endif
    return false;
  }

  #if (DEBUG_MODE == true) 
    Serial.print(F("updating timer - id: "));
    Serial.println(timerId);
  
    Serial.print(F("before "));
    printTimer(RELAY_TIMERS[timerId], timerId);
  #endif

  RELAY_TIMERS[timerId].enabled = true;
  RELAY_TIMERS[timerId].turnOnHour = hourOn;
  RELAY_TIMERS[timerId].turnOnMinute = minuteOn;
  RELAY_TIMERS[timerId].turnOffHour = hourOff;
  RELAY_TIMERS[timerId].turnOffMinute = minuteOff;
  RELAY_TIMERS[timerId].active = true;
  saveTimers();

  #if (DEBUG_MODE == true) 
    Serial.print(F("after "));
    printTimer(RELAY_TIMERS[timerId], timerId);
  #endif

  return true;
}

#if (DEBUG_MODE == true)
    //
    // debug only code
    //

    void printTimers(){
      //
      // print all timers
      //
      for(unsigned int i = 0; i < TIMERS_SIZE; i++) {
        if (RELAY_TIMERS[i].active && is_valid_timer(RELAY_TIMERS[i]))
          printTimer(RELAY_TIMERS[i], i);
      }
    }

    void printTimer(const timer& timer, const unsigned int& idx){
      //
      // print timers
      //
      unsigned int relayPin = 0;
      String relayName = String(TEXT_UNKNOW);
      if (timer.relayIndex < RELAY_SIZE){
          relayPin = RELAY_PINS[timer.relayIndex];
          relayName = getRelayNameByPin(relayPin);
      }
      Serial.print(F(" - timer id: "));
      Serial.print(idx);
      Serial.print(F(" ["));
      Serial.print(timer.enabled ? F("on") :  F("off"));
      Serial.print(F("], relay: "));
      Serial.print(relayName);
      Serial.print(F("[idx: "));
      Serial.print((unsigned int)timer.relayIndex);
      Serial.print(F(", pin: "));
      Serial.print(relayPin);
      Serial.print(F("], timer on: "));
      Serial.print((unsigned int)timer.turnOnHour);
      Serial.print(F(":"));
      Serial.print((unsigned int)timer.turnOnMinute);
      if (!ENABLE_FEEDING || relayPin != RELAY_FEEDER_PIN){
        Serial.print(F(", timer off: "));
        Serial.print((unsigned int)timer.turnOffHour);
        Serial.print(F(":"));
        Serial.println((unsigned int)timer.turnOffMinute);
      }
    }

  #endif

#endif

//--------------------------------------------------------------------------------------------------
//
// alarms
//
//--------------------------------------------------------------------------------------------------
#if (ENABLE_SENSOR_ALARMS == true)
  
  void handleAlarms()
  {
    //
    // Take sensor alarm actions: changes states realys in proper response!!
    //
    if(_status >= STATUS_CYCLE_OFF) 
      // nothing to do: relays disabled or aquarioino turned off
      return;
    
    #if (USE_WATER_LEVEL_SENSORS == true)
      //
      // water reposition (due to evaporation)
      // sump pump water level opened (false) -> alarm sump water level is low (true)
      //
      _waterSumpLevelLow = !digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN); 
    #else
      _waterSumpLevelLow = false;
    #endif

    #if (DEBUG_MODE == true) 
      Serial.println(F("low water level at sump sensor - turning on water reposition pump (fresh water in)"));
    #endif
    
    #if (USE_RELAYS == true) 
      //the  water level on aquarium is low: turn on water reposition 
      // pump (i.e., push some new water into aquarium)
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, _waterSumpLevelLow);
    #endif

    #if (USE_WATER_TEMPERATURE_SENSOR == true)
      //
      // high temperature alarm!
      // set pin=LOW to turn on relay
      // if high temp (true) -> turn on cooler, turn of heater -> set cooler pin LOW, set heater pin HIGH 
      //
      bool highTemp = _measuredWaterTemperature.value > _alarmBoundWaterTemperatureMax;      
    #else
      bool highTemp = false;
    #endif
    
    #if (DEBUG_MODE == true) 
      Serial.println(F("high temperature - turning on cooler fan and turning off heater"));
    #endif

    #if (USE_RELAYS == true)     
      // too hot: turns on cooler and turns off heater
      setRelay(RELAY_COOLER_FAN_PIN, highTemp);
      setRelay(RELAY_HEATER_PIN, !highTemp);
    #endif
  }

  void showAlarms() {
    //
    // shows alarms (second LCD display line)
    //
    if (_status < STATUS_CYCLE_OFF ) {

        #if (DEBUG_MODE == true)  
        // display alarms (in priority order)  
        if (_waterSumpLevelLow && USE_WATER_LEVEL_SENSORS) {
          // low level alarm
          Serial.print(F("ALARM: low water level: "));
        }
        else if (_measuredWaterTemperature.value > _alarmBoundWaterTemperatureMax) {
          // temperature alarms: high
          Serial.print(F("ALARM: high water temperature: "));
          printMeasure(_measuredWaterTemperature);
        }
        else if (_measuredWaterTemperature.value < _alarmBoundWaterTemperatureMin) {
          // temperature alarms: low
          Serial.print(F("ALARM: low water temperature: "));
          printMeasure(_measuredWaterTemperature);
        }
        else if (_measuredWaterPh.value < _alarmBoundWaterPhMin) {
          // pH alarms: low
          Serial.print(F("ALARM: low pH: "));
          printMeasure(_measuredWaterPh);
        }  
        else if (_measuredWaterPh.value > _alarmBoundWaterPhMax) {
          // pH alarms: high
          Serial.print(F("ALARM: high pH: "));
          printMeasure(_measuredWaterPh);
        }
      #endif

      #if (USE_LCD_DISPLAY == true)  
        _lcd.setCursor(0, 1);
        // display alarms (in priority order)  
        if (_waterSumpLevelLow && USE_WATER_LEVEL_SENSORS) {
          // low level alarm
          _lcd.print(TEXT_ALARM_LOW_WATER_LEVEL);
        }
        else if (_measuredWaterTemperature.value > _alarmBoundWaterTemperatureMax) {
          // temperature alarms: high
          _lcd.print(TEXT_ALARM_HIGH_TEMPERATURE);
          _lcd.print(_measuredWaterTemperature.value);
          _lcd.print(F("C"));
        }
        else if (_measuredWaterTemperature.value < _alarmBoundWaterTemperatureMin) {
          // temperature alarms: low
          _lcd.print(TEXT_ALARM_LOW_TEMPERATURE);
          _lcd.print(_measuredWaterTemperature.value);
          _lcd.print(F("C"));
        }
        else if (_measuredWaterPh.value < _alarmBoundWaterPhMin) {
          // pH alarms: low
          _lcd.print(TEXT_ALARM_LOW_PH);
          _lcd.print(_measuredWaterPh.value);
        }  
        else if (_measuredWaterPh.value > _alarmBoundWaterPhMax) {
          // pH alarms: high
          _lcd.print(TEXT_ALARM_HIGH_PH);
          _lcd.print(_measuredWaterPh.value);
        }
      #endif
    
      delay(100);
    }
  }
#endif
//--------------------------------------------------------------------------------------------------
//
// aquarium maintenance routines (PWC, manual cleaning)
//
//--------------------------------------------------------------------------------------------------
void runWaterParcialChangeRoutine() {
  //
  // Auto Water Parcial Change
  //
  delay(1000);
  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_PWC_STARTING);
    delay(1000);  
  #endif 
  
  if (!setAquariumState(STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE))
    // error setting state
    return; 

  //
  // step 1 - drain dirty water (turn on drain pump - see docs above)
  //
  #if (DEBUG_MODE == true)
    Serial.println(TEXT_DRAINING_WATER);
  #endif 
  
  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_DRAINING_WATER);
    delay(500);
  #endif 

  #if (USE_RELAYS == true)
    // turn off all (keep only lights on)
    unsigned int light_index = getRelayIndexByPin(RELAY_LIGHTS_PIN);
    setAllRelays(false, light_index, true); 
    delay(1000);

    setRelay(RELAY_DRAIN_PUMP_PIN, true);
    while (digitalRead(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN)) {
      // while not reach drain level low => keep draining dirty water
      delay(500);
    }

    // turn off drain pump (water is at low drained level) 
    setRelay(RELAY_DRAIN_PUMP_PIN, false);
    delay(1000);
  
    // ensures low water level at sump pump!
    setRelay(RELAY_SUMP_PUMP_PIN, true);
    while (digitalRead(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN)) {
      // while not reach sump level low => keep sump pump on
      delay(500);
    }
    delay(1000);

    // turn of sump pump (sump is at low level)
    setRelay(RELAY_SUMP_PUMP_PIN, false);  
    delay(1000);
  #endif
  
  //
  // step 2 - fills fresh water (turn on reposition pump)
  //
  #if (DEBUG_MODE == true)
    Serial.println(TEXT_DRAINING_WATER);
  #endif 

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_REPLACING_WATER);
    delay(1000);
  #endif 
  
  #if (USE_RELAYS == true)
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, true);
    while (!digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN)) {
      // while not reach sump level high => keep reposition pump on
      delay(500);
    }

    // turn of reposition pump (water flows to sump)
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
    delay(1000);
  #endif

  //
  // ensures sump water level, WITH sump pump on
  //
  #if (DEBUG_MODE == true)
    Serial.println(TEXT_CHECKING);
  #endif 

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_CHECKING);
    delay(1000);
  #endif   

  #if (USE_RELAYS == true)
    // turn on sump pump again (running water on sump)
    setRelay(RELAY_SUMP_PUMP_PIN, true);
    // wait some time to sump pump remove some water
    delay(8000);
    
    while (!digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN)) {
      // keep adding water util its ok with the sump pump turned on
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, true);
      delay(500);
    }

    // turn off both pumps
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
    setRelay(RELAY_DRAIN_PUMP_PIN, false);
  #endif

  #if (DEBUG_MODE == true)
    Serial.println(TEXT_PWC_FINISHED);
  #endif 

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_PWC_FINISHED);
    delay(5000);
  #endif
  
  // restart the display cycle
  restart();
}


void runManualCleaning() {
  //
  // manual cleaning: stop all until reset
  //
  delay(1000);
  
  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_MANUAL_CLEANING);
    delay(1000);
  #endif  
  
  if (!setAquariumState(STATUS_CODE_MANUAL_CLEANING))
    // error on changing aquarium state
    return; 
  
  #if (USE_RELAYS == true)
    // turn off all relays (only lights relay keeps on)
    unsigned int light_index = getRelayIndexByPin(RELAY_LIGHTS_PIN);
    setAllRelays(false, light_index, true);
    delay(1000);    
  #endif
}

void turnOff() {
  //
  // turns off aquarioino
  //
  delay(1000);  

  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_TURN_OFF);  
    delay(1000);  
  #endif
  
  // set status OFF  
  if (!setAquariumState(STATUS_CODE_TURNED_OFF))
    // error on changing aquarium state
    return; 

  #if (USE_RELAYS == true)  
    for(unsigned int i = 0; i < RELAY_SIZE; i++)
      RELAY_FORCED_TURN_ON[i] = false;

    // turn off all relays (only lights relay keeps on)
    unsigned int light_index = getRelayIndexByPin(RELAY_LIGHTS_PIN);
    setAllRelays(false, light_index, true);
    delay(1000);
  #endif
}


void restart() {
  //
  // restarts aquarium
  //
  delay(1000);
  setAquariumState(STATUS_CYCLE_START);
  
  #if (USE_LCD_DISPLAY == true)
    _lcd.clear();
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_TURN_STARTING);
    delay(1000);
  #endif
  
  #if (USE_RELAYS == true)    
    //
    // default: turn on heater, lights and sump pump!
    //
    setRelay(RELAY_HEATER_PIN, true);
    setRelay(RELAY_LIGHTS_PIN, true);
    setRelay(RELAY_SUMP_PUMP_PIN, true);
    setRelay(RELAY_FILTER_UV_PIN, false);
    setRelay(RELAY_DRAIN_PUMP_PIN, false);    
    setRelay(RELAY_COOLER_FAN_PIN, false);
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);  
    #if (ENABLE_FEEDING == true)
      digitalWrite(RELAY_FEEDER_PIN, HIGH);      
    #else
      setRelay(RELAY_FEEDER_PIN, false); 
    #endif
    delay(100);    
  #endif
}

//--------------------------------------------------------------------------------------------------
//
// home assistant
//
//--------------------------------------------------------------------------------------------------
#if (USE_HOME_ASSISTANT == true)
  
  void sendMqttAviability() {
    //
    // Sends aviability message (MQTT)
    //
    if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
        // all sensor at same time!
        device.setAvailability(true);
        // sensorState.setAvailability(!sensorState.isOnline());
        _lastAvailabilityTime = millis();
    }
  }

  void onConnected() {
    //
    // connection to MQTT broker connection is established
    //
    mqtt.subscribe(MQTT_COMMAND_TOPIC);
    delay(100);
    
    mqtt.subscribe(MQTT_TIME_SYNC_TOPIC);
    delay(100);

    #if (DEBUG_MODE)
    Serial.println(F("Home assistant connected"));
    #endif
  }

  void onMessage(const char* topic, const uint8_t* payload, uint16_t length) {
    //
    // device receives an MQTT message
    //
    if (strcmp(topic, MQTT_TIME_SYNC_TOPIC) == 0) {  
      //
      // Date & time synchronization message from home assistant      
      // topic: ha/datetime
      // message format: "dd MM yyyy hh mm ss"
      //
      synchronizeTime(String((const char*)payload));
      return;
    }

    if (strcmp(topic, MQTT_COMMAND_TOPIC) == 0) {
      //
      // commmand topic message
      // topic: aquarioino/cmd
      //
      executeMqttCommand(String((const char*)payload));
      return;
    }
  }

  void onLightStateCommand(bool state, HALight* sender) {
    //
    // MQTT light state handler
    //    
    if (sender == &chkLightSwitch){
      setRelay(RELAY_LIGHTS_PIN, state, false, true);
    }
  }

  void onSwitchCommand(bool state, HASwitch* sender){
    //
    // MQTT switches handler
    //    
    if (sender == &chkFilterSwitch)
      setRelay(RELAY_FILTER_UV_PIN, state, false, true);      
    else if (sender == &chkHeaterSwitch)
      setRelay(RELAY_HEATER_PIN, state, false, true);
    else if (sender == &chkCoolerSwitch)
      setRelay(RELAY_COOLER_FAN_PIN, state, false, true);      
    else if (sender == &chkTimersSwitch) {
      setTimersEnabled(state);
      sender->setState(state); // report state back to the Home Assistant
      sendStatesMessage();
    }
    else if (sender == &chkFeederTimersSwitch) {
      setFeederTimersEnabled(state);
      sender->setState(state); // report state back to the Home Assistant
      sendStatesMessage();
    }        
  }

  void onButtonCommand(HAButton* sender) {
    //
    // MQTT buttons handler
    //
    if (sender == &btnTurnOff) 
        turnOff();            
    else if (sender == &btnRestart) 
        restart();    
    else if (sender == &btnManualCleaning)
        runManualCleaning();
    else if (sender == &btnStartPWC) 
        runWaterParcialChangeRoutine();
    else if (sender == &btnFeed)
        feed();
    else if (sender == &btnSave)
        saveConfigurations();
  }

  void onNumberChange(HANumeric value, HANumber* sender) {
    //
    // MQTT numeric reference values handler
    //
    if (sender == &lblTemperatureMin) {        
        _alarmBoundWaterTemperatureMin = value.toFloat();     
    }
    else if (sender == &lblTemperatureMax) {        
        _alarmBoundWaterTemperatureMax = value.toFloat();
    }
    else if (sender == &lblPhMax) {        
        _alarmBoundWaterPhMax = value.toFloat();
    }
    else if (sender == &lblPhMin) {        
        _alarmBoundWaterPhMin = value.toFloat();
    }
    else if (sender == &lblFeederTurns) {        
        _feederTurns = value.toInt16();
    }
    sender->setState(value);
  }

  String toStr(const bool& value){
    //
    // returns bool as String ("on"/"off")
    //
    return (value ? String(F("on")) : String(F("off")));
  }

  String toStr(const measure& measure, const String& label){
    //
    // returns measure as String
    //
    String json = String(F("\"[LABEL]\": [VALUE], \"[LABEL]_max\": [MAX], \"[LABEL]_min\": [MIN], \"[LABEL]_avg\": [AVG], "));
    json.replace(F("[LABEL]"), label);
    json.replace(F("[VALUE]"), String(measure.value, 1));
    json.replace(F("[AVG]"), String(measure.average, 1));
    json.replace(F("[MAX]"), String(measure.max, 1));
    json.replace(F("[MIN]"), String(measure.min, 1));
    return json;
  }
    
  void sendStatesMessage() {
    //
    // pubishes states on MQTT topic
    //
    String json = String(F("{"));
    String end = String(F(", "));
    
    json += String(F("{"));
    
    // internal states
    json += String(F("\"timers_enabled\":")) + toStr(_isTimersEnabled) + end;
    json += String(F("\"feeding_timers_enabled\":")) + toStr(_isFeedingTimersEnabled) + end;
    json += String(F("\"feeder_turns\":")) + String(_feederTurns) + end;
  
    // measurements
    json += toStr(_measuredWaterTemperature, F("temp"));
    json += toStr(_measuredWaterPh, F("ph"));
    json += toStr(_measuredAirTemperature, F("air_temp"));
    json += toStr(_measuredAirHumidity, F("humidity"));
    json += toStr(_measuredAirPressure, F("pressure"));
    json += toStr(_measuredAirAltitude, F("altitude"));
    json += toStr(_measuredAirSealevelPressure, F("sealevel_pressure"));

    // relays
    json += String(F("\"relays\": {"));    
    for (unsigned int i = 0; i < RELAY_SIZE; i++) {
      unsigned int relayPin = RELAY_PINS[i];
      if (!ENABLE_FEEDING || relayPin != RELAY_FEEDER_PIN){
        String relayStateStr = F("\"[RELAY]\": \"[STATE]\", ");
        relayStateStr.replace(F("[RELAY]"), getRelayNameByPin(relayPin));
        relayStateStr.replace(F("[STATE]"), toStr(RELAY_STATES[i]));
        json += relayStateStr;    
      }
    }          
    // timers
    json += String(F("}, \"timers\": {"));
    for (unsigned int i = 0; i < TIMERS_SIZE; i++) {
      if (RELAY_TIMERS[i].active && is_valid_timer(RELAY_TIMERS[i])) {
        String timerStr = F("\"timer_[ID]\": {\"id\": \"[ID]\", \"enabled\": \"[ENABLED]\", \"relay\": \"[RELAY]\", \"turn on\": \"[HOUR_ON]:[MIN_ON]:00\", \"turn off\": \"[HOUR_OFF]:[MIN_OFF]:00\" }, ");
        unsigned int relayIndex = RELAY_TIMERS[i].relayIndex;
        unsigned int relayPin = RELAY_PINS[relayIndex];
        String relayName = getRelayNameByPin(relayPin);
        timerStr.replace(F("[ID]"), String(i));
        timerStr.replace(F("[ENABLED]"), toStr(RELAY_TIMERS[i].enabled));
        timerStr.replace(F("[RELAY]"), relayName);
        timerStr.replace(F("[HOUR_ON]"), String(RELAY_TIMERS[i].turnOnHour));
        timerStr.replace(F("[MINUTE_ON]"), String(RELAY_TIMERS[i].turnOnMinute));
        timerStr.replace(F("[HOUR_OFF]"), String(RELAY_TIMERS[i].turnOffHour));
        timerStr.replace(F("[MINUTE_OFF]"), String(RELAY_TIMERS[i].turnOffMinute));
        json += timerStr;    
      }
    }

    json += String(F("}}"));
    //
    // TODO: const char*
    //
    mqtt.publish(MQTT_STATES_TOPIC, "test");
  }
  
  bool synchronizeTime(const String& message) {
    //
    // synchronizes RTC data & time with home assistant
    //
    // message format: "dd MM yyyy hh mm ss"
    //
    #if (DEBUG_MODE == true)
      Serial.print(F("home assistant time sync: "));
      Serial.println(message);
    #endif 
    
    #if (USE_RTC_CLOCK == true)
      // parse parameters
      unsigned int day = message.substring(0,2).toInt();   // dd [01-31]
      unsigned int month = message.substring(3,5).toInt();   // MM [01-12]
      unsigned int year =message.substring(8,10).toInt();  // yy [00-99]
      unsigned int hour = 2000 + message.substring(11,13).toInt(); // hh [00-23]
      unsigned int minute = message.substring(14,16).toInt(); // mm [00-59]
      unsigned int second = message.substring(17,19).toInt(); // ss [00-59]

      #if (DEBUG_MODE == true)
        // check parsing
        Serial.print(F("day: "));
        Serial.print(day);
        Serial.print(F(", month: "));
        Serial.print(month);
        Serial.print(F(", year: "));
        Serial.print(year);
        Serial.print(F(", hour: "));
        Serial.print(hour);
        Serial.print(F(", minute: "));
        Serial.print(minute);
        Serial.print(F(", second: "));
        Serial.println(second);
      #endif 

      if (day == 0 || month == 0 || year == 2000)
        // something is wrong!
        return false;
        
      DateTime now = _clock.now();  
      if (_clock.lostPower() || now.day() != day || now.month() != month || now.year() != year 
        || now.hour() != hour || now.minute() != minute) {      
        _clock.adjust(DateTime(year, month, day, hour, minute, second));
        delay(100);
      }

      return true;

    #else
      return false;
    #endif    
  }

  bool executeMqttCommand(const String& command){
    //
    // MQTT topic: aquarioino/cmd  
    //
    #if (DEBUG_MODE == true)
      Serial.print(F("executing mqtt command: "));
      Serial.println(command);
    #endif

    if (command == F("restart")){
      restart();
      return true;
    }

    if (command == F("off")){
      turnOff();
      return true;
    }

    if (command == F("PWC")){
      runWaterParcialChangeRoutine();
      return true;
    }

    if (command == F("manual")){
      runManualCleaning();
      return true;
    }

    if (command == F("save")){
      // save all on EEMPROM
      saveConfigurations(true);
      return true;
    }

    if (command == F("states")){
      // publishes json states
      sendStatesMessage();
      return true;
    }    

    if (command == F("relays on")){
      // turn on all relays (forced)
      setAllRelays(true, RELAY_SIZE, true);
      return true;
    }

    if (command == F("relays off")){
      // turn on all relays (forced)
      setAllRelays(false, RELAY_SIZE, true);
      return true;
    }

    if (command == F("enable timers")){
      // enables timers (and saves it at EEPROM)
      setTimersEnabled(true);
      return true;
    }
    
    if (command == F("disable timers")){
      // disables timers (and saves it at EEPROM)
      setTimersEnabled(false);
      return true;
    }

    if (command == F("enable feeder")){
      // enables timers (and saves it at EEPROM)
      setFeederTimersEnabled(true);
      return true;
    }
    
    if (command == F("disable feeder")){
      // disables timers (and saves it at EEPROM)
      setFeederTimersEnabled(false);
      return true;
    }

    if (command == F("delete all timers")){
      // deletes all timers
      #if (USE_STANDALONE_TIMERS == true)      
        // clean timers
        cleanTimers();
        // saves (no timers)
        saveTimers();
        // disables timers
        setTimersEnabled(false);
        return true;
      #else
        return false;
      #endif
    }

    if (command == F("default timers")){
      #if (USE_STANDALONE_TIMERS == true)
        // loads default timers
        loadDefaultTimers();   
        // saves   
        saveTimers();      
        // enable timers
        setTimersEnabled(true);
        return true;
      #else
        return false;
      #endif
    }

    unsigned int relayPin;
    unsigned int timerId;
    unsigned int len = command.length();

    if (command.startsWith(F("turn on: "))){
      //
      // turn on: [light]
      //
      relayPin = getRelayPinByName(command);    
      if (FEEDING_AS_PUSH_BUTTON && relayPin == RELAY_FEEDER_PIN)
        // do not set feeder relay at "feed as push button" mode
        return true;
      
      setRelay(relayPin, true, false, true);
      return true;
    }

    if (command.startsWith(F("turn off: "))){
      //
      // turn off: [light]
      //
      relayPin = getRelayPinByName(command);
      if (FEEDING_AS_PUSH_BUTTON && relayPin == RELAY_FEEDER_PIN)
        // do not set feeder relay at "feed as push button" mode
        return true;

      setRelay(relayPin, false, false, true);
      return true;
    }

    if (command.startsWith(F("enable timer: ")) && len > 14){
      //
      // enables timer by "id"
      // message format: "enable timer: 01"
      //    
      #if (DEBUG_MODE == true)
        // checks if "timer id" was parsed correctly 
        Serial.print(F("id: ["));
        Serial.print(command.substring(14, len));
        Serial.println(F("]"));
      #endif
      
      timerId = command.substring(14, len).toInt();
      enableTimer(timerId);
      return true;
    }

    if (command.startsWith(F("disable timer: ")) && len > 15){
      //
      // disables timer by "id"
      // message format: "disable timer: 01"
      //
      #if (DEBUG_MODE == true)
        // checks if "timer id" was parsed correctly
        Serial.print(F("id: ["));
        Serial.print(command.substring(15, len));
        Serial.println(F("]"));
      #endif

      timerId = command.substring(15, len).toInt();
      disableTimer(timerId);
      return true;
    }

    if (command.startsWith(F("delete timer: ")) && len > 14){
      //
      // deletes timer by "id"
      // message format: "delete timer: 01"
      //
      #if (DEBUG_MODE == true)
        // checks if "timer id" was parsed correctly
        Serial.print(F("id: ["));
        Serial.print(command.substring(14, len));
        Serial.println(F("]"));
      #endif
      
      timerId = command.substring(14, len).toInt();
      deleteTimer(timerId);
      return true;
    }

    byte hourTurnOn;
    byte minuteTurnOn;
    byte hourTurnOff;
    byte minuteTurnOff;
    String relayName;
    if (command.startsWith(F("add timer: ")) && len > 23){
      //
      // creates new timer
      // message format: "add timer: hh mm hh mm relay"
      //
      #if (DEBUG_MODE == true)
        // checks parsing
        Serial.print(F("h: ["));
        Serial.print(command.substring(12, 14));
        Serial.println(F("]"));

        Serial.print(F("m: ["));
        Serial.print(command.substring(15, 17));
        Serial.println(F("]"));

        Serial.print(F("h: ["));
        Serial.print(command.substring(18, 20));
        Serial.println(F("]"));

        Serial.print(F("m: ["));
        Serial.print(command.substring(21, 23));
        Serial.println(F("]"));
    
        Serial.print(F("relay: ["));
        Serial.print(command.substring(24, len));
        Serial.println(F("]"));
      #endif

      // parse parameters
      hourTurnOn = (byte)(command.substring(12, 14).toInt()); // turn on hour [0-23]
      minuteTurnOn = (byte)(command.substring(15, 17).toInt()); // turn on minute [0-59]
      hourTurnOff = (byte)(command.substring(18, 20).toInt()); // turn off hour [0-23]
      minuteTurnOn = (byte)(command.substring(21, 23).toInt()); // turn off minute [0-59]
      relayName = command.substring(24, len); // relay name [light]
      relayPin = getRelayPinByName(relayName); // relay pin number

      return addTimer(relayPin, hourTurnOn, minuteTurnOn, hourTurnOff, minuteTurnOff);
    }

    if (command.startsWith(F("update timer: ")) && len == 28){
      //
      // creates new timer
      // message format: "update timer: id hh mm hh mm"
      //
      #if (DEBUG_MODE == true)      
        // checks parsing
        Serial.print(F("timer id: ["));
        Serial.print(command.substring(14, 16));
        Serial.println(F("]")); 

        Serial.print(F("h: ["));
        Serial.print(command.substring(17, 19));
        Serial.println(F("]")); 

        Serial.print(F("m: ["));
        Serial.print(command.substring(20, 22));
        Serial.println(F("]")); 

        Serial.print(F("h: ["));
        Serial.print(command.substring(23, 25));
        Serial.println(F("]")); 

        Serial.print(F("m: ["));
        Serial.print(command.substring(26, 28));
        Serial.println(F("]")); 
      #endif

      // parse parameters
      timerId = command.substring(14, 16).toInt(); // timer id [0-49]
      hourTurnOn = (byte)(command.substring(17, 19).toInt()); // turn on hour [0-23]
      minuteTurnOn = (byte)(command.substring(20, 22).toInt()); // turn on minute [0-59]
      hourTurnOff = (byte)(command.substring(23, 25).toInt()); // turn off hour [0-23]
      minuteTurnOff = (byte)(command.substring(26, 28).toInt()); // turn off minute [0-59]

      return updateTimer(timerId, hourTurnOn, minuteTurnOn, hourTurnOff, minuteTurnOff);
    }

    return false;
  }   

#endif

//--------------------------------------------------------------------------------------------------
//
// debug & testing routines
//
//--------------------------------------------------------------------------------------------------
bool testComponents() {
  //
  // execute debug routines (if required)
  //
  // returns true if a testing routine was executed, false otherwise
  //
  #if (USE_LCD_DISPLAY == true && DEBUG_LCD_ROUTINE == true)
    //
    // LCD testing routine: shows message
    //
    #if (DEBUG_MODE == true)
      Serial.println(F("starting LCD testing routine!"));
    #endif
    
    _lcd.print(F("LCD Testing"));
    delay(5000);

    // finish (test executed)
    return true;
  #endif
  
  #if (USE_RTC_CLOCK == true && DEBUG_RTC_ROUTINE == true)
    //
    // RTC testing routine: shows date and time
    //
    #if (DEBUG_MODE == true)
      Serial.println(F("starting RTC testing routine!"));
    #endif

    showDateTime();
    delay(1000);

    // finish (test executed)
    return true;
  #endif

  #if (USE_RELAYS == true && DEBUG_RELAY_ROUTINE == true)
    //
    // Relay testing routine: turn on/off all
    //
    #if (DEBUG_MODE == true)
      Serial.println(F("starting relays testing routine!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 0);
      _lcd.println(F("Relay Testing"));
    #endif

    // repeats indefinitely
    while(true) {
      // for each realy channel
      for(int j = RELAY_FEEDER_PIN; j <= RELAY_HEATER_PIN; j++){
        #if (DEBUG_MODE == true)
          Serial.print(F(" - relay pin:"));
          Serial.println(j);
        #endif
        #if (USE_LCD_DISPLAY == true)
          _lcd.setCursor(0, 1);
          _lcd.print(F("pin: "));
          _lcd.print(j);
          _lcd.print(F("     "));
        #endif        
        // turn on relay of index "j" and turn off all others
        for(int i = RELAY_FEEDER_PIN; i <= RELAY_HEATER_PIN; i++){           
           if(i==j)
               digitalWrite(i, LOW);
           else
               digitalWrite(i, HIGH);
           delay(100);
        }
        // waits
        delay(1000);
      }
      #if (DEBUG_MODE == true)
          Serial.println(F("relays ok!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.setCursor(0, 1);
        _lcd.print(F("Relays Ok"));
        delay(5000);
      #endif
    }
    
    // finish (test executed)
    return true;
  #endif

  #if (USE_PUSH_BUTTONS == true && DEBUG_PUSH_BUTTONS_ROUTINE == true)
    //
    // push buttons testing routine
    //    
    #if (USE_LCD_DISPLAY)
      _lcd.setCursor(0, 0);
      _lcd.print(F("button test"));
      _lcd.setCursor(0, 1);
    #endif
    
    if (digitalRead(PUSH_BUTTON_TURN_OFF_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '01 - turn off' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 1 - OFF"));
      #endif      
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_HEATER_PIN, LOW);
      #endif
    }
    else if (digitalRead(PUSH_BUTTON_RESTART_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '02 - restart' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 2 - RST"));
      #endif
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_LIGHTS_PIN, LOW);
      #endif
    }
    else if (digitalRead(PUSH_BUTTON_LIGHTS_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '03 - lights' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 3 - Ligth"));
      #endif
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_LIGHTS_PIN, LOW);
      #endif
    }
    else if (digitalRead(PUSH_BUTTON_PWC_ROUTINE_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '04 - PWC routine' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 4 - PWC"));
      #endif
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_SUMP_PUMP_PIN, LOW);
      #endif
    }
    else if (digitalRead(PUSH_BUTTON_MANUAL_CLEANING_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '05 - manual cleaning' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 5 - Manual"));
      #endif
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_DRAIN_PUMP_PIN, LOW);
      #endif
    }
    else if (digitalRead(PUSH_BUTTON_FEEDING_PIN) == LOW) {
      #if (DEBUG_MODE == true)
        Serial.println(F("button '06 - feeding' is pressed!"));
      #endif
      #if (USE_LCD_DISPLAY == true)
        _lcd.print(F("btn 6 - Feed"));
      #endif
      #if (USE_RELAYS == true) 
        digitalWrite(RELAY_FEEDER_PIN, LOW);
      #endif
    }
    else
    {
      #if (USE_RELAYS == true) 
        // set pin=LOW to turn on relay
        digitalWrite(RELAY_HEATER_PIN, HIGH);
        digitalWrite(RELAY_LIGHTS_PIN, HIGH);
        digitalWrite(RELAY_FILTER_UV_PIN, HIGH);
        digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
        digitalWrite(RELAY_DRAIN_PUMP_PIN, HIGH);
        digitalWrite(RELAY_FEEDER_PIN, HIGH);
      #endif
    }
    delay(100);    

    // finish (test executed)
    return true;
  #endif


  #if (USE_WATER_LEVEL_SENSORS == true && DEBUG_WATER_LEVEL_SENSORS_ROUTINE == true)
    //
    // water level sensors testing routine
    //
    #if (DEBUG_MODE == true)
      Serial.println(F("starting water level sensors testing routine!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 0);
      _lcd.print(F("water level test"));
    #endif
    //
    while(true){
      for(int i = 0; i < 30; i++){
        _waterSumpLevelLow = !digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN);
        #if (DEBUG_MODE == true)
          Serial.print(F("Sump pump low water level sensor: "));
          Serial.println((_waterSumpLevelLow ? (F"on") : F("off")));
        #endif
        #if (USE_LCD_DISPLAY == true)
          _lcd.setCursor(0, 1);
          _lcd.print(F("Sump low: "));
          _lcd.print((_waterSumpLevelLow ? (F"true  ") : F("false ")));
        #endif
        delay(500);
      }
      for(int i = 0; i < 30; i++){
        _waterDrainLevelLow = !digitalRead(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN);
        #if (DEBUG_MODE == true)
          Serial.print(F("Drain pump low water level sensor: "));
          Serial.println((_waterDrainLevelLow ? (F"on") : F("off")));
        #endif
        #if (USE_LCD_DISPLAY == true)
          _lcd.setCursor(0, 1);
          _lcd.print("Drain low: ");
          _lcd.print((_waterDrainLevelLow ? (F"true  ") : F("false ")));
        #endif
        delay(500);
      }
      delay(100); 
    }        

    // finish (test executed)
    return true;
  #endif

  #if (USE_WATER_LEVEL_SENSORS == true && DEBUG_WATER_REPOSITION_PUMP_ROUTINE == true)
    //
    // reposition pump testing routine: turn on/off
    //
    #if (DEBUG_MODE == true)
      Serial.println(F(" - water reposition pump is on!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 0);
      _lcd.print(F("pump test"));
      _lcd.setCursor(0, 1);
      _lcd.print(F("pump on  "));
    #endif
    digitalWrite(RELAY_WATER_REPOSITION_PUMP_PIN,  LOW);
    delay(2000);        

    #if (DEBUG_MODE == true)
      Serial.println(F(" - water reposition pump is off!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 1);
      _lcd.print(F("pump off "));
    #endif
    digitalWrite(RELAY_WATER_REPOSITION_PUMP_PIN, HIGH);
    delay(8000);    

    // finish (test executed)
    return true;
  #endif
  
  #if (USE_WATER_TEMPERATURE_SENSOR == true && DEBUG_COOLER_FAN_ROUTINE == true)
    //
    // cooler fan testing routine: turn on/off
    //
    #if (DEBUG_MODE == true)
      Serial.println(F(" - cooler fan is on!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 0);
      _lcd.print(F("cooler test"));    
      _lcd.setCursor(0, 1);
      _lcd.print(F("cooler on"));
    #endif
    digitalWrite(RELAY_COOLER_FAN_PIN, LOW);
    delay(5000);        

    #if (DEBUG_MODE == true)
      Serial.println(F(" - cooler fan is off!"));
    #endif
    #if (USE_LCD_DISPLAY == true)
      _lcd.setCursor(0, 1);
      _lcd.print(F("cooler off"));      
    #endif
    digitalWrite(RELAY_COOLER_FAN_PIN, HIGH);
    delay(10000);

    // finish (test executed)
    return true;
  #endif
  
  // no test executed
  return false;
}
