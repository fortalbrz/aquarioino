//
//
// AQUARIO.INO - Aquarium Automation with Home Assistant
//
// TODO: 
//  - programmagle timers
//  - programmagle timers commands
//  - eeprom
//  - states with relays / timers
//
// Source code: https://github.com/fortalbrz/aquarioino
// Making off: https://youtu.be/3CjU-o5LTEI (version 1.0)
// Overview: https://youtu.be/8RszBkeuUlk (version 1.0)
//
// Optimed fpr Arduino Mega 2560 pro mini (CI CH340G)
// - 70 digital I/O, 16 analog inputs, 14 PWM, 4 UART
//
// Features:
// - fully integrated with home assistant (MQTT) or standalone (optional - see configuration flags)
// - automation of the partial water change (PWC / TPA) routine (optional - see configuration flags)
// - automation of water replenishment by evaporation (optional - see configuration flags)
// - lightning and UV filter turn on/off timers (optional - see configuration flags)
// - water temperature control (heating/cooler fan) (optional - see configuration flags)
// - feeding timer (optional - see configuration flags)
// - pH measurement (optional - see configuration flags)
// - alarms (optional - see configuration flags):
//   - temperature
//   - water levels
// - standalone timers (programable with MQTT):
//   - lightning turn on/off
//   - UV filter turn on/off
//   - feeding
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
//   - start partial water change (PWC / TPA) routine button 
//   - turn off / restart / manual cleaning buttons
// - weather station (standalone) (optional - see configuration flags)
// - displays information on LCD 16 x 2 (optional - see configuration flags)
// - push buttons: (optional - see configuration flags)
//  - lightning turn on/off
//  - partial water change (PWC / TPA) routine
//  - manual cleaning button
//  - turn off
//  - restart 
// - local time syncronization with home assistant (optional - see configuration flags)//
//
//  REMARKS: 
//    - the arduino mega pro mini uses the USB/serial CH340G CI, so it's necessary to install the Windows driver: 
//        - CH340G driver: http://bit.ly/44WdzVF (windows 11 compatible)
//        - driver installation instructions http://bit.ly/3ZqIqc0 (pt-BR)
//    - Arduino IDE (2.0) can be obtained at: https://www.arduino.cc/en/software
//    - uses a micro USB cable to connect to the Arduino and select the "Arduino Mega or Mega 2560" board in the IDE ("Tools" -> "Board").
//    - in the library manager, select and install:
//        - "OneWire" by Jim Studt, Tom Pollard, Robin James, etc
//        - "LiquidCrystal" by Arduino, Adafuit
//        - "DallasTemperature" by Milles Burtton, etc
//        - "Adafruit BMP085 Library" by Adafruit
//        - "DHT sensor library" by Adafruit
//        - "RTClib" by Adafruit
//        - "home-assistant-integration" by David Chyrzynsky
//
// Materials:
//
// - 1 box (reused, wooden)
// - 1 arduino mega 2560 pro (mini) - https://produto.mercadolivre.com.br/MLB-2012119523-arduino-mega-pro-mini-lacrado-_JM
// - 1 DS18B20 temperature sensor (waterproof) - https://produto.mercadolivre.com.br/MLB-1659212606-sensor-de-temperatura-ds18b20-prova-dagua-arduino-_JM
// - 1 DHT11 temperature and humidity sensor- https://produto.mercadolivre.com.br/MLB-688214170-sensor-de-umidade-e-temperatura-dht11-com-pci-pic-arduino-_JM
// - 1 BMP180 barometer - https://produto.mercadolivre.com.br/MLB-1335729819-barmetro-bmp180-sensor-de-presso-e-temperatura-arduino-_JM
// - 1 pH sensor PH4502C - https://produto.mercadolivre.com.br/MLB-1894057619-modulo-sensor-de-ph-ph4502c-com-eletrodo-sonda-bnc-arduino-_JM
// - 1 relay module with 8 channels (5 V) - https://produto.mercadolivre.com.br/MLB-1758954385-modulo-rele-rele-5v-8-canais-para-arduino-pic-raspberry-pi- _JM
// - 2 float water level sensors and 2 x 220 olhm resistors - https://produto.mercadolivre.com.br/MLB-1540418150-sensor-de-nivel-de-agua-boia-para-arduino-esp8266-esp32-_JM
// - 1 power source 8v 1A (any voltage between 6 V and 9 V)
// - 1 trimpot 10k
// - 6 tactile push buttom keys and 6 x 1k resistors - https://produto.mercadolivre.com.br/MLB-1858468268-kit-10x-chave-tactil-push-button-6x6x5mm-arduino-eletrnica-_JM
// - 6 female recessed sockets - https://produto.mercadolivre.com.br/MLB-1844503844-6-tomada-embutir-fmea-preta-3-pinos-10a-painel-aparelho-_JM
// - 1 led with
// - 1 power source 12v, 1 fan 12v (cooler), 1 water pump 12v (reposition pump)
// - flexible cab
// 
//   Circuit Wiring Instruction (step by step):
//   -  https://www.circuito.io/static/reply/index.html?solutionId=65010bbd91d445002e8974a5&solutionPath=storage.circuito.io
// 
//  
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
#define USE_LCD_DISPLAY true                   // enables/disables LCD display (disable it to not use the LCD display)
#define USE_HOME_ASSISTANT true                // enables/disables home assistant integration (disable it to not use the LAN ethernet module)
#define USE_PUSH_BUTTONS true                  // enables/disables push buttons (disable it to not use the push buttons)
#define USE_WATER_TEMPERATURE_SENSOR true      // enables/disables water temperature sensor (disable it to not use the DS18B20 temperature sensor)
#define USE_WATER_PH_SENSOR true               // enables/disables water pH sensor (disable it to not use the Ph4502c sensor)
#define USE_AIR_TEMPERATURE_SENSOR true        // enables/disables air temperature and humidity sensor (disable it to not use the DHT11 sensor)
#define USE_AIR_PRESSURE_SENSOR true           // enables/disables air pressure sensor (disable it to not use the BMP085 sensor)
#define USE_WATER_LEVEL_SENSORS true           // enables/disables water level sensors (disable it to not use the water level sensors)
#define USE_RELAYS true                        // enables/disables relays (disable it to not use the relay module)
#define USE_RTC_CLOCK true                     // enables/disables clock (disable it to not use the RTC module)
#define USE_STANDALONE_TIMERS true             // enables/disables timers
#define USE_EEPROM false                       // enables/disables EEPROM 
#define ENABLE_LIGHTS_TIMER true               // enables/disables lights timers
#define ENABLE_UV_FILTER_TIMER true            // enables/disables UV filter timers
#define ENABLE_FEEDING_TIMER true              // enables/disables feeding timers
#define ENABLE_SENSOR_ALARMS true              // enables/disables sensor alarming
#define ENABLE_WEATHER_STATION true            // enables/disables standalone weather station (requires air pressure sensor BMP085)
#define MAX_NUMBER_OF_TIMERS 10
//
// MQTT configuration
//
#define MQTT_BROKER_ADDRESS "192.168.68.93"        // MQTT broker server ip
#define MQTT_BROKER_PORT 1883                      // MQTT broker port
#define MQTT_USERNAME  "mqtt-user"                 // can be omitted if not needed
#define MQTT_PASSWORD  "mqtt"                      // can be omitted if not needed
#define MQTT_DISCOVERY_TOPIC "homeassistant"
#define MQTT_STATES_TOPIC "aquarioino/state"
#define MQTT_COMMAND_TOPIC "aquarioino/cmd"
#define MQTT_TIME_SYNC_TOPIC "ha/datetime"
#define MQTT_AVAILABILITY_TIME 60000
#define LAMBDA_EWMA 0.95                       // exponential weighted mean average lambda
//
// debug & testing flags
//
#define DEBUG_MODE true               // enable/disables serial debugging messages
//#define _debugNoTimers false          // disable timers
#define _debugNoRelay false           // disable relay
#define _debugNoWaterLevel false      // disable water level
// debug routines
#define DEBUG_LCD_ROUTINE false                    // test routine for LCD display (shows test message)
#define DEBUG_RTC_ROUTINE false                    // test routine for RTC (shows curent time)
#define DEBUG_RELAY_ROUTINE false                  // test routine for Relay (activate relay channels in sequence)
#define DEBUG_PUSH_BUTTONS_ROUTINE false           // test routine for push buttons
#define DEBUG_WATER_LEVEL_SENSORS_ROUTINE false    // test routine for water level sensors
#define DEBUG_WATER_REPOSITION_PUMP_ROUTINE false  // test routine for repostion pump
#define DEBUG_COOLER_FAN_ROUTINE false             // test routine for cooler fan
#define ARDUINOHA_DEBUG DEBUG_MODE
//
//
// pins definitions (Arduino Mega 2560 pro mini)
//
//
#define LCD_PIN_RS 12                          // LCD pin 4   
#define LCD_PIN_ENABLED 11                     // LCD pin 6   
#define LCD_PIN_D4 5                           // LCD pin 11
#define LCD_PIN_D5 4                           // LCD pin 12
#define LCD_PIN_D6 3                           // LCD pin 13
#define LCD_PIN_D7 2                           // LCD pin 14
// relay
#define RELAY_SIZE 8
#define RELAY_HEATER_PIN 20                    // Relay heater
#define RELAY_LIGHTS_PIN 19                    //
#define RELAY_FILTER_UV_PIN 18                 //
#define RELAY_SUMP_PUMP_PIN 17                 //
#define RELAY_DRAIN_PUMP_PIN 16                //
#define RELAY_EXTRA_PIN 15                     //
#define RELAY_COOLER_FAN_PIN 14                //
#define RELAY_WATER_REPOSITION_PUMP_PIN 13     //
// water level sensors
#define DRAIN_WATER_LOW_LEVEL_SENSOR_PIN 23    //
#define SUMP_WATER_LOW_LEVEL_SENSOR_PIN 24     //
// LAN ethernet module
#define ETHERNETMODULE_PIN_CS 10               // Ethernet ECN28J60 (CS)
#define ETHERNETMODULE_PIN_INT 3               // Ethernet ECN28J60 (INT)
// analog sensors
#define WATER_TEMPERATURE_SENSOR_DS18B20_PIN 7 // sensor DS18B20 (analog)  
#define AIR_TEMPERATURE_SENSOR_DHT11_PIN 8     // sensor DHT11   (analog)
#define AIR_PRESSURE_SENSOR_BMP180_PIN_SDA 20  // sensor BMP180  (analog) - Arduino Mega 2560
#define AIR_PRESSURE_SENSOR_BMP180_PIN_SCL 21  // sensor BMP180  (analog) - Arduino Mega 2560
#define WATER_PH_SENSOR_PH450C_PIN 6           // sensor Ph4502c (analog)
// buttons and leds
#define PUSH_BUTTON_TURN_OFF_PIN 25
#define PUSH_BUTTON_RESTART_PIN 26
#define PUSH_BUTTON_LIGHTS_PIN 27
#define PUSH_BUTTON_PWC_ROUTINE_PIN 28
#define PUSH_BUTTON_MANUAL_CLEANING_PIN 39
#define PUSH_BUTTON_FEEDING_PIN 30 
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
#define STATUS_CODE_TURNED_OFF 100
#define WEATHER_CODE_NO_FORECAST 0
#define WEATHER_CODE_GOOD 1
#define WEATHER_CODE_DRY 2
#define WEATHER_CODE_NO_CHANGES 3
#define WEATHER_CODE_LIGHT_RAIN 4
#define WEATHER_CODE_RAINING 5
#define WEATHER_CODE_HEAVY_RAIN 6
#define WEATHER_CODE_TUNDERSTORM 7 
//
// LCD text messages (MAX 16 characters)
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
#define TEXT_RELAY_EXTRA F("extra")
//
// EEPROM
//
#define EEPROM_FLAGS_ADDRESS 0
#define EEPROM_FLAG_LIGHT_BIT 0
#define EEPROM_FLAG_TIMERS_ENABLED_BIT 1
#define EEPROM_ALARM_BOUNDS_ADDRESS 2
#define EEPROM_PRESSURE_ADDRESS 10
//
// reference states
//
#define STATUS_CYCLE_START STATUS_CODE_SHOW_DATETIME                         // Cycle start
#define STATUS_CYCLE_END STATUS_CODE_SHOW_FLAGS                              // Cycle end
#define STATUS_CYCLE_OFF STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE  // first non cycle state
#define STATUS_INVALID 100


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
  // MQTT 
  //
  const char str_device_name[] = "aquarioino"; // MQTT device name
  const char str_device_version[] =  "1.0.0"; // MQTT device version
  const char str_device_manufacturer[] =  "Jorge"; // MQTT device manufacturer
  const char str_icon[] = "mdi:water"; // MQTT default cins
  const char str_unit_temperature[] = "C"; // MQTT temperature unit
  const char str_state_sensor[] = "aquarioino_state_sensor";
  const char str_weather_forecast_sensor[] = "aquarioino_weather_forecast_sensor";
  const char str_water_temperature_sensor[] = "aquarioino_water_temperature_sensor";
  const char str_water_ph_sensor[] = "aquarioino_water_ph_sensor";
  const char str_air_temperature_sensor[] = "aquarioino_air_temperature_sensor";
  const char str_air_humidity_sensor[] = "aquarioino_air_humidity_sensor";
  const char str_air_pressure_sensor[] = "aquarioino_air_pressure_sensor";
  const char str_light_switch[] = "aquarioino_lights";
  const char str_heater_switch[] = "aquarioino_heater_switch";
  const char str_cooler_switch[] = "aquarioino_cooler_switch";
  const char str_uv_filter_switch[] = "aquarioino_uv_filter_switch";
  const char str_timers_switch[] = "aquarioino_timers_switch";
  const char str_turn_off_button[] = "aquarioino_turn_off_button";
  const char str_restart_button[] = "aquarioino_restart_button";
  const char str_manual_cleaning_button[] = "aquarioino_manual_cleaning_button";
  const char str_pwc_button[] = "aquarioino_pwc_button";
  const char str_feed_button[] = "aquarioino_feed_button";
  const char str_save_button[] = "aquarioino_save_button";
  const char str_temperature_max[] = "aquarioino_temperature_max";
  const char str_temperature_min[] = "aquarioino_temperature_min";
  const char str_ph_max[] = "aquarioino_ph_max";
  const char str_ph_min[] = "aquarioino_ph_min";
  const char TEXT_MQTT_NAME_STATE[] = "Estado do Aquario";
  const char TEXT_MQTT_NAME_LIGHT[] = "Aquario Luz";
  const char TEXT_MQTT_NAME_HEATER[] = "Aquario Arquecedor";
  const char TEXT_MQTT_NAME_COOLER[] = "Aquario Cooler";
  const char TEXT_MQTT_NAME_TIMERS[] = "Aquario Timers";
  const char TEXT_MQTT_NAME_WEATHER_FORECAST[] = "Aquario Forecast";
  const char TEXT_MQTT_NAME_UV_FILTER[] = "Aquario Filtro UV";
  const char TEXT_MQTT_NAME_TURN_OFF[] = "Desligar Aquario";
  const char TEXT_MQTT_NAME_RESTART[] = "Reiniciar Aquario";
  const char TEXT_MQTT_NAME_MANUAL_CLEANING[] = "Limpeza Manual";
  const char TEXT_MQTT_NAME_PWC[] = "TPA";
  const char TEXT_MQTT_NAME_FEED[] = "Alimentar";
  const char TEXT_MQTT_NAME_RUNNING[] = "Ligado";
  const char TEXT_MQTT_NAME_WATER_TEMPERATURE[] = "Temperatura da agua";
  const char TEXT_MQTT_NAME_WATER_PH[] = "pH da agua";
  const char TEXT_MQTT_NAME_AIR_TEMPERATURE[] = "Temperatura do ar";
  const char TEXT_MQTT_NAME_AIR_HUMIDITY[] = "Umidade do ar";
  const char TEXT_MQTT_NAME_AIR_PRESSURE[] = "Pressao Atmosférica";
  //
  // Home Assistant (MQTT)
  //
  byte _mac[] = {0xF0, 0x24, 0xAF, 0xE6, 0x32, 0xA4};
  HADevice device(_mac, sizeof(_mac));
  HAMqtt mqtt(client, device);
  
  // MQTT general state sensor
  HASensor sensorState(str_state_sensor);
  HASensor sensorWeatherForecast(str_weather_forecast_sensor);

  // MQTT temperature, humidity and pressure sensors
  HASensorNumber sensorWaterTemp(str_water_temperature_sensor, HASensorNumber::PrecisionP1);
  HASensorNumber sensorWaterPh(str_water_ph_sensor, HASensorNumber::PrecisionP1);
  HASensorNumber sensorAirTemp(str_air_temperature_sensor, HASensorNumber::PrecisionP1);
  HASensorNumber sensorAirHumidity(str_air_humidity_sensor, HASensorNumber::PrecisionP0);
  HASensorNumber sensorAirPressure(str_air_pressure_sensor, HASensorNumber::PrecisionP0);

  // MQTT light switches
  HALight chkLightSwitch(str_light_switch);
  HASwitch chkFilterSwitch(str_uv_filter_switch);
  HASwitch chkHeaterSwitch(str_heater_switch);
  HASwitch chkCoolerSwitch(str_cooler_switch);
  HASwitch chkTimersSwitch(str_timers_switch);

  // MQTT buttons
  HAButton btnTurnOff(str_turn_off_button);
  HAButton btnRestart(str_restart_button);
  HAButton btnManualCleaning(str_manual_cleaning_button);
  HAButton btnStartWPC(str_pwc_button);
  HAButton btnFeed(str_feed_button);
  HAButton btnSave(str_save_button);

  // MQTT parameters
  HANumber lblTemperatureMax(str_temperature_max, HASensorNumber::PrecisionP1);
  HANumber lblTemperatureMin(str_temperature_min, HASensorNumber::PrecisionP1);
  HANumber lblPhMax(str_ph_max, HASensorNumber::PrecisionP1);
  HANumber lblPhMin(str_ph_min, HASensorNumber::PrecisionP1);

#endif

struct timer{
   bool enabled = false; 
   bool active = false; 
   byte relay = 0x00;
   byte turnOnHour = 0x00;
   byte turnOnMinute = 0x00;
   byte turnOffHour = 0x00;
   byte turnOffMinute = 0x00;
};

struct measure {
   float value = 0;
   float valueEWMA = 0;
   float valueMax = 0;
   float valueMin = 100000000000;
};

//
// Globals: states and measured values
//
DateTime _now;
int _status = 0;                     // aguarium state code
bool _timersEnabled = true;          // enables/disables timers
bool _statusFeeded = false;          
bool _statusForceLights = false;
bool _waterDrainLevelLow = false;
bool _waterSumpLevelLow = false;
measure _measuredAirTemperature;
measure _measuredAirHumidity;
measure _measuredAirPressure;
measure _measuredAirAltitude;
measure _measuredAirSealevelPressure;
measure _measuredWaterTemperature;
measure _measuredWaterPh;
unsigned long _lastAvailabilityTime = millis();
bool _blink = false;

// required for "setAllRelays()"
const unsigned int RELAY_PINS[] = {RELAY_LIGHTS_PIN, RELAY_HEATER_PIN, RELAY_FILTER_UV_PIN, RELAY_COOLER_FAN_PIN, RELAY_SUMP_PUMP_PIN, RELAY_WATER_REPOSITION_PUMP_PIN, RELAY_DRAIN_PUMP_PIN, RELAY_EXTRA_PIN};
// relay states cache
bool RELAY_STATES[8];
// timers
timer RELAY_TIMERS[MAX_NUMBER_OF_TIMERS];

//
// reference levels (default levels)
//
float _levelWaterTempMin = 26.0;
float _levelWaterTempMax = 29.1;
float _levelWaterPhMin = 7.5;
float _levelWaterPhMax = 8.9;
// timers start/end hours
unsigned int _timerLightStart = 6;
unsigned int _timerLightEnd = 23;
unsigned int _timerUvStart = 8;
unsigned int _timerUvEnd = 15;
unsigned int _timerFeed1 = 7;
unsigned int _timerFeed2 = 12;
unsigned int _timerFeed3 = 18;


//--------------------------------------------------------------------------------------------------
//
// setup
//
//--------------------------------------------------------------------------------------------------

void setup() {
  //
  // initialization
  //
  pinMode(LED_BUILTIN, OUTPUT);  

  #if (DEBUG_MODE == true)
    // serial only in debug mode!
    Serial.begin(9600);
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
    pinMode(RELAY_EXTRA_PIN, OUTPUT);
    pinMode(RELAY_COOLER_FAN_PIN, OUTPUT);
    pinMode(RELAY_WATER_REPOSITION_PUMP_PIN, OUTPUT);

    // initializes relays states (turns off)
    digitalWrite(RELAY_HEATER_PIN, HIGH);
    digitalWrite(RELAY_LIGHTS_PIN, HIGH);
    digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
    digitalWrite(RELAY_FILTER_UV_PIN, HIGH);
    digitalWrite(RELAY_DRAIN_PUMP_PIN, HIGH);
    digitalWrite(RELAY_EXTRA_PIN, HIGH);  
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
      // does not intializes anything more...
      return;
    #endif
  #endif

  #if (USE_RTC_CLOCK)
    //
    // initialize real time clock (RTC)
    //  
    if (! _clock.begin()) {
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
      // RTC debug routine
      _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
      #if (DEBUG_MODE == true)
        Serial.println(F("real time clock debug mode!)");
      #endif
      // does not intializes anything more...
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
      #if (DEBUG_MODE == true)
      Serial.println(F("ERROR: Air pressure sensor BMP180 not found!"));
      #endif
    }
    else {
      updateMeasure(_measuredAirPressure, _sensorAirPresure.readPressure());
      
      #if (USE_EEPROM == true)
      // updateMeasure(_measuredAirPressure.value, (float) EEPROM.read(EEPROM_PRESSURE_ADDRESS));
      // if (_measuredAirPressure.value == 0) 
      //   _measuredAirPressureAvg = _measuredAirPressure;
      
      // // todo avoid writing
      // // EEPROM.update(EEPROM_PRESSURE_ADDRESS, (byte)_measuredAirPressureAvg); 
      // #else
      // _measuredAirPressureAvg = _measuredAirPressure;
      #endif
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

  #if (USE_HOME_ASSISTANT)
    //
    // MQTT initialization
    //
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
    
    btnTurnOff.setName(TEXT_MQTT_NAME_TURN_OFF);
    btnTurnOff.onCommand(onButtonCommand);

    btnRestart.setName(TEXT_MQTT_NAME_RESTART);
    btnRestart.onCommand(onButtonCommand);

    btnManualCleaning.setName(TEXT_MQTT_NAME_MANUAL_CLEANING);
    btnManualCleaning.onCommand(onButtonCommand);

    btnStartWPC.setName(TEXT_MQTT_NAME_PWC);
    btnStartWPC.onCommand(onButtonCommand);

    btnFeed.setName(TEXT_MQTT_NAME_FEED);
    btnFeed.onCommand(onButtonCommand);

    lblTemperatureMax.setMin(10);
    lblTemperatureMax.setMax(40);
    lblTemperatureMax.setStep(0.1);
    lblTemperatureMax.onCommand(onNumberChange);

    lblTemperatureMin.setMin(10);
    lblTemperatureMin.setMax(50);
    lblTemperatureMin.setStep(0.1);
    lblTemperatureMin.onCommand(onNumberChange);

    lblPhMax.setMin(3);
    lblPhMax.setMax(12);
    lblPhMax.setStep(0.1);
    HANumber lblPhMax(str_ph_max, HASensorNumber::PrecisionP1);

    lblPhMin.setMin(3);
    lblPhMin.setMax(12);
    lblPhMin.setStep(0.1);
    HANumber lblPhMin(str_ph_min, HASensorNumber::PrecisionP1);
  
    mqtt.begin(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_PASSWORD);
  #endif

  #if (USE_EEPROM == true)
    _statusForceLights = bitRead(EEPROM.read(EEPROM_FLAGS_ADDRESS), EEPROM_FLAG_LIGHT_BIT);    
  #else
    _statusForceLights = false;
  #endif
  
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
  digitalWrite(LED_BUILTIN, (_blink ? HIGH: LOW));
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
  // a test routines was executed! do not continue!!!
  //
  if (testComponents())    
    return;
  
  //
  // reads sensors data
  //
  readData();

  if (_status == STATUS_CODE_TURNED_OFF) {
    // arquarioino is turned off
    turnOff();
    return;
  }

  #if (USE_LCD_DISPLAY == true)  
    //
    // shows rotating messages on LCD display
    //
    _lcd.setCursor(0, 0);
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
      default:
        // shows date and time
        showDateTime();
        break;
    }
  #endif

  // routines execution status
  switch (_status) {
    case STATUS_CODE_MANUAL_CLEANING:
      runManualCleaning();
      break;
    case STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE:
      runWaterParcialChangeRoutine();
      break;    
  }

  #if (USE_LCD_DISPLAY == true)  
    // sets 2nd LCD line: alarms
    _lcd.setCursor(0, 1);
  #endif
  #if (ENABLE_SENSOR_ALARMS == true)
    showAlarms();
  #endif
  
  // handles actions (5 seconds)
  for (int i = 0; i <= 10 && !readButtons(); i++) {
    
    #if (USE_STANDALONE_TIMERS == true)
      handleTimers();
    #endif

    #if (ENABLE_SENSOR_ALARMS == true)
      handleAlarms();
    #endif

    delay(500);
  }

  // increments states (not for unusual ones: off/cleaning/PWC)
  nextAquariumState();
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
  if (state.valueEWMA == 0)
      state.valueEWMA = state.value;
  state.valueEWMA = (1 - LAMBDA_EWMA) * value + LAMBDA_EWMA * state.valueEWMA; 
  
  // maximum
  if (value > state.valueMax) 
    state.valueMax = value;

  // minimum
  if (value > state.valueMin) 
    state.valueMin = value;
}

void setRelay(unsigned int relayPin, const bool& state, const bool& useIndex = false) {
  //
  // sets relay state by relay pin number (useIndex = false): 
  // set "true" to activates the relay, "false" to deactivates it.
  //
  // alternativelly, set relay state by its index ([0-7]) (useIndex = true)
  //
  #if (USE_RELAYS == true)
    unsigned int index = (useIndex ? relayPin : getRelayIndexByPin(relayPin));
    // ensure valid relay pin
    if (index >= RELAY_SIZE){
      // invalid pin 
      #if (DEBUG_MODE == true) 
        Serial.print(F("Invalid relay pin: "));
        Serial.println(relayPin);    
      #endif
      return;
    }
    if (useIndex)     
      relayPin = RELAY_PINS[index];
    
    // relays are active LOW!
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
      // updates home assistant
      //
      switch(relayPin){

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
    // nothing to do
    return false;

  // stores state
  _status = state;

  switch (state) {    
    case STATUS_CODE_TURNED_OFF:
      _statusForceLights = false;
      break;    

    case STATUS_CODE_MANUAL_CLEANING:
      _statusForceLights = false;
      break;          
  }
  
  #if (DEBUG_MODE == true) 
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
    switch (state) {
      case STATUS_CODE_TURNED_OFF:
        sensorState.setValue(TEXT_MQTT_NAME_TURN_OFF);
        break;

      case STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE:
        sensorState.setValue(TEXT_MQTT_NAME_PWC);
        break;

      case STATUS_CODE_MANUAL_CLEANING:
        sensorState.setValue(TEXT_MQTT_NAME_MANUAL_CLEANING);
        break;

      default:
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
    _status++;

    // circular state transition
    if (_status > STATUS_CYCLE_END)
      _status = STATUS_CYCLE_START;
  }
}

void setAllRelays(const bool& state, const unsigned int& exception = RELAY_SIZE) {
  //
  // sets all relays at specified state
  //
  for (unsigned int index = 0; index < RELAY_SIZE; index++) {
    if (exception < RELAY_SIZE && index == exception)
      setRelay(index, !state, true);
    else
      setRelay(index, state, true);
  }
}

unsigned int getRelayIndexByPin(const unsigned int& relayPin){
  //
  // gets relay index by specified relay pin number
  //
  #if (USE_RELAYS == true)
  for (unsigned int index = 0; index < RELAY_SIZE; index++)
    if (RELAY_PINS[index] == relayPin)
      return index;
  #endif

  // not found
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

//--------------------------------------------------------------------------------------------------
//
// EEPROM
//
//--------------------------------------------------------------------------------------------------
void saveLightState(const boolean& value){
  //
  // Saves light state on EEPROM
  //
  _statusForceLights = value;
  saveConfigurationFlag(EEPROM_FLAG_LIGHT_BIT, value);
}

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
      sensorWaterTemp.setValue(_measuredWaterTemperature.valueEWMA);
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
      sensorAirTemp.setValue(_measuredAirTemperature.valueEWMA);
      sensorAirHumidity.setValue(_measuredAirHumidity.valueEWMA);
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
      sensorAirPressure.setValue(_measuredAirPressure.valueEWMA);
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
  if (!USE_PUSH_BUTTONS)
    // disable buttons
    return false;  
  
  if (digitalRead(PUSH_BUTTON_TURN_OFF_PIN) == LOW) {
    turnOff();
    return true;
  }
  if (digitalRead(PUSH_BUTTON_RESTART_PIN) == LOW) {
    restart();
    return true;
  }
  if (digitalRead(PUSH_BUTTON_LIGHTS_PIN) == LOW) {
    saveLightState(!_statusForceLights);
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
    runManualCleaning();
    // feed();
    return true;
  }

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
  // Shows atmosferic data(1)
  //
  #if (DEBUG_MODE == true)    
    Serial.print(TEXT_AIR_TEMPERATURE);
    Serial.print(_measuredAirTemperature.value);
    Serial.println(str_unit_temperature);
    Serial.print(TEXT_AIR_HUMIDITY);
    Serial.print(_measuredAirHumidity.value, 0);
    Serial.print(F("%"));
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
  // Shows atmosferic data (2)
  //
  #if (DEBUG_MODE == true)    
    Serial.print(TEXT_AIR_PRESSURE);
    Serial.print(_measuredAirPressure.valueEWMA / 100, 0);
    Serial.println(F("mPa"));
    Serial.print(TEXT_ALTITUDE);
    Serial.print(_measuredAirAltitude.valueEWMA, 0);
    Serial.print(F("m"));
  #endif
  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_AIR_PRESSURE);
    _lcd.print(_measuredAirPressure.valueEWMA / 100, 0);
    _lcd.print(F("mPa"));
    
    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_ALTITUDE);
    _lcd.print(_measuredAirAltitude.valueEWMA, 0);
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
      switch(forecast){
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
      switch(forecast){
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
  // Shows temperature
  //
  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_WATER_TEMPERATURE);
    _lcd.print(_measuredWaterTemperature.value);
    _lcd.print(F("C"));
    _lcd.setCursor(0, 1);
    switch (idx) {
      case 1:
        _lcd.print(TEXT_MIN);
        _lcd.print(_measuredWaterTemperature.valueMin);
        break;
      case 2:
        _lcd.print(TEXT_MAX);
        _lcd.print(_measuredWaterTemperature.valueMax);
        break;
      case 3:
        _lcd.print(TEXT_AVG);
        _lcd.print(_measuredWaterTemperature.valueEWMA);
        break;
    }
    _lcd.print(F("C"));
  #endif
}

void showWaterPh() {
  //
  // Shows Ph
  //
  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_WATER_PH);
    _lcd.print(_measuredWaterPh.valueEWMA);
    _lcd.setCursor(0, 1);
    _lcd.print(F("Alcalin: 120 ppm"));
  #endif
}

void showSystemFlags() {
  //
  // Shows system flags
  //
  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_LIGHTS);
    if (_statusForceLights)
      _lcd.print(TEXT_ON);
    else
      _lcd.print(TEXT_TIMER);

    _lcd.setCursor(0, 1);
    _lcd.print(TEXT_FEED);  
    _lcd.print(TEXT_OFF);
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
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 600)  
    return WEATHER_CODE_NO_FORECAST;

  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 550 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 599) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 500 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 549) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 450 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 499) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 400 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 449) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 350 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 399) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 300 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 349) 
    return WEATHER_CODE_GOOD;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 200 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 299) 
    return WEATHER_CODE_DRY;

  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA >= 100 
    && _measuredAirPressure.valueMax - _measuredAirPressure.valueEWMA <= 199) 
    return WEATHER_CODE_NO_CHANGES;

  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueMin >= 200 
    && _measuredAirPressure.valueEWMA - _measuredAirPressure.valueMin <= 299) 
    return WEATHER_CODE_LIGHT_RAIN;

  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueMin >= 300 
    && _measuredAirPressure.valueEWMA - _measuredAirPressure.valueMin <= 399) 
    return WEATHER_CODE_RAINING;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueMin >= 400 
    && _measuredAirPressure.valueEWMA - _measuredAirPressure.valueMin <= 499) 
    return WEATHER_CODE_HEAVY_RAIN;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueMin >= 500 
    && _measuredAirPressure.valueEWMA - _measuredAirPressure.valueMin <= 599) 
    return WEATHER_CODE_TUNDERSTORM;
  
  if (_measuredAirPressure.valueMax - _measuredAirPressure.valueMin >= 600)  
    return WEATHER_CODE_NO_FORECAST;
  
  if (_measuredAirHumidity.valueEWMA < 30)
    return WEATHER_CODE_DRY;

  return WEATHER_CODE_NO_FORECAST;
}

//--------------------------------------------------------------------------------------------------
//
// timers
//
//--------------------------------------------------------------------------------------------------
#if (USE_STANDALONE_TIMERS == true)

  void handleTimers()
  {
    //
    // Handle scheduled tasks actions
    //    
    if (_status < STATUS_CYCLE_OFF) {
      int hour = _now.hour();

      // lights timer, set LOW to turn on lights
      bool lightOnTimer = hour > _timerLightStart && hour < _timerLightEnd ;
      digitalWrite(RELAY_LIGHTS_PIN, !(lightOnTimer || _statusForceLights));

      // UV timer (8h), set LOW to turn on UV lamp)
      digitalWrite(RELAY_FILTER_UV_PIN, !(hour > _timerUvStart && hour < _timerUvEnd));

      // Feeding timer: 7, 12 and 18h
      if (hour == _timerFeed1 || hour == _timerFeed2 || hour == _timerFeed3) {
        if (!_statusFeeded)
          feed();
      }
      else
        _statusFeeded = false;
    }
    else
    {
      // non usual mode: just ensures UV turn off
      digitalWrite(RELAY_FILTER_UV_PIN, HIGH);
    }
  }

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
    // Take sensor alarm actions: change realys in response
    //
    if(!USE_RELAYS || _status >= STATUS_CYCLE_OFF) 
      // nothing to do: relays disabled or aquarioino turned off
      return;
    
    #if (USE_WATER_LEVEL_SENSORS == true)
      //
      // water reposition (due to evaporation)
      // sump pump water level opened (false) -> alarm sump water level is low (true)
      //
      _waterSumpLevelLow = !digitalRead(SUMP_WATER_LOW_LEVEL_SENSOR_PIN); 
      if (_debugNoWaterLevel){
        // disable reposition pump
        _waterSumpLevelLow = false;
      }
      // set pin=LOW to turn on repostion pump relay
      digitalWrite(RELAY_WATER_REPOSITION_PUMP_PIN, !_waterSumpLevelLow);
    #endif

    #if (USE_WATER_TEMPERATURE_SENSOR == true)
      //
      // high temperature alarm!
      // set pin=LOW to turn on relay
      // if high temp (true) -> turn on cooler, turn of heater -> set cooler pin LOW, set heater pin HIGH 
      //
      bool highTemp = _measuredWaterTemperature.value > _levelWaterTempMax;
      digitalWrite(RELAY_COOLER_FAN_PIN, !highTemp);
      digitalWrite(RELAY_HEATER_PIN, highTemp);
    #endif
  }

  void showAlarms()
  {
    //
    // shows alarms (second LCD display line)
    //
    if(ENABLE_SENSOR_ALARMS) {
      // disable alarms
      return;
    }

    if (_status < STATUS_CYCLE_OFF ) {
      
      _lcd.setCursor(0, 1);  
      if (_waterSumpLevelLow && !_debugNoWaterLevel) {
        // low level alarm
        _lcd.print(TEXT_ALARM_LOW_WATER_LEVEL);
      }
      else if (_measuredWaterTemperature.value > _levelWaterTempMax) {
        // temperature alarms
        _lcd.print(TEXT_ALARM_HIGH_TEMPERATURE);
        _lcd.print(_measuredWaterTemperature.value);
        _lcd.print(F("C"));
      }
      else if (_measuredWaterTemperature.value < _levelWaterTempMin) {
        // temperature alarms
        _lcd.print(TEXT_ALARM_LOW_TEMPERATURE);
        _lcd.print(_measuredWaterTemperature.value);
        _lcd.print(F("C"));
      }
      else if (_measuredWaterPh.value < _levelWaterPhMin) {
        // pH alarms
        _lcd.print(TEXT_ALARM_LOW_PH);
        _lcd.print(_measuredWaterPh.value);
      }  
      else if (_measuredWaterPh.value > _levelWaterPhMax) {
        // pH alarms
        _lcd.print(TEXT_ALARM_HIGH_PH);
        _lcd.print(_measuredWaterPh.value);
      }
    
      delay(100);
    }
  }
#endif
//--------------------------------------------------------------------------------------------------
//
// feeding
//
//--------------------------------------------------------------------------------------------------
void feed()
{
  //
  // Feeding procedure (deprecated)
  //
  if (!ENABLE_FEEDING_TIMER && !USE_RELAYS) 
    return;
  
  // set pin=LOW to turn on relay
  digitalWrite(RELAY_EXTRA_PIN, HIGH);
  delay(100);
  digitalWrite(RELAY_EXTRA_PIN, LOW);
  delay(500);
  digitalWrite(RELAY_EXTRA_PIN, HIGH);
  delay(500);
  _statusFeeded = true;
}
//--------------------------------------------------------------------------------------------------
//
// aquarium maintenance routines (WPC, manual cleaning)
//
//--------------------------------------------------------------------------------------------------
void runWaterParcialChangeRoutine()
{
  //
  // Auto TPA
  //
  delay(1000);
  #if (USE_LCD_DISPLAY)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_PWC_STARTING);
    delay(1000);  
  #endif 
  
  if (!setAquariumState(STATUS_CODE_EXECUTION_WATER_PARTIAL_CHANGE_ROUTINE))
    return; 

  //
  // step 1 - drain dirty water (turn on drain pump)
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
    setAllRelays(false, 0); 
    delay(1000);

    setRelay(RELAY_DRAIN_PUMP_PIN, true);
    while (digitalRead(DRAIN_WATER_LOW_LEVEL_SENSOR_PIN)) {
      // while not reach drain level low => keep draining dirty water
      delay(500);
    }
    // turn of drain pump (water is at low drained level) 
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
    return; 
  
  #if (USE_RELAYS == true)
    // turn off all (only lights on)
    setAllRelays(false, 0);
    delay(1000);    
  #endif
}

void turnOff() {
  //
  // turns off aquarioino
  //
  delay(1000);
  
  _statusForceLights = false;
  #if (USE_LCD_DISPLAY == true)
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_TURN_OFF);  
    delay(1000);  
  #endif
  
  // set status OFF  
  if (!setAquariumState(STATUS_CODE_TURNED_OFF))
    return; 

  #if (USE_RELAYS == true)  
    // turn off all (only lights on)
    setAllRelays(false, 0);
    delay(1000);
  #endif
}


void restart() {
  //
  // restarts aquarium
  //
  delay(1000);
  _statusForceLights = false;
  setAquariumState(STATUS_CYCLE_START);
  
  #if (USE_LCD_DISPLAY == true)
    _lcd.clear();
    _lcd.setCursor(0, 0);
    _lcd.print(TEXT_TURN_STARTING);
    delay(1000);
  #endif
  
  #if (USE_RELAYS == true)
    // default: turn on heater, lights and sump pump!
    setRelay(RELAY_HEATER_PIN, true);
    setRelay(RELAY_LIGHTS_PIN, true);
    setRelay(RELAY_SUMP_PUMP_PIN, true);
    setRelay(RELAY_FILTER_UV_PIN, false);
    setRelay(RELAY_DRAIN_PUMP_PIN, false);
    setRelay(RELAY_EXTRA_PIN, false);  
    setRelay(RELAY_COOLER_FAN_PIN, false);
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);  
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
      // Date & time syncronization message from home assistant      
      // message format: "dd MM yyyy hh mm ss"
      //
      String message = String((const char*)payload);
      #if (DEBUG_MODE == true)
        Serial.print(F("Time sync: "));
        Serial.println(message);
      #endif 
      
      unsigned int day = message.substring(0,2).toInt();   // dd [01-31]
      unsigned int month = message.substring(3,5).toInt();   // MM [01-12]
      unsigned int year =message.substring(8,10).toInt();  // yy [00-99]
      unsigned int hour = 2000 + message.substring(11,13).toInt(); // hh [00-23]
      unsigned int minute = message.substring(14,16).toInt(); // mm [00-59]
      unsigned int second = message.substring(17,19).toInt(); // ss [00-59]

      #if (DEBUG_MODE == true)
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
        return;
        
      DateTime now = _clock.now();  
      if (_clock.lostPower() || now.day() != day || now.month() != month || now.year() != year 
        || now.hour() != hour || now.minute() != minute) {      
        _clock.adjust(DateTime(year, month, day, hour, minute, second));
        delay(100);
      }
      return;
    }

    if (strcmp(topic, MQTT_COMMAND_TOPIC) == 0) {
      //
      // Date & time syncronization message from home assistant
      //
      return;
    }
  }

  void onLightStateCommand(bool state, HALight* sender) {
    //
    // MQTT light switch state handler
    //    
    if (sender == &chkLightSwitch)
      setRelay(RELAY_LIGHTS_PIN, state);
  }

  void onSwitchCommand(bool state, HASwitch* sender){
    //
    // MQTT switches handler
    //    
    if (sender == &chkFilterSwitch)
      setRelay(RELAY_FILTER_UV_PIN, state);      
    else if (sender == &chkHeaterSwitch)
      setRelay(RELAY_HEATER_PIN, state);
    else if (sender == &chkCoolerSwitch)
      setRelay(RELAY_COOLER_FAN_PIN, state);      
    else if (sender == &chkTimersSwitch) {

      sender->setState(state); // report state back to the Home Assistant
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
    else if (sender == &btnStartWPC) 
        runWaterParcialChangeRoutine();
    else if (sender == &btnFeed)
        feed();
  }

  void onNumberChange(HANumeric value, HANumber* sender) {
    //
    // MQTT numeric reference values
    //
    if (sender == &lblTemperatureMin) {        
        _levelWaterTempMin = value.toFloat();     
    }
    else if (sender == &lblTemperatureMax) {        
        _levelWaterTempMax = value.toFloat();
    }
    else if (sender == &lblPhMax) {        
        _levelWaterPhMax = value.toFloat();
    }
    else if (sender == &lblPhMin) {        
        _levelWaterPhMin = value.toFloat();
    }
    sender->setState(value);
  }

  String toStr(const bool& value){
    return (value ? String(F("on")) : String(F("off")));
  }

  String toStr(const measure& measure, const String& label){
    String json = String(F("\"[LABEL]\": [VALUE], \"[LABEL]_max\": [MAX], \"[LABEL]_min\": [MIN], \"[LABEL]_avg\": [AVG], "));
    json.replace(F("[LABEL]"), label);
    json.replace(F("[VALUE]"), String(measure.value, 1));
    json.replace(F("[AVG]"), String(measure.valueEWMA, 1));
    json.replace(F("[MAX]"), String(measure.valueMax, 1));
    json.replace(F("[MIN]"), String(measure.valueMin, 1));
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
    json += String(F("\"timer_enabled\":")) + toStr(_timersEnabled) + end;
  
    // measurements
    json += toStr(_measuredWaterTemperature, F("temp"));
    json += toStr(_measuredWaterPh, F("ph"));
    json += toStr(_measuredAirTemperature, F("air_temp"));
    json += toStr(_measuredAirHumidity, F("humidity"));
    json += toStr(_measuredAirPressure, F("pressure"));
    json += toStr(_measuredAirAltitude, F("altitude"));
    json += toStr(_measuredAirSealevelPressure, F("sealevel_pressure"));
    
    // relays
    // for (unsigned int i = 0; i < RELAY_SIZE; i++)
    //   json += String(TEXT_RELAY_LIGHTS) + toStr(RELAY_STATES[0]) + end;    
    
    json += String(F("}"));
    // const char*
    mqtt.publish(MQTT_STATES_TOPIC, "test");
  }


  bool executeMqttCommand(const String& command){
    //
    // MQTT topic: aquarioino/cmd  
    //
    if (command == F("restart")){
      restart();
      return;
    }

    if (command == F("off")){
      turnOff();
      return;
    }

    if (command == F("wpc")){
      runWaterParcialChangeRoutine();
      return;
    }

    if (command == F("manual")){
      runManualCleaning();
      return;
    }

    if (command == F("save")){
      // save all on EEMPROM
      return;
    }

    if (command == F("relays on")){
      // turn on all relays
      setAllRelays(true);
    }

    if (command == F("relays off")){
      // turn on all relays
      setAllRelays(false);
    }

    if (command == F("enable timers")){
      // enables timers
      _timersEnabled = true;
      return;
    }
    
    if (command == F("disable timers")){
      // disables timers
      _timersEnabled = false;
      return;
    }

  if (command == F("delete all timers")){
    // deletes all timers
    return;
  }

  if (command == F("default timers")){
    // set default timers
    return;
  }

  if (command.startsWith(F("turn on: "))){
    //
    // turn on: [light]
    //
    if (command.endsWith(TEXT_RELAY_LIGHTS))
      setRelay(RELAY_LIGHTS_PIN, true);
    else if (command.endsWith(TEXT_RELAY_HEATER))
      setRelay(RELAY_HEATER_PIN, true);
    else if (command.endsWith(TEXT_RELAY_HEATER))
      setRelay(RELAY_HEATER_PIN, true);
    else if (command.endsWith(TEXT_RELAY_FILTER))
      setRelay(RELAY_FILTER_UV_PIN, true);
    else if (command.endsWith(TEXT_RELAY_COOLER))
      setRelay(RELAY_COOLER_FAN_PIN, true);
    else if (command.endsWith(TEXT_RELAY_SUMP_PUMP))
      setRelay(RELAY_SUMP_PUMP_PIN, true);
    else if (command.endsWith(TEXT_RELAY_DRAIN_PUMP))
      setRelay(RELAY_DRAIN_PUMP_PIN, true);
    else if (command.endsWith(TEXT_RELAY_REPOSITION_PUMP))
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, true);
    else if (command.endsWith(TEXT_RELAY_EXTRA))
      setRelay(RELAY_EXTRA_PIN, true);
    return;
  }

  if (command.startsWith(F("turn off: "))){
    //
    // turn off: [light]
    //
    if (command.endsWith(TEXT_RELAY_LIGHTS))
      setRelay(RELAY_LIGHTS_PIN, false);
    else if (command.endsWith(TEXT_RELAY_HEATER))
      setRelay(RELAY_HEATER_PIN, false);
    else if (command.endsWith(TEXT_RELAY_HEATER))
      setRelay(RELAY_HEATER_PIN, false);
    else if (command.endsWith(TEXT_RELAY_FILTER))
      setRelay(RELAY_FILTER_UV_PIN, false);
    else if (command.endsWith(TEXT_RELAY_COOLER))
      setRelay(RELAY_COOLER_FAN_PIN, false);
    else if (command.endsWith(TEXT_RELAY_SUMP_PUMP))
      setRelay(RELAY_SUMP_PUMP_PIN, false);
    else if (command.endsWith(TEXT_RELAY_DRAIN_PUMP))
      setRelay(RELAY_DRAIN_PUMP_PIN, false);
    else if (command.endsWith(TEXT_RELAY_REPOSITION_PUMP))
      setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, false);
    else if (command.endsWith(TEXT_RELAY_EXTRA))
      setRelay(RELAY_EXTRA_PIN, false);
    return;
  }
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
      for(int j = RELAY_EXTRA_PIN; j <= RELAY_HEATER_PIN; j++){
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
        for(int i = RELAY_EXTRA_PIN; i <= RELAY_HEATER_PIN; i++){           
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
        _lcd.print(F("btn 4 - WPC"));
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
        digitalWrite(RELAY_EXTRA_PIN, LOW);
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
        digitalWrite(RELAY_EXTRA_PIN, HIGH);
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
