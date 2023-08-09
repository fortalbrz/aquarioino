//
//
// Aquarium Controller (with Arduino Mega 2560 Pro Mini)
//
// Making off: https://youtu.be/3CjU-o5LTEI
// Overview: https://youtu.be/8RszBkeuUlk
//
// Arduino Mega 2560 pro mini (CI CH340G)
// - 70 digital I/O, 16 analog inputs, 14 PWM, 4 UART
//
//
// Functions:
// - lighting timers and UV filter
// - water temperature control (heating/cooler)
// - feeding timer
// - water level:
//    - automation of water replenishment by evaporation
//    - automation of the partial water change (PWC / TPA) routine
// - pH measurement
// - Weather Station
//
//  REMARK: This arduino (mega pro mini) uses the CH340G CI, so it is necessary to install the driver at: 
//  https://www.blogdarobotica.com/downloads/CH341SER_DRIVER_CASA_DA_ROBOTICA_BLOG_DA_ROBOTICA.EXE
// 
// The Arduino IDE can be obtained at: https://www.arduino.cc/en/software
//
// Use a micro USB cable to connect to the Arduino and select the "Arduino Mega 2560" board in the IDE.
//
// In the library manager, select:
// - wire
// - OneWire
// - Liquid Crystal
// - DallasTemperature
// - Adafruit_BMP085
// - DHT
// - EEPROM
// - RTClib
//
// Material:
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
// 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
//
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <EEPROM.h>
#include "RTClib.h"

//
// debug & testing
//
// disable alarms
#define _debugNoAlarms false 
// disable buttons
#define _debugNoButtons false
// disable timers
#define _debugNoTimers false
// disable feeding
#define _debugNoFeeding true
// disable relay
#define _debugNoRelay false
// disable water level
#define _debugNoWaterLevel false

// tests LCD (shows test message)
#define _debugLCD false   
// tests RTC (shows curent time)
#define _debugRTC false
// tests Relay (activate relay)
#define _debugRelay false
// tests buttons
#define _debugButtons false
// tests water level sensors
#define _debugWaterLevel false
// tests repostion pump
#define _debugRepositionPump false
// tests cooler
#define _debugCooler false


// reference levels
#define _levelWaterTempMin 26.0
#define _levelWaterTempMax 29.1
#define _levelWaterPhMin 7.5
#define _levelWaterPhMax 8.9
// timers start/end hours
#define _timerLightStart 6
#define _timerLightEnd 23
#define _timerUvStart 8
#define _timerUvEnd 15
#define _timerFeed1 7
#define _timerFeed2 12
#define _timerFeed3 18

//
// pins (digital)
//
#define _pinLcdRS 12        // LCD pin 4   
#define _pinLcdEnabled 11   // LCD pin 6   
#define _pinLcdD4 5         // LCD pin 11
#define _pinLcdD5 4         // LCD pin 12
#define _pinLcdD6 3         // LCD pin 13
#define _pinLcdD7 2         // LCD pin 14

#define _pinRelayHeater 20
#define _pinRelayLights 19 
#define _pinRelayUV 18
#define _pinRelayPumpSump 17
#define _pinRelayPumpDrain 16
#define _pinRelayExtra 15
#define _pinRelayCooler 14
#define _pinRelayPumpReposition 13

#define _pinWaterDrainLevelLow 23
#define _pinWaterSumpLevelLow 24

#define _pinSensorWaterTemperature 7   // sensor DS18B20 (analog)  
#define _pinSensorAirTemperature 8     // sensor DHT11   (analog)
#define _pinSensorAirPresureSDA 20     // sensor BMP180  (analog) - Arduino Mega 2560
#define _pinSensorAirPresureSCL 21     // sensor BMP180  (analog) - Arduino Mega 2560
#define _pinSensorWaterPH 6            // sensor Ph4502c (analog)

// pins (buttons and leds)
#define _pinButtonTurnOff 25
#define _pinButtonRestart 26
#define _pinButtonLights 27
#define _pinButtonTPA 28
#define _pinButtonManualCleaning 39
#define _pinButtonFeed 30 


// status codes
#define _statusShowDate 0
#define _statusShowWaterTemp1 1
#define _statusShowWaterTemp2 2
#define _statusShowWaterTemp3 3
#define _statusShowWaterPh 4
#define _statusShowAirTemp1 5
#define _statusShowAirTemp2 6
#define _statusShowForecastLocal 7
#define _statusShowFlags 8
#define _statusTPA 10
#define _statusManualCleaning 11
#define _statusOff 100
 
// Cycle start
#define _statusCycleStart _statusShowDate
// Cycle end
#define _statusCycleEnd _statusShowFlags
// first non cycle state
#define _statusCycleOff _statusTPA


#define _eepromAdrPressure 1
#define _eepromAdrFlags 2
#define _eepromFlagBitLight 1


// LCD pin outs
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
LiquidCrystal _lcd(_pinLcdRS, _pinLcdEnabled, _pinLcdD4, _pinLcdD5, _pinLcdD6, _pinLcdD7);

// Real time clock (RTC DS3231)
// Pin1 (SCL) --> Pin20
// Pin2 (SDA) --> Pin21
// Pin3 (vcc) --> 5v
// Pin4 (gnd) --> Gnd
RTC_DS3231 _clock;

// Water temperature sensor (DS18B20)
// VCC  --> Arduino 5v, plus a 4.7K resistor going from VCC to Data
// Data --> Arduino Pin7
// GND  --> Arduino Gnd
OneWire oneWire(_pinSensorWaterTemperature);
DallasTemperature _sensorWaterTemperature(&oneWire);

// Air temperature sensor (DHT11)
// Pin1 --> 5v and 10k ohm resistor
// Pin2 --> Arduino Pin8 and 10k ohm resistor
// Pin3 --> no connection
// Pin4 --> Gnd
DHT _sensorAirTemperature(_pinSensorAirTemperature, DHT11);

// Air pressure sensor (BMP180)
// Pin1 (SDA) --> Pin20
// Pin2 (SCL) --> Pin21
// Pin3 (gnd) --> Gnd
// Pin4 (vcc) --> 5v
Adafruit_BMP085 _sensorAirPresure; // (I2C)

//
// Globals: states and measured values
//
DateTime _now;
int _status = 0;
bool _statusFeeded = false;
bool _statusForceLights = false;
bool _waterDrainLevelLow = false;
bool _waterSumpLevelLow = false;
float _measureAirTemperature = 0;
float _measureAirTemperatureAvg = 0;
float _measureAirTemperatureMax = 0;
float _measureAirTemperatureMin = 100000000000;
float _measureAirHumidity = 0;
float _measureAirPressure = 0;
float _measureAirAltitude = 0;
float _measureAirSealevelPressure = 0;
float _measureWaterTemperature = 0;
float _measureWaterTemperatureAvg = 0;
float _measureWaterTemperatureMax = 0;
float _measureWaterTemperatureMin = 100000000000;
float _measureWaterPh = 6.8;
float _measureAirPressureAvg = 0;
float _measureAirPressureMax = 0;
float _measureAirPressureMin = 100000000000;


void setup() {
  //
  // Initialization
  //
  // relays
  pinMode(_pinRelayHeater, OUTPUT);
  pinMode(_pinRelayLights, OUTPUT);
  pinMode(_pinRelayUV, OUTPUT);
  pinMode(_pinRelayPumpSump, OUTPUT);
  pinMode(_pinRelayPumpDrain, OUTPUT);
  pinMode(_pinRelayExtra, OUTPUT);
  pinMode(_pinRelayCooler, OUTPUT);
  pinMode(_pinRelayPumpReposition, OUTPUT);
  digitalWrite(_pinRelayHeater, HIGH);
  digitalWrite(_pinRelayLights, HIGH);
  digitalWrite(_pinRelayPumpSump, HIGH);
  digitalWrite(_pinRelayUV, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);
  digitalWrite(_pinRelayExtra, HIGH);  
  digitalWrite(_pinRelayCooler, HIGH);
  digitalWrite(_pinRelayPumpReposition, HIGH);  

  Serial.begin(9600);

  // intialize LCD 16 x 2
  _lcd.begin(16, 2);
  if (_debugLCD){
    // LCD debug routine
    Serial.println("LCD debug mode!");
    return;
  }

  // clock
  if (! _clock.begin()) {
    Serial.println("Clock RTC DS3231 not found! Stop!");
    _lcd.setCursor(0, 0);
    _lcd.print("Clock not found!");
    while (1);
  }
  if (_clock.lostPower()) {
    Serial.println("Set clock RTC DS3231 date and time!");
    _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));  // SKETCH date and time
    //_clock.adjust(DateTime(2018, 9, 29, 15, 00, 45)); //(ANO), (MÊS), (DIA), (HORA), (MINUTOS), (SEGUNDOS)
  }

  _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("Clock RTC DS3231 Ok!");
  if (_debugRTC){
    // RTC debug routine
    _clock.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("Clock debug mode!");
    return;
  }

  // starts sensors
  _sensorWaterTemperature.begin();
  //Serial.println("Water temperature sensor DS18B20 not found!");
  
  _sensorAirTemperature.begin();
  //Serial.println("Air temperature and humidity sensor DHT11 not found!");
  
  if (!_sensorAirPresure.begin()) {
    Serial.println("Air pressure sensor BMP180 not found!");
  }
  else{
     _measureAirPressure = _sensorAirPresure.readPressure();
    
    _measureAirPressureAvg = EEPROM.read(_eepromAdrPressure); 
    if (_measureAirPressureAvg == 0) 
      _measureAirPressureAvg = _measureAirPressure;
    
    EEPROM.write(_eepromAdrPressure,_measureAirPressureAvg); 
    _measureAirPressureMax = _measureAirPressure;
    _measureAirPressureMin = _measureAirPressure;
  }

  // water levels
  pinMode(_pinWaterDrainLevelLow, INPUT);
  pinMode(_pinWaterSumpLevelLow, INPUT);
  // buttons
  pinMode(_pinButtonTurnOff, INPUT_PULLUP);
  pinMode(_pinButtonRestart, INPUT_PULLUP);
  pinMode(_pinButtonLights, INPUT_PULLUP);
  pinMode(_pinButtonTPA, INPUT_PULLUP);
  pinMode(_pinButtonManualCleaning, INPUT_PULLUP);
  pinMode(_pinButtonFeed, INPUT_PULLUP);
  delay(100);

  _statusForceLights = bitRead(EEPROM.read(_eepromAdrFlags), _eepromFlagBitLight);
  
  // set defaults 
  Restart();
}


void loop() {
  //
  // LCD loop (transition between states)
  //
  _lcd.clear();

  if (testComponents()){
    // test and debug routines only
    return;
  }

  // reads sensors data
  readData();

  // sets 1st LCD line
  _lcd.setCursor(0, 0);

  switch (_status) {
    case _statusShowAirTemp1:
      showAirTemperature1();
      break;
    case _statusShowAirTemp2:
      showAirTemperature2();
      break;
    case _statusShowForecastLocal:
      showLocalWeatherForecast();
      break;  
    case _statusShowWaterTemp1:
      showWaterTemperature(1);
      break;
    case _statusShowWaterTemp2:
      showWaterTemperature(2);
      break;
    case _statusShowWaterTemp3:
      showWaterTemperature(3);
      break;
    case _statusShowWaterPh:
      showWaterPh();
      break;
    case _statusShowFlags:
      showSystemFlags();
      break;
    case _statusManualCleaning:
      runManualCleaning();
      break;
    case _statusTPA:
      runTPA();
      break;
    case _statusOff:
      TurnOff();
      return;
    default:
      // shows date and time
      showDateTime();
      break;
  }

  // sets 2nd LCD line: alarms
  _lcd.setCursor(0, 1);
  showAlarms();
  
  // handles actions (5 seconds)
  for (int i = 0; i <= 10 && !readButtons(); i++) {
    handleTimers();
    handleAlarms();
    delay(500);
  }

  // increments states (not for unusual ones: off/cleaning/PWC)
  if (_status < _statusCycleOff) {
    _status++;

    // circular state transition
    if (_status > _statusCycleEnd)
      _status = _statusCycleStart;
  }
}


void readData() {
  //
  // read sensors data
  //
  
  // read real time clock
  _now = _clock.now();

  _measureWaterPh = 7.8;

  // level sensor open (false) => LevelLow alarm = true 
  _waterDrainLevelLow = !digitalRead(_pinWaterDrainLevelLow);
  _waterSumpLevelLow = !digitalRead(_pinWaterSumpLevelLow);

  _sensorWaterTemperature.requestTemperatures();
  _measureWaterTemperature = _sensorWaterTemperature.getTempCByIndex(0);

  _measureAirTemperature = _sensorAirTemperature.readTemperature();
  _measureAirHumidity = _sensorAirTemperature.readHumidity();

  _measureAirPressure = _sensorAirPresure.readPressure();
  _measureAirAltitude = _sensorAirPresure.readAltitude();
  _measureAirSealevelPressure = _sensorAirPresure.readSealevelPressure();

  // EWMA
  if (_measureWaterTemperatureAvg == 0)
    _measureWaterTemperatureAvg = _measureWaterTemperature;
  _measureWaterTemperatureAvg = 0.95 * _measureWaterTemperatureAvg + 0.05 * _measureWaterTemperature;
  if (_measureWaterTemperature > _measureWaterTemperatureMax) 
    _measureWaterTemperatureMax = _measureWaterTemperature;
  if (_measureWaterTemperature < _measureWaterTemperatureMin) 
    _measureWaterTemperatureMin = _measureWaterTemperature;

  if (_measureAirTemperatureAvg == 0)
    _measureAirTemperatureAvg = _measureAirTemperature;
  _measureAirTemperatureAvg = 0.95 * _measureAirTemperatureAvg + 0.05 * _measureAirTemperature;
  if (_measureAirTemperature > _measureAirTemperatureMax) 
    _measureAirTemperatureMax = _measureAirTemperature;
  if (_measureAirTemperature < _measureAirTemperatureMin) 
    _measureAirTemperatureMin = _measureAirTemperature;

  if (_measureAirPressureAvg == 0)
    _measureAirPressureAvg = _measureAirPressure;
  _measureAirPressureAvg = 0.95 * _measureAirPressureAvg + 0.05 * _measureAirPressure; 
  if (_measureAirPressure > _measureAirPressureMax) 
    _measureAirPressureMax = _measureAirPressure;
  if (_measureAirPressure < _measureAirPressureMin) 
    _measureAirPressureMin = _measureAirPressure;
}

bool readButtons() {
  //
  // reads buttons
  //
  if (_debugNoButtons){
    // disable buttons
    return false;
  }
  
  if (digitalRead(_pinButtonTurnOff) == LOW) {
    TurnOff();
    return true;
  }
  if (digitalRead(_pinButtonRestart) == LOW) {
    Restart();
    return true;
  }
  if (digitalRead(_pinButtonLights) == LOW) {
    setFlagLigth(!_statusForceLights);
    return true;
  }
  if (digitalRead(_pinButtonTPA) == LOW) {
    runTPA();
    return true;
  }
  if (digitalRead(_pinButtonManualCleaning) == LOW) {
    runManualCleaning();
    return true;
  }
  if (digitalRead(_pinButtonFeed) == LOW) {
    runManualCleaning();
    // Feed();
    return true;
  }

  // no button pressed
  return false;
}


void showDateTime()
{
  //
  // Shows current date and time
  //
  
  //if (_debugRTC){
  _now = _clock.now();  
  
  _lcd.setCursor(0, 0);
  _lcd.print(_now.day(), DEC);
  _lcd.print("/");
  _lcd.print(_now.month(), DEC);
  _lcd.print("/");
  _lcd.print(_now.year(), DEC);
  _lcd.print(" ");
  _lcd.print(_now.hour(), DEC);
  _lcd.print(":");
  _lcd.print(_now.minute(), DEC);

  int hour = _now.hour(); 
  _lcd.setCursor(0, 1);
  if (hour > 6 && hour < 12) {
    _lcd.print("Bom dia!");
  } 
  else if (hour >= 12 && hour < 18) {
    _lcd.print("Boa tarde!");
  }
  else {
    _lcd.print("Boa noite!");  
  }
}

void showAirTemperature1()
{
  //
  // Shows atmosferic data(1)
  //
  _lcd.setCursor(0, 0);
  _lcd.print("Temp Ar: ");
  _lcd.print(_measureAirTemperature);
  _lcd.print("C");
  _lcd.setCursor(0, 1);
  _lcd.print("Umid Ar: ");
  _lcd.print(_measureAirHumidity);
  _lcd.print("%");
}

void showAirTemperature2()
{
  //
  // Shows atmosferic data (2)
  //
  _lcd.setCursor(0, 0);
  _lcd.print("Pres Atm: ");
  _lcd.print(_measureAirPressure/ 100, 0);
  _lcd.print("mPa");
  
  _lcd.setCursor(0, 1);
  _lcd.print("Altitude: ");
  _lcd.print(_measureAirAltitude,0);
  _lcd.print("m");
  
  //_measureAirSealevelPressure
}

void showLocalWeatherForecast() 
{
  _lcd.setCursor(0, 0);
  _lcd.print("Forecast local:");

  _lcd.setCursor(0, 1);
  if (_measureAirPressureMax - _measureAirPressureAvg >= 600)  
    _lcd.print("Sem previsao");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 550 && _measureAirPressureMax - _measureAirPressureAvg <= 599) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 500 && _measureAirPressureMax - _measureAirPressureAvg <= 549) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 450 && _measureAirPressureMax - _measureAirPressureAvg <= 499) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 400 && _measureAirPressureMax - _measureAirPressureAvg <= 449) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 350 && _measureAirPressureMax - _measureAirPressureAvg <= 399) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 300 && _measureAirPressureMax - _measureAirPressureAvg <= 349) 
    _lcd.print("Tempo BOM");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 200 && _measureAirPressureMax - _measureAirPressureAvg <= 299) 
    _lcd.print("Tempo seco");
  else if (_measureAirPressureMax - _measureAirPressureAvg >= 100 && _measureAirPressureMax - _measureAirPressureAvg <= 199) 
    _lcd.print("Sem alteracao");
  else if (_measureAirPressureAvg - _measureAirPressureMin >= 200 && _measureAirPressureAvg - _measureAirPressureMin <= 299) 
    _lcd.print("Chuva fraca");
  else if (_measureAirPressureAvg - _measureAirPressureMin >= 300 && _measureAirPressureAvg - _measureAirPressureMin <= 399) 
    _lcd.print("Chuva forte");
  else if (_measureAirPressureAvg - _measureAirPressureMin >= 400 && _measureAirPressureAvg - _measureAirPressureMin <= 499) 
    _lcd.print("Chuva torrencial");
  else if (_measureAirPressureAvg - _measureAirPressureMin >= 500 && _measureAirPressureAvg - _measureAirPressureMin <= 599) 
    _lcd.print("TEMPESTADE");
  else if (_measureAirPressureAvg - _measureAirPressureMin >= 600)  
    _lcd.print("Sem previsao");
}



void showWaterTemperature(int idx)
{
  //
  // Shows temperature
  //
  _lcd.setCursor(0, 0);
  _lcd.print("Temp agua: ");
  _lcd.print(_measureWaterTemperature);
  _lcd.print("C");
  _lcd.setCursor(0, 1);
  switch (idx) {
    case 1:
      _lcd.print("Min: ");
      _lcd.print(_measureWaterTemperatureMin);
      break;
    case 2:
      _lcd.print("Max: ");
      _lcd.print(_measureWaterTemperatureMax);
      break;
    case 3:
      _lcd.print("Med: ");
      _lcd.print(_measureWaterTemperatureAvg);
      break;
  }
  _lcd.print("C");
}


void showWaterPh()
{
  //
  // Shows Ph
  //
  _lcd.setCursor(0, 0);
  _lcd.print("pH agua: ");
  _lcd.print(_measureWaterPh);
  _lcd.setCursor(0, 1);
  _lcd.print("Alcalin: 120 ppm");
}

void showSystemFlags()
{
  //
  // Shows system flags
  //
  _lcd.setCursor(0, 0);
  _lcd.print("Luz: ");
  if (_statusForceLights)
    _lcd.print("ON");
  else
    _lcd.print("TIMER");

  _lcd.setCursor(0, 1);
  _lcd.print("Feed: OFF");  
}

void handleTimers()
{
  //
  // Handle scheduled tasks actions
  //
  if (_debugNoTimers || _debugNoRelay) {
     return;
  }

  if (_status < _statusCycleOff) {
    int hour = _now.hour();

    // lights timer, set LOW to turn on lights
    bool lightOnTimer = hour > _timerLightStart && hour < _timerLightEnd ;
    digitalWrite(_pinRelayLights, !(lightOnTimer || _statusForceLights));

    // UV timer (8h), set LOW to turn on UV lamp)
    digitalWrite(_pinRelayUV, !(hour > _timerUvStart && hour < _timerUvEnd));

    // Feeding timer: 7, 12 and 18h
    if (hour == _timerFeed1 || hour == _timerFeed2 || hour == _timerFeed3) {
      if (!_statusFeeded)
        Feed();
    }
    else
      _statusFeeded = false;
  }
  else
  {
    // non usual mode: just ensures UV turn off
    digitalWrite(_pinRelayUV, HIGH);
  }
}

void handleAlarms()
{
  //
  // Take alarm actions
  //
  if(_debugNoAlarms || _debugNoRelay) {
    // disabled alarms
    return;
  }
  
  if (_status < _statusCycleOff ) {
    // water evaporation reposition
    // sump pump water level opened (false) -> alarm sump water level is low (true)
    _waterSumpLevelLow = !digitalRead(_pinWaterSumpLevelLow); 
    if (_debugNoWaterLevel){
       // disable reposition pump
      _waterSumpLevelLow = false;
    }
    // set pin=LOW to turn on repostion pump relay
    digitalWrite(_pinRelayPumpReposition, !_waterSumpLevelLow);

    // high temperature
    // set pin=LOW to turn on relay
    // if high temp (true) -> turn on cooler, turn of heater -> set cooler pin LOW, set heater pin HIGH 
    bool highTemp = _measureWaterTemperature > _levelWaterTempMax;
    digitalWrite(_pinRelayCooler, !highTemp);
    digitalWrite(_pinRelayHeater, highTemp);
  }
}


void showAlarms()
{
  //
  // shows alarms (second LCD display line)
  //
  if(_debugNoAlarms) {
    // disable alarms
    return;
  }

  if (_status < _statusCycleOff ) {
    
    _lcd.setCursor(0, 1);  
    if (_waterSumpLevelLow && !_debugNoWaterLevel) {
      // low level alarm
      _lcd.print("Nivel agua baixo");
    }
    else if (_measureWaterTemperature > _levelWaterTempMax) {
      // temperature alarms
      _lcd.print("Temp alta: ");
      _lcd.print(_measureWaterTemperature);
      _lcd.print("C");
    }
    else if (_measureWaterTemperature < _levelWaterTempMin) {
      // temperature alarms
      _lcd.print("Temp baixa: ");
      _lcd.print(_measureWaterTemperature);
      _lcd.print("C");
    }
    else if (_measureWaterPh < _levelWaterPhMin) {
      // pH alarms
      _lcd.print("Ph baixo: ");
      _lcd.print(_measureWaterPh);
    }  
    else if (_measureWaterPh > _levelWaterPhMax) {
      // pH alarms
      _lcd.print("Ph alto: ");
      _lcd.print(_measureWaterPh);
    }
  
    delay(100);
  }
}


void Feed()
{
  //
  // Feeding procedure (deprecated)
  //
  if (_debugNoFeeding || _debugNoRelay) {
    return;
  }

  // set pin=LOW to turn on relay
  digitalWrite(_pinRelayExtra, HIGH);
  delay(100);
  digitalWrite(_pinRelayExtra, LOW);
  delay(500);
  digitalWrite(_pinRelayExtra, HIGH);
  delay(500);
  _statusFeeded = true;
}


void runTPA()
{
  //
  // Auto TPA
  //
  delay(1000);
  // _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("T P A");
  delay(1000);  
  if (_status == _statusTPA)
    return; 
    
  _status = _statusTPA;  
  if (_debugNoRelay) {
    return;
  }

  _lcd.setCursor(0, 1);
  _lcd.print("Drenando agua...");
  delay(500);
  // set pin=LOW to turn on relay (keep only lights on)
  digitalWrite(_pinRelayHeater, HIGH);
  digitalWrite(_pinRelayLights, LOW);
  digitalWrite(_pinRelayUV, HIGH);
  digitalWrite(_pinRelayExtra, HIGH);
  digitalWrite(_pinRelayCooler, HIGH);
  digitalWrite(_pinRelayPumpSump, HIGH);
  digitalWrite(_pinRelayPumpReposition, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);
  delay(1000);

  //
  // step 1 - drain dirty water (turn on drain pump)
  //
  digitalWrite(_pinRelayPumpDrain, LOW);
  while (digitalRead(_pinWaterDrainLevelLow)) {
    // while not reach drain level low => keep draining dirty water
    delay(500);
  }
  // turn of drain pump (water is at low drained level) 
  digitalWrite(_pinRelayPumpDrain, HIGH);
  delay(1000);
  
  // ensures low water level at sump pump!
  digitalWrite(_pinRelayPumpSump, LOW);
  while (digitalRead(_pinWaterDrainLevelLow)) {
    // while not reach sump level low => keep sump pump on
    delay(500);
  }
  delay(1000);
  // turn of sump pump (sump is at low level)
  digitalWrite(_pinRelayPumpSump, HIGH);  
  delay(1000);

  
  //
  // step 2 - fills fresh water (turn on reposition pump)
  //
  _lcd.setCursor(0, 1);
  _lcd.print("Repondo agua... ");
  delay(1000);
  digitalWrite(_pinRelayPumpReposition, LOW);
  while (!digitalRead(_pinWaterSumpLevelLow)) {
    // while not reach sump level high => keep reposition pump on
    delay(500);
  }
  // turn of reposition pump (water flows to sump)
  digitalWrite(_pinRelayPumpReposition, HIGH);
  delay(1000);


  // ensure sump water level, WITH sump pump on
  _lcd.setCursor(0, 1);
  _lcd.print("Verificando...  ");
  delay(1000);
  digitalWrite(_pinRelayPumpSump, LOW);
  delay(8000);
  
  while (!digitalRead(_pinWaterSumpLevelLow)) {
    digitalWrite(_pinRelayPumpReposition, LOW);
    delay(500);
  }
  digitalWrite(_pinRelayPumpReposition, HIGH);

  // turn off both pumps
  digitalWrite(_pinRelayPumpReposition, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);

  _lcd.setCursor(0, 1);
  _lcd.print("TPA encerrado...");
  delay(5000);
  
  Restart();
}


void runManualCleaning() {
  //
  // manual cleaning: stop all until reset
  //
  delay(1000);
  // _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Limpeza manual");
  delay(1000);  
  if (_status == _statusManualCleaning)
    return; 
  
  _status = _statusManualCleaning;
  if (_debugNoRelay) {
    return;
  }

  // set pin=LOW to turn on relay (only lights on)
  digitalWrite(_pinRelayHeater, HIGH);
  digitalWrite(_pinRelayLights, LOW);
  digitalWrite(_pinRelayUV, HIGH);
  digitalWrite(_pinRelayPumpSump, HIGH);
  digitalWrite(_pinRelayPumpReposition, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);
  digitalWrite(_pinRelayCooler, HIGH);
  digitalWrite(_pinRelayExtra, HIGH);
  delay(1000);
}


void TurnOff() {
  //
  // Disable all
  //
  delay(1000);
  // _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Desligando");
  delay(1000);  
  _statusForceLights = false;
  if (_status == _statusOff)
    return; 
    
  _status = _statusOff;
  if (_debugNoRelay) {
    return;
  }

  // set pin=LOW to turn on relay (only lights on)
  digitalWrite(_pinRelayHeater, HIGH);
  digitalWrite(_pinRelayLights, LOW);
  digitalWrite(_pinRelayUV, HIGH);
  digitalWrite(_pinRelayPumpSump, HIGH);
  digitalWrite(_pinRelayPumpReposition, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);
  digitalWrite(_pinRelayCooler, HIGH);
  digitalWrite(_pinRelayExtra, HIGH);
  delay(1000);
}


void Restart() {
  //
  // Restart all
  //
  delay(1000);
  _status = _statusCycleStart;
  _statusForceLights = false;
  
  _lcd.clear();
  _lcd.setCursor(0, 0);
  _lcd.print("Iniciando...");
  delay(1000);

  // set pin=LOW to turn on relay (default: turn on heater, lights and sump pump)
  if (_debugNoRelay) {
    return;
  }
  digitalWrite(_pinRelayHeater, LOW);
  digitalWrite(_pinRelayLights, LOW);
  digitalWrite(_pinRelayPumpSump, LOW);
  digitalWrite(_pinRelayUV, HIGH);
  digitalWrite(_pinRelayPumpDrain, HIGH);
  digitalWrite(_pinRelayExtra, HIGH);  
  digitalWrite(_pinRelayCooler, HIGH);
  digitalWrite(_pinRelayPumpReposition, HIGH);  
  delay(100);
}


bool testComponents() {
  //
  // debug components
  //
  if (_debugLCD){
    // LCD testing routine: shows message
    _lcd.print("LCD Testing");
    delay(5000);
    return true;
  }
  
  if (_debugRTC){
    // RTC testing routine: shows date and time
    showDateTime();
    delay(1000);
    return true;
  }

  if (_debugRelay){
    // Relay testing routine: enable all
    _lcd.setCursor(0, 0);
    _lcd.print("Relay Testing");

    while(true){
      for(int j=_pinRelayExtra; j<=_pinRelayHeater; j++){
        _lcd.setCursor(0, 1);
        _lcd.print("pin: ");
        _lcd.print(j);
        _lcd.print("     ");
        for(int i=_pinRelayExtra; i<=_pinRelayHeater; i++){
           if(i==j)
               digitalWrite(i, LOW);
           else
               digitalWrite(i, HIGH);
           delay(100);
        }
        delay(1000);
      }
      _lcd.setCursor(0, 1);
      _lcd.print("Relay Ok");
      delay(5000);
    }    
    return true;
  }

  if (_debugButtons){
    // buttons testing routine
    _lcd.setCursor(0, 0);
    _lcd.print("button test");
    
    _lcd.setCursor(0, 1);
    if (digitalRead(_pinButtonTurnOff) == LOW) {
      _lcd.print("btn 1 - OFF");
      digitalWrite(_pinRelayHeater, LOW);
    }
    else if (digitalRead(_pinButtonRestart) == LOW) {
      _lcd.print("btn 2 - RST");
      digitalWrite(_pinRelayLights, LOW);
    }
    else if (digitalRead(_pinButtonLights) == LOW) {
      _lcd.print("btn 3 - Ligth");
      digitalWrite(_pinRelayUV, LOW);
    }
    else if (digitalRead(_pinButtonTPA) == LOW) {
      _lcd.print("btn 4 - TPA");
      digitalWrite(_pinRelayPumpSump, LOW);
    }
    else if (digitalRead(_pinButtonManualCleaning) == LOW) {
      _lcd.print("btn 5 - Manual");
      digitalWrite(_pinRelayPumpDrain, LOW);
    }
    else if (digitalRead(_pinButtonFeed) == LOW) {
      _lcd.print("btn 6 - Feed");
      digitalWrite(_pinRelayExtra, LOW);
    }
    else
    {
      // set pin=LOW to turn on relay
      digitalWrite(_pinRelayHeater, HIGH);
      digitalWrite(_pinRelayLights, HIGH);
      digitalWrite(_pinRelayUV, HIGH);
      digitalWrite(_pinRelayPumpSump, HIGH);
      digitalWrite(_pinRelayPumpDrain, HIGH);
      digitalWrite(_pinRelayExtra, HIGH);
    }
    delay(100);    
    return true;
  }

  if (_debugWaterLevel){
    // water level testing routine
    _lcd.setCursor(0, 0);
    _lcd.print("water level test");
    while(true){
      for(int i = 0; i < 30; i++){
        _waterSumpLevelLow = !digitalRead(_pinWaterSumpLevelLow);
        _lcd.setCursor(0, 1);
        _lcd.print("Sump low: ");
        if(_waterSumpLevelLow)
          _lcd.print("true  ");
        else
          _lcd.print("false ");
        delay(500);
      }
      for(int i = 0; i < 30; i++){
        _waterDrainLevelLow = !digitalRead(_pinWaterDrainLevelLow);
        _lcd.setCursor(0, 1);
        _lcd.print("Drain low: ");
        if(_waterDrainLevelLow)
          _lcd.print("true  ");
        else
          _lcd.print("false ");
        delay(500);
      }
      delay(100); 
    }        
    return true;
  }

  if (_debugRepositionPump){
    // reposition pump testing routine: turn on/off
    _lcd.setCursor(0, 0);
    _lcd.print("pump test");
    
    _lcd.setCursor(0, 1);
    _lcd.print("pump on");
    digitalWrite(_pinRelayPumpReposition,  LOW);
    delay(2000);    
    _lcd.setCursor(0, 1);
    _lcd.print("pump off");
    digitalWrite(_pinRelayPumpReposition, HIGH);
    delay(8000);    
    return true;
  }
  
  if (_debugCooler){
    // cooler fan testing routine: turn on/off
    _lcd.setCursor(0, 0);
    _lcd.print("cooler test");
    
    _lcd.setCursor(0, 1);
    _lcd.print("cooler on");
    digitalWrite(_pinRelayCooler, LOW);
    delay(5000);    
    _lcd.setCursor(0, 1);
    _lcd.print("cooler off");
    digitalWrite(_pinRelayCooler, HIGH);
    delay(10000);
    return true;
  }
  
  return false;
}


void setFlagLigth(boolean value)
{
   _statusForceLights = value;
   saveFlag(_eepromFlagBitLight, value);
}

void saveFlag(int bit, boolean value)
{
   byte flags = EEPROM.read(_eepromAdrFlags);
   bitWrite(flags,  _eepromFlagBitLight, value);
   EEPROM.update(_eepromAdrFlags, flags);
}
