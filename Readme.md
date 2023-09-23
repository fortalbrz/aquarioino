

 # Aquarium.ino (Arduino Aquarium Controller) 
(with Arduino Mega 2560 Pro Mini)

  - Making off: https://youtu.be/3CjU-o5LTEI
  - Overview: https://youtu.be/8RszBkeuUlk

 Tested on Arduino Mega 2560 pro mini (CI CH340G)
 - 70 digital I/O, 16 analog inputs, 14 PWM, 4 UART

 ### Functions
 - lighting timers and UV filter
 - water temperature control (heating/cooler)
 - feeding timer
 - water level:
    - automation of water replenishment by evaporation
    - automation of th[README.md](..%2Ffluxoino%2FREADME.md)e partial water change (PWC / TPA) routine
 - pH measurement
 - Weather Station

## Flashing the code

 **REMARK**: Arduino mega pro mini (used on testing) uses the CH340G CI, so it is necessary to install the [driver](https://www.blogdarobotica.com/downloads/H341SER_DRIVER_CASA_DA_ROBOTICA_BLOG_DA_ROBOTICA.EXE).
 
 The Arduino IDE can be obtained [here](https://www.arduino.cc/en/software).

 Use a micro USB cable to connect to the Arduino and select the "Arduino Mega 2560" board in the IDE.

 In the library manager, select:
 - wire
 - OneWire
 - Liquid Crystal
 - DallasTemperature
 - Adafruit_BMP085
 - DHT
 - EEPROM
 - RTClib

 ## Materials

 - 1 box (reused, wooden)
 - [1 arduino mega 2560 pro (mini)](https://produto.mercadolivre.com.br/MLB-2012119523-arduino-mega-pro-mini-lacrado-_JM)
 - [1 DS18B20 temperature sensor (waterproof)](https://produto.mercadolivre.com.br/MLB-1659212606-sensor-de-temperatura-ds18b20-prova-dagua-arduino-_JM)
 - [1 DHT11 temperature and humidity sensor](https://produto.mercadolivre.com.br/MLB-688214170-sensor-de-umidade-e-temperatura-dht11-com-pci-pic-arduino-_JM)
 - [1 BMP180 barometer](https://produto.mercadolivre.com.br/MLB-1335729819-barmetro-bmp180-sensor-de-presso-e-temperatura-arduino-_JM)
 - [1 pH sensor PH4502C](https://produto.mercadolivre.com.br/MLB-1894057619-modulo-sensor-de-ph-ph4502c-com-eletrodo-sonda-bnc-arduino-_JM)
 - [1 relay module with 8 channels (5 V)](https://produto.mercadolivre.com.br/MLB-1758954385-modulo-rele-rele-5v-8-canais-para-arduino-pic-raspberry-pi-_JM)
 - [2 float water level sensors and 2 x 220 olhm resistors](https://produto.mercadolivre.com.br/MLB-1540418150-sensor-de-nivel-de-agua-boia-para-arduino-esp8266-esp32-_JM)
 - 1 power source 8v 1A (any voltage between 6 V and 9 V)
 - 1 trimpot 10k
 - [6 tactile push button keys and 6 x 1k resistors](https://produto.mercadolivre.com.br/MLB-1858468268-kit-10x-chave-tactil-push-button-6x6x5mm-arduino-eletrnica-_JM)
 - [6 female recessed sockets](https://produto.mercadolivre.com.br/MLB-1844503844-6-tomada-embutir-fmea-preta-3-pinos-10a-painel-aparelho-_JM)
 - 1 led with
 - 1 power source 12v, 1 fan 12v (cooler), 1 water pump 12v (reposition pump)
 - flexible cab
 
## Circuit Connections

### Arduino (Mega 2560 pro mini)

| group            | macro                      | pin (default) | connection                                                                  | 
|------------------|----------------------------|---------------|-----------------------------------------------------------------------------|
| LCD              | _pinLcdRS                  | 12            | LCD pin 4 (see LCD connections bellow)                                      |
|                  | _pinLcdEnabled             | 11            | LCD pin 6 (see LCD connections bellow)                                      |
|                  | _pinLcdD4                  | 5             | LCD pin 11 (see LCD connections bellow)                                     |
|                  | _pinLcdD5                  | 4             | LCD pin 12 (see LCD connections bellow)                                     |
|                  | _pinLcdD6                  | 3             | LCD pin 13 (see LCD connections bellow)                                     |
|                  | _pinLcdD7                  | 2             | LCD pin 14 (see LCD connections bellow)                                     |
| Relay            | _pinRelayHeater            | 20            | heater relay pin (power plug)                                               |
|                  | _pinRelayLights            | 19            | lights relay pin (power plug)                                               |
|                  | _pinRelayUV                | 18            | UV filter relay pin (power plug)                                            |
|                  | _pinRelayPumpSump          | 17            | Sump pump relay pin (power plug)                                            |
|                  | _pinRelayPumpDrain         | 16            | drain pump relay pin (power plug) - automatic TPA                           |
|                  | _pinRelayExtra             | 15            | relay pin (power plug) - (extension: 8 power plugs)                         |
|                  | _pinRelayCooler            | 14            | cooler relay pin (12V power source into cooler fan)                         |
|                  | _pinRelayPumpReposition    | 13            | water reposition pump relay pin (power plug)                                |
| Water Level      | _pinWaterDrainLevelLow     | 23            | water level sensor (upper level)                                            |
|                  | _pinWaterSumpLevelLow      | 24            | water level sensor (lower level)                                            |
| Sensors          | _pinSensorWaterTemperature | 7             | sensor DS18B20 (analog) - see all sensor connections bellow                 |
|                  | _pinSensorAirTemperature   | 8             | sensor DHT11   (analog) - see all sensor connections bellow                 |
|                  | _pinSensorAirPresureSDA    | 20            | sensor BMP180  (analog) - Arduino Mega 2560                                 |
|                  | _pinSensorAirPresureSCL    | 21            | sensor BMP180  (analog) - Arduino Mega 2560                                 |
|                  | _pinSensorWaterPH          | 6             | sensor Ph4502c (analog)                                                     |
| Buttons and leds | _pinButtonTurnOff          | 25            | tactile button (pull-up) for turn off all features                          |
|                  | _pinButtonRestart          | 26            | tactile button (pull-up) for restart                                        |
|                  | _pinButtonLights           | 27            | tactile button (pull-up) for turn on/off the lights                         |
|                  | _pinButtonTPA              | 28            | tactile button (pull-up) for start TPA routine                              |
|                  | _pinButtonManualCleaning   | 39            | tactile button (pull-up) for turn off pumps, heater and leave lights on [1] |
|                  | _pinButtonFeed             | 30            | tactile button (pull-up) for manual feeding                                 | 

 [1] intended to water aspiration (manual)

### LCD pin outs
| Pin | Connection                                                             |
|-----|------------------------------------------------------------------------|
| 1   | Gnd                                                                    |
| 2   | 5v                                                                     |
| 3   | 10k ohm potentiometer middle pin (the other two pins go to 5v and Gnd) |
| 4   | Arduino Pin 12                                                         |                                                           
| 5   | Gnd                                                                    |
| 6   | Arduino Pin 11                                                         |                                                           
| 7   | no connection                                                          |                                                         
| 8   | no connection                                                          |                                                         
| 9   | no connection                                                          |                                                         
| 10  | no connection                                                          |                                                         
| 11  | Arduino Pin 5                                                          |                                                          
| 12  | Arduino Pin 4                                                          |                                                           
| 13  | Arduino Pin 3                                                          |                                                          
| 14  | Arduino Pin 2                                                          |                                                          
| 15  | 5v                                                                     |                                                                     
| 16  | Tactile button (other side of tack button goes to Gnd)                 |                 

### Real time clock (RTC DS3231)

| Pin      | Connection     |
|----------|----------------|
| 1 (SCL)  | Arduino Pin 20 |
| 2 (SDA)  | Arduino Pin 21 |
| 3 (vcc)  | 5v             |
| 4 (gnd)  | Gnd            | 

### Water temperature sensor (DS18B20)

| Pin     | Connection                                              |
|---------|---------------------------------------------------------|
| VCC     | Arduino 5v, plus a 4.7K resistor going from VCC to Data |
| Data    | Arduino Pin 7                                           |
| GND     | Arduino Gnd                                             |                                            

### Air temperature sensor (DHT11)

| Pin | Connection                       |
|----|-----------------------------------|
| 1  | 5v and 10k ohm resistor           |
| 2  | Arduino Pin8 and 10k ohm resistor | 
| 3  | no connection                     |
| 4  | Gnd                               |



## Reference Levels 

Hard coded setup

| macro              | value (default) | description                                         |
|--------------------|-----------------|-----------------------------------------------------|
| _levelWaterTempMin | 26.0            | minimum water temperature in Celsius (alarm/heater) |
| _levelWaterTempMax | 29.1            | maximum water temperature in Celsius (alarm/cooler) |
| _levelWaterPhMin   | 7.5             | minimum Ph (alarm)                                  |
| _levelWaterPhMax   | 8.9             | maximum Ph (alarm)                                  |
| _timerLightStart   | 6               | ligth turn on time (hour of the day)                |               
| _timerLightEnd     | 23              | ligth turn off time (hour of the day)               |
| _timerUvStart      | 8               | UV filter turn on time (hour of the day)            |
| _timerUvEnd        | 15              | UV filter turn off time (hour of the day)           |
| _timerFeed1        | 7               | 1st Feeding time (hour of the day)                  |
| _timerFeed2        | 12              | 2nd Feeding time (hour of the day)                  |
| _timerFeed3        | 18              | 3th Feeding time (hour of the day)                  |

## Flags (debug)

For information on test routines, checks the [video](https://youtu.be/3CjU-o5LTEI) (pt-BR).

| flag                 | description                                  |
|----------------------|----------------------------------------------|
| _debugNoButtons      | disable all buttons                          |
| _debugNoTimers       | disable all timers                           |
| _debugNoFeeding      | disable feeding                              |
| _debugNoRelay        | disable all relays                           |
| _debugNoWaterLevel   | disable water level monitor                  |
| _debugLCD            | LCD test routine (shows test message on LCD) |
| _debugRTC            | RTC test routine (shows current time on LCD) |
| _debugRelay          | relays test routine (activates all relays)   |
| _debugButtons        | buttons test routine                         |
| _debugWaterLevel     | water level sensors test routine             |
| _debugRepositionPump | reposition pump test routine                 |
| _debugCooler         | cooler (fan) test routine                    |


---
 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
