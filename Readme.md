

# Aquarium.ino 

## Arduino Aquarium Automation (with Home Assistant) 

**(with Arduino Mega 2560 Pro Mini and NodeMCU ESP-32)**

  - Making off video: version 2.0 ([version 1.0](https://youtu.be/3CjU-o5LTEI))
  - Overview video: version 2.0 ([version 1.0](https://youtu.be/8RszBkeuUlk))

 Tested on Arduino Mega 2560 pro mini (with IC CH340G)
 - 70 digital I/O, 16 analog inputs, 14 PWM, 4 UART


**source code**: https://github.com/fortalbrz/aquarioino ([backlog](https://github.com/users/fortalbrz/projects/1))

### Related Projects:

  - [Betta.ino](https://github.com/fortalbrz/aquarioino/tree/main/bettaino): light version of *aquarium.ino* (uses only a NodeMCU and a 4-channels relay module)
  - [Wellspring.ino](https://github.com/fortalbrz/aquarioino/tree/main/wellspring): small "Buddha" water fountain (with a aquarium water pump and led lights)


### Change log

   - version 2.0: IoT 
     - home assistant integration (MQTT)
     - 200 programmable timers
     - refactored code for enabling/disabling most of the features (adaptation for use needs)
     - stepper motor feeder
   - version 1.0: standalone controller 
     - water temperature control (cooler/heater)
     - pH measurements
     - relays timers (lightning, UV filter)
     - automated partial water change (PWC) routine     
     - automated water replenishment (due evaporation losses)
     - feeding control (timers)
     - water levels, water temperature and pH alarms
     - weather station (air temperature, humidity and pressure sensors, local / standalone forecasting)
     - LCD display
     

## Features:
- fully integrated with home assistant (MQTT) or standalone functionality (*optional, see [config flags](https:/github.com/fortalbrz/aquarioino/blob/main/README.md#configuration-flags)*)
- automation of the partial water change (PWC) routine (*optional, see [config flags](https:/github.com/fortalbrz/aquarioino/blob/main/README.md#configuration-flags)*)
- automation of water replenishment, due evaporation losses (*optional, see [config flags](https:/github.com/fortalbrz/aquarioino/blob/main/README.md#configuration-flags)*)
- lightning and UV filter turn on/off timers (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
- water temperature control (heating/cooler fan) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
- pH measurement (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
- up to 200 standalone timers, fully remotely programmable with MQTT: (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
  - lightning, UV filter, heater, cooler, pumps (i.e., all relays) turn on/off timers  
  - feeding timers
- alarms: (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
  - temperature (max/min)
  - pH (max/min)
  - water levels (aquarium low water level, sump pump low water level)
- home assistant sensors: (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
  - water temperature
  - water pH
  - water levels (aquarium and sump)
  - air temperature
  - air humidity
  - air pressure
  - weather station forecasting
- home assistant switches: (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
  - lightning switch
  - feeding button
  - UV filter switch
  - heater switch
  - cooler fan switch
  - sump,  "drain" and "water reposition" pumps switches
  - enables/disables timers and feeding switches
  - start partial water change (PWC) routine button 
  - turn off / restart / "manual cleaning" / save configurations buttons
- weather station (standalone weather forecasting) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*, requires the barometer sensor)
- displays information on LCD 16 x 2 (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
- push buttons: (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
  - lightning turn on/off button
  - partial water change (PWC) routine button
  - manual cleaning button 
  - turn off button
  - restart  button
- protects sump pump of running out of water
- local time synchronization with home assistant: home assistant as "time server" (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)


 ## Materials
 
 - [1 arduino mega 2560 pro (mini)](https://produto.mercadolivre.com.br/MLB-2012119523-arduino-mega-pro-mini-lacrado-_JM)
 - [1 temperature sensor DS18B20](https://produto.mercadolivre.com.br/MLB-1659212606-sensor-de-temperatura-ds18b20-prova-dagua-arduino-_JM) (waterproof) and 1 resistor 4.7k olhms (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 temperature and humidity sensor DHT11](https://produto.mercadolivre.com.br/MLB-688214170-sensor-de-umidade-e-temperatura-dht11-com-pci-pic-arduino-_JM) and 1 resistor 10k olhms (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 barometer sensor BMP180](https://produto.mercadolivre.com.br/MLB-1335729819-barmetro-bmp180-sensor-de-presso-e-temperatura-arduino-_JM) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 pH sensor PH4502C](https://produto.mercadolivre.com.br/MLB-1894057619-modulo-sensor-de-ph-ph4502c-com-eletrodo-sonda-bnc-arduino-_JM) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 relay module with 8 channels (5v)](https://produto.mercadolivre.com.br/MLB-1758954385-modulo-rele-rele-5v-8-canais-para-arduino-pic-raspberry-pi-_JM) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [2 float water level sensors](https://produto.mercadolivre.com.br/MLB-1540418150-sensor-de-nivel-de-agua-boia-para-arduino-esp8266-esp32-_JM) and 2 x resistors 220 olhm  (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 cooler fan 12v](https://produto.mercadolivre.com.br/MLB-777458962-micro-ventilador-80x80x25-gc-fan-cooler-12v-015a-80mm-_JM) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [2 immersible flow water pump](https://produto.mercadolivre.com.br/MLB-2205913740-motor-bomba-submersa-para-aquario-fontes-agua-bivolt-_JM) (cheap / low flow pump of around 3w - *optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 external water pump 12v](https://produto.mercadolivre.com.br/MLB-2138007025-mini-bomba-6v-a-12v-agua-ar-vacuo-aquario-rs-385-_JM) (*water / air / vacuum*, i.e. "water reposition" pump - *optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [1 LCD 16 x 2 display module](https://produto.mercadolivre.com.br/MLB-2679096309-display-lcd-16x2-1602-fundo-verde-_JM) and 1 trimpot 10k olhm (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*) 
 - [6 tactile push button keys](https://produto.mercadolivre.com.br/MLB-1858468268-kit-10x-chave-tactil-push-button-6x6x5mm-arduino-eletrnica-_JM) and 6 x resistors 1k olhm (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - [6 female recessed sockets](https://produto.mercadolivre.com.br/MLB-1844503844-6-tomada-embutir-fmea-preta-3-pinos-10a-painel-aparelho-_JM) (*optional: pretty relay outputs on wooden case...*) 
 - [1 stepper motor with driver](https://produto.mercadolivre.com.br/MLB-2205913740-motor-bomba-submersa-para-aquario-fontes-agua-bivolt-_JM) (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*)
 - 1 led and resistor 10k olhm (*optional: indicates* **power on**) 
 - 1 power source 8v 1A (any voltage between 6 V and 9 V: arduino power source, i.e., Vin)
 - 1 power source 12v 1A (optional: cooler and water pump independent 12v power source)
 - flexible cab (agw22)
 - 1 box (reused, wooden - as you wish...)
 - plastic hoses (connect bumps)
 - 1 large plastic water container (reused, *optional as water reposition container*)

**remark**: assuming that sump tank, sump pump, sump water buoy, etc are already in place...

## Flashing the code

   - the *Arduino Mega Pro Mini* uses the USB/serial IC *CH340G*, so it's necessary to install the Windows driver: 
       - [CH340G driver](http:|bit.ly/44WdzVF) (windows 11 compatible)
       - driver installation instructions ([pt-BR](http:|bit.ly/3ZqIqc0))
   - Download [Arduino IDE](https://www.arduino.cc/en/software) (2.0) 
   - uses a micro USB cable to connect to the Arduino and select the "Arduino Mega or Mega 2560" board in the IDE ("*Tools*" -> "*Board*").
   - in the library manager, select and install:
       - "OneWire" by Jim Studt, Tom Pollard, Robin James, etc
       - "LiquidCrystal" by Arduino, Adafuit
       - "DallasTemperature" by Milles Burtton, etc
       - "Adafruit BMP085 Library" by Adafruit
       - "DHT sensor library" by Adafruit
       - "RTClib" by Adafruit
       - "home-assistant-integration" by David Chyrzynsky


 
## Assembly 

### Water level sensors

The water levels sensors are used for:
 - protects the sump pump (preventing it from running without water)
 - automation of water replenishment due evaporation losses
 - automation of the partial water change (PWC) routine

The water levels sensors are optional, in the sense that their use can be disabled using the [configuration flags](https://github.com/fortalbrz/aquarioino#configuration-flags). 
However, the features above will be suppressed. As shown in the figure bellow, the sensors are expected to be mounted as *closed* for full water level and *opened* for low water level. 

![water sensor](https://github.com/fortalbrz/aquarioino/blob/main/water_level_sensor_001.jpg?raw=true)

### Pumps and sensors schema

The figure bellow, shows the main solution components with focus on water pumps and water level sensors:

![solution diagram](https://github.com/fortalbrz/aquarioino/blob/main/water_level_diagram.png?raw=true)

- **"drain" pump** and **"drain" water level sensor** are mounted at aquarium/fisk tank as show (with the "drain" water level sensors at maximum aquarium level)
- **"sump" pump** and **"sump" water level sensor** are mounted at sump tank as show (with the "sump" water level sensors at minimum level for sump pump safety)
- **"water reposition" pump** is mounted a external water container ("water reposition tank)

The **"drain" water level sensor** and **"water reposition" pump** work together in the automation of water replenishment due evaporation losses (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*). 
Whenever the water level drops (at water level sensor), the water reposition pump pushes more water into the fisk tank (keeping it stable).
This feature is optional and can be disabled using the [configuration flags](https://github.com/fortalbrz/aquarioino#configuration-flags). 
In that case, "water reposition container", "water reposition pump" and the water level sensor can be dismissed. 
Notice that the flow performance demand over the "water reposition pump" is very low, and a really cheap / low flow pump (~ 3w) is strongly recommended.  

As expected, the **"sump" pump** pushed water from sump back into fisk tank. The **"sump" water sensor level** 
turn of the sump pump case water level is to low (preventing it from running without water).

More over, **"sump" pump**, **"sumo" water level sensor**, **"drain" pump** and **"drain" water level sensor** 
work together in the automation of the partial water change (PWC) routine (*optional, see [config flags](https://github.com/fortalbrz/aquarioino#configuration-flags)*). 
Using a ingenious teamwork, the solution perform 2 main steps:
   - *step 1*: removes dirt water from aquarium / fisk tank into waste/sink
   - *step 2*: pushes water from the external water container (with conditioned water) into aquarium / fisk tank, until the original water level.

This feature is also optional and can be disabled using the [configuration flags](https://github.com/fortalbrz/aquarioino#configuration-flags). 
In that case, "water reposition container", "water reposition pump", "drain" pump and "drain" water level sensor can be dismissed. 
Notice that the flow performance demand over the "drain pump" is also very low (a slower PWC is better for the fishes), and a really cheap / low flow pump (~ 3w) is also strongly recommended.  

Furthermore, the code was designed to use a minimum amount of water level sensors. However, you can improve the setup adding independent water level sensors for maximum aquarium level and PWC stop level.
You can also include a water level sensor at external water container ("water reposition tank") in order to alarm and/or protect the "water reposition" pump.

### Feeder

Using feeder feature (ENABLE_FEEDING true) requires an fish food container that can dispense it to the fisk tank. There are two possible options:

- integrates an external feeder (e.g., Boyu feeder) (FEEDING_AS_PUSH_BUTTON true)
- making a feeder from scratch using a stepper motor (FEEDING_AS_PUSH_BUTTON false)

#### Integrating external feeder

This configuration (FEEDING_AS_PUSH_BUTTON true) assumes to control an external feeder (e.g., Boyu - that I already have!), 
and requires disassembly it and a little wiring. Connects (parallel) the "feed" push button of your feeder (both terminals) 
to the "normally open" pins of your relay pins (RELAY_FEEDER_PIN). 
Therefore, the relay can simulate a user button press (and the feeder push button still working independently, just as before...).

#### Stepper motor feeder

This configuration (FEEDING_AS_PUSH_BUTTON false) assumes making a feeder from scratch using a stepper motor (and driver).  
On that step, the stepper motor will the specified number of turns in order to feed the fishes.
The simpler way of do the "mechanical" parts of the feeder is to fix and small plastic bottle on stepper motor axis.
Make one hole (transversal) in the bottle in order to pass the food as the bottle rotates,  
like [this](https://www.instructables.com/Simplest-Automatic-Fish-Feeder/) or [this](https://www.instructables.com/Arduino-Fish-Feeder/). 
Alternatively, you can use any kind of [drill](https://hackaday.com/2014/10/14/diy-auto-fish-feeder-feeds-fish-automatically/) or even [wire](https://flourishingplants.com/how-to-make-an-automatic-fish-feeder-with-simple-materials/) do drive the food.
In this step up, the relay pin (RELAY_FEEDER_PIN) is free to be used as an extra relay for any application 
(the same if feeding routine is disabled at all - ENABLE_FEEDING false).


# Configuration flags

| flag / macro                         | default            | description                                                                                                                               |
|--------------------------------------|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| USE_LCD_DISPLAY                      | true               | enables/disables LCD display (disable it to not use the LCD display)                                                                      |
| USE_HOME_ASSISTANT                   | true               | enables/disables home assistant integration (disable it to not use the LAN ethernet module)                                               |
| USE_WATER_PH_SENSOR                  | true               | enables/disables water pH sensor (disable it to not use the Ph4502c sensor)                                                               |
| USE_WATER_TEMPERATURE_SENSOR         | true               | enables/disables water temperature sensor (disable it to not use the DS18B20 temperature sensor)                                          |
| USE_AIR_TEMPERATURE_SENSOR           | true               | enables/disables air temperature and humidity sensor (disable it to not use the DHT11 sensor)                                             |
| USE_AIR_PRESSURE_SENSOR              | true               | enables/disables air pressure sensor (disable it to not use the BMP085 sensor)                                                            |
| USE_WATER_LEVEL_SENSORS              | true               | enables/disables water level sensors (disable it to not use the water level sensors)                                                      |
| USE_RTC_CLOCK                        | true               | enables/disables clock (disable it to not use the RTC module)                                                                             |
| USE_RELAYS                           | true               | enables/disables relays (disable it to not use the relay module)                                                                          |
| USE_PUSH_BUTTONS                     | true               | enables/disables push buttons (disable it to not use the push buttons)                                                                    |
| USE_STANDALONE_TIMERS                | true               | enables/disables standalone timers (you may use home assistant calendars automation instead, as you wish...)                              |
| USE_EEPROM                           | true               | enables/disables EEPROM (saves configuration data with persistence - recommended)                                                         |
| ENABLE_FEEDING                       | true               | true for uses "feeding" relay as feeder, false for uses it as regular extra relay                                                         |
| ENABLE_FEEDING_TIMER                 | true               | enables/disables feeding timers                                                                                                           |
| ENABLE_LIGHTS_TIMER                  | true               | enables/disables lights timers                                                                                                            |
| ENABLE_UV_FILTER_TIMER               | true               | enables/disables UV filter timers                                                                                                         |
| FEEDING_AS_PUSH_BUTTON               | true               | true for feeding using a external feeder push button or false for use a stepper motor as feeder (see above), requires ENABLE_FEEDING true |
| ENABLE_SENSOR_ALARMS                 | true               | enables/disables sensor alarming                                                                                                          |
| ENABLE_WEATHER_STATION               | true               | enables/disables standalone weather station (requires air pressure sensor BMP085 to be enabled)                                           |
| MAX_NUMBER_OF_TIMERS                 | 50                 | maximum number of programmable timers  (in range: 5 to 200)                                                                               |
| MQTT_BROKER_ADDRESS                  |                    | MQTT broker server ip (e.g. mosquitto broker)                                                                                             |
| MQTT_BROKER_PORT                     | 1883               | MQTT broker port                                                                                                                          |
| MQTT_USERNAME                        |                    | MQTT username, can be omitted if not needed                                                                                               |
| MQTT_PASSWORD                        |                    | MQTT password, can be omitted if not needed                                                                                               |
| MQTT_DISCOVERY_TOPIC                 | "homeassistant"    | MQTT discovery topic prefix                                                                                                               |
| MQTT_STATES_TOPIC                    | "aquarioino/state" | MQTT state topic prefix                                                                                                                   |
| MQTT_COMMAND_TOPIC                   | "aquarioino/cmd"   | MQTT topic for custom text commands                                                                                                       |
| MQTT_TIME_SYNC_TOPIC                 | "ha/datetime"      | home assistant MQTT topic that broadcast current date/time (see bellow)                                                                   |
| MQTT_AVAILABILITY_TIME               | 60000              | time to send MQTT availability (default: 1 min)                                                                                           |
| FEEDER_STEPPER_MOTOR_PULSER_PER_TURN | 200                | number of pulses per turn on stepper motor feeder (see above), requires FEEDING_AS_PUSH_BUTTON false                                      |
| STEPPER_MOTOR_DELAY                  | 700                | time delay (in microseconds) between the motor steps (use it for change the rotation speed), requires FEEDING_AS_PUSH_BUTTON false        |
| FEEDER_TURNS                         | 1                  | default number of stepper motor feeder turns (complete cycles), requires FEEDING_AS_PUSH_BUTTON false                                     |
| LAMBDA_EWMA                          | 0.95               | exponential weighted mean average (EWMA) lambda (used on averaged measurements)                                                           |
| DEBUG_MODE                           | false              | enable/disables serial debugging messages                                                                                                 |
| DEBUG_LCD_ROUTINE                    | false              | runs testing routine for LCD display (shows a test message)                                                                               |
| DEBUG_RTC_ROUTINE                    | false              | runs testing routine for RTC (shows current time)                                                                                         |
| DEBUG_RELAY_ROUTINE                  | false              | runs testing routine for Relay (activate relay channels in sequence)                                                                      |
| DEBUG_PUSH_BUTTONS_ROUTINE           | false              | runs testing routine for push buttons                                                                                                     |
| DEBUG_WATER_LEVEL_SENSORS_ROUTINE    | false              | runs testing routine for water level sensors                                                                                              |
| DEBUG_WATER_REPOSITION_PUMP_ROUTINE  | false              | runs testing routine for reposition pump                                                                                                  |
| DEBUG_COOLER_FAN_ROUTINE             | false              | runs testing routine for cooler fan                                                                                                       |



For information on test routines, checks the [video](https://youtu.be/3CjU-o5LTEI) (pt-BR).

# Home Assistant

The solution will be discoverable as MQTT device with:

## MQTT sensors

| Name                   | uid                                        | description                  |
|------------------------|--------------------------------------------|------------------------------|
| Tank Water Temperature | sensor.aquarioino_water_temperature_sensor | water temperature (celsius)  |
| Tank pH                | sensor.aquarioino_water_ph_sensor          | water pH                     |
| Air Temperature        | sensor.aquarioino_air_temperature_sensor   | air temperature (celsius)    |
| Humidity               | sensor.aquarioino_air_humidity_sensor      | air humidity (%)             |
| Atmospheric Pressure   | sensor.aquarioino_air_pressure_sensor      | atmospheric pressure (mPa)   |
| Tank State             | sensor.aquarioino_state_sensor             | running / off / pwc / manual |
| Weather Forecast       | sensor.aquarioino_weather_forecast_sensor  | local weather forecasting    |

## MQTT switches and buttons

| Name                 | uid                                      | description                                           |
|----------------------|------------------------------------------|-------------------------------------------------------|
| Tank Light           | light.aquarioino_lights                  | lights switch (on/off)                                |
| Tank Heater          | switch.aquarioino_heater_switch          | heater switch (on/off)                                |
| Tank Cooler          | switch.aquarioino_cooler_switch          | cooler switch (on/off)                                |
| Tank UV Filter       | switch.aquarioino_uv_filter_switch       | UV filter switch (on/off)                             |
| Tank Timers          | switch.aquarioino_timers_switch          | timers switch (enables/disables timers)               |
| Tank Feeder          | switch.aquarioino_feeder_timers_switch   | feeder timers switch (enables/disables feeding)       |
| Tank Feed            | button.aquarioino_feed_button            | feed the fishes button                                |
| Tank Turn Off        | button.aquarioino_turn_off_button        | turn off button (press "restart" to go back)          |
| Tank Restart         | button.aquarioino_restart_button         | restart button                                        |
| Tank PWC             | button.aquarioino_pwc_button             | start automated partial water change (PWC) button     |
| Tank Manual Cleaning | button.aquarioino_manual_cleaning_button | "manual cleaning" button (press "restart" to go back) |
| Save Tank Setup      | button.aquarioino_save_button            | "save" configurations on EEPROM                       |



## MQTT configurations (numbers)

| Name                 | uid                               | description                            |
|----------------------|-----------------------------------|----------------------------------------|
| Tank Temperature Max | number.aquarioino_temperature_max | high water temperature alarm (celsius) |
| Tank Temperature Min | number.aquarioino_temperature_min | low water temperature alarm (celsius)  |
| Tank pH Max          | number.aquarioino_ph_max          | high pH alarm                          |
| Tank pH Min          | number.aquarioino_ph_min          | low pH alarm                           |
| Tank Feeder Turns    | number.aquarioino_feeder_turns    | number of stepper motor feeder turns   |


## MQTT topics

- **aquarioino/available**: sensors availability ("*online*" / "*offline*")
- **aquarioino/cmd**: pushes commands to solution
  - "off": turn off the solution ("restart" to go back)
  - "restart": reboot the solution
  - "pwc": starts the water partial change (PWC) routine
  - "manual": enter the "manual cleaning" state ("restart" to go back)
  - "feed": feeds the fishes
  - "turn on/off: [relay name]": turn on/off relay by name
    - relay name: "light", "heater", "filter", "cooler", "repo", "sump", "drain", "feed"
  - "enable/disable timers": enables/disables relay timers
  - "enable/disable feeder": enables/disables feeding timers
  - "relays on/off": turn on/off all relays   
  - "enable/disable timer: [timer id]": disable timer by timer "id" (see "states" for gets "timer id", e.g., 4)
  - "add timer: [hh] [mm] [hh] [mm] [relay name]": adds new relay timer with parameters:
    - turn on hour [hh] (in range 0 to 23, e.g., 12)
    - turn on minute [mm] (in range 0 to 59, e.g., 30)
    - turn off hour [hh] (in range 0 to 23, e.g., 14)
    - turn off minute [mm] (in range 0 to 59, e.g., 00)
    - relay name: "light", "heater", "filter", "cooler", "repo", "sump", "drain", "feed" (feeder timers ignores the turn off data and executes once at turn on data)
  - "update timer: [timer id] [hh] [mm] [hh] [mm]": updates relay timer by timer "id"
    - timer id: see "states" for gets "timer id" (e.g., 4)
    - turn on hour [hh] (in range 0 to 23, e.g., 12)
    - turn on minute [mm] (in range 0 to 59, e.g., 30)
    - turn off hour [hh] (in range 0 to 23, e.g., 14)
    - turn off minute [mm] (in range 0 to 59, e.g., 00)
  - "delete timer: [timer id]": delete timer by timer "id" (e.g. 4)
  - "delete all timers": deletes all relay timers (warning: all timers will be erased)
  - "default timers": sets default relay timers (i.e. timers "factory reset", warning: all timers will be erased)
  - "save": saves current configurations on EEPROM
  - "states": publishes internal states at topic **aquarioino/state** (see bellow)

- **aquarioino/state**: retrieves states as json
   ```
        {
           "timers_enabled": "on",
           "feeding_timers_enabled": "on",
           "feeder_turns": 1
           "temp": 26.3,
           "temp_max": 29.3,
           "temp_min": 23.3,
           "temp_avg": 27.3,
           "ph": 6.8,
           "ph_max": 6.9,
           "ph_min": 6.7,
           "ph_avg": 6.8,
           "air_temp": 28.3,
           "air_temp_max": 31.3,
           "air_temp_min": 23.3,
           "air_temp_avg": 27.3,
           "humidity": 68,
           "humidity_max": 80,
           "humidity_min": 48,
           "humidity_avg": 71,
           "pressure": 1004,
           "pressure_max": 1080,
           "pressure_min": 1001,
           "pressure_avg": 1040,
           "altitude": 80,
           "sealevel_pressure": 1040,
           "relays": {
                "light": "on",
                "heater": "on",
                "filter": "off",
                "cooler": "off", 
                "repo": "off",
                "sump": "off",
                "drain": "off",
                "feed": "off"
           },
           "timers": {
                "timer_01": {
                     "id": 1,
                     "enabled": "on", 
                     "relay": "light", 
                     "turn on": "06:00:00", 
                     "turn off": "23:00:00"},
                 "timer_02": {
                     "id": 2,
                     "enabled": "on", 
                     "relay": "filter", 
                     "turn on": "08:00:00", 
                     "turn off": "15:00:00"},
                 "timer_03": {
                     "id": 3,
                     "enabled": "on", 
                     "relay": "feed", 
                     "turn on": "07:00:00", 
                     "turn off": "00:00:00"},
                 "timer_04": {
                     "id": 4,
                     "enabled": "on", 
                     "relay": "feed", 
                     "turn on": "12:00:00", 
                     "turn off": "00:00:00"}
                 "timer_05": {
                     "id": 5,
                     "enabled": "on", 
                     "relay": "feed", 
                     "turn on": "18:00:00", 
                     "turn off": "00:00:00"}                    
                 ...           
           },   
        }
   ```
   
- **ha/datetime**: (optional) creates a automation on home assistant in order to synchronize data & time with the solution

```
           alias: MQTT date time sync
           description: "synchronize data & time on client MQTT devices"
           trigger:
             - platform: time_pattern
               minutes: /1
           condition: []
           action:
             - service: mqtt.publish
               data:
                 qos: "0"
                 retain: false
                 topic: ha/datetime
                 payload_template: "{{ now().timestamp() | timestamp_custom('%d %m %Y %H %M %S') }}"
           mode: single
```

## Circuit wiring

### Arduino (Mega 2560 pro mini)

| group            | macro                                | pin (default) | connection                                                                                               | 
|------------------|--------------------------------------|---------------|----------------------------------------------------------------------------------------------------------|
| LCD              | LCD_PIN_RS                           | 12            | LCD pin 4 (see LCD connections bellow)                                                                   |
|                  | LCD_PIN_ENABLED                      | 11            | LCD pin 6 (see LCD connections bellow)                                                                   |
|                  | LCD_PIN_D4                           | 5             | LCD pin 11 (see LCD connections bellow)                                                                  |
|                  | LCD_PIN_D5                           | 4             | LCD pin 12 (see LCD connections bellow)                                                                  |
|                  | LCD_PIN_D6                           | 3             | LCD pin 13 (see LCD connections bellow)                                                                  |
|                  | LCD_PIN_D7                           | 2             | LCD pin 14 (see LCD connections bellow)                                                                  |
| Relay            | RELAY_HEATER_PIN                     | 20            | heater relay pin (normally open, from 110vac to case power plug)                                         |
|                  | RELAY_LIGHTS_PIN                     | 19            | lights relay pin (normally open, from 110vac to case power plug)                                         |
|                  | RELAY_FILTER_UV_PIN                  | 18            | UV filter relay pin (normally open, from 110vac to case power plug)                                      |
|                  | RELAY_SUMP_PUMP_PIN                  | 17            | sump pump relay pin (normally open, from 110vac to case power plug)                                      |
|                  | RELAY_DRAIN_PUMP_PIN                 | 16            | "drain" pump relay pin (normally open, from 110vac to case power plug)                                   |
|                  | RELAY_WATER_REPOSITION_PUMP_PIN      | 13            | water reposition pump relay pin (normally open, from 12vdc power source to water reposition pump)        |
|                  | RELAY_COOLER_FAN_PIN                 | 14            | cooler relay pin (normally open, 12vdc power source into cooler fan terminal)                            |
|                  | RELAY_FEEDER_PIN                     | 15            | feeder relay pin (normally open, to boch terminal of external feeder 'feed' push button)                 |
| Water Level      | DRAIN_WATER_LOW_LEVEL_SENSOR_PIN     | 23            | "drain" water level sensor terminal (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc)    |
|                  | SUMP_WATER_LOW_LEVEL_SENSOR_PIN      | 24            | "sump" water level sensor terminal (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc)     |
| Sensors          | WATER_TEMPERATURE_SENSOR_DS18B20_PIN | 7             | sensor DS18B20 (analog) - see all sensor connections bellow                                              |
|                  | AIR_TEMPERATURE_SENSOR_DHT11_PIN     | 8             | sensor DHT11   (analog) - see all sensor connections bellow                                              |
|                  | AIR_PRESSURE_SENSOR_BMP180_PIN_SDA   | 20            | sensor BMP180  (analog - Arduino Mega 2560) - see all sensor connections bellow                          |
|                  | AIR_PRESSURE_SENSOR_BMP180_PIN_SCL   | 21            | sensor BMP180  (analog - Arduino Mega 2560) - see all sensor connections bellow                          |
|                  | WATER_PH_SENSOR_PH450C_PIN           | 6             | sensor Ph4502c (analog) - see all sensor connections bellow                                              |
| Buttons and leds | PUSH_BUTTON_TURN_OFF_PIN             | 25            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): turn off button        |
|                  | PUSH_BUTTON_RESTART_PIN              | 26            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): restart button         |
|                  | PUSH_BUTTON_LIGHTS_PIN               | 27            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): turn on lights button  |
|                  | PUSH_BUTTON_PWC_ROUTINE_PIN          | 28            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): pwc button             |
|                  | PUSH_BUTTON_MANUAL_CLEANING_PIN      | 39            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): manual cleaning button |
|                  | PUSH_BUTTON_FEEDING_PIN              | 30            | tactile button (pull-up, i.e, other terminal to 10k olhms resistor and then Vcc): manual feeding button  | 

 
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


## Furthermore

#### Default timers (basic timers - without home assistant integration)

Timers and alarms can be programmed (number entities and/or home assistant command topic) and stored at EEPROM, however, default values will be saved otherwise (change that if required for standalone use):

| macro                        | value (default) | description                                         |
|------------------------------|-----------------|-----------------------------------------------------|
| DEFAULT_ALARM_BOUND_TEMP_MAX | 26.0            | minimum water temperature in Celsius (alarm/heater) |
| DEFAULT_ALARM_BOUND_TEMP_MIN | 29.1            | maximum water temperature in Celsius (alarm/cooler) |
| DEFAULT_ALARM_BOUND_PH_MAX   | 8.9             | maximum Ph (alarm)                                  |
| DEFAULT_ALARM_BOUND_PH_MIN   | 7.5             | minimum Ph (alarm)                                  |
| DEFAULT_TIMER_LIGHT_ON       | 6               | light turn on time (hour of the day)                |               
| DEFAULT_TIMER_LIGHT_OFF      | 23              | light turn off time (hour of the day)               |
| DEFAULT_TIMER_UV_ON          | 8               | UV filter turn on time (hour of the day)            |
| DEFAULT_TIMER_UV_OFF         | 15              | UV filter turn off time (hour of the day)           |
| DEFAULT_TIMER_FEED_1         | 7               | 1st Feeding time (hour of the day)                  |
| DEFAULT_TIMER_FEED_2         | 12              | 2nd Feeding time (hour of the day)                  |
| DEFAULT_TIMER_FEED_3         | 18              | 3th Feeding time (hour of the day)                  |


#### Other Arduino/Home Assistant projects
   
MIT licence:

   - https://github.com/fortalbrz/fluxoino: automated robot to prevent water leakages with arduino and home assistant
   - https://github.com/fortalbrz/gardenino: automated garden robot with arduino and home assistant

---
 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
