# BETTA.INO 
## :house_with_garden: Aquarium Automation with Home Assistant and NodeMCU

**Optimized for NodeMCU 1.0 (ESP-12E Module)** 

<center>

![logo](https://github.com/fortalbrz/aquarioino/blob/main/bettaino/img/logo.png?raw=true)

</center>

[Home Assistant](https://home-assistant.io) is free and open-source software used for home automation. It serves as an integration platform and smart home hub, allowing users to control smart home devices. This project objective is to control and integrate a fisk tank with Home Assistant using the [MQTT]((https://www.home-assistant.io/integrations/mqtt/)) protocol. The MQTT (aka MQ Telemetry Transport) is a machine-to-machine or "*Internet of Things*" connectivity protocol on top of TCP/IP. It allows extremely lightweight publish/subscribe messaging transport.

 - **Video**: **[YouTube](https://youtu.be/uQc_9umpfpI)** [pt]



## Features:
 - fully integrated with Home Assistant (MQTT)
 - automation of water replenishment by evaporation (optional, see ENABLE_WATER_REPOSITION configuration flag)
 - four relays: fish feeder, lights, sump pump, water reposition pump, etc
 - automated fish feeding 
 - push buttons: 
   - lightning turn on/off button
   - feeding button
 - home assistant switches:
   - feeding button
   - lightening switch
   - "sump" and "water reposition" pumps automation switches

### Water Replenishment (due evaporation loses):

This routine uses a water level sensor to refill the fisk tank with clean water (from an external container) when the water level is low.


## MQTT broker

This project should communicate with a MQTT broker (e.g., *mosquitto broker*), ideally using **[home assistant](https://home-assistant.io)**

![mqtt diagram](https://github.com/fortalbrz/gardenino/blob/main/img/schema.png?raw=true)


## Materials:
 - 1 x NodeMCU (ESP 8266-12e) - [[25 BRL](https://produto.mercadolivre.com.br/MLB-1211973212-modulo-wifi-esp8266-nodemcu-esp-12e-_JM)]
 - 1 x relay module 5v 4-ch - [[24 BRL](https://produto.mercadolivre.com.br/MLB-1758894951-modulo-rele-rele-5v-4-canais-para-arduino-pic-raspberry-pi-_JM)]
 - 1 x water level sensor - [[R$ 14](https://produto.mercadolivre.com.br/MLB-3164716754-sensor-de-nivel-de-agua-interruptor-de-boia-e-mini-boia-c-_JM)] (*optional*, see **ENABLE_WATER_REPOSITION** configuration flag)
 - 1 x active buzzer 5v - [[2.30 BRL](https://www.a2robotics.com.br/buzzer-ativo-5v)]
 - 1 x NPN BC548 transistor - [[1.20  BRL](https://produto.mercadolivre.com.br/MLB-1712833525-transistor-bc548-npn-para-projetos-10-pecas-_JM )]
 - 2 x tactile push button - [[0.20 BRL](https://www.a2robotics.com.br/chave-tactil-6x6x5mm-4-terminais)]
 - 3 x resistor 10k ohms (1/8w) 
 - 1 x resistor 1k ohms (1/8w) 
 - 1 x power supply 5vdc (1A) - [[14 BRL](https://produto.mercadolivre.com.br/MLB-3445635491-fonte-alimentaco-5v-1a-bivolt-roteador-wireles-modem-d-link-_JM)]
 - 1 x led and resistor 10k ohms (optional, indicates "power on")
 - 1 x electrolytic capacitor 100 uF (optional)
 - 1 x large plastic water container (reused, optional as "water reposition container")
 - plastic hoses (connect bumps)
 - flexible cab (22 agw)
 

## Circuit Wiring Instruction (step by step):


![project resources](https://github.com/fortalbrz/aquarioino/blob/main/bettaino/img/wiring_diagram.png?raw=true)

[[wiring diagram](https://www.circuito.io/app?components=513,13322,360216,442979)]:
  - power:
    - NodeMCU (GND) -> power supply 5vdc (negative/Gnd)
    - NodeMCU (Vin) -> power supply 5vdc (positive/Vcc)
  - relays (4-channels module):
    - Relay 4 ch (VCC) -> power supply 5vdc (negative/Gnd)
    - Relay 4 ch (GND) -> power supply 5vdc (positive/Vcc)
    - Relay 4 ch (In 1) -> NodeMCU (D4)
    - Relay 4 ch (In 2) -> NodeMCU (D5)
    - Relay 4 ch (In 3) -> NodeMCU (D6)
    - Relay 4 ch (In 4) -> NodeMCU (D7)
  - push buttons:
    - NodeMCU "D1" -> resistor 10k ohms "A" terminal 1 
    - resistor 10k ohms "A" terminal 2 -> NodeMCU (3.3V)
    - NodeMCU "D2" -> resistor 10k ohms "B" terminal 1
    - resistor 10k ohms "B" terminal 2 -> NodeMCU (3.3V)
    - NodeMCU "D1" -> tactile push button "FEED" terminal 1 (NC)
    - tactile push button "FEED" terminal 2 (NC) -> -5 V power source (GND)
    - NodeMCU "D2" -> tactile push button "LIGHT" terminal 1 (NC)
    - tactile push button "LIGHT" terminal 2 (NC) -> -5 V power source (GND)
  - water level sensor (*optional*, see **ENABLE_WATER_REPOSITION** configuration flag):
    - NodeMCU "A0" -> water level sensor terminal 1
    - water level sensor terminal 2 -> NodeMCU (3.3V)
    - NodeMCU "A0" -> resistor 10k ohms "C" terminal 1
    - resistor 10k ohms "C" terminal 2 -> NodeMCU "GND"
    - NodeMCU "D3" -> resistor 1k ohms terminal 1
  - buzzer  (*optional*, see **PLAY_TUNE** configuration flag)
    - resistor 1k ohms terminal 2 -> BC548 transistor base (pin 2 - leg at middle)
    - BC548 transistor collector (pin 1 - left leg) -> buzzer 5v negative terminal (-)
    - buzzer 5v positive terminal (+) -> +5 V power source (Vcc)
    - BC548 transistor emitter (pin 3 - right leg) -> -5 V power source (GND)
  - "power on led" (*optional*)
    - Led terminal 1 (positive) -> +5 V power source (VCC)
    - Led terminal 2 (negative/bevel) -> resistor 10k ohms "D" terminal 1
    - resistor 10k ohms "D" terminal 2 -> -5 V power source (GND)
  - power supply filter capacitor (*optional*)
    - capacitor 100uF (positive) -> +5 V power source (VCC)
    - capacitor 100uF (negative/"minus sign") -> resistor 10k ohms "D" terminal 2




## Source code:
 - **https://github.com/fortalbrz/aquarioino/tree/main/bettaino**


## Flashing the code

**Drivers (CH340g)** for NodeMCU:
  - [CH340g USB/Serial driver](https://bit.ly/44WdzVF) (windows 11 compatible driver)  
  - driver install instructions ([pt](https://bit.ly/3ZqIqc0))

The ESP-01 module should be programed with the sketch with the [Arduino IDE](https://www.arduino.cc/en/software):
  - go to File > Preferences
  - on "Additional boards manager", set the value "http://arduino.esp8266.com/stable/package_esp8266com_index.json"
  - go to Tools > Board > Board Manager
  - search for "**ESP8266**"
  - install the ESP8266 Community package ("**esp8266**" by *ESP8266 Community*)//   
  - select board "**NodeMCU 1.0 (ESP-12E Module)**" and connected COM port (checks at Windows "device manager")
  - select Sketch > Upload


### Error Codes (buzzer)
   - 2 fast bips: Wifi error
   - 3 fast bips: MQTT broker error
   - 5 bips: low water level alarm


### Wiring Testing:

Sets the macro "**WIRING_TEST_MODE**" as true in order to check buttons, relays and water sensor connections (testing only)

### Serial Monitor:
Sets the macro "**DEBUG_MODE**" as true in order to debug on serial monitor (testing only) - ensure the baud rate setup!


## Home Assistant Configuration

Adds the line on *configuration.yaml*: 


     mqtt: !include mqtt.yaml


And creates a file "*[mqtt.yaml](https://github.com/fortalbrz/aquarioino/blob/main/bettaino/mqtt.yaml)*" as follow:


      - binary_sensor:
          #
          # Fish tank - low water level: on/off
          #
          name: "Fish Tank Low Water Level"
          state_topic: "bettaino/state"
          value_template: "{{ value_json.water_low }}"
          payload_on: "on"
          payload_off: "off"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          qos: 0
          device_class: "problem"
          icon: mdi:hydraulic-oil-level
      - switch:
          #
          # Fish tank lights: on/off
          #
          name: "Fish Tank Lights"
          unique_id: aquarium_home_lights
          state_topic: "bettaino/state"
          value_template: "{{ value_json.light }}"
          state_on: "on"
          state_off: "off"
          command_topic: "bettaino/cmd"
          payload_on: "light on"
          payload_off: "light off"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          availability_mode: latest
          enabled_by_default: true
          optimistic: false
          qos: 0
          retain: true
          device_class: "outlet"
          icon: mdi:spotlight-beam
      - switch:
          #
          # Fish tank (home) - enables low water level to stop sump pump: on/off
          #
          name: "Fisk Tank Low Water Level Blocks Sump Pump Enabled"
          unique_id: fisk_tank_home_low_water_level_block_sump_pump_enabled
          state_topic: "bettaino/state"
          value_template: "{{ value_json.sump_en }}"
          state_on: "on"
          state_off: "off"
          command_topic: "bettaino/cmd"
          payload_on: "sump enable"
          payload_off: "sump disable"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          availability_mode: latest
          enabled_by_default: true
          optimistic: false
          qos: 0
          retain: true
          device_class: "outlet"
          icon: mdi:pump
      - switch:
          #
          # Fish tank - enables water reposition pump: on/off
          #
          name: "Fisk Tank Water Reposition Pump Enabled"
          unique_id: fisk_tank_home_water_reposition_pump_enabled
          state_topic: "bettaino/state"
          value_template: "{{ value_json.repo_en }}"
          state_on: "on"
          state_off: "off"
          command_topic: "bettaino/cmd"
          payload_on: "repo enable"
          payload_off: "repo disable"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          availability_mode: latest
          enabled_by_default: true
          optimistic: false
          qos: 0
          retain: true
          device_class: "outlet"
          icon: mdi:pump
      - button:
          #
          # fisk tank feeding (home)
          #
          name: "Fisk Tank Feed"
          unique_id: aquarium_home_feed
          command_topic: "bettaino/cmd"
          payload_press: "feed"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          qos: 0
          retain: false
          entity_category: "config"
          device_class: "restart"
      - button:
          #
          # plays music
          #
          name: "Fisk Tank Plays Music"
          unique_id: aquario_home_play
          command_topic: "bettaino/cmd"
          payload_press: "play"
          availability:
            - topic: "bettaino/available"
              payload_available: "online"
              payload_not_available: "offline"
          qos: 0
          retain: false
          entity_category: "config"
          device_class: "restart"



## MQTT topics:

   - **bettaino/available**: sensors availability [*"online"/"offline"*]

   - **bettaino/cmd**: pushes commands to NodeMCU [*home assistant -> bettaino*]:
     - "*feed*": feeding routine (simulate a use button press on relay #1 - connect to feeder button)
     - "*light on/off*": turns on/off the lights (i.e. relay #2)
     - "*sump enable/disable*": enables/disables sump pump automation routine
     - "*repo enable/disable*": enables/disables water reposition pump automation routine
     - "*alarm on/off*": turns on/off a low water level alarm sound
     - "*beep*": plays a beep sound [for any notification]
     - "*play [sw/dv/tetris/mario/got/gf/nokia/notice]*": plays an tune by name [for any notification] (*none for random*)
     - "*emergency on/off*": emergency alarm [for any notification]
     - "*sensor on/off*": enables/disables the water level sensor to block the sump pump [debug only]
     - "*relays on/off*": turn on/off all relays [debug only]
     - "*refresh*": update MQTT state [debug only]


   - **bettaino/state**: retrieves NodeMCU states as json [*bettaino -> home assistant*]


         {
             "light": "on",                // relay #2 state (lights): [on/off]
             "sump": "on",                 // relay #3 state (sump pump): [on/off]
             "repo": "off",                // relay #4 state (water reposition pump): [on/off]
             "sump_en": "on",              // sump pump automation routine enabled: [on/off]
             "repo_en": "off",             // water reposition pump automation routine enabled: [on/off]
             "alarm": "on",                // play a alarm sound on low water level: [on/off]
             "sensor": "on",               // water level sensor enabled to block the sump pump: [on/off]
             "water_low": "off"            // low water level: [on/off]
             "rssi": -68,                  // wifi signal power [db]
             "ip": "192.168.0.58",         // ip address
             "mac": "E3:3A:21:34:FE:21",   // mac address
         } 



### Configuration flags
  

| macro                      | default | description                                                                                   |
|----------------------------|---------|-----------------------------------------------------------------------------------------------|
| WIFI_SSID                  |         | Wi-fi SSID                                                                                    |
| WIFI_PASSWORD              |         | Wi-fi password                                                                                |
| MQTT_BROKER_ADDRESS MQTT   |         | MQTT broker server ip address                                                                 |
| MQTT_BROKER_PORT           | 1883    | MQTT broker port                                                                              |
| MQTT_USERNAME              |         | MQTT broker username                                                                          |
| MQTT_PASSWORD              |         | MQTT broker password                                                                          |
| MQTT_DEVICE_ID             |         | MQTT session identifier (changes for more then one gardeino on the same MQTT broker)          |
| ENABLE_WATER_REPOSITION    | true    | enables/disables water level sensors (disable it to not use the water level sensors)          |
| DEBUG_MODE                 | false   | true to debug on serial monitor (debug), false otherwise                                      |
| WIRING_TEST_MODE           | false   | enables/disables a wiring test routine                                                        |
| PLAY_TUNE                  | true    | enables play music themes                                                                     |
| KEEP_SILENCE_TIME          | true    | true to not play sounds at dawn, false otherwise                                              |
| SILENCE_HOUR_START         | 20      | silence time starting hour                                                                    |
| SILENCE_HOUR_END           | 8       | silence time finishing hour                                                                   |

<hr>

*[Jorge Albuquerque](https://linkedin.com/in/jorgealbuquerque) (2024)*
