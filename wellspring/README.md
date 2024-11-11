# WELLSPRING.INO 
## :house_with_garden: Water Fountain Automation with Home Assistant and NodeMCU

**Optimized for NodeMCU 1.0 (ESP-12E Module)** 


[Home Assistant](https://home-assistant.io) is free and open-source software used for home automation. It serves as an integration platform and smart home hub, allowing users to control smart home devices. This project objective is to control and integrate a small "Buddha" water fountain with Home Assistant using the [MQTT]((https://www.home-assistant.io/integrations/mqtt/)) protocol. The MQTT (aka MQ Telemetry Transport) is a machine-to-machine or "*Internet of Things*" connectivity protocol on top of TCP/IP. It allows extremely lightweight publish/subscribe messaging transport.

 - **Video**: **[YouTube](https://youtu.be/uQc_9umpfpI)** [pt]



## Features:
 - fully integrated with Home Assistant (MQTT)
 - led lights
 - two relays: water pump and extra outlet
 - plays music! 
 - push buttons: 
   - water pump turn on/off button
   - lightning turn on/off button   
 - home assistant switches:
   - water pump switch
   - lightening switch
   - extra outlet switch


## MQTT broker

This project should communicate with a MQTT broker (e.g., *mosquitto broker*), ideally using **[home assistant](https://home-assistant.io)**

![mqtt diagram](https://github.com/fortalbrz/gardenino/blob/main/img/schema.png?raw=true)


## Materials:
 - 1x NodeMCU (ESP 8266-12e) - [[25 BRL](https://produto.mercadolivre.com.br/MLB-1211973212-modulo-wifi-esp8266-nodemcu-esp-12e-_JM)]
 - 1x relay module 5v 2-ch - [[24 BRL](https://produto.mercadolivre.com.br/MLB-1758894951-modulo-rele-rele-5v-4-canais-para-arduino-pic-raspberry-pi-_JM)]
 - 1x active buzzer 5v - [[2.30 BRL](https://www.a2robotics.com.br/buzzer-ativo-5v)]
 - 2x NPN BC548 transistor - [[1.20  BRL](https://produto.mercadolivre.com.br/MLB-1712833525-transistor-bc548-npn-para-projetos-10-pecas-_JM )]
 - 2x tactile push button - [[0.20 BRL](https://www.a2robotics.com.br/chave-tactil-6x6x5mm-4-terminais)]
 - 2x resistor 1k ohms (1/8w) 
 - 1x power supply 5vdc (1A) - [[14 BRL](https://produto.mercadolivre.com.br/MLB-3445635491-fonte-alimentaco-5v-1a-bivolt-roteador-wireles-modem-d-link-_JM)]
 - 20x led and resistor 480 ohms (optional, lights)
 - 1x on/off switch 
 - 1x electrolytic capacitor 100 uF (optional)
 - 1x Small buddha statue
 - 1x Aquarium Pump (3w, 200L/h)
 - 1x Ceramic Plate (base) 
 - 4x Small Clay Pots
 - 1x White Stones (1kg)
 - flexible cab (22 agw)
 - "Durepox" epoxy resin (100g)
 - Silicone Glue (50g)
 

 

## Circuit Wiring Instruction (step by step):


[[wiring diagram]]:
  - power:
    - NodeMCU (GND) -> power supply 5vdc (negative/Gnd)
    - NodeMCU (Vin) -> power supply 5vdc (positive/Vcc)  
  - relay module (2-channels - sets "NUMBER_OF_RELAYS" as "1" for use a single relay module)):
    - Relay 2 ch (VCC) -> power supply 5vdc (negative/Gnd)
    - Relay 2 ch (GND) -> power supply 5vdc (positive/Vcc)
    - Relay 2 ch (In 1) [water pump] -> NodeMCU (D4)
    - Relay 2 ch (In 2) [extra outlet] -> NodeMCU (D5)  
  - push buttons (optional - set "USE_PUSH_BUTTONS" as false to ignore):
    - NodeMCU "D1" -> resistor 1k ohms "A" terminal 1 
    - resistor 1k ohms "A" terminal 2 -> NodeMCU (3.3V)
    - NodeMCU "D2" -> resistor 1k ohms "B" terminal 1
    - resistor 1k ohms "B" terminal 2 -> NodeMCU (3.3V)
    - NodeMCU "D1" -> tactile push button "water pump on/off" terminal 1 (NC)
    - tactile push button "water pump on/off" terminal 2 (NC) -> -5 V power source (GND)
    - NodeMCU "D2" -> tactile push button "light on/off" terminal 1 (NC)
    - tactile push button "light on/off" terminal 2 (NC) -> -5 V power source (GND)
  - leds (up to 20 leds):
    - NodeMCU "D3" -> resistor 1k ohms terminal 1
    - resistor 1k ohms terminal 2 -> BC548 transistor "A" base (pin 2 - leg at middle)
    - BC548 transistor "A" emitter (pin 3 - right leg) -> -5 V power source (GND)
    - BC548 transistor "A" collector (pin 1 - left leg) -> 480 ohms resistor "led" terminal 1 (connects 20 up to leds in parallel at this point)
    - 480 ohms resistor "led" terminal 1 -> Led terminal 2 (negative/bevel) (connects up to 20 leds in parallel)
    - Led terminal 1 (positive) -> +5 V power source (Vcc) (connects 20 up to leds in parallel at this point)    
  - buzzer: (optional - set "PLAY_TUNES" as false to ignore)
    - NodeMCU "D6" -> resistor 1k ohms terminal 1
    - resistor 1k ohms terminal 2 -> BC548 transistor "B" base (pin 2 - leg at middle)
    - BC548 transistor "B" collector (pin 1 - left leg) -> buzzer 5v negative terminal (-)
    - buzzer 5v positive terminal (+) -> +5 V power source (Vcc)
    - BC548 transistor "B" emitter (pin 3 - right leg) -> -5 V power source (GND)
  - power on indicator led (optional)
    - Led terminal 1 (positive) -> +5 V power source (VCC) (optional, "power on led")
    - Led terminal 2 (negative/bevel) -> resistor 10k ohms "D" terminal 1 (optional, "power on led")
    - resistor 10k ohms "D" terminal 2 -> -5 V power source (GND) (optional, "power on led")
  - filter capacitor (optional)
    - capacitor 100uF (positive) -> +5 V power source (VCC) (optional)
    - capacitor 100uF (negative/"minus sign") -> resistor 10k ohms "D" terminal 2 (optional)



## Source code:
 - **https://github.com/fortalbrz/aquarioino/tree/main/wellspring**


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


### Serial Monitor:
Sets the macro "**DEBUG_MODE**" as true in order to debug on serial monitor (testing only) - ensure the baud rate setup!


## Home Assistant Configuration

Adds the line on *configuration.yaml*: 


     mqtt: !include mqtt.yaml


And creates a file "*[mqtt.yaml](https://github.com/fortalbrz/aquarioino/blob/main/bettaino/mqtt.yaml)*" as follow:


      - switch:
        #
        # buddha fountain (main bedroom) - enables water pump: on/off
        #
        name: "Buddha Fountain Pump"
        unique_id: buddha_wellspring_pump
        state_topic: "wellspring/status"
        value_template: "{{ value_json.pump }}"
        state_on: "on"
        state_off: "off"
        json_attributes_topic: "wellspring/status"
        json_attributes_template: "{{ value_json | tojson }}"
        command_topic: "wellspring/cmd"
        payload_on: "pump on"
        payload_off: "pump off"
        availability:
          - topic: "wellspring/available"
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
        # buddha fountain (main bedroom) - enables led lights: on/off
        #
        name: "Buddha Fountain Light"
        unique_id: buddha_wellspring_light
        state_topic: "wellspring/status"
        value_template: "{{ value_json.light }}"
        state_on: "on"
        state_off: "off"
        command_topic: "wellspring/cmd"
        payload_on: "light on"
        payload_off: "light off"
        availability:
          - topic: "wellspring/available"
            payload_available: "online"
            payload_not_available: "offline"
        availability_mode: latest
        enabled_by_default: true
        optimistic: false
        qos: 0
        retain: true
        device_class: "outlet"
        icon: mdi:led-strip
      - switch:
        #
        # buddha fountain (main bedroom) - enables power outlet (on/off)
        #
        name: "Buddha Fountain Outlet"
        unique_id: buddha_wellspring_relay2
        state_topic: "wellspring/status"
        value_template: "{{ value_json.relay2 }}"
        state_on: "on"
        state_off: "off"
        command_topic: "wellspring/cmd"
        payload_on: "relay2 on"
        payload_off: "relay2 off"
        availability:
          - topic: "wellspring/available"
            payload_available: "online"
            payload_not_available: "offline"
        availability_mode: latest
        enabled_by_default: true
        optimistic: false
        qos: 0
        retain: true
        device_class: "outlet"
        icon: mdi:light-switch
      


## MQTT topics:

   - **bettaino/available**: sensors availability [*"online"/"offline"*]

   - **wellspring/cmd**: pushes commands to NodeMCU [*home assistant -> wellspring*]:
     - "*pump on/off*": turns on/off the water pump
     - "*light on/off*": turns on/off the led lights
     - "*relay2 on/off*": turns on/off the extra power outlet
     - "*beep*": plays a beep sound [for any notification]
     - "*play [sw/dv/tetris/mario/got/gf/brahms/cannon]*": plays an tune by name [for any notification] (*none for random*)
     - "*refresh*": update MQTT state [debug only]


   - **bettaino/state**: retrieves NodeMCU states as json [*bettaino -> home assistant*]


         {
             "light": "on",                // led lights state: [on/off]
             "pump": "on",                 // relay #1 state (water pump): [on/off]
             "relay2": "off",              // relay #2 state (extra power outlet): [on/off]
             "rssi": -60,                  // wifi signal power
             "ip": "192.168.0.100",        // ip address
             "mac": "E3:3A:21:34:FE:21"    // mac address
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
| PLAY_TUNES                 | true    | enables play music themes (disables to not assembly the buzzer)                               |
| USE_PUSH_BUTTONS           | true    | enables play tactile push buttons (disables to not assembly the push buttons)                 |
| NUMBER_OF_RELAYS           | 2       | 1 for single relay module, 2 for 2-channels relay module                                      |
| DEBUG_MODE                 | false   | true to debug on serial monitor (debug), false otherwise                                      |

<hr>

*[Jorge Albuquerque](https://linkedin.com/in/jorgealbuquerque) (2024)*
