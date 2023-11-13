//------------------------------------------------------------------------------------------------------------------
//
//
// BETA.INO - Beta Aquarium Automation with Home Assistant
// Optimized for Arduino Nano and ESP-01
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
// Materials:
// - 1 arduino Nano
// - 1 relay module with 4 channels (optional, see USE_RELAYS)
// - 1 water level sensor and 1 resistor 10k olhms (optional, see USE_WATER_LEVEL_SENSORS)
// - 1 Wi-fi module ESP 8266-01 (optional, see USE_HOME_ASSISTANT)
// - 1 bidirectional digital level shifter - I2C 5v/3.3v OR 2 voltage divider with 10K and 20K olhms resistors (optional, see USE_HOME_ASSISTANT)
// - 1 stepper motor 28BYJ-48 with driver ULN2023 (optional, see USE_STEPPER_MOTOR)
// - 1 power source 5v (1 A)
// - 2 tactile push buttom keys and 2 x resistor 10k olhms (optional, see USR_PUSH_BUTTONS)
// - 1 led and resistor 10k olhms (optional, indicates "power on")
// - 1 electrolytic capacitor 100 uF (optional)
// - plastic hoses (connect bumps)
// - flexible cab (22 agw)
// - 1 large plastic water container (reused, optional as "water reposition container")
//
// Circuit Wiring Instruction:
//  - Arduino Nano "Vin" --> +5 V power source (VCC)
//  - Arduino Nano "Gnd" --> -5 V power source (GND)
//  - Arduino Nano "D2" --> resistor 10k olhms "A" terminal 1 (optional, see USE_PUSH_BUTTONS)
//  - resistor 10k olhms "A" terminal 2 --> +5 V power source (VCC) (optional, see USE_PUSH_BUTTONS)
//  - Arduino Nano "D3" --> resistor 10k olhms "B" terminal 1 (optional, see USE_PUSH_BUTTONS)
//  - resistor 10k olhms "B" terminal 2 --> +5 V power source (VCC) (optional, see USE_PUSH_BUTTONS)
//  - Arduino Nano "D2" --> tactile push buttom "FEED" terminal 1 (NC) (optional, see USE_PUSH_BUTTONS)
//  - tactile push buttom "FEED" terminal 2 (NC) --> -5 V power source (GND) (optional, see USE_PUSH_BUTTONS)
//  - Arduino Nano "D3" --> tactile push buttom "LIGHT" terminal 1 (NC) (optional, see USE_PUSH_BUTTONS)
//  - tactile push buttom "LIGHT" terminal 2 (NC) --> -5 V power source (GND) (optional, see USE_PUSH_BUTTONS)
//  - Arduino Nano "D4" --> digital level shifter "HV1" (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "D4" --> digital level shifter "HV2" (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "+5V" --> digital level shifter "HV" (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "+3.3V" --> digital level shifter "LV" (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "GND" --> digital level shifter "GND" (optional, see USE_HOME_ASSISTANT)
//  - digital level shifter "LV1" -> ESP-01 RX (optional, see USE_HOME_ASSISTANT)
//  - digital level shifter "LV2" -> ESP-01 TX (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "+3.3V" --> ESP-01 VCC (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "GND" --> ESP-01 GND (optional, see USE_HOME_ASSISTANT)
//  - Arduino Nano "D6" --> water level sensor terminal 1 (optional, see USE_WATER_LEVEL_SENSORS)
//  - water level sensor terminal 2 --> +5 V power source (VCC) (optional, see USE_WATER_LEVEL_SENSORS)
//  - Arduino Nano "D6" --> resistor 10k olhms "C" terminal 1 (optional, see USE_WATER_LEVEL_SENSORS)
//  - resistor 10k olhms "C" terminal 2 --> Arduino Nano "GND" (optional, see USE_WATER_LEVEL_SENSORS)
//  - Relay 4 ch "VCC" --> +5 V power source (VCC) (optional, see USE_RELAYS)
//  - Relay 4 ch "GND" --> -5 V power source (GND) (optional, see USE_RELAYS)
//  - Arduino Nano "D7" --> Relay 4 ch "IN 1" (optional, see USE_RELAYS)
//  - Arduino Nano "D8" --> Relay 4 ch "IN 2" (optional, see USE_RELAYS)
//  - Arduino Nano "D9" --> Relay 4 ch "IN 3" (optional, see USE_RELAYS)
//  - Arduino Nano "D10" --> Relay 4 ch "IN 4" (optional, see USE_RELAYS)
//  - stepper motor driver ULN2023 "+5V" --> +5 V power source (VCC) (optional, see USE_STEPPER_MOTOR)
//  - stepper motor driver ULN2023 "-5V" --> -5 V power source (GND) (optional, see USE_STEPPER_MOTOR)
//  - Arduino Nano "D11" --> stepper motor driver ULN2023 "IN 1" (optional, see USE_STEPPER_MOTOR)
//  - Arduino Nano "D12" --> stepper motor driver ULN2023 "IN 2" (optional, see USE_STEPPER_MOTOR)
//  - Arduino Nano "D13" --> stepper motor driver ULN2023 "IN 3" (optional, see USE_STEPPER_MOTOR)
//  - Arduino Nano "A0" (pin 14) --> stepper motor driver ULN2023 "IN 4" (optional, see USE_STEPPER_MOTOR)
//  - Led terminal 1 (positive) --> +5 V power source (VCC) (optional, "power on led")
//  - Led terminal 2 (negative/bevel) --> resistor 10k olhms "D" terminal 1 (optional, "power on led")
//  - resistor 10k olhms "D" terminal 2 --> -5 V power source (GND) (optional, "power on led")
//  - capacitor 100uF (positive) --> +5 V power source (VCC) (optional)
//  - capacitor 100uF (negative/"minus sign") --> resistor 10k olhms "D" terminal 2 (optional)
//
// Flashing the code:
//   - the Arduino Nano may use the USB/serial IC CH340G, so it's necessary to install the Windows driver: 
//       - CH340G driver: http://bit.ly/44WdzVF (windows 11 compatible)
//       - driver installation instructions (pt-BR): http://bit.ly/3ZqIqc0
//   - download Arduino IDE (ver 2.0): https://www.arduino.cc/en/software
//   - install WifiEsp library:
//       - On arduino IDE select: "File" -> "Preferences" -> "Additional boards manager URL":
//          - add the URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
//       - On arduino IDE select: "Tools" -> "Manage Libraries":
//          - search and install: "WifiEsp" (by bportaluri)
//   - uses a micro USB cable to connect to the Arduino
//   - select the "Arduino Nano" board in the arduino IDE: "Tools" -> "Board".
//   - select "Sketch" -> "Upload"
//
// Wiring Testing:
//  sets the macro "TESTING_MODE" as true in order to check buttons, relays and water sensor connections (testing only)
//
// Serial Monitor:
//  sets the macro "DEBUG_MODE" as true in order to debug on serial monitor (testing only)
//
// Setup:
//    - partial assembly:
//        USE_HOME_ASSISTANT                     enables/disables home assistant integration (disable it to not use the ESP-01 module)
//        USE_WATER_LEVEL_SENSORS                enables/disables water level sensors (disable it to not use the water level sensors)
//        USE_STEPPER_MOTOR                      enables/disables stepper motor feeder (disable it to not use the stepper motor)
//        USE_PUSH_BUTTONS                       enables/disables push buttons (disable it to not use the push buttons)
//        USE_RELAYS                             enables/disables relays (disable it to not use the relays module)
//    - configurations:
//        ENABLE_LIGHTS                          enables/disables lights relay
//        ENABLE_HEATER                          enables/disables heater relay
//        ENABLE_FEEDING                         enables/disables feeding routine
//        ENABLE_WATER_REPOSITION                true for uses "feeding" relay as feeder, false for uses it as regular extra relay
//        STEPPER_MOTOR_STEPS_PER_REVOLUTION     number of pulses per turn on stepper motor feeder (usually 32 or 64)
//        STEPPER_MOTOR_SPEED                    the rotation speed
//        FEEDER_TURNS                           default number of feeder turns
//    - network:
//        WIFI_SSID                              Wi-fi SSID
//        WIFI_PASSWORD                          Wi-fi password
//        MQTT_BROKER_ADDRESS                    MQTT broker server ip
//        MQTT_BROKER_PORT                       MQTT broker port
//        MQTT_USERNAME                          MQTT username, can be omitted if not needed
//        MQTT_PASSWORD                          MQTT password, can be omitted if not needed
//
//------------------------------------------------------------------------------------------------------------------
//
// 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
// https://jorgealbuquerque.com
//
#define DEBUG_MODE true                             // enables/disables serial debugging messages
#define TESTING_MODE false                          // enables/disables testing mode
//
// Configuration flags (enables or disables features in order to "skip" unwanted hardware)
//
#define USE_HOME_ASSISTANT true                      // enables/disables home assistant integration (disable it to not use the ESP-01 module)
#define USE_WATER_LEVEL_SENSORS true                 // enables/disables water level sensors (disable it to not use the water level sensors)
#define USE_STEPPER_MOTOR true                       // enables/disables stepper motor (disable it to not use the stepper motor)
#define USE_PUSH_BUTTONS true                        // enables/disables push buttons (disable it to not use the push buttons)
#define USE_RELAYS true                              // enables/disables relays (disable it to not use the relays module)
#define ENABLE_LIGHTS true                           // enables/disables lights relays
#define ENABLE_HEATER true                           // enables/disables heater relays
#define ENABLE_FEEDING true                          // enables/disables feeding routine
#define ENABLE_WATER_REPOSITION true                 // true for uses "feeding" relay as feeder, false for uses it as regular extra relay
#define STEPPER_MOTOR_STEPS_PER_REVOLUTION 200       // number of pulses per turn on stepper motor feeder (usually 32 or 64)
#define STEPPER_MOTOR_SPEED 60                       // the rotation speed
#define FEEDER_TURNS 1                               // default number of feeder turns (comple cycles)
#define WIFI_SSID "wifi-ssid"                        // Wi-fi SSID
#define WIFI_PASSWORD "wifi-password"                // Wi-fi password
#define MQTT_BROKER_ADDRESS "192.168.68.93"          // MQTT broker server ip
#define MQTT_BROKER_PORT 1883                        // MQTT broker port
#define MQTT_USERNAME "mqtt-user"                    // can be omitted if not needed
#define MQTT_PASSWORD "mqtt-password"                // can be omitted if not needed
#define MQTT_DEVICE_ID "betaino_12fmo43iowerwe"      // MQTT device id (unique)
#define MQTT_COMMAND_TOPIC "betaino/cmd"             // MQTT topic for input commands
#define MQTT_STATES_TOPIC "betaino/state"            // MQTT topic for publish states
#define MQTT_AVAILABILITY_TOPIC "betaino/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_AVAILABILITY_TIME 60000                 // time to send MQTT availability (default: 1 min)
#define USE_SUMP_ON_OFF false 
#define SUMP_TIME_ON 5000 
#define SUMP_TIME_OFF 20000
//
//
// pins definitions (Arduino Nano)
// - digital: 2 to 13 (interruptions digital pins 2 and 3)
// - analog: A0 to A7
#define PUSH_BUTTON_FEEDING_PIN 2                  // D2: feeding push button (interption 0)
#define PUSH_BUTTON_LIGHT_PIN 3                    // D3: light push button (interption 1)
#define ESP01_SERIAL_RX_PIN 4                      // D4: ESP01 RX
#define ESP01_SERIAL_TX_PIN 5                      // D5: ESP01 TX 
#define WATER_LOW_LEVEL_SENSOR_PIN A0              // A0: water level sensor (should be located at desired aquarium level: open when water is low)
#define RELAY_LIGHTS_PIN 7                         // D7: relay sump pump (normally open, common on 110vac)
#define RELAY_HEATER_PIN 8                         // D8: relay sump pump (normally open, common on 110vac)
#define RELAY_SUMP_PUMP_PIN 9                      // D9: relay sump pump (normally open, common on 110vac)
#define RELAY_WATER_REPOSITION_PUMP_PIN 10         // D10: relay water reposition pump (normally open, common on 110vac)
#define STEPPER_MOTOR_IN1_PIN 11                   // D11: stepper motor ULN2003 - IN1
#define STEPPER_MOTOR_IN2_PIN 12                   // D12: stepper motor ULN2003 - IN2
#define STEPPER_MOTOR_IN3_PIN 6                    // D6: stepper motor ULN2003 - IN3
#define STEPPER_MOTOR_IN4_PIN 13                   // D13: stepper motor ULN2003 - IN4

#define SERIAL_ON (DEBUG_MODE || TESTING_MODE)

#if (USE_STEPPER_MOTOR == true)
  //
  // Wifi (ESP-01) libraries
  //
  #include <Stepper.h>
  Stepper FeederStepper(STEPPER_MOTOR_STEPS_PER_REVOLUTION, 
    STEPPER_MOTOR_IN1_PIN, STEPPER_MOTOR_IN2_PIN, 
    STEPPER_MOTOR_IN3_PIN, STEPPER_MOTOR_IN4_PIN);
#endif

#if (USE_HOME_ASSISTANT == true)
  //
  // Wifi (ESP-01) libraries
  //
  #include <WiFiEsp.h>
  #include <PubSubClient.h>
  #ifndef HAVE_HWSERIAL1
    #include <SoftwareSerial.h>
    SoftwareSerial SerialEsp01(ESP01_SERIAL_RX_PIN, ESP01_SERIAL_TX_PIN);
  #endif  

  WiFiEspClient espClient;
  PubSubClient client(espClient);
  int status = WL_IDLE_STATUS;
#else
  int status = 0;
#endif


//
// internal states (globals)
//
bool _blink = false;                                    // builtin led state
bool _ligthOn = false;                                  // lights: default OFF
bool _heaterOn = true;                                  // heater: default ON
bool _sumpPumpOn = true;                                // sump pump: default ON
bool _repoPumpOn = false;                               // repo pump: default OFF
bool _sumpPumpPwmOn = true;
bool _testOk = false;
unsigned long _lastAvailabilityTime = millis();
unsigned long _lastSumpPwmTime = millis();

//--------------------------------------------------------------------------------------------------
//
// main code
//
//--------------------------------------------------------------------------------------------------

void setup() {
  //
  // initialization
  //
  pinMode(LED_BUILTIN, OUTPUT); // blinking led 

  #if (SERIAL_ON == true)
    // serial output only in debug mode!
    Serial.begin(9600);
    Serial.println();
    Serial.println(F("starting..."));
  #endif

  #if (USE_RELAYS == true)
    //
    // initializes relays
    //
    pinMode(RELAY_LIGHTS_PIN, OUTPUT);    
    pinMode(RELAY_HEATER_PIN, OUTPUT);
    pinMode(RELAY_SUMP_PUMP_PIN, OUTPUT);
    pinMode(RELAY_WATER_REPOSITION_PUMP_PIN, OUTPUT);

    // initializes relays states (HIGH = turns off)
    digitalWrite(RELAY_LIGHTS_PIN, (_ligthOn ? LOW : HIGH));
    digitalWrite(RELAY_HEATER_PIN, (_heaterOn ? LOW : HIGH));
    digitalWrite(RELAY_SUMP_PUMP_PIN, (_sumpPumpOn ? LOW : HIGH));
    digitalWrite(RELAY_WATER_REPOSITION_PUMP_PIN, (_repoPumpOn ? LOW : HIGH));  
  #endif

  #if (USE_WATER_LEVEL_SENSORS == true)
    //
    // water level sensors
    //
    pinMode(WATER_LOW_LEVEL_SENSOR_PIN, INPUT);
  #endif

  
  #if (USE_PUSH_BUTTONS == true)
    //
    // initializes push buttons (interruptions)
    //
    pinMode(PUSH_BUTTON_FEEDING_PIN, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_LIGHT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_FEEDING_PIN), onFeedPushButton, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_LIGHT_PIN), onLightPushButton, FALLING);
  #endif
  
  #if (USE_HOME_ASSISTANT == true && TESTING_MODE == false)
    //
    // MQTT initialization
    //
    connect();
    client.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
    client.setCallback(onMessage);
  #endif

}

void loop() {
  //
  // main loop
  //
  digitalWrite(LED_BUILTIN, (_blink ? HIGH: LOW)); // blinking led
  _blink = !_blink;

  #if (TESTING_MODE == true)
    if (!_testOk)
        test();
  #endif

  #if (USE_HOME_ASSISTANT == true)
    //
    // keeps MQTT connection alive
    //
    cli();
    
    if (!client.connected()) 
      reconnect();
    
    client.loop();

    if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
      // sends MQTT "availability" message
      client.publish(MQTT_AVAILABILITY_TOPIC, "online");
      _lastAvailabilityTime = millis();
    }

    sei();
  #endif

  #if (USE_WATER_LEVEL_SENSORS == true && USE_RELAYS == true)
    //
    // water sensor level watchdog
    //
    cli();
    
    // level sensor open (false) => low water level!     
    bool lowLevel = !digitalRead(WATER_LOW_LEVEL_SENSOR_PIN);

    // turns on/off the water reposition pump (if low level detected)
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, lowLevel);
    
    sei();
  #endif

  #if (USE_SUMP_ON_OFF)
    //
    // Sump on/off routine
    //
    if (_sumpPumpOn) {
      cli();
      // only changes states if sump if on
      if (_repoPumpOn){
        _sumpPumpPwmOn = false;
        digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
      }
      else if (_sumpPumpPwmOn) {
        if ((millis() - _lastSumpPwmTime) > SUMP_TIME_ON) {
          // turn off sump pump
          _sumpPumpPwmOn = false;
          digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
          _lastSumpPwmTime = millis();
        }
      } else {
        if ((millis() - _lastSumpPwmTime) > SUMP_TIME_OFF) {
          // turn on sump pump
          _sumpPumpPwmOn = true;
          digitalWrite(RELAY_SUMP_PUMP_PIN, LOW);
          _lastSumpPwmTime = millis();
        }
      }
      sei();
    }
  #endif

  sei();
  delay(400);
}



//--------------------------------------------------------------------------------------------------
//
// feed push button
//
//--------------------------------------------------------------------------------------------------    
#if (USE_PUSH_BUTTONS == true)
  
  void onFeedPushButton(){
    //
    // feeding button handler: feed!
    //
    #if (TESTING_MODE == true)
      Serial.println(F("Button feed pressed"));
    #endif

    feed();
  }

  void onLightPushButton(){
    //
    // lightening button handler: invertd light state
    //
    #if (TESTING_MODE == true)
      Serial.println(F("Button light pressed"));
    #endif    
    
    setRelay(RELAY_LIGHTS_PIN, !_ligthOn);
  }

#endif

//--------------------------------------------------------------------------------------------------
//
// home assistant
//
//--------------------------------------------------------------------------------------------------
#if (USE_HOME_ASSISTANT == true)
  
  void connect() {
    //
    // initialize WiFi connection
    //  
    SerialEsp01.begin(9600);
    WiFi.init(&SerialEsp01);  
    delay(10);
    
    #if (SERIAL_ON == true)
      Serial.println();
      Serial.print(F("connecting to wifi: "));
      Serial.println(WIFI_SSID);
    #endif

    // connects to the WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      #if (SERIAL_ON == true)
        Serial.print(F("."));
      #endif
    }

    #if (SERIAL_ON == true)
      Serial.print(F("wifi connected: "));
      Serial.println(WiFi.localIP());
    #endif
  }

  void reconnect() {
    //
    // reconnect to MQTT broker
    //
    while (!client.connected()) {
      #if (SERIAL_ON == true)
        Serial.println(F("MQTT reconnection..."));
      #endif
    
      // attempts to connect
      if (client.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        #if (SERIAL_ON == true)
          Serial.println(F("connected"));
        #endif

        // subscribes to command topic
        client.subscribe(MQTT_COMMAND_TOPIC);
      } else {
        #if (SERIAL_ON == true)
          Serial.print(F("failed: rc="));
          Serial.print(client.state());
          Serial.println(F(". trying again in 5 seconds."));
        #endif
        // waits
        delay(5000);
      }
    }
  }

  void onMessage(char* topic, byte* payload, unsigned int length) {
    //
    // on MQTT message received callback
    //
    #if (SERIAL_ON == true)
      Serial.print(F(" - mqtt command: ["));
      for (unsigned int i = 0; i < length; i++) 
        Serial.print((char)payload[i]);
      Serial.print(F("] ("));
      Serial.print(topic);
      Serial.println(F(")"));
    #endif

    if (strcmp(payload, "feed") == 0){
      feed();
    } else if (strcmp(payload, "light on") == 0){        
      setRelay(RELAY_LIGHTS_PIN, true);
    } else if (strcmp(payload, "light off") == 0){        
      setRelay(RELAY_LIGHTS_PIN, false);
    } else if (strcmp(payload, "heater on") == 0){        
      setRelay(RELAY_HEATER_PIN, true);
    } else if (strcmp(payload, "heater off") == 0){        
      setRelay(RELAY_HEATER_PIN, false);
    } else if (strcmp(payload, "sump on") == 0){        
      setRelay(RELAY_SUMP_PUMP_PIN, true);
    } else if (strcmp(payload, "sump off") == 0){        
      setRelay(RELAY_SUMP_PUMP_PIN, false);
    }   
}

String toStr(const char* label, const bool& state) {
  //
  // creates a line as: 
  //     "label": "on/off",
  // 
  String text = String(F("\"[LABEL]\": \"[VALUE]\", "));
  text.replace(F("[LABEL]"), label);
  text.replace(F("[VALUE]"), (state? F("on") : F("off")));
  return text;
}

void updateState(){
  //
  // MQTT publish state update
  //
  String json = String(F("{"));
  json += toStr("light", _ligthOn);
  json += toStr("heater", _heaterOn);
  json += toStr("sump", _sumpPumpOn);
  json += String(F("}"));
    
  unsigned int n = json.length();
  char message[n];
  json.toCharArray(message, n);

  #if (SERIAL_ON == true)
    Serial.print(F("mqtt update: "));
    Serial.println(message);
  #endif

  client.publish(MQTT_STATES_TOPIC, message);  
}

#endif



void feed() {  
  //
  // Feeding procedure
  //
  #if (USE_STEPPER_MOTOR == true && ENABLE_FEEDING == true)  
    sei();

    #if (SERIAL_ON == true)
      Serial.println(F("stating feeding"));
    #endif
    
    for (unsigned int i = 0 ; i < FEEDER_TURNS; i++) {
      #if (SERIAL_ON == true)
        Serial.print(F("feeding turn: "));
        Serial.println(i);
      #endif

      FeederStepper.setSpeed(STEPPER_MOTOR_SPEED);
      FeederStepper.step(2048);
    }

    #if (SERIAL_ON == true)
      Serial.println(F("feeding finished"));
      Serial.println();
    #endif

  #endif
}


void setRelay(const unsigned int& relayPin, const bool& state){
  //
  // sets relay state
  //
  #if (USE_RELAYS == true)
    cli();

    switch (relayPin) {
      
      case RELAY_LIGHTS_PIN:              
        // lightening relay
        if (!ENABLE_LIGHTS || _ligthOn == state)
          return;
        _ligthOn = state;
        #if (SERIAL_ON == true)
          Serial.print(F(" - relay light: "));
          Serial.println(state ? F("on") : F("off"));
        #endif
        break;

      case RELAY_HEATER_PIN:
        // heater relay
        if (_heaterOn == state)
          return;
        _heaterOn = state;        
        #if (SERIAL_ON == true)
          Serial.print(F(" - relay heater: "));
          Serial.println(state ? F("on") : F("off"));
        #endif
        break;

      case RELAY_SUMP_PUMP_PIN:
        if (_sumpPumpOn == state)
          return;
        _sumpPumpOn = state;
        #if (SERIAL_ON == true)
          Serial.print(F(" - relay sump: "));
          Serial.println(state ? F("on") : F("off"));
          if (USE_SUMP_ON_OFF)
            Serial.print(F(" [pwm mode]"));
        #endif
        break;

      case RELAY_WATER_REPOSITION_PUMP_PIN:
        if (_repoPumpOn == state)
          return;
        _repoPumpOn = state;
        #if (SERIAL_ON == true)
          Serial.print(F(" - relay repo: "));
          Serial.println(state ? F("on") : F("off"));
        #endif
        break;
    }

    // relays are active LOW  
    digitalWrite(relayPin, (state ? LOW : HIGH));

    #if (USE_HOME_ASSISTANT == true)
      // update home assistant states
      updateState();
    #endif

  #endif
}


#if (TESTING_MODE == true)
  
  void test() {
    //
    // Testing routine
    //            
    while (true) {
      sei();
      Serial.println(F("TESTING MODE"));
      Serial.println(F("1) Water Level Sensor"));
      Serial.println(F("2) Relay 1 - Light"));
      Serial.println(F("3) Relay 2 - Heater"));
      Serial.println(F("4) Relay 3 - Sump Pump"));
      Serial.println(F("5) Relay 4 - Water Repo Pump"));
      Serial.println(F("6) Feeder"));
      Serial.println(F("7) Exit"));
      Serial.println(F("Select one option [1-7]:"));
      Serial.println();
      while(Serial.available() < 2)  {
        delay(300);
      }

      // can be 0 if read error      
      int input = Serial.parseInt();

      Serial.print(F("option: "));
      Serial.println(input);
      
      switch(input) { 
        case 1:
          // water level sensor testing
          while(Serial.available() > 0)  {
             Serial.read();
          } 
          while(Serial.available() == 0)  {
            if (digitalRead(WATER_LOW_LEVEL_SENSOR_PIN))
              Serial.println(F("Water Level: ok"));
            else
              Serial.println(F("Water Level: LOW"));
            Serial.println(F("press any key to exit..."));
            delay(1000);
          }   
          Serial.read();
          blink();
          break;
        case 2:
          // relay 1 testing
          setRelay(RELAY_LIGHTS_PIN, !_ligthOn);
          break;
        case 3:
          // relay 2 testing
          setRelay(RELAY_HEATER_PIN, !_heaterOn);
          break;
        case 4:
          // relay 3 testing
          setRelay(RELAY_SUMP_PUMP_PIN, !_sumpPumpOn);
          break;
        case 5:
          // relay 4 testing
          setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, !_repoPumpOn);
          break;
        case 6:
          // feeding testing
          feed();
          break;        
        case 7:
          // Exit
          _testOk = true;
          return;          
        default:          
          Serial.println(F("Invalid option: [1-7]."));
          break; 
      }
    }
  }

  void blink() {
    // blinking led
    digitalWrite(LED_BUILTIN, LOW); 
    delay(30);
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(500);
    digitalWrite(LED_BUILTIN, LOW); 
    delay(30);  
  }
#endif