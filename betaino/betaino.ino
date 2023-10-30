//------------------------------------------------------------------------------------------------------------------
//
//
// BETA.INO - Beta Aquarium Automation with Home Assistant
// Optimized for Arduino Pro Mini and ESP-01
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
// - 1 arduino pro mini
// - 1 relay module with 4 channels
// - 1 water level sensor
// - 1 power source 5v
// - 1 stepper motor with driver
// - 2 tactile push buttom keys and 2 x resistor 1k olhms
// - 1 led and resistor 10k olhms (indicates "power on")
// - plastic hoses (connect bumps)
// - flexible cab (22 agw)
// - 1 large plastic water container (reused, optional as "water reposition container")
//
//------------------------------------------------------------------------------------------------------------------
//
// 2023 - Jorge Albuquerque (jorgealbuquerque@gmail.com)
//
#define DEBUG_MODE true                              // enable/disables serial debugging messages
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
#define STEPPER_MOTOR_PULSER_PER_TURN 200            // number of pulses per turn on stepper motor feeder
#define STEPPER_MOTOR_DELAY 700                      // time delay (in microseconds) between the motor steps (use it for change the rotation speed)
#define FEEDER_TURNS 1                               // default number of feeder turns (comple cycles)
#define WIFI_SSID "ssid"                             // Wi-fi SSID
#define WIFI_PASSWORD "password"                     // Wi-fi password
#define MQTT_BROKER_ADDRESS "192.168.68.93"          // MQTT broker server ip
#define MQTT_BROKER_PORT 1883                        // MQTT broker port
#define MQTT_USERNAME "mqtt-user"                    // can be omitted if not needed
#define MQTT_PASSWORD "mqtt-pass"                    // can be omitted if not needed
#define MQTT_DEVICE_ID "betaino_12fmo43iowerwe"      // MQTT device id (unique)
#define MQTT_COMMAND_TOPIC "betaino/cmd"             // MQTT topic for input commands
#define MQTT_STATES_TOPIC "betaino/state"            // MQTT topic for publish states
#define MQTT_AVAILABILITY_TOPIC "betaino/available"  // MQTT topic for availability notification (home assistant "unavailable" state)
#define MQTT_AVAILABILITY_TIME 60000                 // time to send MQTT availability (default: 1 min)
#define USE_SUMP_PWM true 
#define SUMP_PWM_ON 5000 
#define SUMP_PWM_OFF 60000
//
//
// pins definitions (Arduino pro mini)
// - digital: 2 to 13 (interruptions digital pins 2 and 3)
// - analog: A0 to A7
#define PUSH_BUTTON_FEEDING_PIN 2                  // D2: feeding push button (interption 0)
#define PUSH_BUTTON_LIGHT_PIN 3                    // D3: light push button (interption 1)
#define ESP01_SERIAL_RX_PIN 4                      // D4: ESP01 RX
#define ESP01_SERIAL_TX_PIN 5                      // D5: ESP01 TX 
#define WATER_LOW_LEVEL_SENSOR_PIN 6               // D6: water level sensor (should be located at desired aquarium level: open when water is low)
#define RELAY_LIGHTS_PIN 7                         // D7: relay sump pump (normally open, common on 110vac)
#define RELAY_HEATER_PIN 8                         // D8: relay sump pump (normally open, common on 110vac)
#define RELAY_SUMP_PUMP_PIN 9                      // D9: relay sump pump (normally open, common on 110vac)
#define RELAY_WATER_REPOSITION_PUMP_PIN 10         // D10: relay water reposition pump (normally open, common on 110vac)
#define STEPPER_MOTOR_STEP_PIN 11                  // D11: stepper motor 
#define STEPPER_MOTOR_DIRECTION_PIN  12            // D12: stepper motor


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
#endif


//
// internal states (globals)
//
int status = WL_IDLE_STATUS;                            // wifi radio status
bool _blink = false;                                    // builtin led state
bool _ligthOn = false;                                  // lights: default OFF
bool _heaterOn = true;                                  // heater: default ON
bool _sumpPumpOn = true;                                // sump pump: default ON
bool _repoPumpOn = false;                               // repo pump: default OFF
unsigned long _lastAvailabilityTime = millis();
bool _sumpPumpPwmOn = true;
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

  #if (DEBUG_MODE == true)
    // serial output only in debug mode!
    Serial.begin(9600);    
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

  #if (USE_STEPPER_MOTOR == true) 
    //
    // initializes stepper motor
    //
    pinMode(STEPPER_MOTOR_STEP_PIN, OUTPUT); 
    pinMode(STEPPER_MOTOR_DIRECTION_PIN, OUTPUT);
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
  
  #if (USE_HOME_ASSISTANT == true)
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

  #if (USE_HOME_ASSISTANT == true)
    //
    // keeps MQTT connection alive
    //
    if (!client.connected()) 
      reconnect();
    
    client.loop();

    if ((millis() - _lastAvailabilityTime) > MQTT_AVAILABILITY_TIME) {
      // sends MQTT "availability" message
      client.publish(MQTT_AVAILABILITY_TOPIC, "online");
      _lastAvailabilityTime = millis();
    }

  #endif

  #if (USE_WATER_LEVEL_SENSORS == true && USE_RELAYS == true)
    //
    // water sensor level watchdog
    //

    // level sensor open (false) => low water level!     
    bool lowLevel = !digitalRead(WATER_LOW_LEVEL_SENSOR_PIN);

    // turns on/off the water reposition pump (if low level detected)
    setRelay(RELAY_WATER_REPOSITION_PUMP_PIN, lowLevel);
  #endif

  #if (USE_SUMP_PWM)
    //
    // Sump on/off routine
    //
    if (_sumpPumpOn) {
      // only changes states if sump if on
      if (_sumpPumpPwmOn) {
        if ((millis() - _lastSumpPwmTime) > SUMP_PWM_ON) {
          // turn off sump pump
          _sumpPumpPwmOn = false;
          digitalWrite(RELAY_SUMP_PUMP_PIN, HIGH);
          _lastSumpPwmTime = millis();
        }
      } else {
        if ((millis() - _lastSumpPwmTime) > SUMP_PWM_OFF) {
          // turn on sump pump
          _sumpPumpPwmOn = true;
          digitalWrite(RELAY_SUMP_PUMP_PIN, LOW);
          _lastSumpPwmTime = millis();
        }
      }
    }
  #endif
  
  delay(300);
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
    feed();
  }

  void onLightPushButton(){
    //
    // lightening button handler: invertd light state
    //
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
    
    #if (DEBUG_MODE == true)
      Serial.println();
      Serial.print(F("connecting to wifi: "));
      Serial.println(WIFI_SSID);
    #endif

    // connects to the WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      #if (DEBUG_MODE == true)
        Serial.print(F("."));
      #endif
    }

    #if (DEBUG_MODE == true)
      Serial.print(F("wifi connected: "));
      Serial.println(WiFi.localIP());
    #endif
  }

  void reconnect() {
    //
    // reconnect to MQTT broker
    //
    while (!client.connected()) {
      #if (DEBUG_MODE == true)
        Serial.println(F("MQTT reconnection..."));
      #endif
    
      // attempts to connect
      if (client.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
        #if (DEBUG_MODE == true)
          Serial.println(F("connected"));
        #endif

        // subscribes to command topic
        client.subscribe(MQTT_COMMAND_TOPIC);
      } else {
        #if (DEBUG_MODE == true)
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
    #if (DEBUG_MODE == true)
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

  #if (DEBUG_MODE == true)
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

    #if (DEBUG_MODE == true)
      Serial.println(F("stating feeding"));
    #endif

    digitalWrite(STEPPER_MOTOR_DIRECTION_PIN, HIGH);
    delay(30);

    for (unsigned int i = 0 ; i < FEEDER_TURNS; i++) {
      for(unsigned int j = 0 ; j < STEPPER_MOTOR_PULSER_PER_TURN; j++) { 
        // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(STEPPER_MOTOR_STEP_PIN, HIGH); 
        delayMicroseconds(STEPPER_MOTOR_DELAY);
        digitalWrite(STEPPER_MOTOR_STEP_PIN, LOW); 
        delayMicroseconds(STEPPER_MOTOR_DELAY); 
      }
    }

    #if (DEBUG_MODE == true)
      Serial.println(F("feeding finished"));
    #endif

  #endif
}


void setRelay(const unsigned int& relayPin, const bool& state){
  //
  // sets relay state
  //
  #if (USE_RELAYS == true)

    switch (relayPin) {
      
      case RELAY_LIGHTS_PIN:              
        // lightening relay
        if (!ENABLE_LIGHTS || _ligthOn == state)
          return;
        _ligthOn = state;
        #if (DEBUG_MODE == true)
          Serial.print(F(" - relay light: "));
          Serial.println(state ? F("on") : F("off"));
        #endif
        break;

      case RELAY_HEATER_PIN:
        // heater relay
        if (_heaterOn == state)
          return;
        _heaterOn = state;        
        #if (DEBUG_MODE == true)
          Serial.print(F(" - relay heater: "));
          Serial.println(state ? F("on") : F("off"));
        #endif
        break;

      case RELAY_SUMP_PUMP_PIN:
        if (_sumpPumpOn == state)
          return;
        _sumpPumpOn = state;
        #if (DEBUG_MODE == true)
          Serial.print(F(" - relay sump: "));
          Serial.println(state ? F("on") : F("off"));
          if (USE_SUMP_PWM)
            Serial.print(F(" [pwm mode]"));
        #endif
        break;

      case RELAY_WATER_REPOSITION_PUMP_PIN:
        #if (DEBUG_MODE == true)
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
