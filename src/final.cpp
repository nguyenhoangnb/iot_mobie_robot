#include <calc_distance.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Hoang";
const char* password = "12345678";
const char* mqtt_server = "192.168.43.111";
const char* controlTopic = "robot/control";
const char *stopTopic = "stop";
const char *stateTopic = "state_data";
const char *handleTopic = "handleTopic";
const char *handleTopicManual = "handleTopicManual";
const char *autoTopic = "automatic";
const char *currentStateTopic = "currentStateTopic";
int currentState = 0;

char* currentStateNow;

WiFiClient espClient;
PubSubClient client(espClient);

struct StateData {
  String state;
  long time;
  long timeOut;
  String led1Status;
  long led1Time;
  long led1TimeOut;
  String led2Status;
  long led2Time;
  long led2TimeOut;
};

StateData stateData;

// Motor Pins
const int IN1_L = 27;
const int IN2_L = 26;
const int ENA_L = 25;
const int IN1_R = 13;
const int IN2_R = 12;
const int ENA_R = 14;

// LED Pins
const int led1Pin = 5;
const int led2Pin = 22;
const int led3Pin = 23
;
// LED Timers
const int timeOutled1Pin = 15000;
const int timeOutled2Pin = 10000;

int timeStartLed1 = 0;
int timeStartLed2 = 0;

bool led1Sts = false;
bool led2Sts = false;

unsigned int startStateTime = 0;

bool led_status1 = true;
bool led_status2 = true;

bool checkReceiveTopic = false;

// Distance Sensor Parameters
const float safeDistance = 30.0;

bool checkStateAuto = false;
bool checkStateControl = false;
bool checkControl = false;
bool checkStop = false;

// PWM Settings
const int pwmFrequency = 1000;
const int pwmResolution = 8;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;

// Robot Motion Parameters
const float wheelBase = 0.1; // Distance between wheels (meters)
const float maxSpeed = 0.2823; // Max speed (meters/second)
const int maxPWM = 255;
const int minPWM = 130;
const int maxTurnTime = 240; // Max time for turning (milliseconds)
const float R = 0.0325; // Wheel radius (meters)

// Distance Sensor Pins
#define TRIG_PIN 16
#define ECHO_PIN 17
#define SERVO_PIN 21

// Create Distance Sensor Object
calc_distance sensor(TRIG_PIN, ECHO_PIN, SERVO_PIN);

unsigned long timeRequire = 0;

// Motor Control Functions
void controlLeftMotor(int speed, bool direction) {
    digitalWrite(IN1_L, direction ? LOW : HIGH);
    digitalWrite(IN2_L, direction ? HIGH : LOW);
    if (speed == 0) {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, LOW);
    }
    ledcWrite(pwmChannelLeft, speed);
}

void controlRightMotor(int speed, bool direction) {
    digitalWrite(IN1_R, direction ? HIGH : LOW);
    digitalWrite(IN2_R, direction ? LOW : HIGH);
    if (speed == 0) {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, LOW);
    }
    ledcWrite(pwmChannelRight, speed);
}

void setRobotSpeed(float linearVelocity, float angularVelocity) {
    // Calculate wheel velocities
    float vL = linearVelocity + (angularVelocity * wheelBase / 2.0);
    float vR = linearVelocity - (angularVelocity * wheelBase / 2.0);

    // Constrain velocities
    vL = constrain(vL, -maxSpeed, maxSpeed);
   vR = constrain(vR, -maxSpeed, maxSpeed);

    // Map velocities to PWM values
    int leftPWM = map(fabs(vL) * 1000, 0, maxSpeed * 1000, minPWM, maxPWM);
    int rightPWM = map(fabs(vR) * 1000, 0, maxSpeed * 1000, minPWM, maxPWM);

    // Determine required turn time
    if (fabs(angularVelocity) > 1e-6) {
        timeRequire = map(fabs(angularVelocity) * 1000, 0, 1.57 * 1000, 0, maxTurnTime);
    }

    // Ensure minimum PWM threshold
    leftPWM = (fabs(vL) > 1e-6) ? constrain(leftPWM, minPWM, maxPWM) : 0;
    rightPWM = (fabs (vR) > 1e-6) ? constrain(rightPWM, minPWM, maxPWM) : 0;

    // Determine motor directions
    bool directionL = vL >= 0;
    bool directionR = vR >= 0;

    // Control motors
    controlLeftMotor(leftPWM, directionL);
    controlRightMotor(rightPWM, directionR);
}


// Movement Functions
void turnLeft() {
    setRobotSpeed(0, PI / 2);
}

void turnRight() {
    setRobotSpeed(0, -PI / 2);
}

void moveForward() {
    setRobotSpeed(0.1, 0);
}

void moveBackward() {
    setRobotSpeed(-0.1, 0);
}

void stop() {
    setRobotSpeed(0, 0);
}

void reset(){
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  stop();
}

// Function to publish state data
void publishState(int stateNumber, const char *topicPub) {
  JsonDocument doc;

  doc["state_now"]["state"] = stateData.state;
  doc["state_now"]["timeOut"] = stateData.timeOut > stateData.time ? 0 : stateData.timeOut;

  if (stateNumber == 1 || stateNumber == 2) {
    doc["led_1"]["status"] = stateData.led1Status;
    doc["led_1"]["timeOut"] = stateData.led1TimeOut > stateData.led1Time ? 0 : stateData.led1TimeOut;
  }

  if (stateNumber == 2 || stateNumber == 4) {
    doc["led_2"]["status"] = stateData.led2Status;
    doc["led_2"]["timeOut"] = stateData.led2TimeOut > stateData.led2Time ? 0 : stateData.led2TimeOut;
  }
  if (stateNumber == 1){
    Serial.print("Led 2");Serial.println(stateData.led2Status);
    doc["led_2"]["status"] = stateData.led2Status;
  }
  if (stateNumber == 3){
    doc["led_1"]["status"] = stateData.led1Status;
    doc["led_2"]["status"] = stateData.led2Status;
  }
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish(topicPub, jsonBuffer);

  Serial.println("Published state data.");
}

void automatic() {
    float forwardDistance = sensor.getDistanceForward();
    if (forwardDistance > safeDistance) {
        moveForward();
    } else {
        moveBackward();
        delay(500);
        stop();
        float leftDistance = sensor.getDistanceLeft();
        float rightDistance = sensor.getDistanceRight();
        if (leftDistance > rightDistance) {
            turnLeft();
            delay(timeRequire);
        } else {
            turnRight();
            delay(timeRequire);
        }
        stop();
    }
}

void waitingTopic(){
    Serial.print("check receive topic: ");
    Serial.println(checkReceiveTopic);
    if (!checkReceiveTopic && (checkStateControl || checkControl || checkStop)){
      unsigned int startTimeWait = millis();
      checkStateControl = checkControl = checkStop = false;
      while (millis() - startTimeWait <= 2000 && !checkReceiveTopic)
      {
        client.loop();
      }
      if (!checkReceiveTopic){
         Serial.println("Tao gửi topic lên mày có nhận cho thầy không?");
        client.publish("check_auto", "1");
      }
    }
}

void state_1() {
    currentStateNow = "trạng thái 1";
    client.publish(currentStateTopic, currentStateNow);
    timeStartLed1 = millis();
    unsigned int duration = stateData.time - stateData.timeOut;
    startStateTime = millis();
    led1Sts = stateData.led1Status == "on";
    digitalWrite(led1Pin, led1Sts ? HIGH : LOW);
    led2Sts = stateData.led2Status == "on";
    digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
    currentState = 1;
    checkReceiveTopic = false;
    while (millis() - startStateTime < duration && !checkReceiveTopic) {
        client.loop();
        if (millis() - timeStartLed1 > stateData.led1Time - stateData.led1TimeOut) {
            led1Sts = !led1Sts;
            stateData.led1Status = led1Sts ? "on" : "off";
            digitalWrite(led1Pin, led1Sts ? HIGH : LOW);
            timeStartLed1 = millis();
        }
        
        digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
        automatic();
      
    }
    
    if (!(checkControl || checkStateControl || checkStop))checkStateAuto = false;

    reset();
}

void state_2() {
    timeStartLed1 = millis();
    timeStartLed2 = millis();
    currentStateNow = "trạng thái 2";
    client.publish(currentStateTopic, currentStateNow);
    unsigned int duration = stateData.time - stateData.timeOut;
    startStateTime = millis();
    led1Sts = stateData.led1Status == "on";
    digitalWrite(led1Pin, led1Sts ? HIGH : LOW);
    led2Sts = stateData.led2Status == "on";
    digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
    checkReceiveTopic = false;
    currentState = 2;
    while (millis() - startStateTime < duration && !checkReceiveTopic) {
        client.loop();
        if (millis() - timeStartLed1 > stateData.led1Time - stateData.led1TimeOut) {
            led1Sts = !led1Sts;
            stateData.led1Status = led1Sts ? "on" : "off";
            digitalWrite(led1Pin, led1Sts ? HIGH : LOW);
            timeStartLed1 = millis();
        }
        
        if (millis() - timeStartLed2 > stateData.led2Time - stateData.led2TimeOut) {
            led2Sts = !led2Sts;
            stateData.led2Status = led2Sts ? "on" : "off";
            digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
            timeStartLed2 = millis();
        }
   
        automatic();
        
    }
    
    if (!(checkControl || checkStateControl || checkStop))checkStateAuto = false;
    reset();
}
void state_3() {
  checkReceiveTopic = false;
  unsigned long duration = stateData.time - stateData.timeOut ;
  currentStateNow = "trạng thái 3";
  client.publish(currentStateTopic, currentStateNow);
  startStateTime = millis();
  currentState = 3;

  while (millis() - startStateTime < duration && !checkReceiveTopic){
    client.loop();
    led1Sts = stateData.led1Status == "on";
    led2Sts = stateData.led2Status == "on";
    digitalWrite(led1Pin, led1Sts ? HIGH : LOW);
    digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
    automatic();
  }
  if (!(checkControl || checkStateControl || checkStop))checkStateAuto = false;

  reset();
    
}

void state_4() {  
    unsigned int duration = stateData.time - stateData.timeOut;
    timeStartLed2 = millis();
    checkReceiveTopic = false;
    startStateTime = millis();
    currentStateNow = "trạng thái 4";
    client.publish(currentStateTopic, currentStateNow);
    led2Sts = stateData.led2Status == "on";
    digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
    currentState = 4;
    while (millis() - startStateTime < duration &&!checkReceiveTopic){
      client.loop();
        if (millis() - timeStartLed2 > stateData.led1Time - stateData.led2TimeOut) {
            led2Sts = !led2Sts;
            stateData.led2Status = led2Sts ? "on" : "off";
            digitalWrite(led2Pin, led2Sts ? HIGH : LOW);
            timeStartLed2 = millis();
        }
        automatic();
    }
    reset();
    if (!(checkControl || checkStateControl || checkStop)) checkStateAuto = false;
    
}

void checkStateNow(){
  bool t = (checkStateAuto && (checkControl || checkStateControl)) || checkStop;
  Serial.print("check: "); Serial.println(t);
  Serial.print("check state auto: "); Serial.println(checkStateAuto);
  Serial.print("check control: "); Serial.println(checkControl);
  if (t){
    if (currentState == 1){
      stateData.timeOut = millis() - startStateTime;
      stateData.led1TimeOut = millis() - timeStartLed1;
      stateData.led1Status = led1Sts ? "on" : "off";
      stateData.led2Status = led2Sts ? "on" : "off";
    } else if (currentState == 2){
      stateData.timeOut = millis() - startStateTime;
      stateData.led1TimeOut = millis() - timeStartLed1;
      stateData.led2TimeOut = millis() - timeStartLed2;
      stateData.led1Status = led1Sts ? "on" : "off";
      stateData.led2Status = led2Sts ? "on" : "off";
    } else if (currentState == 3){
      stateData.timeOut = millis() - startStateTime;
      stateData.led1Status = led1Sts ? "on" : "off";
      stateData.led2Status = led2Sts ? "on" : "off";
    } else if (currentState == 4){
      stateData.timeOut = millis() - startStateTime;
      stateData.led2TimeOut = millis() - timeStartLed2;
      stateData.led2Status = led2Sts ? "on" : "off";
      }
  }
  if (checkStateAuto && t){
     publishState(currentState, handleTopic);
     Serial.print("State auto");Serial.println(checkStateAuto);
     Serial.print("Control ");Serial.println(checkControl);
     checkStateAuto = false;
     reset();
  }
  if (checkStateControl && checkStop) {
    publishState(currentState, handleTopicManual);
    checkStateControl = checkStop = false;
    reset();
  }
}


void handleState(JsonDocument& doc, int stateNumber) {
  // Parse common state data
  stateData.time = doc["state_now"]["time"] | 0;
  stateData.timeOut = doc["state_now"]["timeOut"] | 0;
  stateData.state = doc["state_now"]["state"] | "";

  // Parse LED 1 data for applicable states
  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 3) {
    stateData.led1Status = doc["led_1"]["status"] | "";
    if (stateNumber == 1 || stateNumber == 2) {
      stateData.led1Time = doc["led_1"]["time"] | 0;
      stateData.led1TimeOut = doc["led_1"]["timeOut"] | 0;
    }
  }

  // Parse LED 2 data for applicable states
  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 3 || stateNumber == 4) {
    stateData.led2Status = doc["led_2"]["status"] | "";
    if (stateNumber == 2 || stateNumber == 4) {
      stateData.led2Time = doc["led_2"]["time"] | 0;
      stateData.led2TimeOut = doc["led_2"]["timeOut"] | 0;
    }
  }

  // Log parsed state data
  Serial.println("Handling state:");
  Serial.print("State: "); Serial.println(stateData.state);
  Serial.print("Time: "); Serial.println(stateData.time);
  Serial.print("TimeOut: "); Serial.println(stateData.timeOut);

  // Log LED 1 data if applicable
  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 3) {
    Serial.print("LED 1 Status: "); Serial.println(stateData.led1Status);
    if (stateNumber == 1 || stateNumber == 2) {
      Serial.print("LED 1 Time: "); Serial.println(stateData.led1Time);
      Serial.print("LED 1 TimeOut: "); Serial.println(stateData.led1TimeOut);
    }
  }

  // Log LED 2 data if applicable
  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 3 || stateNumber == 4) {
    Serial.print("LED 2 Status: "); Serial.println(stateData.led2Status);
    if (stateNumber == 2 || stateNumber == 4) {
      Serial.print("LED 2 Time: "); Serial.println(stateData.led2Time);
      Serial.print("LED 2 TimeOut: "); Serial.println(stateData.led2TimeOut);
    }
  }
}


void handleState1(JsonDocument& doc) {
  checkStateNow();
  handleState(doc, 1);
  state_1();
  if (checkStateControl){
    waitingTopic();
  }
}

void handleState2(JsonDocument& doc) {
  checkStateNow();
  handleState(doc, 2);
  state_2();
  if (checkStateControl){
    waitingTopic();
  }
}


void handleState3(JsonDocument& doc) {
  checkStateNow();
  handleState(doc, 3); 
  state_3();
  if (checkStateControl){
    waitingTopic();
  }
}

void handleState4(JsonDocument& doc) {
  checkStateNow();
  handleState(doc, 4);
  state_4();
  if (checkStateControl){
    waitingTopic();
  }
}

// Initialization Functions
void setupMotors() {
    pinMode(IN1_L, OUTPUT);
    pinMode(IN2_L, OUTPUT);
    pinMode(IN1_R, OUTPUT);
    pinMode(IN2_R, OUTPUT);

    ledcSetup(pwmChannelLeft, pwmFrequency, pwmResolution);
    ledcAttachPin(ENA_L, pwmChannelLeft);

    ledcSetup(pwmChannelRight, pwmFrequency, pwmResolution);
    ledcAttachPin(ENA_R, pwmChannelRight);
}

void setupLEDs() {
    pinMode(led1Pin, OUTPUT);
    pinMode(led2Pin, OUTPUT);
    pinMode(led3Pin, OUTPUT);

    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, LOW);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

// Main Logic for Autonomous Movement


void handleStateTopic(JsonDocument& doc) {
  const char* state = doc["state_now"]["state"];
  if (state) {
    // Ensure the state is safely copied

    // Dispatch to the appropriate state handler
    if (strcmp(state, "state_1") == 0) {
      handleState1(doc);
    } else if (strcmp(state, "state_2") == 0) {
      handleState2(doc);
    } else if (strcmp(state, "state_3") == 0) {
      handleState3(doc);
    } else if (strcmp(state, "state_4") == 0) {
      handleState4(doc);
    } else {
      Serial.println("Unknown state received.");
    }
  } else {
    Serial.println("State field is missing in the message.");
  }
}

void handleControlTopic(JsonDocument& doc) {
  checkStateNow();
  checkReceiveTopic = false;
  currentStateNow = "Điều khiển";
    client.publish(currentStateTopic, currentStateNow);
  if (doc.containsKey("linear_velocity") && doc.containsKey("angular_velocity")) {
    float linearVelocity = doc["linear_velocity"];
    float angular = doc["angular_velocity"];
    unsigned int startime = millis();
    while (millis() - startime < 2000){
      client.loop();
      if (fabs(angular) > 1e-6){
        setRobotSpeed(linearVelocity, angular);
        delay(timeRequire);
        linearVelocity = maxSpeed;
      }
      setRobotSpeed(linearVelocity, angular);
    }
    reset();


    Serial.println("Control command received and processed.");
  } else {
    Serial.println("Missing fields in controlTopic message.");
  }
  waitingTopic();
} 

void handleStopTopic(JsonDocument& doc) {
  checkStateNow(); 
  currentStateNow = "Dừng";
  client.publish(currentStateTopic, currentStateNow);
  Serial.println("Stop topic message received. Logic not implemented.");
}

// LED Blinking and State Logic
bool isJson(String payload) {
    payload.trim();  // Remove any surrounding whitespace
    return (payload.startsWith("{") && payload.endsWith("}")) || 
           (payload.startsWith("[") && payload.endsWith("]"));
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0;i<length;++i){
    message+=(char)payload[i];
  }
  JsonDocument doc; // Use a StaticJsonDocument with a defined size
  if (isJson(message)){
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
      return;
    }
  }
  
  Serial.println(topic);
  // Check for JSON deserialization errors
  
  digitalWrite(led3Pin, HIGH);
  delay(200);
  digitalWrite(led3Pin, LOW);
  if (strcmp(topic, "") != 0){
    checkReceiveTopic = true;
  }
  if (strcmp(topic, autoTopic) == 0) {
    checkStateAuto = true;
    checkStateControl = false;

    Serial.println("Topic auto received");
    reset();
    handleStateTopic(doc);
  }
  else if (strcmp(topic, stateTopic) == 0) {
    checkStateControl = true;
    Serial.println("Control state received");
    handleStateTopic(doc);
  } else if (strcmp(topic, controlTopic) == 0) {
      checkControl = true;
      handleControlTopic(doc);
  } else if (strcmp(topic, stopTopic) == 0) {
      checkStop = true;
      Serial.print("check state control: "); Serial.println(checkStateControl);
      Serial.println("Stop topic receive");
      handleStopTopic(doc);
  } 
  else {
      Serial.println("Unknown topic received.");
  }
  
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT");
      digitalWrite(led3Pin, HIGH);
      delay(200);
      digitalWrite(led3Pin, LOW);
      client.subscribe(controlTopic);
      client.subscribe(autoTopic);
      client.subscribe(stateTopic);
      client.subscribe(stopTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  reconnect();
}

// Arduino Setup
void setup() {
    Serial.begin(115200);
    setupMotors();
    setupLEDs();
    sensor.init();
    setupWiFi();
    setupMQTT();
}

// Main Loop
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    if (!(checkReceiveTopic && checkControl && checkStateAuto && checkStop && checkStateControl)){
      client.publish(currentStateTopic, "Chờ");
    }
    client.loop();
}
