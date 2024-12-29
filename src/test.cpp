#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include<calc_distance.h>
const char* ssid = "NQT";
const char* password = "11345678";
const char* mqttServer = "192.168.1.114";
const char* controlTopic = "robot/control";
const char *stopTopic = "stop";
const char *stateTopic = "state_data";
String currentState = "";
calc_distance getDistance(16, 17, 21);
WiFiClient espClient;
PubSubClient client(espClient);

const int IN1_L = 27;
const int IN2_L = 26;
const int ENA_L = 25;
const int IN1_R = 13;
const int IN2_R = 12;
const int ENA_R = 14;
const int ledPin = 23;

const float safe_distance = 30;

const int pwmFrequency = 1000;
const int pwmResolution = 8;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;

const int maxTurnTime = 240;
const float wheelBase = 0.1;
const float maxSpeed = 0.2823;
const int maxPWM = 255;
const int minPWM = 130;
const unsigned long timeoutDuration = 1000;
const float R = 0.0325;
unsigned long time_require = 0;
unsigned long lastCommandTime = 0;
float linearVelocity = 0.0;
float angular = 0.0;
bool ledOn = false;
unsigned long ledOffTime = 0;
bool commandReceived = false;

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

// Global variables
StateData stateData;



// Function to handle state from JSON
void handleState(JsonDocument& doc, int stateNumber) {
  stateData.time = doc["state_now"]["time"] | 0;
  stateData.timeOut = doc["state_now"]["timeOut"] | 0;
  stateData.state = doc["state_now"]["state"] | "";

  if (stateNumber == 1 || stateNumber == 2) {
    stateData.led1Status = doc["led_1"]["status"] | "";
    stateData.led1Time = doc["led_1"]["time"] | 0;
    stateData.led1TimeOut = doc["led_1"]["timeOut"] | 0;
  }

  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 4) {
    stateData.led2Status = doc["led_2"]["status"] | "";
    stateData.led2Time = doc["led_2"]["time"] | 0;
    stateData.led2TimeOut = doc["led_2"]["timeOut"] | 0;
  }

  // Log state data
  Serial.println("Handling state:");
  Serial.print("State: "); Serial.println(stateData.state);
  Serial.print("Time: "); Serial.println(stateData.time);
  Serial.print("TimeOut: "); Serial.println(stateData.timeOut);

  if (stateNumber == 1 || stateNumber == 2) {
    Serial.print("LED 1 Status: "); Serial.println(stateData.led1Status);
    Serial.print("LED 1 Time: "); Serial.println(stateData.led1Time);
    Serial.print("LED 1 TimeOut: "); Serial.println(stateData.led1TimeOut);
  }

  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 4) {
    Serial.print("LED 2 Status: "); Serial.println(stateData.led2Status);
    Serial.print("LED 2 Time: "); Serial.println(stateData.led2Time);
    Serial.print("LED 2 TimeOut: "); Serial.println(stateData.led2TimeOut);
  }
}

// Function to publish state data
void publishState(int stateNumber) {
  JsonDocument doc;

  doc["state_now"]["state"] = stateData.state;
  doc["state_now"]["time"] = stateData.time;
  doc["state_now"]["timeOut"] = stateData.timeOut;

  if (stateNumber == 1 || stateNumber == 2) {
    doc["led_1"]["status"] = stateData.led1Status;
    doc["led_1"]["time"] = stateData.led1Time;
    doc["led_1"]["timeOut"] = stateData.led1TimeOut;
  }

  if (stateNumber == 1 || stateNumber == 2 || stateNumber == 4) {
    doc["led_2"]["status"] = stateData.led2Status;
    doc["led_2"]["time"] = stateData.led2Time;
    doc["led_2"]["timeOut"] = stateData.led2TimeOut;
  }

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish(stateTopic, jsonBuffer);

  Serial.println("Published state data.");
}
void handleState1(JsonDocument& doc) {
  handleState(doc, 1);
}

void publish_state_1() {
  publishState(1);
}

void handleState2(JsonDocument& doc) {
  handleState(doc, 2);
}

void publish_state_2() {
  publishState(2);
}

void handleState3(JsonDocument& doc) {
  handleState(doc, 3);
}

void publish_state_3() {
  publishState(3);
}

void handleState4(JsonDocument& doc) {
  handleState(doc, 4);
}

void publish_state_4() {
  publishState(4);
}

void controlLeftMotor(int speed, bool direction) {
    // Control the left motor's direction and speed
    digitalWrite(IN1_L, direction ? LOW : HIGH);
    digitalWrite(IN2_L, direction ? HIGH : LOW);
    if (speed == 0) {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, LOW);
    }
    ledcWrite(pwmChannelLeft, speed);
}

void controlRightMotor(int speed, bool direction) {
    // Control the right motor's direction and speed
    digitalWrite(IN1_R, direction ? HIGH : LOW);
    digitalWrite(IN2_R, direction ? LOW : HIGH);
    if (speed == 0) {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, LOW);
    }
    ledcWrite(pwmChannelRight, speed);
}

void setRobotSpeed(float linearVelocity1, float angularVelocity1) {
    // Constants
    // const float wheelBase = 0.2;  // Distance between wheels (meters)
    // const float maxSpeed = 1.0;   // Maximum speed (meters/second)
    // const int minPWM = 100;        // Minimum PWM to overcome motor stall
    // const int maxPWM = 255;       // Maximum PWM value

    // Calculate wheel velocities
    float v_L = linearVelocity1 + (angularVelocity1 * wheelBase / 2.0); // Left wheel velocity
    float v_R = linearVelocity1 - (angularVelocity1 * wheelBase / 2.0); // Right wheel velocity

    // Constrain velocities to maxSpeed
    v_L = constrain(v_L, -maxSpeed, maxSpeed);
    v_R = constrain(v_R, -maxSpeed, maxSpeed);

    // Map velocities to PWM values
    int leftPWM = map(fabs(v_L) * 1000, 0, maxSpeed * 1000, minPWM, maxPWM);
    int rightPWM = map(fabs(v_R) * 1000, 0, maxSpeed * 1000, minPWM, maxPWM);

    // int time_require = 0;
    if (fabs(angularVelocity1) > 1e-6) {
        time_require = map(fabs(angularVelocity1) * 1000, 0, 1.57 * 1000, 0, maxTurnTime);
    }
    // Ensure PWM values meet minimum threshold for movement
    leftPWM = (fabs(v_L) > 1e-6) ? constrain(leftPWM, minPWM, maxPWM) : 0;
    rightPWM = (fabs(v_R) > 1e-6) ? constrain(rightPWM, minPWM, maxPWM) : 0;

    // Determine motor directions
    bool directionL = v_L >= 0;  // True for forward, False for reverse
    bool directionR = v_R >= 0;  // True for forward, False for reverse
    Serial.print("Left PWM: ");
    Serial.println(leftPWM);

    Serial.print("Right PWM: ");
    Serial.println(rightPWM);
    
    Serial.print("Direction left: ");
    Serial.println(directionL);

    Serial.print("Direction right: ");
    Serial.println(directionR);

    Serial.print("Time require: ");
    Serial.println(time_require);
    // Control motors
    controlLeftMotor(leftPWM, directionL);
    controlRightMotor(rightPWM, directionR);
}


void stopRobot() {
  setRobotSpeed(0, 0);
}

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
  if (doc.containsKey("linear_velocity") && doc.containsKey("angular")) {
    linearVelocity = doc["linear_velocity"];
    angular = doc["angular"];
    lastCommandTime = millis();
    commandReceived = true;

    // Indicate command received using LED
    digitalWrite(ledPin, HIGH);
    ledOn = true;
    ledOffTime = millis() + 100;

    Serial.println("Control command received and processed.");
  } else {
    Serial.println("Missing fields in controlTopic message.");
  }
}

void handleStopTopic(JsonDocument& doc) {
  // Implement stop logic here if required
  Serial.println("Stop topic message received. Logic not implemented.");
}

void callback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc; // Use a StaticJsonDocument with a defined size
  DeserializationError error = deserializeJson(doc, payload, length);

  // Check for JSON deserialization errors
  if (error) {
    Serial.print("Error parsing JSON: ");
    Serial.println(error.c_str());
    return;
  }
  // Route messages based on topic
  if (strcmp(topic, stateTopic) == 0) {
    handleStateTopic(doc);
  } else if (strcmp(topic, controlTopic) == 0) {
    handleControlTopic(doc);
  } else if (strcmp(topic, stopTopic) == 0) {
    handleStopTopic(doc);
  } else {
    Serial.println("Unknown topic received.");
  }
}




void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

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

void setupLED() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}

void turnLeft(){
  setRobotSpeed(0, PI/2);
}

void turnRight(){
  setRobotSpeed(0, -PI/2);
}

void moveForward(){
  setRobotSpeed(0.1, 0);
}

void moveBackward(){
  setRobotSpeed(-0.1, 0);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT");
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      client.subscribe(controlTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void automatic() {
  float forwardDistance = getDistance.getDistanceForward();
    if (forwardDistance >=30){
      moveForward();
      delay(10);
    }
    if(forwardDistance < 30){
        float leftDistance = getDistance.getDistanceLeft();
        Serial.print("Left Distance: ");
        Serial.print(leftDistance);
        Serial.println(" cm");

    }
    // delay(1000);
    // float rightDistance = getDistance.getDistanceRight();

    Serial.print("Forward Distance: ");
    Serial.print(forwardDistance);
    Serial.println(" cm");

    // Serial.print("Right Distance: ");
    // Serial.print(rightDistance);
    // Serial.println(" cm");

    delay(1000);
}



void setupMQTT() {
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
  reconnect();
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  getDistance.init();
  // setupWiFi();
  // setupLED();
  // setupMQTT();
}

void loop(){
  automatic();
  
}

