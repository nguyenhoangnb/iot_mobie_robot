#include <calc_distance.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "NQT";
const char* password = "11345678";
const char* mqtt_server = "192.168.1.114";
const char* controlTopic = "robot/control";
const char *stopTopic = "stop";
const char *stateTopic = "state_data";
const char *autoTopic = "automatic";
String currentState = "";
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
  long led2PinTime;
  long led2PinTimeOut;
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
int timeStartled2 = 0;

bool led_status1 = true;
bool led_status2 = true;

bool checkReceiveTopic = false;

// Distance Sensor Parameters
const float safeDistance = 30.0;

bool stateAuto = false;
bool stateControl = false;
bool control = true;

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



// LED Blinking and State Logic
bool isJson(String payload) {
    payload.trim();  // Remove any surrounding whitespace
    return (payload.startsWith("{") && payload.endsWith("}")) || 
           (payload.startsWith("[") && payload.endsWith("]"));
}

void callback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc; // Use a StaticJsonDocument with a defined size
  String payloadStr = String((char*)payload, length);
  if (isJson(payloadStr)){
    Serial.println("Json data");
  } else {
    Serial.println("Stirng data");
    Serial.println(payloadStr);
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
      client.subscribe(stopTopic);
      client.subscribe(autoTopic);
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
    client.loop();
}
