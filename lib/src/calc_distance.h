#ifndef calc_distance_h
#define calc_distance_h

#include <Arduino.h>
#include <ESP32Servo.h>

class calc_distance {
public:
    calc_distance(int trigPin, int echoPin, int servoPin);
    void init();
    float measureDistance();
    float getDistanceForward();
    float getDistanceLeft();
    float getDistanceRight();

private:
    int _trigPin;
    int _echoPin;
    int _servoPin;
    Servo _myServo;

    const int SAFE_DISTANCE = 30;
    const int FORWARD_ANGLE = 90;
    const int LEFT_ANGLE = 135;
    const int RIGHT_ANGLE = 45;

    const unsigned long TIMEOUT = 5000;

    void moveServo(int angle);
};

#endif
