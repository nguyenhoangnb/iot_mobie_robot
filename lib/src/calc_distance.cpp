#include "calc_distance.h"

calc_distance::calc_distance(int trigPin, int echoPin, int servoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
    _servoPin = servoPin;
}

void calc_distance::init() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    _myServo.setPeriodHertz(50);
    _myServo.attach(_servoPin, 500, 2400);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    _myServo.write(FORWARD_ANGLE);
    delay(1000);
}

float calc_distance::measureDistance() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    long duration = pulseIn(_echoPin, HIGH, TIMEOUT);

    float distance = (duration / 2.0) / 29.1;

    return (distance > 0) ? distance : 5000;
}

void calc_distance::moveServo(int angle) {
    _myServo.write(angle);
    delay(300);
}

float calc_distance::getDistanceForward() {
    // moveServo(FORWARD_ANGLE);
    return measureDistance();
}

float calc_distance::getDistanceLeft() {
    moveServo(LEFT_ANGLE);
    float distance = measureDistance();
    moveServo(FORWARD_ANGLE);
    delay(300);
    return distance;
}

float calc_distance::getDistanceRight() {
    moveServo(RIGHT_ANGLE);
    float distance = measureDistance();
    moveServo(FORWARD_ANGLE);
    delay(300);
    return distance;
}
