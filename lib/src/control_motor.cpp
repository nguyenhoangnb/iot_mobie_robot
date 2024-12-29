// #include "control_motor.h"

// // Constructor to initialize stepper motors with references
// control_motor::control_motor(AccelStepper &step1, AccelStepper &step2) : stepper1(step1), stepper2(step2) {}

// void control_motor::init() {
//     // Set maximum speed and acceleration for the motors
//     stepper1.setMaxSpeed(1000);  // Adjust the max speed as needed
//     stepper2.setMaxSpeed(1000);  // Adjust the max speed as needed
//     stepper1.setAcceleration(500);  // Set acceleration for stepper1
//     stepper2.setAcceleration(500);  // Set acceleration for stepper2
// }

// void control_motor::run() {
//     stepper1.run();
//     stepper2.run();
// }

// void control_motor::stopMotors() {
//     stepper1.stop();
//     stepper2.stop();
//     Serial.println("Motors stopped.");
//     run();  // Run to stop movement (without blocking)
// }

// void control_motor::turnLeft(int speed) {
//     stepper1.setSpeed(0);      // Motor 1 speed
//     stepper2.setSpeed(speed);     // Motor 2 speed (opposite direction)
//     unsigned int startTime = millis();
//     while (millis() - startTime < 10000){
//         stepper1.setSpeed(0);     // Both motors in the opposite direction
//         stepper2.setSpeed(speed);
//         run();
//         Serial.println("Turning left.");
//     }
// }

// void control_motor::turnRight(int speed) {
//     stepper1.setSpeed(speed);     // Motor 1 speed (opposite direction)
//     stepper2.setSpeed(0);      // Motor 2 speed
    
//     unsigned int startTime = millis();
//     while (millis() - startTime < 10000){
//         stepper1.setSpeed(speed);     // Both motors in the opposite direction
//         stepper2.setSpeed(0);
//         run();
//         Serial.println("Turning right.");
//     }

// }

// void control_motor::moveForward(int speed) {
//     stepper1.setSpeed(-speed);      // Both motors in the same direction
//     stepper2.setSpeed(speed);
//     run();
//     unsigned int startTime = millis();
//     while (millis() - startTime < 10000){
//         stepper1.setSpeed(-speed);      // Both motors in the same direction
//         stepper2.setSpeed(speed);
//         run();
//         Serial.println("Moving forward.");
//     }
//     Serial.println("Moving forward.");
// }

// void control_motor::moveBackward(int speed) {
//     stepper1.setSpeed(-speed);     // Both motors in the opposite direction
//     stepper2.setSpeed(-speed);
//     unsigned int startTime = millis();
//     while (millis() - startTime < 10000){
//         stepper1.setSpeed(speed);     // Both motors in the opposite direction
//         stepper2.setSpeed(-speed);
//         run();
//         Serial.println("Moving backward.");
//     }
// }
