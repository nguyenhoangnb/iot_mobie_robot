// #include <Arduino.h>
// #include <AccelStepper.h>

// // Robot parameters
// const float wheelRadius = 0.4; // meters
// const float wheelBase = 0.825; // meters
// const float dt = 0.033;        // Control loop time step in seconds (~30Hz)

// // Robot state
// float x = 0.0;       // X position
// float y = 0.0;       // Y position
// float theta = 0.0;   // Orientation in radians

// // Stepper parameters
// const int stepsPerRevolution = 200; // Steps per revolution of the stepper
// const float gearRatio = 1.0;        // Gear ratio
// const float wheelCircumference = 2 * PI * wheelRadius;

// // Stepper driver pins
// #define LEFT_STEP_PIN 2
// #define LEFT_DIR_PIN 3
// #define RIGHT_STEP_PIN 4
// #define RIGHT_DIR_PIN 5

// // AccelStepper instances for left and right motors
// AccelStepper leftStepper(AccelStepper::DRIVER, LEFT_STEP_PIN, LEFT_DIR_PIN);
// AccelStepper rightStepper(AccelStepper::DRIVER, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

// void setup() {
//   // Initialize stepper motors
//   leftStepper.setMaxSpeed(1000);    // Maximum speed in steps per second
//   leftStepper.setAcceleration(500); // Acceleration in steps per second^2
//   rightStepper.setMaxSpeed(1000);
//   rightStepper.setAcceleration(500);

//   Serial.begin(115200); // Initialize serial for debugging
// }

// void loop() {
//   static unsigned long lastUpdate = 0;

//   // Time control loop
//   if (millis() - lastUpdate >= dt * 1000) {
//     lastUpdate = millis();

//     // Calculate odometry (not directly using encoders here)
//     updateOdometry();

//     // Set desired velocity (example: move forward with 0.5 m/s, no rotation)
//     float linearVelocity = 0.5;  // meters/second
//     float angularVelocity = 0.0; // radians/second
//     controlMotors(linearVelocity, angularVelocity);

//     // Print odometry for debugging
//     Serial.print("X: ");
//     Serial.print(x);
//     Serial.print(" Y: ");
//     Serial.print(y);
//     Serial.print(" Theta: ");
//     Serial.println(theta);
//   }

//   // Run the stepper motors
//   leftStepper.run();
//   rightStepper.run();
// }

// // Update odometry based on stepper positions
// void updateOdometry() {
//   static long lastLeftPosition = 0;
//   static long lastRightPosition = 0;

//   // Get current stepper positions
//   long leftPosition = leftStepper.currentPosition();
//   long rightPosition = rightStepper.currentPosition();

//   // Calculate distances moved
//   float leftWheelDistance = (leftPosition - lastLeftPosition) * (wheelCircumference / stepsPerRevolution) / gearRatio;
//   float rightWheelDistance = (rightPosition - lastRightPosition) * (wheelCircumference / stepsPerRevolution) / gearRatio;

//   // Update for next iteration
//   lastLeftPosition = leftPosition;
//   lastRightPosition = rightPosition;

//   // Calculate robot motion
//   float distance = (leftWheelDistance + rightWheelDistance) / 2.0;
//   float angleChange = (rightWheelDistance - leftWheelDistance) / wheelBase;

//   // Update robot state
//   x += distance * cos(theta);
//   y += distance * sin(theta);
//   theta += angleChange;

//   // Keep theta in the range [-pi, pi]
//   if (theta > PI) theta -= 2 * PI;
//   if (theta < -PI) theta += 2 * PI;
// }

// // Control stepper motors based on desired linear and angular velocity
// void controlMotors(float linearVelocity, float angularVelocity) {
//   // Calculate wheel speeds in meters/second
//   float leftWheelSpeed = (2 * linearVelocity - angularVelocity * wheelBase) / 2.0;
//   float rightWheelSpeed = (2 * linearVelocity + angularVelocity * wheelBase) / 2.0;

//   // Convert wheel speeds to steps per second
//   float leftStepsPerSecond = (leftWheelSpeed / wheelCircumference) * stepsPerRevolution * gearRatio;
//   float rightStepsPerSecond = (rightWheelSpeed / wheelCircumference) * stepsPerRevolution * gearRatio;

//   // Set stepper speeds
//   leftStepper.setSpeed(leftStepsPerSecond);
//   rightStepper.setSpeed(rightStepsPerSecond);
// }
