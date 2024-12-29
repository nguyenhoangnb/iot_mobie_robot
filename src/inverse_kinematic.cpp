// #include<Arduino.h>
// const float WHEEL_BASE = 0.2;    
// const float WHEEL_RADIUS = 0.05; 
// const float MAX_LINEAR_VELOCITY = 1.0; 
// const float MAX_PWM = 255.0;          

// const int LEFT_MOTOR_PWM = 3;   
// const int LEFT_MOTOR_DIR = 4;   
// const int RIGHT_MOTOR_PWM = 5;  
// const int RIGHT_MOTOR_DIR = 6;  

// void calculateWheelVelocities(float v, float omega, float& v_left, float& v_right) {
//     v_left = v - (WHEEL_BASE / 2.0) * omega;
//     v_right = v + (WHEEL_BASE / 2.0) * omega;
// }

// int velocityToPWM(float velocity) {
//     return (int)(velocity / MAX_LINEAR_VELOCITY * MAX_PWM);
// }

// void setMotorSpeed(int pwmPin, int dirPin, float velocity) {
//     int pwmValue = abs(velocityToPWM(velocity));
//     pwmValue = constrain(pwmValue, 0, 255);

//     if (velocity >= 0) {
//         digitalWrite(dirPin, HIGH);
//     } else {
//         digitalWrite(dirPin, LOW);
//     }
//     analogWrite(pwmPin, pwmValue);
// }

// void setup() {
//     pinMode(LEFT_MOTOR_PWM, OUTPUT);
//     pinMode(LEFT_MOTOR_DIR, OUTPUT);
//     pinMode(RIGHT_MOTOR_PWM, OUTPUT);
//     pinMode(RIGHT_MOTOR_DIR, OUTPUT);
//     Serial.begin(9600);
// }

// void loop() {
//     float linear_velocity = 0.5;
//     float angular_velocity = 1.0;

//     float v_left, v_right;
//     calculateWheelVelocities(linear_velocity, angular_velocity, v_left, v_right);

//     Serial.print("Left Wheel Velocity (m/s): ");
//     Serial.println(v_left);
//     Serial.print("Right Wheel Velocity (m/s): ");
//     Serial.println(v_right);

//     setMotorSpeed(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, v_left);
//     setMotorSpeed(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR, v_right);

//     delay(100);
// }
