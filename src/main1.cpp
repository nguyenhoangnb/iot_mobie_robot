// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <AccelStepper.h>
// #include<ArduinoJson.h>
// #include<calc_distance.h>
// #include<control_motor.h>
// // Cấu hình WiFi
// const char* ssid = "NQT";           // WiFi SSID
// const char* password = "11345678";  // WiFi Password

// calc_distance get_distance(17, 15, 4);


// // Cấu hình MQTT
// const char* mqtt_server = "192.168.1.114"; // MQTT Broker IP
// const int mqttPort = 1883;
// const char* mqttTopic = "commands";     

// const float save_distance = 10;
// float left_distance;
// float right_distance;
// float distance ;
// // Thời gian mặc định
// const int interval = 10000; 

// #define HALFSTEP 4

// int speed = 1000;

// // AccelStepper stepper1(HALFSTEP, 32, 33, 25, 32);
// AccelStepper stepper1(HALFSTEP, 21, 18, 19, 5);
// AccelStepper stepper2(HALFSTEP, 27, 12, 14, 13);

// control_motor motor(stepper1, stepper2);
// float wheelBase = 0.11;
// enum MotorState { STOP, TURN_LEFT, TURN_RIGHT, FORWARD, BACKWARD };
// MotorState currentState = STOP;

// // MQTT và WiFi clients
// WiFiClient espClient;
// PubSubClient client(espClient);

// // MQTT topic status flag
// bool topic_status = false; 



// // Định nghĩa stack lưu lệnh
// #define MAX_COMMAND 1000
// struct CommandNode{
//     String Command;
//     CommandNode* next;
// };
// class CommandStack{
//     private:
//         CommandNode *top;
//     public:
//         CommandStack():top(nullptr){}
//     void push(const String& cmd){
//         CommandNode *newNode = new CommandNode;
//         newNode->Command = cmd;
//         newNode->next = top;
//         top = newNode;
//         delete newNode;

//     }
//     String pop(){
//         if (top == nullptr){
//             return "";
//         }
//         // CommandNode* temp = top;
//         String cmd = top->Command;
//         top = top->next;
//         return cmd;
//     }
//     bool isEmpty(){
//         return top == nullptr;
//     }
// };

// String generate_message(char mode, unsigned int time, int speed, float dis);
// void callback(char* topic, byte* payload, unsigned int length );
// void reconnect();
// void servoScan();
// void automatic();
// void executeCommand();
// void saveZone();
// CommandStack commandStack;
// void setup() {
//     Serial.begin(115200);
//     get_distance.init();
//     motor.init();
//     motor.stopMotors();
//     Serial.print("Kết nối WiFi...");
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED) {
//         delay(500);
//         Serial.print(".");
//     }
//     Serial.println("\nKết nối WiFi thành công!");

//     client.setServer(mqtt_server, mqttPort);
//     client.setCallback(callback);

//     reconnect();
// }

// void loop() {
//     if (!client.connected()) {
//         reconnect();
//     }
//     client.loop();

//     if (!topic_status) {
//         //commandStack.freeStack();
//         automatic();
//     }
//     else if (topic_status){
//         executeCommand();
//     }
  
// }


// // Callback nhận lệnh từ MQTT
// void callback(char* topic, byte* payload, unsigned int length) {
//     Serial.print("Nhận lệnh từ [");
//     Serial.print(topic);
//     Serial.print("]: ");
//     String message;
//     for (int i = 0; i < length; i++) {
//         message += (char)payload[i];
//         Serial.print((char)payload[i]);
//     }
//     Serial.println();
//     commandStack.push(message);

    
// }



// String generate_message(char mode, unsigned int time, int speed, float dis) {
//     JsonDocument buffer;
//     buffer["Mode"] = String(mode);
//     buffer["time"] = time;
//     // buffer["speed"] = speed;
//     // buffer["distance"] = dis;
//     String message;
//     serializeJson(buffer, message);
//     return message;
// }

// // Thực thi lệnh
// void executeCommand() {
//     String message = commandStack.pop();
//     JsonDocument docs;
//     deserializeJson(docs, message);
//     unsigned cmd = (unsigned char)docs["Mode"];
//     unsigned int time = docs["time"] | 10000;
//     // int speed = docs["speed"];
//     // float distance_control = docs["distance"];
//     int speed = 1000;
//     float dis  = get_distance.get_distance_forward();
//     bool auto_mate = true;
//     if (topic_status){
//         auto_mate = false;
//     }
//     switch (cmd) {
//         case 'A': {
//             Serial.println("Command A: Go ahead");
//             topic_status = false;
//             // digitalWrite(led1, HIGH);
//             motor.moveForward(speed);
//             unsigned long startMillis = millis();
//             while (millis() - startMillis < time && !topic_status) {
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, speed, dis));
//             }
//             break;
//         }
//         case 'B': {
//             Serial.println("Command B: Go Behind.");
//             topic_status = false;
//             motor.moveBackward(speed);
//             unsigned long startMillis = millis();
//             while (millis() - startMillis < time && !topic_status){
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, speed, dis));
//             }
//             break;
//         }
//         case 'L': {
//             Serial.println("Command L: Turn Left.");
//             unsigned long startMillis = millis();
//             motor.turnLeft(speed);
//             while (millis() - startMillis < time && !topic_status){
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, speed, dis));
//             }
            
//             break;
//         }
//         case 'R': {
//             Serial.println("Command R: Turn Right.");
//             unsigned long startMillis = millis();
//             motor.turnRight(speed);
//             while (millis() - startMillis < time && !topic_status){
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, speed, dis));
//             }
//             break;
//         }
//         case 'S': {
//             Serial.println("Command S: Stop.");
//             unsigned long startMillis = millis();
//             motor.stopMotors();
//             while (millis() - startMillis < time && !topic_status){
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, 0, dis));
//             }
//             break;
//         }

//         case 'C': {
//             Serial.println("Command C: Turn Right.");
//             unsigned long startMillis = millis();
//             while (millis() - startMillis < time && !topic_status){
//                 client.loop(); // Duy trì kết nối MQTT
//             }
//             if (topic_status && auto_mate){
//                 unsigned int time_break = millis() - startMillis;
//                 commandStack.push(generate_message(cmd, time_break, speed, dis));
//             }
//             break;
//         }

//         default: {
//             Serial.println("Lệnh không xác định.");
//             break;
//         }
//     }
// }

// // Kết nối lại MQTT Broker
// void reconnect() {
//     while (!client.connected()) {
//         Serial.print("Đang kết nối lại MQTT...");
//         if (client.connect("ESP32Client")) {
//             Serial.println("Kết nối MQTT thành công.");
//             client.subscribe(mqttTopic);
//         } else {
//             Serial.print("Thất bại, rc=");
//             Serial.print(client.state());
//             Serial.println(" Thử lại sau 5 giây.");
//             delay(5000);
//         }
//     }
// }




// // Chế độ tự động
// void automatic() {
//     Serial.println("Chuyển sang chế độ tự động...");
//     while (!topic_status){
//         float distanceL = 0;
//         float distanceR = 0;
//         distance = get_distance.get_distance_forward();
//         Serial.print("Distance");
//         Serial.println(distance);
//         if (distance > save_distance){
//           motor.moveForward(speed);
//           distance = get_distance.get_distance_forward();

//         } else if (distance < save_distance){
//             motor.stopMotors();
//             get_distance.DELAY(500);
//             motor.moveBackward(speed);
//             get_distance.DELAY(500);
//             motor.stopMotors();
//             get_distance.DELAY(500);
//             distanceR = get_distance.get_distance_right();
//             distanceL = get_distance.get_distance_left();
//             if (distanceL < distanceR){
//                 motor.turnRight(speed);
//                 motor.stopMotors();
//             } else {
//                 motor.turnLeft(speed);
//                 motor.stopMotors();
//             }
//         }
//     }
    
//     Serial.println("Thoát chế độ tự động do có lệnh MQTT.");
// }


