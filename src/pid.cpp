// #include<Arduino.h>
// #include "DHT.h"

// // Khai báo loại cảm biến và chân kết nối
// #define DHTPIN 33     // Chân DATA của DHT22
// #define DHTTYPE DHT22 // Chọn loại cảm biến (DHT11, DHT22, v.v.)

// DHT dht(DHTPIN, DHTTYPE); // Tạo đối tượng DHT

// void setup() {
//   Serial.begin(115200);
//   Serial.println("Khởi động DHT22...");
  
//   dht.begin(); // Khởi tạo cảm biến
// }

// void loop() {
//   // Đọc nhiệt độ và độ ẩm
//   float humidity = dht.readHumidity();    // Đọc độ ẩm (%)
//   float temperature = dht.readTemperature(); // Đọc nhiệt độ (°C)

//   // Kiểm tra lỗi đọc dữ liệu
//   if (isnan(humidity) || isnan(temperature)) {
//     Serial.println("Lỗi đọc cảm biến!");
//     return;
//   }

//   // In dữ liệu ra Serial Monitor
//   Serial.print("Độ ẩm: ");
//   Serial.print(humidity);
//   Serial.println(" %");

//   Serial.print("Nhiệt độ: ");
//   Serial.print(temperature);
//   Serial.println(" °C");

//   delay(2000); // Đọc dữ liệu mỗi 2 giây
// }
