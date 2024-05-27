#include <Arduino.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x30, 0xC9, 0x22, 0x32, 0xD4, 0x40};
constexpr char WIFI_SSID[] = "A14.13";
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}
// Chân kết nối
#define gasSensorPin  34 // Chân AO của MQ-5 nối với A0 trên ESP8266
const int buzzerPin = 22;    // Chân dương của buzzer nối với D1 trên ESP8266
const int greenLEDPin = 4;  // Chân dương của LED xanh nối với D2 trên ESP8266
const int redLEDPin = 27;    // Chân dương của LED đỏ nối với D3 trên ESP8266
const int relayPin = 23;     // Chân điều khiển relay nối với D4 trên ESP8266
const int switchPin = 15;    // Chân điều khiển công tắc nối với D5 trên ESP8266
const int servoPin = 33; //////////////
#define servoSwitchPin 14  ////////////
// Ngưỡng nồng độ khí gas để kích hoạt buzzer và LED đỏ
const int gasThreshold = 500; 
//Góc quay tối đa của servo
const int servoClosedAngle = 0;    // Góc đóng cửa
const int servoOpenAngle = 180;     // Góc mở cửa
Servo doorServo;   // Khởi tạo đối tượng Servo
int currentServoAngle = servoClosedAngle; // Theo dõi góc hiện tại của servo
//Must match the receiver structure
typedef struct struct_message_send1 {
    int id;
    bool lightState3;
    int gasValue;
    int currentServo;
} struct_message_send1;

struct_message_send1 lastReceivedData;

typedef struct struct_message_coming {
  bool lightStatefromMaster;
  int  ServoAnglefromMaster;
} struct_message_coming;

struct_message_coming incomingmessage;

bool LED_State_Receive;
int  Servo_Angle_Receive;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Callback when data is received
//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingmessage, incomingData, sizeof(incomingmessage));
  Serial.println("Hello from master");
  Serial.print("Bytes received: ");
  Serial.println(len);
  LED_State_Receive = incomingmessage.lightStatefromMaster;
  Serial.print("Light State: ");
  Serial.println(LED_State_Receive);
  digitalWrite(relayPin, LED_State_Receive);
  Servo_Angle_Receive = incomingmessage.ServoAnglefromMaster;
  Serial.print("Servo Angle: ");
  Serial.println(Servo_Angle_Receive);
  doorServo.write(Servo_Angle_Receive);
}

void setup() {
  // Khởi tạo chân buzzer và LED là OUTPUT
  pinMode(buzzerPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);

  pinMode(relayPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); // Sử dụng pull-up nội bộ
  
  pinMode(servoSwitchPin, INPUT_PULLUP);
  doorServo.attach(servoPin);       // Gắn chân điều khiển servo
  // Khởi tạo kết nối Serial để debug
  Serial.begin(115200);
  // Khởi tạo relay ở trạng thái tắt
  digitalWrite(relayPin, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  lastReceivedData.id =3;
  // Đọc giá trị từ cảm biến MQ-5
  static bool lastStateLight = HIGH;
  static bool lastStateServo = HIGH;
  static bool servoOpen = false;
  static bool autoServoOpen = false;
  static unsigned long lastButtonPress = 0;
  static unsigned long lastPrintTime = 0;
  static unsigned long lastSendTime = 0;
  int gasValue = analogRead(gasSensorPin);
  bool currentStateLight = digitalRead(switchPin);
  bool currentStateServo = digitalRead(servoSwitchPin);
  unsigned long currentMillis = millis();

  if (currentMillis - lastSendTime >= 3000) {
    lastSendTime = currentMillis;
    // Debounce check for light button
    lastReceivedData.lightState3 = digitalRead(relayPin);
    lastReceivedData.gasValue = gasValue;
    lastReceivedData.currentServo = Servo_Angle_Receive;
    // lastReceivedData.currentServo = currentServoAngle;
    Serial.println("Sending data...");
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &lastReceivedData, sizeof(lastReceivedData));
  } 
  // Kiểm tra trạng thái công tắc để bật/tắt relay
  if (millis() - lastButtonPress > 200) {
    if (currentStateLight == LOW && lastStateLight == HIGH) {
      digitalWrite(relayPin, !digitalRead(relayPin));
      lastButtonPress = millis();
    }
    lastStateLight = currentStateLight;
  }
    // Kiểm tra trạng thái công tắc để đóng/mở servo
  if (millis() - lastButtonPress > 200) {  
    if (currentStateServo == LOW && lastStateServo == HIGH) {
      servoOpen = !servoOpen; // Đảo trạng thái servo
      autoServoOpen = false;  // Tắt chế độ tự động khi công tắc được nhấn
      if (servoOpen) {
        currentServoAngle = servoOpenAngle;
        doorServo.write(currentServoAngle); // Mở servo
        Servo_Angle_Receive = currentServoAngle;
        Serial.println("Servo opened.");
      } else {
        currentServoAngle = servoClosedAngle;
        doorServo.write(currentServoAngle); // Đóng servo
        Servo_Angle_Receive = currentServoAngle;
        Serial.println("Servo closed.");
      }
      lastButtonPress = millis();
    }
    lastStateServo = currentStateServo;
  }


  if (currentMillis - lastPrintTime >= 5000) {
    lastPrintTime = currentMillis;
  // In giá trị đọc được ra Serial Monitor
  Serial.print("Gas Sensor Value: ");
  Serial.println(gasValue);

  // Kiểm tra nếu giá trị đọc được vượt ngưỡng
  if (gasValue > gasThreshold) {
    // Bật buzzer và LED đỏ, tắt LED xanh
    digitalWrite(buzzerPin, LOW);
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
    Serial.println("Gas detected! Buzzer and Red LED ON!");
  // Mở cửa nếu chưa mở và chưa ở chế độ tự động
  if (!servoOpen || !autoServoOpen) {
      currentServoAngle = servoOpenAngle;
      doorServo.write(currentServoAngle);
      servoOpen = true;
      autoServoOpen = true;
      Serial.println("Servo opened automatically.");
    }
  } else {
    // Tắt buzzer và LED đỏ, bật LED xanh
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    digitalWrite(greenLEDPin, HIGH);
    Serial.println("Gas level normal. Green LED ON.");
    Serial.print("Light State: ");
    Serial.print(lastReceivedData.lightState3);
    // Đóng cửa nếu đang mở và ở chế độ tự động
    if (servoOpen && autoServoOpen) {
        currentServoAngle = servoClosedAngle;
        doorServo.write(currentServoAngle);
        servoOpen = false;
        autoServoOpen = false;
        Serial.println("Servo closed automatically.");
      }
    }
  }
}