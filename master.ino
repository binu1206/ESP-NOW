#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFiClientSecure.h>
#include <esp_crt_bundle.h>
#include <ssl_client.h>
#include <PubSubClient.h>

#define  BUTTON_LIGHT_PIN1 21//4
#define  BUTTON_FAN_PIN1  19//12
#define  RELAY_LIGHT_PIN1 32//15
#define  RELAY_FAN_PIN1  33

#define  BUTTON_LIGHT_PIN2 18//14
#define  BUTTON_FAN_PIN2  5//32
#define  RELAY_LIGHT_PIN2 25//13
#define  RELAY_FAN_PIN2  26//33

#define  BUTTON_LIGHT_PIN3 4//5
#define  BUTTON_SERVO_PIN3 15//18
#define  RELAY_LIGHT_PIN3  27//21
#define  RELAY_SERVO_PIN3  14//25
// boolean  updateState1 = 0; 
// boolean  updateStte2 a= 0;
// Replace with your network credentials
const char* ssid = "A14.13";
const char* password = "0855707879";
// MQTT Broker
const char* mqtt_server = "8f3a4feef3e14ed7821b4ae60986fca6.s1.eu.hivemq.cloud"; // Using HiveMQ public broker
const int   mqtt_port = 8883;
const char* mqtt_username = "pynux1206";
const char* mqtt_password = "Sktt1faker";
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress1[] = {0x30, 0xC9, 0x22, 0x32, 0x59, 0x90};
uint8_t broadcastAddress2[] = {0xD8, 0x13, 0x2A, 0x7F, 0x9C, 0x6C};
uint8_t broadcastAddress3[] = {0xD8, 0x13, 0x2A, 0x7F, 0xA9, 0x18};
WiFiClientSecure espClient;
PubSubClient client(espClient);
const int  servoClosedAngle = 0;
const int  servoOpenAngle   = 180;
//Structure example to send data
//Must match the receiver structure
typedef struct struct_message_send1 {
    bool lightStatetoslave1;
    bool fanStatetoslave1;
} struct_message_send1;
typedef struct struct_message_send2 {
    bool lightStatetoslave2;
    bool fanStatetoslave2;
} struct_message_send2;
typedef struct struct_message_send3 {
    bool lightStatetoslave3;
    int  ServoAngle;
} struct_message_send3;


typedef struct struct_message_coming1 {
  int   id;
  float temp;
  float humidity;
  bool  lightState;
  bool  fanState;
}struct_message_coming1;

typedef struct struct_message_coming2 {
  int id;
  bool lightState3;
  int gasValue;
  int currentServo;
} struct_message_coming2;

struct_message_coming1 mylastReceivedDhtData;
struct_message_coming2 mylastReceivedKitchenData;


struct_message_coming1 board1;
struct_message_coming1 board2;


struct_message_send1 send_Data_slave1;
struct_message_send2 send_Data_slave2;
struct_message_send3 send_Data_slave3;
// Create an array with all the structures
struct_message_coming1 boardsStruct[2] = {board1,board2};
// struct_message_coming2 boardsStruct2[1] = {board3};
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
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  // int incomingId = incomingData[0];
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  if (len == sizeof(struct_message_coming1)) {
  memcpy(&mylastReceivedDhtData, incomingData, sizeof(mylastReceivedDhtData));
  Serial.printf("Received from Board ID %d: ",mylastReceivedDhtData.id);
  Serial.printf("Board ID %u: %u bytes\n", mylastReceivedDhtData.id, len);
  if (mylastReceivedDhtData.id == 1 || mylastReceivedDhtData.id == 2){
  // Update the structures with the new incoming data
  boardsStruct[mylastReceivedDhtData.id-1].temp = mylastReceivedDhtData.temp;
  boardsStruct[mylastReceivedDhtData.id-1].humidity = mylastReceivedDhtData.humidity;
  Serial.printf("Temperature value: %f \n", boardsStruct[mylastReceivedDhtData.id-1].temp);
  Serial.printf("Humidity value: %f \n", boardsStruct[mylastReceivedDhtData.id-1].humidity);
  Serial.println();
  boardsStruct[mylastReceivedDhtData.id-1].lightState = mylastReceivedDhtData.lightState;
  boardsStruct[mylastReceivedDhtData.id-1].fanState = mylastReceivedDhtData.fanState;
  Serial.printf("lightState value: %d \n", boardsStruct[mylastReceivedDhtData.id-1].lightState);
  Serial.printf("fanState value: %d \n", boardsStruct[mylastReceivedDhtData.id-1].fanState);
  digitalWrite(RELAY_LIGHT_PIN1,boardsStruct[0].lightState);
  digitalWrite(RELAY_FAN_PIN1,boardsStruct[0].fanState);
  digitalWrite(RELAY_LIGHT_PIN2,boardsStruct[1].lightState);
  digitalWrite(RELAY_FAN_PIN2,boardsStruct[1].fanState);
  Serial.println();
   }
  } 
  ////////////////////////////////////////////////////////////////////////////////////////////////
  if (len == sizeof(struct_message_coming2)) {
    memcpy(&mylastReceivedKitchenData, incomingData, sizeof(mylastReceivedKitchenData));
    Serial.printf("Received from Board ID %d: \n",mylastReceivedKitchenData.id);
    if (mylastReceivedKitchenData.id == 3){
    Serial.printf("Gas value: %d \n",mylastReceivedKitchenData.gasValue);
    Serial.printf("Light State: %d \n",mylastReceivedKitchenData.lightState3);
    digitalWrite(RELAY_LIGHT_PIN3,mylastReceivedKitchenData.lightState3);
    Serial.printf("Current Servo: %d \n",mylastReceivedKitchenData.currentServo);
    if (mylastReceivedKitchenData.currentServo == 180){
      digitalWrite(RELAY_SERVO_PIN3,HIGH);
    }  
    if (mylastReceivedKitchenData.currentServo == 0) {
      digitalWrite(RELAY_SERVO_PIN3,LOW);
    }
    Serial.println();   
    }
  }  
}
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    int32_t channel = WiFi.channel();
    Serial.print("WiFi Channel: ");
    Serial.println(channel);
    randomSeed(micros());
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}
//------------Connect to MQTT Broker--------------------//
void reconnect() {
    // Loop until we're reconnected
    Serial.println("Connecting to MQTT Broker...");
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        String clientID = "ESP32Client-";
        clientID += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected");
            // Once connected, publish an announcement...
            client.publish("outTopic", "hello world");
            // ... and resubscribe
            client.subscribe("ESP32/client");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            switch (client.state()) {
                  case -4: Serial.println(" MQTT_CONNECTION_TIMEOUT"); break;
                  case -3: Serial.println(" MQTT_CONNECTION_LOST"); break;
                  case -2: Serial.println(" MQTT_CONNECT_FAILED"); break;
                  case -1: Serial.println(" MQTT_DISCONNECTED"); break;
                  case 0: Serial.println(" MQTT_CONNECTED"); break;
                  case 1: Serial.println(" MQTT_CONNECT_BAD_PROTOCOL"); break;
                  case 2: Serial.println(" MQTT_CONNECT_BAD_CLIENT_ID"); break;
                  case 3: Serial.println(" MQTT_CONNECT_UNAVAILABLE"); break;
                  case 4: Serial.println(" MQTT_CONNECT_BAD_CREDENTIALS"); break;
                  case 5: Serial.println(" MQTT_CONNECT_UNAUTHORIZED"); break;
            }
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}
//-----Call back Method for Receiving MQTT massage---------//
void callback (char* topic, byte* payload, unsigned int length) {
  static unsigned long lastHandleTime = 0;
  unsigned long currentMillis = millis();
  static unsigned long lastButtonPress = 0;
  // Now handle the message
  String incomingMessage ="";
  for(int i=0; i<length;i++) incomingMessage += (char)payload[i];
  Serial.println("Massage arived ["+String(topic)+"]"+incomingMessage);

  DynamicJsonDocument doc(100);
  DeserializationError error = deserializeJson(doc, incomingMessage);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
  JsonObject obj = doc.as<JsonObject>();
  if(obj.containsKey("out1")){
    boolean p = obj["out1"];
    digitalWrite(RELAY_LIGHT_PIN1,p);
    send_Data_slave1.lightStatetoslave1 = digitalRead(RELAY_LIGHT_PIN1);
    esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *) &send_Data_slave1, sizeof(struct_message_send1));
    Serial.println("out1: "+String(p));
  }
  if(obj.containsKey("out2")){
    boolean p = obj["out2"];
    digitalWrite(RELAY_FAN_PIN1,p);
    send_Data_slave1.fanStatetoslave1 = digitalRead(RELAY_FAN_PIN1);
    esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *) &send_Data_slave1, sizeof(struct_message_send1));
    Serial.println("out2: "+String(p));
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(obj.containsKey("out3")){
    boolean p = obj["out3"];
    digitalWrite(RELAY_LIGHT_PIN2,p);
    send_Data_slave2.lightStatetoslave2 = digitalRead(RELAY_LIGHT_PIN2);
    esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &send_Data_slave2, sizeof(struct_message_send2));
    Serial.println("out3: "+String(p));
  }
  if(obj.containsKey("out4")){
    boolean p = obj["out4"];
    digitalWrite(RELAY_FAN_PIN2,p);
    send_Data_slave2.fanStatetoslave2 = digitalRead(RELAY_FAN_PIN2);
    esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &send_Data_slave2, sizeof(struct_message_send2));
    Serial.println("out4: "+String(p));
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(obj.containsKey("out5")){
    boolean p = obj["out5"];
    digitalWrite(RELAY_LIGHT_PIN3,p);
    send_Data_slave3.lightStatetoslave3 = digitalRead(RELAY_LIGHT_PIN3);
    esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3));
    Serial.println("out5: "+String(p));
  }
  if(obj.containsKey("out6")){
    int p = obj["out6"];
    if (p == 180) {
    send_Data_slave3.ServoAngle = servoOpenAngle;
    esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3));
    } else if (p == 0) {
     send_Data_slave3.ServoAngle = servoClosedAngle;
    esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3));
    }
  }
  if(obj.containsKey("out7")) {
    boolean t = obj["out7"];
    digitalWrite(RELAY_SERVO_PIN3,t);
    Serial.println("out7: "+String(t));
  }
}
void publishMessage(const char* topic, String message, boolean retained){
  if(client.publish(topic,message.c_str(),true))
    Serial.println("Message published ["+String(topic)+"]: "+message);
}
void setupMQTT(){
    espClient.setInsecure();
    client.setServer(mqtt_server,mqtt_port);
    client.setCallback(callback);
    reconnect();
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  // Slave 1
  pinMode(BUTTON_LIGHT_PIN1, INPUT_PULLUP);
  pinMode(RELAY_LIGHT_PIN1,OUTPUT);
  pinMode(BUTTON_FAN_PIN1, INPUT_PULLUP);
  pinMode(RELAY_FAN_PIN1,OUTPUT);
  digitalWrite(RELAY_LIGHT_PIN1,LOW);
  digitalWrite(RELAY_FAN_PIN1,LOW);
  
  
  // Slave 2
  pinMode(BUTTON_LIGHT_PIN2, INPUT_PULLUP);
  pinMode(RELAY_LIGHT_PIN2,OUTPUT);
  pinMode(BUTTON_FAN_PIN2, INPUT_PULLUP);
  pinMode(RELAY_FAN_PIN2,OUTPUT);
  digitalWrite(RELAY_LIGHT_PIN2,LOW);
  digitalWrite(RELAY_FAN_PIN2,LOW);

  // Slave 3
  pinMode(BUTTON_LIGHT_PIN3, INPUT_PULLUP);
  pinMode(RELAY_LIGHT_PIN3,OUTPUT);
  pinMode(BUTTON_SERVO_PIN3, INPUT_PULLUP);
  pinMode(RELAY_SERVO_PIN3,OUTPUT);
  digitalWrite(RELAY_LIGHT_PIN3,LOW);
  digitalWrite(RELAY_SERVO_PIN3,LOW);
  // Initialize WiFi connection
  Serial.println("Initializing WiFi...");
  setup_wifi();
  ////////////////////////////////////////
  // Delay after WiFi setup to ensure stability
  Serial.println("Stabilizing WiFi connection...");
  delay(2000);  // Delay for 2 seconds
  ////////////////////////////////////////////
  // Initialize MQTT
  Serial.println("Setting up MQTT...");
  setupMQTT();
  ////////////////////////////////////////////
  // Delay after MQTT setup to reduce overlap and allow connection establishment
  Serial.println("Stabilizing MQTT connection...");
  delay(2000);  // Delay for 2 seconds
  ////////////////////////////////////////////////////////
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi mode set to AP STA for ESP-NOW");
  WiFi.softAP("A14.13", "0855707879", 6);
  ////////////////////////////////////////////////////////
  // Initialize ESP-NOW
  Serial.println("Initializing ESP-NOW...");
  esp_err_t sendStatus = esp_now_init();
  if (sendStatus == ESP_OK) {
  Serial.println("ESP-NOW Initialized Successfully");
  } else {
  Serial.print("ESP-NOW Initialization Failed: ");
  Serial.println(sendStatus);
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
    // register third peer  
  memcpy(peerInfo.peer_addr, broadcastAddress3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
void publishBoardData(int boardId) {
    if (boardId < 0 || boardId >= 3) return;  // Check for valid board ID

    DynamicJsonDocument sensorDoc(512);
    DynamicJsonDocument actuatorDoc(512);
    DynamicJsonDocument kitchengasDoc(512);
    DynamicJsonDocument kitchenDoc(512);
    char jsonOutput[128];

    // Sensor Data
    sensorDoc["temperature"] = boardsStruct[boardId].temp;
    sensorDoc["humidity"] = boardsStruct[boardId].humidity;
    serializeJson(sensorDoc, jsonOutput);
    publishMessage((String("ESP32/dht11/") + String(boardId + 1)).c_str(), jsonOutput, true);

    // Actuator States
    actuatorDoc["outled"] = boardsStruct[boardId].lightState;
    actuatorDoc["outfan"] = boardsStruct[boardId].fanState;
    serializeJson(actuatorDoc, jsonOutput);
    publishMessage((String("ESP32/outledfan/") + String(boardId + 1)).c_str(), jsonOutput, true);

    // Kitchen Data
    kitchengasDoc["gas"] = mylastReceivedKitchenData.gasValue;
    serializeJson(kitchengasDoc, jsonOutput);
    publishMessage("ESP32/kitchen",jsonOutput,true);


    kitchenDoc["outledkitchen"] = mylastReceivedKitchenData.lightState3;
    kitchenDoc["outServo"] = digitalRead(RELAY_SERVO_PIN3);
    serializeJson(kitchenDoc, jsonOutput);
    publishMessage("ESP32/outkitchen",jsonOutput,true);

}
void loop() {
  // Slave 1
  static bool lastStateLight1 = HIGH;
  static bool lastStateFan1 = HIGH;
  bool currentStateLight1 = digitalRead(BUTTON_LIGHT_PIN1);
  bool currentStateFan1 = digitalRead(BUTTON_FAN_PIN1);
  // Slave 2
  static bool lastStateLight2 = HIGH;
  static bool lastStateFan2 = HIGH;
  bool currentStateLight2 = digitalRead(BUTTON_LIGHT_PIN2);
  bool currentStateFan2 = digitalRead(BUTTON_FAN_PIN2);
  // Slave 3
  static bool lastStateLight3 = HIGH;
  static bool servoOpen = false;
  static bool lastStateServo = HIGH;
  int currentStateServo = digitalRead(BUTTON_SERVO_PIN3);
  bool currentStateLight3 = digitalRead(BUTTON_LIGHT_PIN3);
  
  
  
  if (!client.connected()) {
        reconnect();
  }
  client.loop();
  static unsigned long lastPublishTime = 0;
  static unsigned long lastButtonPress1 = 0;
  static unsigned long lastButtonPress2 = 0;
  static unsigned long lastButtonPress3 = 0;
  if (millis() - lastPublishTime > 5000) {
        lastPublishTime = millis();
        publishBoardData(0);  // For board 1
        publishBoardData(1);  // For board 2
  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(millis()-lastButtonPress1 > 200) {
    if (currentStateLight1 == LOW && lastStateLight1 == HIGH) {
    digitalWrite(RELAY_LIGHT_PIN1, !digitalRead(RELAY_LIGHT_PIN1));
    lastButtonPress1 = millis();
  send_Data_slave1.lightStatetoslave1 = digitalRead(RELAY_LIGHT_PIN1);
  esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *) &send_Data_slave1, sizeof(struct_message_send1));
  }
  lastStateLight1 = currentStateLight1;
  }
  if (millis() - lastButtonPress1 > 200) {
    if (currentStateFan1 == LOW && lastStateFan1 == HIGH) {
      digitalWrite(RELAY_FAN_PIN1, !digitalRead(RELAY_FAN_PIN1));
      lastButtonPress1 = millis();
    send_Data_slave1.fanStatetoslave1 = digitalRead(RELAY_FAN_PIN1);
    esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *) &send_Data_slave1, sizeof(struct_message_send1));
    }
    lastStateFan1 = currentStateFan1;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(millis()-lastButtonPress2 > 200) {
    if (currentStateLight2 == LOW && lastStateLight2 == HIGH) {
    digitalWrite(RELAY_LIGHT_PIN2, !digitalRead(RELAY_LIGHT_PIN2));
    lastButtonPress2 = millis();
  send_Data_slave2.lightStatetoslave2 = digitalRead(RELAY_LIGHT_PIN2);
  esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &send_Data_slave2, sizeof(struct_message_send2)); 
    }
  lastStateLight2 = currentStateLight2;
  }
  if (millis() - lastButtonPress2 > 200) {
    if (currentStateFan2 == LOW && lastStateFan2 == HIGH) {
      digitalWrite(RELAY_FAN_PIN2, !digitalRead(RELAY_FAN_PIN2));
      lastButtonPress2 = millis();
  send_Data_slave2.fanStatetoslave2 = digitalRead(RELAY_FAN_PIN2);
  esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &send_Data_slave2, sizeof(struct_message_send2));  
    }
    lastStateFan2 = currentStateFan2;
  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(millis()-lastButtonPress3 > 200) {
    if (currentStateLight3 == LOW && lastStateLight3 == HIGH) {
    digitalWrite(RELAY_LIGHT_PIN3, !digitalRead(RELAY_LIGHT_PIN3));
    lastButtonPress3 = millis();
  send_Data_slave3.lightStatetoslave3 = digitalRead(RELAY_LIGHT_PIN3);
  esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3)); 
    }
  lastStateLight3 = currentStateLight3;
  }
  if (millis() - lastButtonPress3 > 200) {
    if (currentStateServo == LOW && lastStateServo == HIGH) {
      digitalWrite(RELAY_SERVO_PIN3, !digitalRead(RELAY_SERVO_PIN3));
      servoOpen = !servoOpen;
      if (servoOpen) {
        send_Data_slave3.ServoAngle = servoOpenAngle ;
        esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3));
        Serial.println("Servo opened.");
      } else {
        send_Data_slave3.ServoAngle = servoClosedAngle;
        esp_err_t result3 = esp_now_send(broadcastAddress3, (uint8_t *) &send_Data_slave3, sizeof(struct_message_send3));
        Serial.println("Servo closed.");
      }
      lastButtonPress3 = millis();
    }
    lastStateServo = currentStateServo;
  }
}