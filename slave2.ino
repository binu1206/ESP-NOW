#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <DHT.h>
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
//Button setup
#define RELAY_LIGHT_PIN 14
#define RELAY_FAN_PIN 27
#define BUTTON_LIGHT_PIN 12
#define BUTTON_FAN_PIN 13
#define TEMP_THRESHOLD 33  // Temperature threshold for fan activation
// DHT Setup
#define DHTPIN 4  //  GPIO4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


//Structure example to send data
//Must match the receiver structure
typedef struct struct_message_send2 {
    int id;
    float temp2;
    float humidity2;
     bool lightState2;
    bool fanState2;
} struct_message_send2;

struct_message_send2 lastReceivedDhtData2;


typedef struct struct_message_coming {
  bool lightStatefromMaster;
  bool fanStatefromMaster;
} struct_message_coming;

struct_message_coming incomingmessage;

bool LED_State_Receive;
bool FAN_State_Receive;

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
  digitalWrite(RELAY_LIGHT_PIN, LED_State_Receive);
  FAN_State_Receive = incomingmessage.fanStatefromMaster;
  Serial.print("Fan State: ");
  Serial.println(FAN_State_Receive);
  digitalWrite(RELAY_FAN_PIN, FAN_State_Receive);
}


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  dht.begin();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);


  pinMode(RELAY_LIGHT_PIN, OUTPUT);
  pinMode(RELAY_FAN_PIN, OUTPUT);
  pinMode(BUTTON_LIGHT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_FAN_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_LIGHT_PIN, LOW);
  digitalWrite(RELAY_FAN_PIN, LOW);
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
// Global variables for fan control
bool fanAutoOn = false;
bool fanManualOverride = false;  
void loop() {
  lastReceivedDhtData2.id =2;
  static bool lastStateLight = HIGH;
  static bool lastStateFan = HIGH;
  static unsigned long lastPrintTime = 0;
  static unsigned long lastSendTime = 0;
  static unsigned long lastButtonPress = 0;


  bool currentStateLight = digitalRead(BUTTON_LIGHT_PIN);
  bool currentStateFan = digitalRead(BUTTON_FAN_PIN);
  unsigned long currentMillis = millis();

  if (currentMillis - lastSendTime >= 3000) {
    lastSendTime = currentMillis;
      // Debounce check for light button
    lastReceivedDhtData2.lightState2 = digitalRead(RELAY_LIGHT_PIN);
    lastReceivedDhtData2.fanState2 = digitalRead(RELAY_FAN_PIN);
    Serial.println("Sending data...");
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &lastReceivedDhtData2, sizeof(lastReceivedDhtData2));
  }
/////////////////////////////////////////////////////////////////////////////////////////////  
  if (millis()- lastButtonPress > 200) {
      if (currentStateLight == LOW && lastStateLight == HIGH) {
      digitalWrite(RELAY_LIGHT_PIN, !digitalRead(RELAY_LIGHT_PIN));
      lastButtonPress = millis();
    }  
  lastStateLight = currentStateLight;
  }

    
  if (millis() - lastButtonPress > 200) {
    if (currentStateFan == LOW && lastStateFan == HIGH) {
      digitalWrite(RELAY_FAN_PIN, !digitalRead(RELAY_FAN_PIN));
      lastButtonPress = millis();
    }
    lastStateFan = currentStateFan;
  }
//////////////////////////////////////////////////////////////////////////////////////////////////


  // Debounce check for fan button
  if (currentStateFan == LOW && lastStateFan == HIGH) {
    fanManualOverride = !fanManualOverride;
    digitalWrite(RELAY_FAN_PIN, fanManualOverride); // Toggle fan state on button press
    delay(200); // Debounce delay
  }
  lastStateFan = currentStateFan;
  // Temperature control for fan
  if (!isnan(lastReceivedDhtData2.temp2)) {
    if (lastReceivedDhtData2.temp2 > TEMP_THRESHOLD && !fanManualOverride) {
      digitalWrite(RELAY_FAN_PIN, HIGH);
      fanAutoOn = true;
    } else if (lastReceivedDhtData2.temp2 <= TEMP_THRESHOLD && fanAutoOn && !fanManualOverride) {
      digitalWrite(RELAY_FAN_PIN, LOW);
      fanAutoOn = false;
    }
  }
  // Read and print temperature and humidity at defined intervals
  if (currentMillis - lastPrintTime >= 5000) {
    lastPrintTime = currentMillis;
    lastReceivedDhtData2.temp2 = dht.readTemperature();  // Read temperature in Celsius
    lastReceivedDhtData2.humidity2 = dht.readHumidity(); // Read humidity

    if (isnan(lastReceivedDhtData2.temp2) || isnan(lastReceivedDhtData2.humidity2)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(lastReceivedDhtData2.temp2);
      Serial.print(" °C, Humidity: ");
      Serial.print(lastReceivedDhtData2.humidity2);
      Serial.println(" %");
       // Update struct to hold new data
      // lastReceivedDhtData2.lightState2 = digitalRead(RELAY_LIGHT_PIN);
      // lastReceivedDhtData2.fanState2 = digitalRead(RELAY_FAN_PIN);
      Serial.print("Light State: ");
      Serial.print(lastReceivedDhtData2.lightState2);
      Serial.print(", Fan State: ");
      Serial.println(lastReceivedDhtData2.fanState2);
    }
  }
}
