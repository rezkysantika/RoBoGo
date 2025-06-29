#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// [LED] Include the library for the built-in RGB LED
#include <Adafruit_NeoPixel.h>

// ====== Configuration ======
// -- UART --
#define RXD2 16
#define TXD2 17
const int UART_BAUD_RATE = 115200;

// -- [LED] LED Indicator --
#define BUILTIN_NEOPIXEL_PIN 48     // Pin for the built-in RGB LED on most ESP32-S3 boards
#define BUILTIN_NEOPIXEL_NUM 1      // There is only one built-in LED
const long DATA_TIMEOUT_MS = 2000;  // 2 seconds. If no data is received in this time, blink red.
const long BLINK_INTERVAL_MS = 500; // Blink the LED every 500ms

// ====== Struct Definition ======
// IMPORTANT: This struct MUST exactly match the sender's SensorData struct!
typedef struct SensorData {
  uint32_t timestamp;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float pitch, roll, yaw;
  float heading;
  char direction[16];
  float positionX, positionY;
  float velocity, velocityX, velocityY;
  float distanceTraveled;
  float distX, distY;
  float rotationRate;
  bool obstacleDetected;
  float accelerationMagnitude;
  float linearAcceleration;
  float ultrasonic_distance;
} SensorData;

// ====== Global Variables ======
SensorData incomingSensorData;
int lastRSSI = -128; // Default worst value

// -- [LED] Global variables for LED status management --
Adafruit_NeoPixel builtin_neopixel(BUILTIN_NEOPIXEL_NUM, BUILTIN_NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastDataRecvTime = 0;
unsigned long lastBlinkTime = 0;
bool ledIsOn = false;

// ====== Function Prototypes ======
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void sendSensorDataAsJSON(const SensorData& data, int rssi, const String& macAddress);
float estimateDistanceFromRSSI(int rssi, int txPower = -40, float pathLossExponent = 2.0);
float safeFloat(float val);
float clampFloat(float val, float minVal, float maxVal);
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type);
void updateLedIndicator(); // [LED] New function to handle LED logic

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RXD2, TXD2);

  // [LED] Initialize the built-in NeoPixel LED
  builtin_neopixel.begin();
  builtin_neopixel.setBrightness(20); // Set a dim brightness (0-255)
  builtin_neopixel.clear();           // Ensure the LED is off at startup
  builtin_neopixel.show();

  Serial.println("\n[INFO] ESP-NOW Receiver with LED Status");

  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    // [LED] Blink RED rapidly on fatal error
    builtin_neopixel.setPixelColor(0, 255, 0, 0); // Red
    builtin_neopixel.show();
    while (true) { delay(100); }
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Initialize promiscuous mode for RSSI sniffing
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);

  Serial.println("[INFO] ESP-NOW Initialized. Waiting for data...");
  lastDataRecvTime = millis(); // [LED] Initialize the timer
}

// ====== Main Loop ======
void loop() {
  // [LED] The main loop now only needs to call the LED handler.
  // This approach is non-blocking and responsive.
  updateLedIndicator();
  delay(10); // A small delay to prevent the loop from running too fast
}

// ====== ESP-NOW Receive Callback ======
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // [LED] A packet has been received, so update the timestamp.
  lastDataRecvTime = millis();

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  String macString = String(macStr);

  if (len == sizeof(SensorData)) {
    memcpy(&incomingSensorData, incomingData, sizeof(SensorData));
    // Forward the data as JSON over UART
    sendSensorDataAsJSON(incomingSensorData, lastRSSI, macString);
  } else {
    Serial.printf("[WARN] Unexpected data size: %d (expected %d) from MAC: %s\n", len, sizeof(SensorData), macStr);
  }
}

// ====== [LED] LED Indicator Logic ======
void updateLedIndicator() {
  // This function checks two things based on millis():
  // 1. Has the data reception timed out?
  // 2. Is it time to toggle the blink state?

  if (millis() - lastBlinkTime > BLINK_INTERVAL_MS) {
    lastBlinkTime = millis();
    ledIsOn = !ledIsOn; // Toggle the blink state

    if (ledIsOn) {
      // Check if data reception has timed out
      if (millis() - lastDataRecvTime > DATA_TIMEOUT_MS) {
        // NO DATA: Connection is considered lost. Blink RED.
        builtin_neopixel.setPixelColor(0, 255, 0, 0); // (R, G, B) -> Red
      } else {
        // DATA OK: Data is being received. Blink GREEN.
        builtin_neopixel.setPixelColor(0, 0, 255, 0); // (R, G, B) -> Green
      }
    } else {
      // Turn the LED off for the "blink" effect
      builtin_neopixel.clear();
    }
    // Send the updated color/state to the LED hardware
    builtin_neopixel.show();
  }
}

// ====== Promiscuous callback for RSSI ======
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_CTRL) return;
  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  lastRSSI = ppkt->rx_ctrl.rssi;
}

// ====== Helper and JSON Functions ======
float estimateDistanceFromRSSI(int rssi, int txPower, float pathLossExponent) {
  if (rssi >= -45) return 0.5;
  else if (rssi >= -50) return 1.0;
  else if (rssi >= -55) return 1.5;
  else if (rssi >= -60) return 2.0;
  else if (rssi >= -65) return 2.5;
  else if (rssi >= -70) return 3.5;
  else if (rssi >= -75) return 5.0;
  else if (rssi >= -80) return 7.0;
  else if (rssi >= -85) return 10.0;
  else return 12.0;
}

float safeFloat(float val) {
  return (isnan(val) || isinf(val)) ? 0.0f : val;
}

float clampFloat(float val, float minVal, float maxVal) {
  if (isnan(val) || isinf(val)) return 0.0f;
  return constrain(val, minVal, maxVal);
}

void sendSensorDataAsJSON(const SensorData& data, int rssi, const String& macAddress) {
  StaticJsonDocument<1536> doc;

  doc["timestamp"] = data.timestamp;
  doc["senderMac"] = macAddress;
  doc["obstacle"] = data.obstacleDetected;
  doc["ultrasonic"] = safeFloat(data.ultrasonic_distance);
  doc["heading"] = safeFloat(data.heading);
  doc["direction"] = data.direction;
  doc["accelerationMagnitude"] = safeFloat(data.accelerationMagnitude);
  doc["linearAcceleration"] = safeFloat(data.linearAcceleration);
  doc["accelX"] = safeFloat(data.accel_x);
  doc["accelY"] = safeFloat(data.accel_y);
  doc["accelZ"] = safeFloat(data.accel_z);
  doc["gyroX"] = safeFloat(data.gyro_x);
  doc["gyroY"] = safeFloat(data.gyro_y);
  doc["gyroZ"] = safeFloat(data.gyro_z);
  doc["pitch"] = safeFloat(data.pitch);
  doc["roll"] = safeFloat(data.roll);
  doc["yaw"] = safeFloat(data.yaw);
  doc["rotationRate"] = safeFloat(data.rotationRate);
  doc["distanceTraveled"] = safeFloat(data.distanceTraveled);
  doc["velocity"] = clampFloat(data.velocity, 0.0f, 1000.0f);
  doc["velocityX"] = clampFloat(data.velocityX, -1000.0f, 1000.0f);
  doc["velocityY"] = clampFloat(data.velocityY, -1000.0f, 1000.0f);
  doc["magX"] = safeFloat(data.mag_x);
  doc["magY"] = safeFloat(data.mag_y);
  doc["magZ"] = safeFloat(data.mag_z);
  doc["positionX"] = clampFloat(data.positionX, -100000.0f, 100000.0f);
  doc["positionY"] = clampFloat(data.positionY, -100000.0f, 100000.0f);
  doc["distX"] = clampFloat(data.distX, -1000.0f, 1000.0f);
  doc["distY"] = clampFloat(data.distY, -1000.0f, 1000.0f);
  doc["rssi"] = rssi;
  doc["rssiDistance"] = estimateDistanceFromRSSI(rssi);

  String output;
  if (serializeJson(doc, output) > 0) {
    Serial.println(output);  // Print JSON to main debug serial
    Serial2.println(output); // Send JSON over UART2
    Serial2.flush();
  } else {
    Serial.println("[ERROR] JSON serialization failed!");
  }
}
