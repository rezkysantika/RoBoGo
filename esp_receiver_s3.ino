#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <base64.h>

// ====== UART Configuration ======
#define RXD2 16
#define TXD2 17

// ====== Struct Definition ======
typedef struct SensorData {
  uint32_t timestamp;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float heading;
  char direction[16];
  float distance;
  float locationStartX;
  float locationStartY;
  float positionX;
  float positionY;
  float distX;
  float distY;
  float velocity;
  float velocityX;
  float velocityY;
  float distanceTraveled;
  float rotationRate;
  bool obstacleDetected;
  float accelerationMagnitude;
  float linearAcceleration;
  float pitch;
  float roll;
  float yaw;
} SensorData;

// ====== Global Variables ======
SensorData incomingSensorData;

// ====== Function Prototypes ======
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void printSensorData(const SensorData& data);
void sendSensorDataAsJSON(const SensorData& data);
float safeFloat(float val);
float clampFloat(float val, float minVal, float maxVal) {
  if (isnan(val) || isinf(val)) return 0.0f;
  return constrain(val, minVal, maxVal);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // UART2 for forwarding

  Serial.println("\n[INFO] ESP-NOW Receiver + UART Forwarder Starting...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW initialization failed!");
    while (true);
  }

  Serial.println("[INFO] ESP-NOW Initialized.");
  esp_now_register_recv_cb(OnDataRecv);
}

// ====== Main Loop ======
void loop() {
  delay(10); // idle
}

// ====== ESP-NOW Receive Callback ======
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("\n[INFO] Data received from MAC: %s | Size: %d bytes\n", macStr, len);

  if (len == sizeof(SensorData)) {
    memcpy(&incomingSensorData, incomingData, sizeof(SensorData));
    printSensorData(incomingSensorData);
    sendSensorDataAsJSON(incomingSensorData);
  } else {
    Serial.printf("[WARN] Data size mismatch: expected %d bytes, got %d bytes\n", sizeof(SensorData), len);
  }
}

// ====== Print the received Sensor Data ======
void printSensorData(const SensorData& data) {
  Serial.println("\n====== RECEIVED SENSOR DATA ======");
  Serial.printf("Timestamp: %lu ms\n", data.timestamp);
  Serial.println("-- Acceleration (m/s^2) --");
  Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\n", data.accel_x, data.accel_y, data.accel_z);
  Serial.println("-- Gyroscope (deg/s) --");
  Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\n", data.gyro_x, data.gyro_y, data.gyro_z);
  Serial.println("-- Magnetometer (uT) --");
  Serial.printf("X: %.2f\tY: %.2f\tZ: %.2f\n", data.mag_x, data.mag_y, data.mag_z);
  Serial.println("-- Orientation --");
  Serial.printf("Heading: %.2f° (%s)\n", data.heading, data.direction);
  Serial.printf("Pitch: %.2f°\tRoll: %.2f°\tYaw: %.2f°\n", data.pitch, data.roll, data.yaw);
  Serial.println("-- Movement --");
  Serial.printf("Distance: %.2f cm\n", data.distance);
  Serial.printf("Start Position: (%.2f, %.2f) cm\n", data.locationStartX, data.locationStartY);
  Serial.printf("Current Position: (%.2f, %.2f) cm\n", data.positionX, data.positionY);
  Serial.printf("Relative Movement: (%.2f, %.2f) cm\n", data.distX, data.distY);
  Serial.printf("Velocity: %.2f cm/s | Vx: %.2f | Vy: %.2f\n", data.velocity, data.velocityX, data.velocityY);
  Serial.printf("Distance Traveled: %.2f cm\n", data.distanceTraveled);
  Serial.println("-- Other --");
  Serial.printf("Rotation Rate: %.2f deg/s\n", data.rotationRate);
  Serial.printf("Acceleration Magnitude: %.2f m/s^2\n", data.accelerationMagnitude);
  Serial.printf("Linear Acceleration: %.2f m/s^2\n", data.linearAcceleration);
  Serial.printf("Obstacle Detected: %s\n", data.obstacleDetected ? "YES" : "NO");
  Serial.println("===================================");
}

// ====== Safe Float Helper ======
float safeFloat(float val) {
  return (isnan(val) || isinf(val)) ? 0.0f : val;
}

// ====== Convert to JSON and Send Over UART (FLAT STRUCTURE) ======
void sendSensorDataAsJSON(const SensorData& data) {
  StaticJsonDocument<1536> doc;

  // Copy values directly to the JSON
  doc["obstacle"] = data.obstacleDetected ? "true" : "false";  // Convert boolean to string
  doc["ultrasonic"] = safeFloat(data.distance);
  doc["heading"] = safeFloat(data.heading);
  doc["direction"] = data.direction;
  doc["accelerationMagnitude"] = safeFloat(data.accelerationMagnitude);
  doc["rotationRate"] = safeFloat(data.rotationRate);
  doc["distanceTraveled"] = safeFloat(data.distanceTraveled);
  doc["linearAcceleration"] = safeFloat(data.linearAcceleration);
  doc["pitch"] = safeFloat(data.pitch);
  doc["roll"] = safeFloat(data.roll);
  doc["yaw"] = safeFloat(data.yaw);
  doc["distTotal"] = clampFloat(data.distanceTraveled, 0.0f, 10000.0f);
  doc["distX"] = clampFloat(data.distX, -1000.0f, 1000.0f);
  doc["distY"] = clampFloat(data.distY, -1000.0f, 1000.0f);
  doc["velocity"] = clampFloat(data.velocity, 0.0f, 1000.0f);
  doc["velocityX"] = clampFloat(data.velocityX, -1000.0f, 1000.0f);
  doc["velocityY"] = clampFloat(data.velocityY, -1000.0f, 1000.0f);
  doc["magnetometerX"] = safeFloat(data.mag_x);
  doc["magnetometerY"] = safeFloat(data.mag_y);
  doc["magnetometerZ"] = safeFloat(data.mag_z);
  doc["positionX"] = clampFloat(data.positionX, -1000.0f, 1000.0f);
  doc["positionY"] = clampFloat(data.positionY, -1000.0f, 1000.0f);

  String output;
  if (serializeJson(doc, output) > 0) {
    Serial2.println(output);  // Send to UART for third ESP32
    Serial2.flush();
    Serial.println("[DEBUG] Sent JSON to UART:");
    Serial.println(output);
  } else {
    Serial.println("[ERROR] Failed to serialize JSON.");
  }
}
