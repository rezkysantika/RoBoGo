#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "leon";
const char* password = "leon1975";

// API endpoint 
// const char* serverName = "https://api.robogo.website/api/v1/reports/gallery/images";
const char* serverName = "https://api.robogo.website/api/v1/reports/gallery/images/metadata";

const uint32_t JSON_BUFFER_SIZE = 1024;
const int HTTP_RETRY_COUNT = 3;
const int UART_BAUD_RATE = 115200;
const uint8_t RX_PIN = 16;
const uint8_t TX_PIN = 17;

void setup() {
  Serial.begin(115200); // Debug
  Serial2.begin(UART_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  connectToWiFi();
}

void loop() {
  static String inputBuffer;

  while (Serial2.available()) {
    char ch = Serial2.read();
    if (ch == '\n') {
      inputBuffer.trim();
      Serial.println("[DEBUG] Full line received:");
      Serial.println(inputBuffer);

      if (inputBuffer.length() > 0) {
        StaticJsonDocument<JSON_BUFFER_SIZE> doc;
        DeserializationError error = deserializeJson(doc, inputBuffer);
        if (!error) {
          sendToApi(doc);
        } else {
          Serial.print("[ERROR] JSON Deserialization failed: ");
          Serial.println(error.c_str());
        }
      }
      inputBuffer = ""; // Clear buffer for next line
    } else {
      inputBuffer += ch;
      // Prevent overflow
      if (inputBuffer.length() > JSON_BUFFER_SIZE) {
        Serial.println("[ERROR] Input too large, discarding.");
        inputBuffer = "";
      }
    }
  }

  // Optional: reconnect Wi-Fi if lost
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WARN] WiFi disconnected. Reconnecting...");
    connectToWiFi();
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n❌ WiFi connection failed.");
  }
}

void sendToApi(const JsonDocument& sensorDoc) {
  StaticJsonDocument<2048> fullDoc;

  // Dummy fields required by the API
  // fullDoc["filename"] = "dummy.jpg";
  // fullDoc["path"] = "images/dummy.jpg";
  // fullDoc["imageUrl"] = "https://example.com/images/dummy.jpg";
  // fullDoc["timestamp"] = "2025-05-04T07:14:42.317Z";
  fullDoc["sessionId"] = 1;
  fullDoc["takenWith"] = "ESP32-CAM";
  // fullDoc["createdAt"] = "2025-05-04T07:14:42.319Z";

  // Directly add the sensor data to the top level (no "metadata" field)
  fullDoc["obstacle"] = sensorDoc["obstacle"];
  fullDoc["ultrasonic"] = sensorDoc["ultrasonic"];
  fullDoc["heading"] = sensorDoc["heading"];
  fullDoc["direction"] = sensorDoc["direction"];
  fullDoc["accelerationMagnitude"] = sensorDoc["accelerationMagnitude"];
  fullDoc["rotationRate"] = sensorDoc["rotationRate"];
  fullDoc["distanceTraveled"] = sensorDoc["distanceTraveled"];
  fullDoc["linearAcceleration"] = sensorDoc["linearAcceleration"];
  fullDoc["pitch"] = sensorDoc["pitch"];
  fullDoc["roll"] = sensorDoc["roll"];
  fullDoc["yaw"] = sensorDoc["yaw"];
  fullDoc["distTotal"] = sensorDoc["distTotal"];
  fullDoc["distX"] = sensorDoc["distX"];
  fullDoc["distY"] = sensorDoc["distY"];
  fullDoc["velocity"] = sensorDoc["velocity"];
  fullDoc["velocityX"] = sensorDoc["velocityX"];
  fullDoc["velocityY"] = sensorDoc["velocityY"];
  fullDoc["magnetometerX"] = sensorDoc["magnetometerX"];
  fullDoc["magnetometerY"] = sensorDoc["magnetometerY"];
  fullDoc["magnetometerZ"] = sensorDoc["magnetometerZ"];
  fullDoc["positionX"] = sensorDoc["positionX"];
  fullDoc["positionY"] = sensorDoc["positionY"];

  // Copy top-level status fields
  fullDoc["status"] = "success";
  fullDoc["code"] = 200;
  fullDoc["message"] = "Dummy image + sensor metadata";

  // Serialize to string
  String payload;
  serializeJson(fullDoc, payload);

  Serial.println("[DEBUG] Sending full JSON:");
  Serial.println(payload);

  // Send to API
  for (int attempt = 1; attempt <= HTTP_RETRY_COUNT; attempt++) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[ERROR] WiFi not connected.");
      return;
    }

    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      Serial.printf("[INFO] POST success (code %d)\n", httpResponseCode);
      Serial.println("[INFO] API response:");
      Serial.println(http.getString());
      http.end();
      return;
    } else {
      Serial.printf("[WARN] POST attempt %d failed: %s\n", attempt, http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    delay(1000);
  }

  Serial.println("[ERROR] All POST attempts failed.");
}

