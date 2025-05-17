#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownout detector
#include "soc/rtc_cntl_reg.h"  // Disable brownout detector
#include <WiFiClientSecure.h>
#include <WebServer.h>         // For MJPEG streaming

// WiFi credentials
const char* ssid = "leon";
const char* password = "leon1975";

// Server URL
const char* serverUrl = "https://api.robogo.website/api/v1/reports/gallery/images";

// LED Pin (on-board flash LED for ESP32-CAM)
#define LED_FLASH_PIN 4

// Web server for streaming
WebServer server(80);

void startCameraServer2(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void startCameraServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", "<html><body><img src=\"/stream\"/></body></html>");
  });

  server.on("/stream", HTTP_GET, []() {
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (client.connected()) {
      camera_fb_t * fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        continue;
      }

      response = "--frame\r\n";
      response += "Content-Type: image/jpeg\r\n\r\n";
      server.sendContent(response);
      server.client().write(fb->buf, fb->len);
      server.sendContent("\r\n");
      esp_camera_fb_return(fb);

      delay(100); // control frame rate
    }
  });

  server.begin();
  Serial.println("Camera web server started. Stream at http://<ESP_IP>/");
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

  Serial.begin(115200);

  pinMode(LED_FLASH_PIN, OUTPUT);
  digitalWrite(LED_FLASH_PIN, LOW); // Turn it off initially

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1; // No reset pin
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    while(true) { delay(100); }
  }

  Serial.println("Camera initialized!");

  startCameraServer(); // <-- Start streaming server
  startCameraServer2();
}

void loop() {
  server.handleClient(); // Handle incoming web requests

  // Check WiFi, reconnect if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.begin(ssid, password);
    int retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < 10) {
      delay(500);
      Serial.print(".");
      retryCount++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected!");
    } else {
      Serial.println("\nWiFi reconnect failed.");
      delay(5000);
      return;
    }
  }

  // Turn on flash LED
  digitalWrite(LED_FLASH_PIN, HIGH);
  delay(200); // Wait a bit for light to stabilize

  camera_fb_t * fb = esp_camera_fb_get();
  digitalWrite(LED_FLASH_PIN, LOW); // Turn off flash

  if (!fb) {
    Serial.println("Camera capture failed");
    delay(5000);
    return;
  }

  uploadImage(fb);
  esp_camera_fb_return(fb);

  delay(10000); // Wait 10 seconds between uploads
}

void uploadImage(camera_fb_t * fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure(); // 🚨 WARNING: skips certificate validation

  Serial.println("[HTTP] Connecting to server...");
  if (!client.connect("api.robogo.website", 443)) {
    Serial.println("Connection to server failed!");
    return;
  }

  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  String contentType = "multipart/form-data; boundary=" + boundary;

  String bodyStart = "--" + boundary + "\r\n";
  bodyStart += "Content-Disposition: form-data; name=\"image\"; filename=\"photo.jpg\"\r\n";
  bodyStart += "Content-Type: image/jpeg\r\n\r\n";

  String bodyEnd = "\r\n--" + boundary + "--\r\n";

  int totalLength = bodyStart.length() + fb->len + bodyEnd.length();

  client.println("POST /api/v1/reports/gallery/images HTTP/1.1");
  client.println("Host: api.robogo.website");
  client.println("User-Agent: ESP32CAM");
  client.println("Content-Type: " + contentType);
  client.print("Content-Length: ");
  client.println(totalLength);
  client.println("Connection: close");
  client.println();

  client.print(bodyStart);
  client.write(fb->buf, fb->len);
  client.print(bodyEnd);

  Serial.println("[HTTP] Request sent. Awaiting response...");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }

  String response = client.readString();
  Serial.println("Server Response:");
  Serial.println(response);

  client.stop();
}
