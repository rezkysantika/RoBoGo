// 🎥 ESP32-CAM Stable WebSocket Streaming
// Fokus: Stabilitas streaming WebSocket + Auto device registration

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SD_MMC.h>

const char* ssid = "leon";
const char* password = "leon1975";

// � API Configuration for Device Registration
const char* apiHost = "api.robogo.website";
const int apiPort = 443;
const char* deviceRegisterPath = "/api/v1/devices";
const char* deviceName = "esp32-48BB88";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);  

// 🎥 Real-time Streaming Configuration
bool streamingActive = false;
unsigned long streamDelay = 100; // 100ms delay = ~10 FPS
unsigned long lastStreamTime = 0;

//  Offline Photo Capture Configuration
unsigned long wifiCheckInterval = 10000; // Check WiFi every 10 seconds - balance antara responsif dan hemat daya
unsigned long lastWifiCheck = 0;
unsigned long wifiTimeoutDuration = 10000; // 10 seconds timeout for WiFi connection
unsigned long offlineCaptureInterval = 5000; // Capture photo every 5 seconds in offline mode
unsigned long lastOfflineCapture = 0;
bool sdCardInitialized = false;
int offlinePhotoCount = 0;
bool offlineModeActive = false;

// � Streaming Statistics
unsigned long totalFramesSent = 0;
unsigned long streamingStartTime = 0;
bool deviceRegistered = false;


// � Device Registration to API
void registerDevice() {
  if (WiFi.status() != WL_CONNECTED || deviceRegistered) {
    return;
  }

  Serial.println("📡 Registering device to API...");
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10);

  if (!client.connect(apiHost, apiPort)) {
    Serial.println("❌ Failed to connect to API server for registration");
    return;
  }

  // Prepare JSON payload
  String websocketUrl = "ws://" + WiFi.localIP().toString() + ":81/";
  String jsonPayload = "{";
  jsonPayload += "\"deviceName\":\"" + String(deviceName) + "\",";
  jsonPayload += "\"camera\":\"ON\",";
  jsonPayload += "\"cameraStreamUrl\":\"" + websocketUrl + "\"";
  jsonPayload += "}";

  // Send HTTP POST request
  client.print("POST " + String(deviceRegisterPath) + " HTTP/1.1\r\n");
  client.print("Host: " + String(apiHost) + "\r\n");
  client.print("Content-Type: application/json\r\n");
  client.print("Content-Length: " + String(jsonPayload.length()) + "\r\n");
  client.print("Connection: close\r\n\r\n");
  client.print(jsonPayload);

  // Read response
  unsigned long timeout = millis() + 5000;
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line.startsWith("HTTP/")) {
        Serial.println("📡 Registration response: " + line);
        if (line.indexOf("200") > 0 || line.indexOf("201") > 0) {
          deviceRegistered = true;
          Serial.println("✅ Device registered successfully!");
          Serial.println("📋 Device: " + String(deviceName));
          Serial.println("📋 Stream URL: " + websocketUrl);
        }
        break;
      }
    }
    delay(10);
  }

  client.stop();
  
  if (!deviceRegistered) {
    Serial.println("⚠️ Device registration failed, will retry later");
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("🔌 WebSocket [%u] Disconnected!\n", num);
      if (webSocket.connectedClients() == 0) {
        streamingActive = false;
        Serial.println("📹 Streaming stopped - no clients");
      }
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("✅ WebSocket [%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        // Start streaming when first client connects
      if (!streamingActive) {
        streamingActive = true;
        Serial.println("📹 Streaming started");
      }
      
      // Send welcome message
      String welcomeMsg = "ESP32-CAM Real-time Stream - Device: " + String(WiFi.getHostname());
      webSocket.sendTXT(num, welcomeMsg);
      break;
    }
    
    case WStype_TEXT:
      Serial.printf("💬 WebSocket [%u] Command: %s\n", num, payload);
        if (strcmp((char*)payload, "start") == 0) {
        streamingActive = true;
        Serial.println("📹 Streaming started via command");
      } else if (strcmp((char*)payload, "stop") == 0) {
        streamingActive = false;
        Serial.println("⏹️ Streaming stopped via command");
      } else if (strcmp((char*)payload, "fast") == 0) {
        streamDelay = 50; // ~20 FPS
        Serial.println("🚀 Fast streaming: ~20 FPS");
      } else if (strcmp((char*)payload, "normal") == 0) {
        streamDelay = 100; // ~10 FPS
        Serial.println("⚡ Normal streaming: ~10 FPS");
      } else if (strcmp((char*)payload, "slow") == 0) {
        streamDelay = 200; // ~5 FPS
        Serial.println("🐌 Slow streaming: ~5 FPS");
      } else if (strcmp((char*)payload, "stats") == 0) {
        String stats = "📊 Streaming Stats: Frames:" + String(totalFramesSent) +
                      " Clients:" + String(webSocket.connectedClients()) + 
                      " Registered:" + String(deviceRegistered ? "YES" : "NO");
        webSocket.sendTXT(num, stats);
        Serial.println(stats);
      }
      break;
      
    default:
      break;
  }
}

void streamFrame() {
  if (!streamingActive || webSocket.connectedClients() == 0) {
    return;
  }
  
  // Check if enough time has passed for next frame
  if (millis() - lastStreamTime < streamDelay) {
    return;
  }
  
  lastStreamTime = millis();
  
  // Capture frame
  camera_fb_t * fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("❌ Frame capture failed");
    return;
  }
    // Send compressed frame to all connected clients
  webSocket.broadcastBIN(fb->buf, fb->len);
  
  // Update statistics
  totalFramesSent++;
  
  // Register device if not already registered
  if (!deviceRegistered && totalFramesSent % 50 == 0) { // Try every 50 frames
    registerDevice();
  }
  
  // Optional: Print compression info (every 100 frames to reduce spam)
  if (totalFramesSent % 100 == 0) {
    Serial.printf("📡 Frame #%lu sent: %d bytes (%dx%d) to %d clients\n", 
                  totalFramesSent, fb->len, fb->width, fb->height, webSocket.connectedClients());
  }
  
  esp_camera_fb_return(fb);
}

void startRealtimeServer() {  // Root endpoint with streaming information
  server.on("/", HTTP_GET, []() {
    String info = "🎥 ESP32-CAM Stable WebSocket Streaming\n";
    info += "Device: " + String(deviceName) + "\n";
    info += "WebSocket URL: ws://" + WiFi.localIP().toString() + ":81/\n";
    info += "Connected Clients: " + String(webSocket.connectedClients()) + "\n";
    info += "Streaming: " + String(streamingActive ? "ACTIVE" : "INACTIVE") + "\n";
    info += "Frame Rate: ~" + String(1000/streamDelay) + " FPS\n";
    info += "Resolution: VGA (640x480) - Compressed\n";
    info += "Device Registered: " + String(deviceRegistered ? "YES" : "NO") + "\n";
    info += "Total Frames Sent: " + String(totalFramesSent) + "\n";
    info += "SD Card: " + String(sdCardInitialized ? "READY" : "NOT AVAILABLE") + "\n";
    info += "Offline Photos: " + String(offlinePhotoCount) + "\n";
    info += "Offline Mode: " + String(offlineModeActive ? "ACTIVE" : "INACTIVE") + "\n";
    info += "\nWebSocket Commands:\n";
    info += "- 'start' = Start streaming\n";
    info += "- 'stop' = Stop streaming\n";
    info += "- 'fast' = ~20 FPS\n";
    info += "- 'normal' = ~10 FPS\n";
    info += "- 'slow' = ~5 FPS\n";
    info += "- 'stats' = Show streaming statistics\n";    server.send(200, "text/plain", info);
  });
  
  // Info endpoint with CORS
  server.on("/info", HTTP_GET, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    
    String json = "{";
    json += "\"websocket_url\":\"ws://" + WiFi.localIP().toString() + ":81/\",";
    json += "\"status\":\"online\",";
    json += "\"connected_clients\":" + String(webSocket.connectedClients()) + ",";
    json += "\"device\":\"ESP32-CAM\",";
    json += "\"device_name\":\"" + String(deviceName) + "\",";
    json += "\"mode\":\"stable_websocket_streaming\",";
    json += "\"streaming\":" + String(streamingActive ? "true" : "false") + ",";
    json += "\"fps\":" + String(1000/streamDelay) + ",";
    json += "\"resolution\":\"VGA\",";
    json += "\"compression\":\"JPEG_Q8\",";
    json += "\"device_registered\":" + String(deviceRegistered ? "true" : "false") + ",";
    json += "\"total_frames_sent\":" + String(totalFramesSent) + ",";
    json += "\"sd_card_ready\":" + String(sdCardInitialized ? "true" : "false") + ",";
    json += "\"offline_photos\":" + String(offlinePhotoCount) + ",";
    json += "\"offline_mode\":" + String(offlineModeActive ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
  });
  // Control endpoints
  server.on("/start", HTTP_GET, []() {
    streamingActive = true;
    server.send(200, "text/plain", "📹 Real-time streaming STARTED");
  });

  server.on("/stop", HTTP_GET, []() {
    streamingActive = false;
    server.send(200, "text/plain", "⏹️ Real-time streaming STOPPED");
  });

  // Speed control endpoints
  server.on("/fast", HTTP_GET, []() {
    streamDelay = 50;
    server.send(200, "text/plain", "🚀 Fast mode: ~20 FPS");
  });

  server.on("/normal", HTTP_GET, []() {
    streamDelay = 100;
    server.send(200, "text/plain", "⚡ Normal mode: ~10 FPS");
  });

  server.on("/slow", HTTP_GET, []() {
    streamDelay = 200;
    server.send(200, "text/plain", "🐌 Slow mode: ~5 FPS");
  });

  // Manual photo capture endpoint
  server.on("/capture", HTTP_GET, []() {
    if (sdCardInitialized) {
      if (capturePhotoToSD()) {
        server.send(200, "text/plain", "📷 Photo captured and saved to SD card!");
      } else {
        server.send(500, "text/plain", "❌ Failed to capture photo");
      }
    } else {
      server.send(500, "text/plain", "❌ SD Card not available");
    }
  });

  // SD card status endpoint
  server.on("/sdcard", HTTP_GET, []() {
    String status = "💾 SD Card Status:\n";
    status += "Initialized: " + String(sdCardInitialized ? "YES" : "NO") + "\n";
    status += "Offline Photos: " + String(offlinePhotoCount) + "\n";
    status += "Offline Mode: " + String(offlineModeActive ? "ACTIVE" : "INACTIVE") + "\n";
    
    if (sdCardInitialized) {
      uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
      uint64_t usedBytes = SD_MMC.usedBytes() / (1024 * 1024);
      status += "Card Size: " + String((unsigned long)cardSize) + " MB\n";
      status += "Used Space: " + String((unsigned long)usedBytes) + " MB\n";
    }
    
    server.send(200, "text/plain", status);
  });
  server.begin();
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("🎥 === ESP32-CAM Stable WebSocket Streaming ===");
  Serial.println("Device Name: " + String(deviceName));
  Serial.print("📡 WebSocket URL: ws://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/");
  Serial.print("🌐 Control panel: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Serial.println("🚀 Mode: Stable WebSocket Streaming");
  Serial.println("📹 Frame Rate: ~" + String(1000/streamDelay) + " FPS");
  Serial.println("📺 Stream Resolution: VGA (640x480) - High Compression");
  Serial.println("✅ Camera ready! Registering device to API...");
    // Register device to API
  registerDevice();
}

void initCamera() {
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
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;  config.frame_size   = FRAMESIZE_VGA; // VGA (640x480) untuk kualitas lebih baik
  config.jpeg_quality = 8; // Quality 8 untuk kompresi lebih tinggi (was 12)
  config.fb_count     = 2; // Double buffer untuk smooth streaming

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed with error 0x%x", err);
    while (true) delay(100);
  }
  
  // Camera settings optimized untuk VGA
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 2);     // Brightness: -2 to 2 (0 = normal)
    s->set_contrast(s, 1);       // Contrast: -2 to 2 (sedikit tinggi untuk VGA)
    s->set_saturation(s, 0);     // Saturation: -2 to 2
    s->set_special_effect(s, 0); // Special effects: 0-6 (normal)
    s->set_whitebal(s, 1);       // White balance: 0-1 (enable)
    s->set_awb_gain(s, 1);       // AWB gain: 0-1 (enable)
    s->set_wb_mode(s, 1);        // WB mode: 0-4 (auto)
    s->set_exposure_ctrl(s, 1);  // Exposure control: 0-1 (enable)
    s->set_aec2(s, 0);           // AEC2: 0-1
    s->set_ae_level(s, 0);      // AE level: -2 to 2
    s->set_aec_value(s, 800);    // AEC value: 0-1200
    s->set_gain_ctrl(s, 1);      // Gain control: 0-1 (enable)
    s->set_agc_gain(s, 5);       // AGC gain: 0-30
    s->set_gainceiling(s, (gainceiling_t)2); // Gain ceiling: 0-6 (low)
    s->set_bpc(s, 0);            // BPC: 0-1 (disable)
    s->set_wpc(s, 1);            // WPC: 0-1 (enable)
    s->set_raw_gma(s, 1);        // Gamma: 0-1 (enable)
    s->set_lenc(s, 0);           // Lens correction: 0-1 (enable)
    s->set_hmirror(s, 0);        // Horizontal mirror: 0-1
    s->set_vflip(s, 0);          // Vertical flip: 0-1
    s->set_dcw(s, 1);            // DCW: 0-1 (enable)
    s->set_colorbar(s, 0);       // Color bar: 0-1 (disable)
  }
  
  Serial.println("📷 Camera initialized: VGA (640x480), Quality 8 (High Compression)");
}

bool initSDCard() {
  if (!SD_MMC.begin()) {
    Serial.println("❌ SD Card Mount Failed");
    return false;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("❌ No SD card attached");
    return false;
  }
  
  Serial.print("✅ SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("💾 SD Card Size: %lluMB\n", cardSize);
  
  return true;
}

bool capturePhotoToSD() {
  if (!sdCardInitialized) {
    Serial.println("❌ SD Card not initialized");
    return false;
  }
  
  // Capture photo
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Camera capture failed");
    return false;
  }
  
  // Generate filename with timestamp
  offlinePhotoCount++;
  String filename = "/photo_" + String(millis()) + "_" + String(offlinePhotoCount) + ".jpg";
  
  // Save to SD card
  File file = SD_MMC.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("❌ Failed to open file for writing");
    esp_camera_fb_return(fb);
    return false;
  }
  
  file.write(fb->buf, fb->len);
  file.close();
  
  Serial.printf("📷 Photo saved: %s (%d bytes)\n", filename.c_str(), fb->len);
  
  esp_camera_fb_return(fb);
  
  return true;
}

void checkWiFiAndOfflineMode() {
  unsigned long currentTime = millis();
  
  // Handle auto capture in two scenarios:
  // 1. Offline mode (no WiFi)
  // 2. Online but no WebSocket clients connected
  bool shouldAutoCapture = false;
  
  if (offlineModeActive && sdCardInitialized) {
    // Scenario 1: Offline mode
    shouldAutoCapture = true;
  } else if (WiFi.status() == WL_CONNECTED && sdCardInitialized && webSocket.connectedClients() == 0) {
    // Scenario 2: Online but no clients
    shouldAutoCapture = true;
  }
  
  if (shouldAutoCapture) {
    if (currentTime - lastOfflineCapture >= offlineCaptureInterval) {
      lastOfflineCapture = currentTime;
      if (offlineModeActive) {
        Serial.println("📷 Auto capturing offline photo...");
      } else {
        Serial.println("📷 Auto capturing photo (no WebSocket clients)...");
      }
      
      if (capturePhotoToSD()) {
        Serial.printf("✅ Photo #%d captured successfully\n", offlinePhotoCount);
      } else {
        Serial.println("❌ Failed to capture photo");
      }
    }
  }
  
  // Only check WiFi at specified intervals
  if (currentTime - lastWifiCheck < wifiCheckInterval) {
    return;
  }
  
  lastWifiCheck = currentTime;
  
  if (WiFi.status() != WL_CONNECTED) {
    if (!offlineModeActive) {
      Serial.println("📡 WiFi disconnected, entering offline mode...");
      Serial.println("📷 Starting auto capture every 5 seconds...");
      offlineModeActive = true;
      lastOfflineCapture = currentTime; // Reset capture timer
    }
    
    // Try to reconnect WiFi
    Serial.println("🔄 Attempting to reconnect WiFi...");
    WiFi.reconnect();
    
    // Wait for connection with timeout
    unsigned long connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && 
           (millis() - connectStart) < wifiTimeoutDuration) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ WiFi reconnected!");
      Serial.print("📍 IP Address: ");
      Serial.println(WiFi.localIP());
      if (offlineModeActive) {
        Serial.println("📷 WiFi restored - will auto capture only when no clients connected");
        offlineModeActive = false;
      }
      
      // Try to register device again
      deviceRegistered = false;
      registerDevice();
    } else {
      Serial.println("\n❌ WiFi reconnection failed, staying in offline mode");
    }
  } else {
    if (offlineModeActive) {
      Serial.println("✅ WiFi connection restored, exiting offline mode");
      Serial.println("📷 Will auto capture only when no WebSocket clients connected");
      offlineModeActive = false;
    }
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  
  Serial.println("\n🎥 ESP32-CAM Real-time WebSocket Stream - Starting...");

  // Initialize SD Card
  Serial.println("💾 Initializing SD Card...");
  sdCardInitialized = initSDCard();
  if (sdCardInitialized) {
    Serial.println("✅ SD Card ready for offline photo capture");
  } else {
    Serial.println("⚠️ SD Card initialization failed - offline capture disabled");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("🔗 Connecting to WiFi");
  
  // WiFi connection with 10 second timeout
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime) < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("📍 IP Address: ");
    Serial.println(WiFi.localIP());
    
    initCamera();
    startRealtimeServer();
  } else {
    Serial.println("\n❌ WiFi connection failed after 10 seconds!");
    Serial.println("📡 Entering offline mode...");
    Serial.println("📷 Starting auto capture every 5 seconds...");
    offlineModeActive = true;
    lastOfflineCapture = millis();
    
    initCamera();
    
    if (!sdCardInitialized) {
      Serial.println("⚠️ WARNING: SD Card not available for offline capture!");
      Serial.println("💡 Please check SD Card connection");
    } else {    Serial.println("✅ Offline mode ready - will capture photos every 5 seconds");
    Serial.println("💡 When WiFi connects, will auto capture only when no WebSocket clients");
  }
  }
}

void loop() {
  // Handle HTTP server only if WiFi is connected
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
    webSocket.loop();
    
    // Continuous frame streaming
    streamFrame();
  }
  
  // Check WiFi status and handle offline mode
  checkWiFiAndOfflineMode();
  
  // Small delay to prevent overwhelming the system
  delayMicroseconds(100);
}
