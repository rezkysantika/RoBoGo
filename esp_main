#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

// --- I2C addresses ---
#define MPU6050_ADDR 0x68
#define MAG_ADDR 0x1E // HMC5883L

// --- Ultrasonic pins ---
#define TRIG_PIN 5
#define ECHO_PIN 18

// --- Calibration values ---
float mag_bias_x = 10.0;
float mag_bias_y = -5.0;
float mag_bias_z = 2.0;

float mag_scale_x = 1.02;
float mag_scale_y = 0.98;
float mag_scale_z = 1.05;

// --- Variables ---
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float ultrasonic_distance_cm;

// Struct to send data
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

// Struct to receive ACK
typedef struct AckMessage {
  char message[32];
} AckMessage;

SensorData dataToSend;
AckMessage ackMessage;

// Receiver MAC Address
uint8_t receiverAddress[] = {0xCC, 0x8D, 0xA2, 0x0C, 0xE1, 0x7C};

// --- Function declarations ---
void enableBypassMode();
void setupMPU6050();
void setupMagnetometer();
void readAccelerometerAndGyro();
void readMagnetometer();
void readUltrasonic();
String getDirection(float heading);
void printData();
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len); // <-- Corrected

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  enableBypassMode();
  setupMPU6050();
  setupMagnetometer();

  WiFi.mode(WIFI_STA);
  Serial.println("[INFO] ESP-NOW initializing...");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while (1);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer!");
    while (1);
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("[INFO] ESP-NOW ready. Sending data...");
}

void loop() {
  static unsigned long lastBypassCheck = 0;
  if (millis() - lastBypassCheck > 5000) {
    enableBypassMode();
    lastBypassCheck = millis();
  }

  readAccelerometerAndGyro();
  readMagnetometer();
  readUltrasonic();

  // --- Important: normalize accelerometer here first ---
  float accel_x_g = ax / 16384.0;
  float accel_y_g = ay / 16384.0;
  float accel_z_g = az / 16384.0;

  // --- Initialize tracking variables ---
  static bool initialized = false;
  static float lastUpdateTime = 0;
  static float lastPosX = 0;
  static float lastPosY = 0;
  static float totalDistance = 0;

  if (!initialized) {
    dataToSend.locationStartX = 0;
    dataToSend.locationStartY = 0;
    dataToSend.positionX = 0;
    dataToSend.positionY = 0;
    lastPosX = 0;
    lastPosY = 0;
    lastUpdateTime = millis() / 1000.0;
    initialized = true;
  }

  // --- Timing ---
  float currentTime = millis() / 1000.0;
  float dt = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  // --- Estimate velocity ---
  float accelX_mps2 = accel_x_g * 9.81;
  float accelY_mps2 = accel_y_g * 9.81;

  // Update velocity (integrate acceleration)
  dataToSend.velocityX += accelX_mps2 * dt;
  dataToSend.velocityY += accelY_mps2 * dt;
  dataToSend.velocity = sqrt(dataToSend.velocityX * dataToSend.velocityX +
                            dataToSend.velocityY * dataToSend.velocityY);

  // --- Compute relative movement (delta distance) ---
  dataToSend.distX = dataToSend.velocityX * dt * 100; // in cm
  dataToSend.distY = dataToSend.velocityY * dt * 100; // in cm

  // --- Update position (absolute) using delta movement ---
  dataToSend.positionX += dataToSend.distX;
  dataToSend.positionY += dataToSend.distY;

  // --- Calculate distance traveled ---
  float deltaX = dataToSend.positionX - lastPosX;
  float deltaY = dataToSend.positionY - lastPosY;
  float segmentDistance = sqrt(deltaX * deltaX + deltaY * deltaY);
  totalDistance += segmentDistance;

  lastPosX = dataToSend.positionX;
  lastPosY = dataToSend.positionY;

  dataToSend.distanceTraveled = totalDistance;

  // --- Magnetometer calibration ---
  float mx_cal = (mx - mag_bias_x) * mag_scale_x;
  float my_cal = (my - mag_bias_y) * mag_scale_y;
  float mz_cal = (mz - mag_bias_z) * mag_scale_z;

  // --- Gyroscope conversion ---
  float gyro_x_dps = gx / 131.0;
  float gyro_y_dps = gy / 131.0;
  float gyro_z_dps = gz / 131.0;

  // --- Tilt compensation ---
  float norm = sqrt(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
  float ax_f = accel_x_g / norm;
  float ay_f = accel_y_g / norm;
  float az_f = accel_z_g / norm;

  float Xh = mx_cal * ax_f + my_cal * ay_f - mz_cal * az_f;
  float Yh = mx_cal * (-ay_f) + my_cal * ax_f;

  float computedHeading = atan2(Yh, Xh) * (180.0 / PI);
  if (computedHeading < 0) computedHeading += 360.0;

  String computedDirection = getDirection(computedHeading);

  // --- Fill dataToSend ---
  dataToSend.timestamp = millis();
  dataToSend.accel_x = accel_x_g;
  dataToSend.accel_y = accel_y_g;
  dataToSend.accel_z = accel_z_g;
  dataToSend.gyro_x = gyro_x_dps;
  dataToSend.gyro_y = gyro_y_dps;
  dataToSend.gyro_z = gyro_z_dps;
  dataToSend.mag_x = mx_cal;
  dataToSend.mag_y = my_cal;
  dataToSend.mag_z = mz_cal;
  dataToSend.heading = computedHeading;
  strncpy(dataToSend.direction, computedDirection.c_str(), sizeof(dataToSend.direction));
  dataToSend.direction[sizeof(dataToSend.direction) - 1] = '\0';
  dataToSend.distance = ultrasonic_distance_cm;

  // Extra calculations
  dataToSend.rotationRate = sqrt(gyro_x_dps * gyro_x_dps +
                                 gyro_y_dps * gyro_y_dps +
                                 gyro_z_dps * gyro_z_dps);

  dataToSend.obstacleDetected = (ultrasonic_distance_cm > 0 && ultrasonic_distance_cm < 30.0);

  dataToSend.accelerationMagnitude = sqrt(accel_x_g * accel_x_g +
                                          accel_y_g * accel_y_g +
                                          accel_z_g * accel_z_g);

  dataToSend.linearAcceleration = sqrt(accel_x_g * accel_x_g +
                                       accel_y_g * accel_y_g);

  dataToSend.pitch = atan2(accel_y_g, sqrt(accel_x_g * accel_x_g + accel_z_g * accel_z_g)) * (180.0 / PI);
  dataToSend.roll  = atan2(-accel_x_g, accel_z_g) * (180.0 / PI);
  dataToSend.yaw   = computedHeading;

  // --- Send data ---
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));

  if (result == ESP_OK) {
    Serial.println("[INFO] Data sent successfully.");
    printData(); 
  } else {
    Serial.println("[ERROR] Failed to send data.");
  }

  // printData();
  delay(300);
}

// --- Functions ---

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckMessage)) {
    memcpy(&ackMessage, incomingData, sizeof(AckMessage));
    Serial.print("[INFO] Received ACK from ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", recv_info->src_addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.print(" -> Message: ");
    Serial.println(ackMessage.message);
  } else {
    Serial.println("[WARN] Received unknown data format!");
  }
}

void enableBypassMode() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
}

void setupMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void setupMagnetometer() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readAccelerometerAndGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  if (Wire.available() == 14) {
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temperature (ignored)
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
  }
}

void readMagnetometer() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 6);

  if (Wire.available() == 6) {
    mx = (Wire.read() << 8) | Wire.read();
    my = (Wire.read() << 8) | Wire.read();
    mz = (Wire.read() << 8) | Wire.read();
  }
}

void readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    ultrasonic_distance_cm = -1.0;
  } else {
    ultrasonic_distance_cm = duration * 0.0343 / 2.0;
  }
}

String getDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5) return "North";
  else if (heading >= 22.5 && heading < 67.5) return "North-East";
  else if (heading >= 67.5 && heading < 112.5) return "East";
  else if (heading >= 112.5 && heading < 157.5) return "South-East";
  else if (heading >= 157.5 && heading < 202.5) return "South";
  else if (heading >= 202.5 && heading < 247.5) return "South-West";
  else if (heading >= 247.5 && heading < 292.5) return "West";
  else return "North-West";
}

void printData() {
  Serial.println("====== SENT DATA ======");
  Serial.printf("Timestamp: %lu ms\n", dataToSend.timestamp);
  Serial.printf("Accel (g): X=%.3f Y=%.3f Z=%.3f\n", dataToSend.accel_x, dataToSend.accel_y, dataToSend.accel_z);
  Serial.printf("Gyro (dps): X=%.2f Y=%.2f Z=%.2f\n", dataToSend.gyro_x, dataToSend.gyro_y, dataToSend.gyro_z);
  Serial.printf("Mag (uT): X=%.2f Y=%.2f Z=%.2f\n", dataToSend.mag_x, dataToSend.mag_y, dataToSend.mag_z);
  Serial.printf("Heading: %.2f ° (%s)\n", dataToSend.heading, dataToSend.direction);
  Serial.printf("Ultrasonic Distance: %.2f cm\n", dataToSend.distance);
  Serial.printf("Start Pos (X, Y): (%.2f, %.2f) cm\n", dataToSend.locationStartX, dataToSend.locationStartY);
  Serial.printf("Current Pos (X, Y): (%.2f, %.2f) cm\n", dataToSend.positionX, dataToSend.positionY);
  Serial.printf("Velocity: %.2f cm/s (Vx=%.2f, Vy=%.2f)\n", dataToSend.velocity, dataToSend.velocityX, dataToSend.velocityY);
  Serial.printf("Distance Traveled: %.2f cm\n", dataToSend.distanceTraveled);
  Serial.printf("Distance: X=%.2f Y=%.2f\n", dataToSend.distX, dataToSend.distY);
  Serial.printf("Rotation Rate: %.2f dps\n", dataToSend.rotationRate);
  Serial.printf("Obstacle Detected: %s\n", dataToSend.obstacleDetected ? "YES" : "NO");
  Serial.printf("Acceleration Magnitude: %.2f g\n", dataToSend.accelerationMagnitude);
  Serial.printf("Linear Acceleration: %.2f g\n", dataToSend.linearAcceleration);
  Serial.printf("Pitch: %.2f° Roll: %.2f° Yaw: %.2f°\n", dataToSend.pitch, dataToSend.roll, dataToSend.yaw);
  Serial.println("========================\n");
}

