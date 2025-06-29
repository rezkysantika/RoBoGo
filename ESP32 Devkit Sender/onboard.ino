#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define MPU6050_ADDR 0x68
#define ACCEL_SENSITIVITY_2G 16384.0
#define GYRO_SENSITIVITY_250DPS 131.0

#define MAG_ADDR 0x1E

#define TRIG_PIN 5
#define ECHO_PIN 18

#define DEG_TO_RAD 0.01745329251994329576923690768489F

int16_t rawAx, rawAy, rawAz;
int16_t rawGx, rawGy, rawGz;
int16_t rawMx, rawMy, rawMz;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float pitch, roll, yaw;

float accelOffsetX = 0;
float accelOffsetY = 0;
float accelOffsetZ = 0;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float magBiasX = -24.5;
float magBiasY = -29.5;
float magBiasZ = 409.0;
float magScaleX = 0.75196;
float magScaleY = 0.62558;
float magScaleZ = 13.9583;

static float KF_positionX_m = 0.0;
static float KF_velocityX_mps = 0.0;
static float P_x[2][2] = {{1, 0}, {0, 1}};
static float KF_positionY_m = 0.0;
static float KF_velocityY_mps = 0.0;
static float P_y[2][2] = {{1, 0}, {0, 1}};

const float Q_pos_var = 0.0001;
const float Q_vel_var = 0.01;
const float R_accel_var = 0.5;

static float totalDistanceTraveled_cm = 0;
static float lastPosX_KF_m_for_distance = 0;
static float lastPosY_KF_m_for_distance = 0;

static float lastPosX_KF_cm_previous_iteration = 0;
static float lastPosY_KF_cm_previous_iteration = 0;

static float debugScalarVelocity_mps = 0.0;
static float debugScalarDistance_m = 0.0;

static unsigned long lastUpdateTime = 0;
static unsigned long lastStationaryTime = 0;

const unsigned long STATIONARY_RESET_DELAY_MS = 5000;
const float ACCEL_MOVEMENT_THRESHOLD_MPS2 = 0.3;
const float ROTATION_THRESHOLD_DPS = 0.5;
const float ACCEL_DEADBAND_MPS2 = 0.15;
const float VELOCITY_DEADBAND_MPS = 0.03;
const float ALPHA_ACCEL_FILTER = 0.9;
static float filteredLinearAccelX_mps2 = 0;
static float filteredLinearAccelY_mps2 = 0;
const float VELOCITY_DECAY_FACTOR = 0.98;

bool isMoving = false;

// New variables for X/Y STILL/MOVE logic
const float ACCEL_AXIS_STILL_THRESHOLD_G = 5.0; // Your requested 5.0g threshold
const unsigned long AXIS_STILL_DURATION_MS = 5000; // 5 seconds
static unsigned long lastAccelXWithinThresholdTime = 0;
static unsigned long lastAccelYWithinThresholdTime = 0;


float ultrasonicDistance_cm = 0;
bool obstacleDetected = false;
const float OBSTACLE_THRESHOLD_CM = 30.0;

uint8_t receiverAddress[] = { 0xCC, 0x8D, 0xA2, 0x0C, 0xE1, 0x7C };

typedef struct {
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

typedef struct {
  char message[32];
} AckMessage;

SensorData dataToSend;
AckMessage ackMessage;

bool checkAndEnableBypassMode();
bool setupMPU6050();
bool setupMagnetometer();
void calibrateMPU6050();
void readRawAccelerometerAndGyro();
void readRawMagnetometer();
void processSensorData();
void calculateOrientation();
String getCardinalDirection(float angle);
void readUltrasonicDistance();
void printData();
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, -1, 4);
  Wire.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("Initializing Sensors and ESP-NOW...");

  if (!checkAndEnableBypassMode()) {
    Serial.println("Failed to enable MPU6050 bypass mode after multiple attempts. Check wiring!");
    while (true);
  }

  if (!setupMPU6050()) {
    Serial.println("Failed to configure MPU6050 after multiple attempts. Check wiring!");
    while (true);
  }

  if (!setupMagnetometer()) {
    Serial.println("Failed to configure HMC5883L after multiple attempts. Check wiring!");
    while (true);
  }

  calibrateMPU6050();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed! Halting.");
    while (true);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer! Halting.");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW ready. Sending data...");
}

void loop() {
  static unsigned long lastBypassCheck = 0;
  if (millis() - lastBypassCheck > 5000) {
    checkAndEnableBypassMode();
    lastBypassCheck = millis();
  }

  float currentTime = millis() / 1000.0;
  float dt = currentTime - lastUpdateTime;
  lastUpdateTime = currentTime;

  if (dt <= 0) dt = 0.001;

  readRawAccelerometerAndGyro();
  readRawMagnetometer();
  processSensorData();
  calculateOrientation();
  readUltrasonicDistance();

  float pitchRad = pitch * DEG_TO_RAD;
  float rollRad = roll * DEG_TO_RAD;

  float gravityX_mps2 = sin(rollRad) * 9.81;
  float gravityY_mps2 = -sin(pitchRad) * cos(rollRad) * 9.81;

  float accel_x_mps2_biased = accelX * 9.81;
  float accel_y_mps2_biased = accelY * 9.81;

  float linearAccelX_mps2 = accel_x_mps2_biased - gravityX_mps2;
  float linearAccelY_mps2 = accel_y_mps2_biased - gravityY_mps2;

  filteredLinearAccelX_mps2 = ALPHA_ACCEL_FILTER * filteredLinearAccelX_mps2 + (1 - ALPHA_ACCEL_FILTER) * linearAccelX_mps2;
  filteredLinearAccelY_mps2 = ALPHA_ACCEL_FILTER * filteredLinearAccelY_mps2 + (1 - ALPHA_ACCEL_FILTER) * linearAccelY_mps2;

  if (fabs(filteredLinearAccelX_mps2) < ACCEL_DEADBAND_MPS2) filteredLinearAccelX_mps2 = 0;
  if (fabs(filteredLinearAccelY_mps2) < ACCEL_DEADBAND_MPS2) filteredLinearAccelY_mps2 = 0;

  float yawRad = yaw * DEG_TO_RAD;
  float cosYaw = cos(yawRad);
  float sinYaw = sin(yawRad);

  float accelGlobalX = filteredLinearAccelX_mps2 * cosYaw - filteredLinearAccelY_mps2 * sinYaw;
  float accelGlobalY = filteredLinearAccelX_mps2 * sinYaw + filteredLinearAccelY_mps2 * cosYaw;

  float currentLinearAccelerationMagnitude = sqrt(filteredLinearAccelX_mps2 * filteredLinearAccelX_mps2 + filteredLinearAccelY_mps2 * filteredLinearAccelY_mps2);
  float currentRotationRateMagnitude = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  isMoving = (currentLinearAccelerationMagnitude > ACCEL_MOVEMENT_THRESHOLD_MPS2) || (currentRotationRateMagnitude > ROTATION_THRESHOLD_DPS);

  // New X axis stillness logic
  if (fabs(accelX) <= ACCEL_AXIS_STILL_THRESHOLD_G) {
    if (lastAccelXWithinThresholdTime == 0) {
      lastAccelXWithinThresholdTime = millis();
    }
    if (millis() - lastAccelXWithinThresholdTime >= AXIS_STILL_DURATION_MS) {
      Serial.println("X STILL");
    } else {
      Serial.println("X MOVE");
    }
  } else {
    Serial.println("X MOVE");
    lastAccelXWithinThresholdTime = 0; // Reset timer if outside threshold
  }

  // New Y axis stillness logic
  if (fabs(accelY) <= ACCEL_AXIS_STILL_THRESHOLD_G) {
    if (lastAccelYWithinThresholdTime == 0) {
      lastAccelYWithinThresholdTime = millis();
    }
    if (millis() - lastAccelYWithinThresholdTime >= AXIS_STILL_DURATION_MS) {
      Serial.println("Y STILL");
    } else {
      Serial.println("Y MOVE");
    }
  } else {
    Serial.println("Y MOVE");
    lastAccelYWithinThresholdTime = 0; // Reset timer if outside threshold
  }

  KF_positionX_m += KF_velocityX_mps * dt + 0.5 * accelGlobalX * dt * dt;
  KF_velocityX_mps += accelGlobalX * dt;
  KF_positionY_m += KF_velocityY_mps * dt + 0.5 * accelGlobalY * dt * dt;
  KF_velocityY_mps += accelGlobalY * dt;

  P_x[0][0] += dt * dt * Q_pos_var;
  P_x[1][1] += dt * dt * Q_vel_var;
  P_y[0][0] += dt * dt * Q_pos_var;
  P_y[1][1] += dt * dt * Q_vel_var;

  if (!isMoving) {
    KF_velocityX_mps *= VELOCITY_DECAY_FACTOR;
    KF_velocityY_mps *= VELOCITY_DECAY_FACTOR;

    if (fabs(KF_velocityX_mps) < VELOCITY_DEADBAND_MPS) KF_velocityX_mps = 0;
    if (fabs(KF_velocityY_mps) < VELOCITY_DEADBAND_MPS) KF_velocityY_mps = 0;

    if (millis() - lastStationaryTime > STATIONARY_RESET_DELAY_MS) {
      Serial.println("POSITION & DISTANCE RESET (ZUPT) due to prolonged stillness.");
      KF_positionX_m = 0;
      KF_positionY_m = 0;
      KF_velocityX_mps = 0;
      KF_velocityY_mps = 0;
      totalDistanceTraveled_cm = 0;
      lastPosX_KF_m_for_distance = 0;
      lastPosY_KF_m_for_distance = 0;
      lastPosX_KF_cm_previous_iteration = 0;
      lastPosY_KF_cm_previous_iteration = 0;

      P_x[0][0] = 0.01; P_x[0][1] = 0;
      P_x[1][0] = 0;    P_x[1][1] = 0.01;
      P_y[0][0] = 0.01; P_y[0][1] = 0;
      P_y[1][0] = 0;    P_y[1][1] = 0.01;
    }
  } else {
    lastStationaryTime = millis();
    if (fabs(KF_velocityX_mps) < VELOCITY_DEADBAND_MPS) KF_velocityX_mps = 0;
    if (fabs(KF_velocityY_mps) < VELOCITY_DEADBAND_MPS) KF_velocityY_mps = 0;
  }

  float deltaX_m_for_distance = KF_positionX_m - lastPosX_KF_m_for_distance;
  float deltaY_m_for_distance = KF_positionY_m - lastPosY_KF_m_for_distance;
  float segmentDistance_m = sqrt(deltaX_m_for_distance * deltaX_m_for_distance + deltaY_m_for_distance * deltaY_m_for_distance);

  if (isMoving && (sqrt(KF_velocityX_mps*KF_velocityX_mps + KF_velocityY_mps*KF_velocityY_mps) > VELOCITY_DEADBAND_MPS)) {
      totalDistanceTraveled_cm += segmentDistance_m * 100.0;
  }
  lastPosX_KF_m_for_distance = KF_positionX_m;
  lastPosY_KF_m_for_distance = KF_positionY_m;

  debugScalarVelocity_mps += currentLinearAccelerationMagnitude * dt;
  debugScalarDistance_m += debugScalarVelocity_mps * dt;

  dataToSend.timestamp = millis();
  dataToSend.accel_x = accelX;
  dataToSend.accel_y = accelY;
  dataToSend.accel_z = accelZ;

  dataToSend.gyro_x = gyroX;
  dataToSend.gyro_y = gyroY;
  dataToSend.gyro_z = gyroZ;

  dataToSend.mag_x = magX;
  dataToSend.mag_y = magY;
  dataToSend.mag_z = magZ;

  dataToSend.pitch = pitch;
  dataToSend.roll = roll;
  dataToSend.yaw = yaw;
  dataToSend.heading = yaw;

  String cardinalDirection = getCardinalDirection(yaw);
  strncpy(dataToSend.direction, cardinalDirection.c_str(), sizeof(dataToSend.direction));
  dataToSend.direction[sizeof(dataToSend.direction) - 1] = '\0';

  dataToSend.ultrasonic_distance = ultrasonicDistance_cm;
  dataToSend.obstacleDetected = (ultrasonicDistance_cm > 0 && ultrasonicDistance_cm < OBSTACLE_THRESHOLD_CM);
  if (dataToSend.obstacleDetected) {
    Serial2.println("obstacleDetected:true");
  }

  dataToSend.positionX = KF_positionX_m * 100.0;
  dataToSend.positionY = KF_positionY_m * 100.0;
  dataToSend.velocityX = KF_velocityX_mps * 100.0;
  dataToSend.velocityY = KF_velocityY_mps * 100.0;
  dataToSend.velocity = sqrt(dataToSend.velocityX * dataToSend.velocityX + dataToSend.velocityY * dataToSend.velocityY);

  dataToSend.distanceTraveled = totalDistanceTraveled_cm;

  dataToSend.distX = (dataToSend.positionX - lastPosX_KF_cm_previous_iteration);
  dataToSend.distY = (dataToSend.positionY - lastPosY_KF_cm_previous_iteration);
  lastPosX_KF_cm_previous_iteration = dataToSend.positionX;
  lastPosY_KF_cm_previous_iteration = dataToSend.positionY;

  dataToSend.rotationRate = currentRotationRateMagnitude;

  dataToSend.accelerationMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  dataToSend.linearAcceleration = currentLinearAccelerationMagnitude;

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend));
  if (result == ESP_OK) {
    printData();
  } else {
    Serial.println("Failed to send data.");
  }

  delay(300);
}

bool checkAndEnableBypassMode() {
  int attempts = 0;
  const int maxAttempts = 5;
  bool success = false;

  while (!success && attempts < maxAttempts) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
        Serial.printf("MPU6050 PWR_MGMT_1 write failed, attempt %d\n", attempts + 1);
        attempts++;
        delay(50);
        continue;
    }
    delay(10);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x37);
    Wire.write(0x02);
    if (Wire.endTransmission(true) != 0) {
        Serial.printf("MPU6050 INT_PIN_CFG write failed, attempt %d\n", attempts + 1);
        attempts++;
        delay(50);
        continue;
    }
    delay(10);
    success = true;
    Serial.println("MPU6050 bypass mode enabled.");
  }
  return success;
}

bool setupMPU6050() {
  int attempts = 0;
  const int maxAttempts = 5;
  bool success = false;
  while (!success && attempts < maxAttempts) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
      Serial.printf("MPU6050 ACCEL_CONFIG write failed, attempt %d\n", attempts + 1);
      attempts++;
      delay(50);
      continue;
    }
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
      Serial.printf("MPU6050 GYRO_CONFIG write failed, attempt %d\n", attempts + 1);
      attempts++;
      delay(50);
      continue;
    }
    Serial.println("MPU6050 configured successfully.");
    success = true;
  }
  return success;
}

bool setupMagnetometer() {
  int attempts = 0;
  const int maxAttempts = 5;
  bool success = false;
  while (!success && attempts < maxAttempts) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x00);
    Wire.write(0x70);
    if (Wire.endTransmission(true) != 0) {
      Serial.printf("HMC5883L CRA write failed, attempt %d\n", attempts + 1);
      attempts++;
      delay(50);
      continue;
    }
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x01);
    Wire.write(0xA0);
    if (Wire.endTransmission(true) != 0) {
      Serial.printf("HMC5883L CRB write failed, attempt %d\n", attempts + 1);
      attempts++;
      delay(50);
      continue;
    }
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x02);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
      Serial.printf("HMC5883L Mode write failed, attempt %d\n", attempts + 1);
      attempts++;
      delay(50);
      continue;
    }
    Serial.println("HMC5883L configured successfully.");
    success = true;
  }
  return success;
}

void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050 Accelerometer and Gyroscope.");
  Serial.println("Keep the board ABSOLUTELY STILL for 3 seconds.");
  Serial.println("Ensure the Z-axis is pointing perpendicular to the ground (e.g., board lying flat).");
  delay(100);

  long sumRawAx = 0, sumRawAy = 0, sumRawAz = 0;
  long sumRawGx = 0, sumRawGy = 0, sumRawGz = 0;
  int num_samples = 1000;

  for (int i = 0; i < num_samples; i++) {
    readRawAccelerometerAndGyro();
    sumRawAx += rawAx;
    sumRawAy += rawAy;
    sumRawAz += rawAz;
    sumRawGx += rawGx;
    sumRawGy += rawGy;
    sumRawGz += rawGz;
    delay(3);
  }

  accelOffsetX = (float)sumRawAx / num_samples / ACCEL_SENSITIVITY_2G;
  accelOffsetY = (float)sumRawAy / num_samples / ACCEL_SENSITIVITY_2G;
  accelOffsetZ = ((float)sumRawAz / num_samples / ACCEL_SENSITIVITY_2G) - 1.0;

  gyroOffsetX = (float)sumRawGx / num_samples / GYRO_SENSITIVITY_250DPS;
  gyroOffsetY = (float)sumRawGy / num_samples / GYRO_SENSITIVITY_250DPS;
  gyroOffsetZ = (float)sumRawGz / num_samples / GYRO_SENSITIVITY_250DPS;

  Serial.printf("Accel Bias (g): X=%.4f, Y=%.4f, Z=%.4f (Z bias adjusted for 1g gravity)\n", accelOffsetX, accelOffsetY, accelOffsetZ);
  Serial.printf("Gyro Bias (deg/s): X=%.4f, Y=%.4f, Z=%.4f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.println("MPU6050 calibration complete. These biases will be subtracted from readings.");

  KF_positionX_m = 0.0;
  KF_positionY_m = 0.0;
  KF_velocityX_mps = 0.0;
  KF_velocityY_mps = 0.0;
  totalDistanceTraveled_cm = 0;
  lastPosX_KF_m_for_distance = 0;
  lastPosY_KF_m_for_distance = 0;
  lastPosX_KF_cm_previous_iteration = 0;
  lastPosY_KF_cm_previous_iteration = 0;
  lastUpdateTime = millis() / 1000.0;
  lastStationaryTime = millis();
  lastAccelXWithinThresholdTime = millis(); // Initialize for new manual thresholding
  lastAccelYWithinThresholdTime = millis(); // Initialize for new manual thresholding

  P_x[0][0] = 1.0; P_x[0][1] = 0;
  P_x[1][0] = 0;   P_x[1][1] = 1.0;
  P_y[0][0] = 1.0; P_y[0][1] = 0;
  P_y[1][0] = 0;   P_y[1][1] = 1.0;

  debugScalarVelocity_mps = 0.0;
  debugScalarDistance_m = 0.0;
}

void readRawAccelerometerAndGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDR, 14);
  if (Wire.available() == 14) {
    rawAx = (Wire.read() << 8) | Wire.read();
    rawAy = (Wire.read() << 8) | Wire.read();
    rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    rawGx = (Wire.read() << 8) | Wire.read();
    rawGy = (Wire.read() << 8) | Wire.read();
    rawGz = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Failed to read MPU6050 data. Check wiring or I2C address.");
  }
}

void readRawMagnetometer() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);

  Wire.requestFrom(MAG_ADDR, 6);
  if (Wire.available() == 6) {
    rawMx = (Wire.read() << 8) | Wire.read();
    rawMz = (Wire.read() << 8) | Wire.read();
    rawMy = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Failed to read HMC5883L data. Check wiring or I2C address or MPU6050 bypass.");
  }
}

void processSensorData() {
  accelX = ((float)rawAx / ACCEL_SENSITIVITY_2G) - accelOffsetX;
  accelY = ((float)rawAy / ACCEL_SENSITIVITY_2G) - accelOffsetY;
  accelZ = ((float)rawAz / ACCEL_SENSITIVITY_2G) - accelOffsetZ;

  gyroX = ((float)rawGx / GYRO_SENSITIVITY_250DPS) - gyroOffsetX;
  gyroY = ((float)rawGy / GYRO_SENSITIVITY_250DPS) - gyroOffsetY;
  gyroZ = ((float)rawGz / GYRO_SENSITIVITY_250DPS) - gyroOffsetZ;

  magX = ((float)rawMx - magBiasX) * magScaleX;
  magY = ((float)rawMy - magBiasY) * magScaleY;
  magZ = ((float)rawMz - magBiasZ) * magScaleZ;
}

void calculateOrientation() {
  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0 / PI);
  roll = atan2(-accelX, accelZ) * (180.0 / PI);

  float cosRoll = cos(roll * PI / 180.0);
  float sinRoll = sin(roll * PI / 180.0);
  float cosPitch = cos(pitch * PI / 180.0);
  float sinPitch = sin(pitch * PI / 180.0);

  float BY_h = magY * cosRoll - magZ * sinRoll;
  float BX_h = magX * cosPitch + magY * sinRoll * sinPitch + magZ * cosRoll * sinPitch;

  float currentYaw = atan2(-BX_h, BY_h) * (180.0 / PI);

  if (currentYaw < 0) {
    currentYaw += 360.0;
  }

  yaw = currentYaw;
}

String getCardinalDirection(float angle) {
  const char* directions[] = {
    "North", "North-Northeast", "Northeast", "East-Northeast",
    "East", "East-Southeast", "Southeast", "South-Southeast",
    "South", "South-Southwest", "Southwest", "West-Southwest",
    "West", "West-Northwest", "Northwest", "North-Northwest"
  };

  angle += 11.25;
  if (angle >= 360) angle -= 360;

  int index = (int)(angle / 22.5);

  return String(directions[index]);
}

void readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  ultrasonicDistance_cm = (duration == 0) ? -1.0 : duration * 0.0343 / 2.0;
}

void printData() {
  Serial.println("====== SENT DATA ======");
  Serial.printf("Timestamp: %lu ms\n", dataToSend.timestamp);
  Serial.printf("Accel (bias-corr g): X=%.3f Y=%.3f Z=%.3f\n", dataToSend.accel_x, dataToSend.accel_y, dataToSend.accel_z);
  Serial.printf("Gyro (dps): X=%.2f Y=%.2f Z=%.2f\n", dataToSend.gyro_x, dataToSend.gyro_y, dataToSend.gyro_z);
  Serial.printf("Mag (uT): X=%.2f Y=%.2f Z=%.2f\n", dataToSend.mag_x, dataToSend.mag_y, dataToSend.mag_z);
  Serial.printf("Pitch: %.2f° Roll: %.2f° Yaw: %.2f°\n", dataToSend.pitch, dataToSend.roll, dataToSend.yaw);
  Serial.printf("Heading: %.2f ° (%s)\n", dataToSend.heading, dataToSend.direction);
  Serial.printf("Ultrasonic Distance: %.2f cm\n", dataToSend.ultrasonic_distance);
  Serial.printf("Obstacle Detected: %s\n", dataToSend.obstacleDetected ? "YES" : "NO");

  Serial.printf("Current Pos (KF Displacement): (%.2f, %.2f) cm\n", dataToSend.positionX, dataToSend.positionY);
  Serial.printf("Velocity (KF): %.2f cm/s (Vx=%.2f, Vy=%.2f)\n", dataToSend.velocity, dataToSend.velocityX, dataToSend.velocityY);
  Serial.printf("Total Distance Traveled (KF): %.2f cm\n", dataToSend.distanceTraveled);
  Serial.printf("Incremental Displacement (KF): X=%.2f Y=%.2f cm\n", dataToSend.distX, dataToSend.distY);
  Serial.printf("Rotation Rate: %.2f dps\n", dataToSend.rotationRate);
  Serial.printf("Acceleration Magnitude: %.2f g\n", dataToSend.accelerationMagnitude);
  Serial.printf("Linear Acceleration: %.2f m/s^2\n", dataToSend.linearAcceleration);
  Serial.printf("DEBUG: Scalar Distance from Lin Accel Mag: %.2f m\n", debugScalarDistance_m);
  Serial.printf("Accel X/Y Status: Threshold=%.3f, X_Duration=%lu ms, Y_Duration=%lu ms\n", ACCEL_AXIS_STILL_THRESHOLD_G, millis() - lastAccelXWithinThresholdTime, millis() - lastAccelYWithinThresholdTime);
  Serial.println("========================\n");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(AckMessage)) {
    memcpy(&ackMessage, data, sizeof(AckMessage));
    Serial.print("Received ACK from ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", info->src_addr[i]);
      if (i < 5) Serial.print(":");
    }
    Serial.print(" -> Message: ");
    Serial.println(ackMessage.message);
  } else {
    Serial.printf("Received unknown data format! Length: %d bytes\n", len);
  }
}
