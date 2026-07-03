#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "esp_log.h"

#include <WiFi.h>
#include <WebServer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =========================
// OLED CONFIG
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// =========================
// ESP32 I2C PIN
// =========================
#define SDA_PIN 21
#define SCL_PIN 22

// =========================
// WIFI CONFIG
// =========================
const char* WIFI_SSID = "AB4_Plus";
const char* WIFI_PASSWORD = "nafisa1107";

WebServer server(80);
String espIpAddress = "0.0.0.0";

// =========================
// OBJECTS
// =========================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// =========================
// SHOCK CONFIG
// =========================
const float GRAVITY_MS2 = 9.80665f;

const float SHOCK_EVENT_DYNAMIC_G = 1.0f;
const float SHOCK_RELEASE_DYNAMIC_G = 0.35f;

const float CHECK_DYNAMIC_G = 1.0f;
const float REJECT_DYNAMIC_G = 2.5f;

const float CHECK_TILT_DEG = 45.0f;
const float REJECT_TILT_DEG = 70.0f;

const int CHECK_SHOCK_COUNT = 2;
const int REJECT_SHOCK_COUNT = 5;

const unsigned long SHOCK_DEBOUNCE_MS = 600;
const unsigned long RESET_IGNORE_SHOCK_MS = 2000;

const float FILTER_ALPHA = 0.85f;

// =========================
// RTOS CONFIG
// =========================
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t i2cMutex;

TaskHandle_t taskMPUHandle = NULL;
TaskHandle_t taskOLEDHandle = NULL;
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskWiFiHandle = NULL;

// =========================
// DATA STRUCTURE
// =========================
struct QCData {
  String packageId = "WAIT";
  String qrStatus = "WAIT";
  String side = "NONE";
  String visualStatus = "WAIT";

  int shockCount = 0;

  float accelG = 1.0f;
  float dynamicG = 0.0f;
  float maxAccelG = 1.0f;
  float maxDynamicG = 0.0f;

  float tiltAngle = 0.0f;

  String handlingStatus = "PASS";
  String finalStatus = "READY";
};

QCData qcData;

bool mpuReady = false;
bool shockArmed = false;

uint8_t mpuAddress = 0x68;

unsigned long lastShockTime = 0;
unsigned long ignoreShockUntil = 0;

String serialBuffer = "";

// Baseline posisi awal sensor
float baseAx = 0.0f;
float baseAy = 0.0f;
float baseAz = GRAVITY_MS2;

// Filtered acceleration
float filtAx = 0.0f;
float filtAy = 0.0f;
float filtAz = GRAVITY_MS2;

// =========================
// HELPER
// =========================
float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float vectorMagnitude(float x, float y, float z) {
  return sqrtf((x * x) + (y * y) + (z * z));
}

float angleBetweenVectors(
  float ax, float ay, float az,
  float bx, float by, float bz
) {
  float magA = vectorMagnitude(ax, ay, az);
  float magB = vectorMagnitude(bx, by, bz);

  if (magA <= 0.001f || magB <= 0.001f) {
    return 0.0f;
  }

  float dot = (ax * bx) + (ay * by) + (az * bz);
  float cosTheta = dot / (magA * magB);
  cosTheta = clampFloat(cosTheta, -1.0f, 1.0f);

  return acosf(cosTheta) * 180.0f / PI;
}

// =========================
// DECISION ENGINE
// =========================
void updateDecision(QCData &data) {
  if (!mpuReady) {
    data.handlingStatus = "MPU_ERR";
  }
  else if (
    data.shockCount >= REJECT_SHOCK_COUNT ||
    data.maxDynamicG >= REJECT_DYNAMIC_G ||
    data.tiltAngle >= REJECT_TILT_DEG
  ) {
    data.handlingStatus = "REJECT";
  }
  else if (
    data.shockCount >= CHECK_SHOCK_COUNT ||
    data.maxDynamicG >= CHECK_DYNAMIC_G ||
    data.tiltAngle >= CHECK_TILT_DEG
  ) {
    data.handlingStatus = "CHECK";
  }
  else {
    data.handlingStatus = "PASS";
  }

  if (data.qrStatus == "WAIT") {
    data.finalStatus = "READY";
  }
  else if (data.qrStatus == "INVALID") {
    data.finalStatus = "TAMPER";
  }
  else if (data.handlingStatus == "MPU_ERR") {
    data.finalStatus = "ERROR";
  }
  else if (data.handlingStatus == "REJECT" || data.visualStatus == "REJECT") {
    data.finalStatus = "REJECT";
  }
  else if (data.handlingStatus == "CHECK" || data.visualStatus == "CHECK") {
    data.finalStatus = "CHECK";
  }
  else if (data.visualStatus == "WAIT") {
    data.finalStatus = "READY";
  }
  else {
    data.finalStatus = "PASS";
  }
}

// =========================
// SERIAL OUTPUT
// =========================
void printStatusToSerial() {
  QCData data;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    data = qcData;
    xSemaphoreGive(dataMutex);
  } else {
    Serial.println("ERROR: dataMutex busy");
    return;
  }

  Serial.print("{");
  Serial.print("\"id\":\""); Serial.print(data.packageId); Serial.print("\",");
  Serial.print("\"qr\":\""); Serial.print(data.qrStatus); Serial.print("\",");
  Serial.print("\"side\":\""); Serial.print(data.side); Serial.print("\",");
  Serial.print("\"visual\":\""); Serial.print(data.visualStatus); Serial.print("\",");
  Serial.print("\"shock\":"); Serial.print(data.shockCount); Serial.print(",");
  Serial.print("\"accelG\":"); Serial.print(data.accelG, 2); Serial.print(",");
  Serial.print("\"dynamicG\":"); Serial.print(data.dynamicG, 2); Serial.print(",");
  Serial.print("\"maxDynamicG\":"); Serial.print(data.maxDynamicG, 2); Serial.print(",");
  Serial.print("\"tilt\":"); Serial.print(data.tiltAngle, 1); Serial.print(",");
  Serial.print("\"handling\":\""); Serial.print(data.handlingStatus); Serial.print("\",");
  Serial.print("\"final\":\""); Serial.print(data.finalStatus); Serial.print("\"");
  Serial.println("}");
}

void printHelp() {
  Serial.println();
  Serial.println("========== COMMAND LIST ==========");
  Serial.println("HELP");
  Serial.println("STATUS?");
  Serial.println("I2C?");
  Serial.println("CAL");
  Serial.println("RESET");
  Serial.println("RESTART");
  Serial.println("ID:FRAGILE-001");
  Serial.println("QR:VALID");
  Serial.println("QR:INVALID");
  Serial.println("SIDE:TOP");
  Serial.println("VIS:PASS");
  Serial.println("VIS:CHECK");
  Serial.println("VIS:REJECT");
  Serial.println("==================================");
  Serial.println();
}

// =========================
// OLED DISPLAY
// =========================
void renderOLED(QCData data) {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("SMART QC STATION");

    display.setCursor(0, 10);
    display.print("ID:");
    String shortId = data.packageId;
    if (shortId.length() > 15) {
      shortId = shortId.substring(shortId.length() - 15);
    }
    display.println(shortId);

    display.setCursor(0, 20);
    display.print("QR:");
    display.print(data.qrStatus);
    display.print(" S:");
    display.println(data.side);

    display.setCursor(0, 30);
    display.print("SHK:");
    display.print(data.shockCount);
    display.print(" D:");
    display.print(data.dynamicG, 1);
    display.println("g");

    display.setCursor(0, 40);
    display.print("MAX:");
    display.print(data.maxDynamicG, 1);
    display.print(" T:");
    display.print(data.tiltAngle, 0);

    display.setCursor(0, 52);
    display.print("ST:");
    display.print(data.finalStatus);
    display.print(" V:");
    display.println(data.visualStatus);

    display.display();
    xSemaphoreGive(i2cMutex);
  }
}

void showBootScreen() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("SMART QC STATION");

    display.setCursor(0, 14);
    display.println("ESP32 READY");

    display.setCursor(0, 28);
    display.println("OLED READY");

    display.setCursor(0, 42);
    display.print("MPU:");
    display.print(mpuReady ? "OK " : "ERR ");

    display.print("WiFi:");
    display.println(WiFi.status() == WL_CONNECTED ? "OK" : "NO");

    display.setCursor(0, 56);
    display.println("Starting...");

    display.display();
    xSemaphoreGive(i2cMutex);
  }
}

// =========================
// I2C TOOLS
// =========================
bool i2cDeviceAvailable(uint8_t address) {
  bool ok = false;

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission(true);
    ok = (error == 0);
    xSemaphoreGive(i2cMutex);
  }

  return ok;
}

void scanI2C() {
  Serial.println("Scanning I2C...");

  int found = 0;

  for (uint8_t address = 1; address < 127; address++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission(true);
      xSemaphoreGive(i2cMutex);

      if (error == 0) {
        Serial.print("I2C device found at 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
        found++;
      }
    }
  }

  if (found == 0) {
    Serial.println("No I2C device found.");
  }

  Serial.println("I2C scan done.");
}

// =========================
// MPU6050 INIT
// =========================
bool initMPU6050() {
  if (mpu.begin(0x68, &Wire)) {
    mpuAddress = 0x68;
    Serial.println("MPU6050 detected at address 0x68");
  }
  else if (mpu.begin(0x69, &Wire)) {
    mpuAddress = 0x69;
    Serial.println("MPU6050 detected at address 0x69");
  }
  else {
    Serial.println("MPU6050 tidak terdeteksi.");
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

// =========================
// MPU6050 CALIBRATION
// =========================
bool calibrateMPU6050() {
  if (!mpuReady) {
    Serial.println("CAL failed: MPU6050 not ready.");
    return false;
  }

  Serial.println("Calibrating MPU6050. Diamkan sensor...");

  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;

  const int samples = 60;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    if (!i2cDeviceAvailable(mpuAddress)) {
      delay(10);
      continue;
    }

    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(80)) == pdTRUE) {
      mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
      xSemaphoreGive(i2cMutex);

      float ax = accelEvent.acceleration.x;
      float ay = accelEvent.acceleration.y;
      float az = accelEvent.acceleration.z;

      float accelG = vectorMagnitude(ax, ay, az) / GRAVITY_MS2;

      if (!isnan(accelG) && accelG >= 0.5f && accelG <= 1.8f) {
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        validSamples++;
      }
    }

    delay(10);
  }

  if (validSamples < 20) {
    Serial.println("Calibration failed: invalid MPU readings.");
    return false;
  }

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    baseAx = sumAx / validSamples;
    baseAy = sumAy / validSamples;
    baseAz = sumAz / validSamples;

    filtAx = baseAx;
    filtAy = baseAy;
    filtAz = baseAz;

    qcData.shockCount = 0;
    qcData.accelG = 1.0f;
    qcData.dynamicG = 0.0f;
    qcData.maxAccelG = 1.0f;
    qcData.maxDynamicG = 0.0f;
    qcData.tiltAngle = 0.0f;

    shockArmed = false;
    lastShockTime = millis();
    ignoreShockUntil = millis() + RESET_IGNORE_SHOCK_MS;

    updateDecision(qcData);
    xSemaphoreGive(dataMutex);
  }

  Serial.println("Calibration done.");
  return true;
}

// =========================
// READ MPU6050
// =========================
void readMPU6050Once() {
  if (!mpuReady) {
    return;
  }

  if (!i2cDeviceAvailable(mpuAddress)) {
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;

  bool readSuccess = false;

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    xSemaphoreGive(i2cMutex);
    readSuccess = true;
  }

  if (!readSuccess) {
    return;
  }

  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;

  float accelMag = vectorMagnitude(ax, ay, az);
  float accelG = accelMag / GRAVITY_MS2;
  float dynamicG = fabsf(accelG - 1.0f);

  if (isnan(accelG) || isnan(dynamicG) || accelG < 0.2f || accelG > 8.5f) {
    return;
  }

  unsigned long now = millis();

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    qcData.accelG = accelG;
    qcData.dynamicG = dynamicG;

    if (accelG > qcData.maxAccelG) {
      qcData.maxAccelG = accelG;
    }

    if (dynamicG > qcData.maxDynamicG) {
      qcData.maxDynamicG = dynamicG;
    }

    filtAx = (FILTER_ALPHA * filtAx) + ((1.0f - FILTER_ALPHA) * ax);
    filtAy = (FILTER_ALPHA * filtAy) + ((1.0f - FILTER_ALPHA) * ay);
    filtAz = (FILTER_ALPHA * filtAz) + ((1.0f - FILTER_ALPHA) * az);

    qcData.tiltAngle = angleBetweenVectors(
      filtAx, filtAy, filtAz,
      baseAx, baseAy, baseAz
    );

    if (now < ignoreShockUntil) {
      updateDecision(qcData);
      xSemaphoreGive(dataMutex);
      return;
    }

    if (dynamicG >= SHOCK_EVENT_DYNAMIC_G) {
      if (shockArmed && (now - lastShockTime >= SHOCK_DEBOUNCE_MS)) {
        qcData.shockCount++;
        lastShockTime = now;
        shockArmed = false;

        Serial.print("Shock detected. Count: ");
        Serial.print(qcData.shockCount);
        Serial.print(" | dynamicG: ");
        Serial.println(dynamicG, 2);
      }
    }

    if (dynamicG <= SHOCK_RELEASE_DYNAMIC_G) {
      shockArmed = true;
    }

    updateDecision(qcData);
    xSemaphoreGive(dataMutex);
  }
}

// =========================
// RESET
// =========================
void resetQCData() {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    qcData.packageId = "WAIT";
    qcData.qrStatus = "WAIT";
    qcData.side = "NONE";
    qcData.visualStatus = "WAIT";

    qcData.shockCount = 0;

    qcData.accelG = 1.0f;
    qcData.dynamicG = 0.0f;
    qcData.maxAccelG = 1.0f;
    qcData.maxDynamicG = 0.0f;

    qcData.tiltAngle = 0.0f;

    qcData.handlingStatus = "PASS";
    qcData.finalStatus = "READY";

    shockArmed = false;
    lastShockTime = millis();
    ignoreShockUntil = millis() + RESET_IGNORE_SHOCK_MS;

    updateDecision(qcData);
    xSemaphoreGive(dataMutex);
  }

  Serial.println("System reset. Shock ignored for 2 seconds.");
  printStatusToSerial();
}

// =========================
// WIFI
// =========================
void connectWiFi() {
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retry = 0;

  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    espIpAddress = WiFi.localIP().toString();

    Serial.println("WiFi connected.");
    Serial.print("ESP32 IP Address: ");
    Serial.println(espIpAddress);
  } else {
    Serial.println("WiFi connection failed.");
    espIpAddress = "NO_WIFI";
  }
}

void updateQCFromNetwork(String id, String qr, String side, String vis) {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (id.length() > 0) {
      qcData.packageId = id;
      qcData.packageId.trim();
    }

    if (qr.length() > 0) {
      qcData.qrStatus = qr;
      qcData.qrStatus.trim();
      qcData.qrStatus.toUpperCase();
    }

    if (side.length() > 0) {
      qcData.side = side;
      qcData.side.trim();
      qcData.side.toUpperCase();
    }

    if (vis.length() > 0) {
      qcData.visualStatus = vis;
      qcData.visualStatus.trim();
      qcData.visualStatus.toUpperCase();
    }

    updateDecision(qcData);
    xSemaphoreGive(dataMutex);
  }
}

// =========================
// HTTP SERVER
// =========================
void handleRoot() {
  String msg = "SMART QC ESP32 READY\n";
  msg += "IP: " + espIpAddress + "\n";
  msg += "Use:\n";
  msg += "/update?id=FRAGILE-001&qr=VALID&side=TOP&vis=PASS\n";
  msg += "/status\n";
  msg += "/reset\n";
  msg += "/cal\n";

  server.send(200, "text/plain", msg);
}

void handleUpdate() {
  String id = server.arg("id");
  String qr = server.arg("qr");
  String side = server.arg("side");
  String vis = server.arg("vis");

  updateQCFromNetwork(id, qr, side, vis);

  Serial.println("WiFi update received:");
  Serial.print("ID: "); Serial.println(id);
  Serial.print("QR: "); Serial.println(qr);
  Serial.print("SIDE: "); Serial.println(side);
  Serial.print("VIS: "); Serial.println(vis);

  server.send(200, "application/json", "{\"ok\":true,\"message\":\"updated\"}");
}

void handleResetHttp() {
  resetQCData();
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"reset\"}");
}

void handleCalHttp() {
  bool ok = calibrateMPU6050();

  if (ok) {
    server.send(200, "application/json", "{\"ok\":true,\"message\":\"calibrated\"}");
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"message\":\"calibration_failed\"}");
  }
}

void handleStatusHttp() {
  QCData data;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    data = qcData;
    xSemaphoreGive(dataMutex);
  }

  String json = "{";
  json += "\"id\":\""; json += data.packageId; json += "\",";
  json += "\"qr\":\""; json += data.qrStatus; json += "\",";
  json += "\"side\":\""; json += data.side; json += "\",";
  json += "\"visual\":\""; json += data.visualStatus; json += "\",";
  json += "\"shock\":"; json += String(data.shockCount); json += ",";
  json += "\"accelG\":"; json += String(data.accelG, 2); json += ",";
  json += "\"dynamicG\":"; json += String(data.dynamicG, 2); json += ",";
  json += "\"maxDynamicG\":"; json += String(data.maxDynamicG, 2); json += ",";
  json += "\"tilt\":"; json += String(data.tiltAngle, 1); json += ",";
  json += "\"handling\":\""; json += data.handlingStatus; json += "\",";
  json += "\"final\":\""; json += data.finalStatus; json += "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleI2CHttp() {
  String text = "I2C scan:\n";

  for (uint8_t address = 1; address < 127; address++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      Wire.beginTransmission(address);
      uint8_t error = Wire.endTransmission(true);
      xSemaphoreGive(i2cMutex);

      if (error == 0) {
        text += "Device found at 0x";
        if (address < 16) text += "0";
        text += String(address, HEX);
        text += "\n";
      }
    }
  }

  server.send(200, "text/plain", text);
}

void setupHttpServer() {
  server.on("/", handleRoot);
  server.on("/update", handleUpdate);
  server.on("/reset", handleResetHttp);
  server.on("/cal", handleCalHttp);
  server.on("/status", handleStatusHttp);
  server.on("/i2c", handleI2CHttp);

  server.begin();
  Serial.println("HTTP server started.");
}

// =========================
// SERIAL COMMAND
// =========================
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  Serial.print("CMD RECEIVED: ");
  Serial.println(cmd);

  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd == "HELP") {
    printHelp();
    return;
  }

  if (upperCmd == "RESTART") {
    Serial.println("Restarting ESP32...");
    delay(300);
    ESP.restart();
    return;
  }

  if (upperCmd == "RESET") {
    resetQCData();
    return;
  }

  if (upperCmd == "CAL") {
    calibrateMPU6050();
    printStatusToSerial();
    return;
  }

  if (upperCmd == "STATUS?") {
    printStatusToSerial();
    return;
  }

  if (upperCmd == "I2C?") {
    scanI2C();
    return;
  }

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (upperCmd.startsWith("ID:")) {
      qcData.packageId = cmd.substring(3);
      qcData.packageId.trim();
    }
    else if (upperCmd.startsWith("QR:")) {
      qcData.qrStatus = cmd.substring(3);
      qcData.qrStatus.trim();
      qcData.qrStatus.toUpperCase();
    }
    else if (upperCmd.startsWith("SIDE:")) {
      qcData.side = cmd.substring(5);
      qcData.side.trim();
      qcData.side.toUpperCase();
    }
    else if (upperCmd.startsWith("VIS:")) {
      qcData.visualStatus = cmd.substring(4);
      qcData.visualStatus.trim();
      qcData.visualStatus.toUpperCase();
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }

    updateDecision(qcData);
    xSemaphoreGive(dataMutex);
  } else {
    Serial.println("ERROR: command skipped, dataMutex busy.");
  }

  printStatusToSerial();
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    }
    else if (c >= 32 && c <= 126) {
      serialBuffer += c;

      if (serialBuffer.length() > 100) {
        Serial.println("Serial buffer cleared: command too long.");
        serialBuffer = "";
      }
    }
  }
}

// =========================
// RTOS TASKS
// =========================
void taskMPU(void *parameter) {
  while (true) {
    readMPU6050Once();
    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

void taskOLED(void *parameter) {
  while (true) {
    QCData snapshot;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snapshot = qcData;
      xSemaphoreGive(dataMutex);
      renderOLED(snapshot);
    }

    vTaskDelay(pdMS_TO_TICKS(400));
  }
}

void taskSerial(void *parameter) {
  while (true) {
    readSerialCommand();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void taskWiFi(void *parameter) {
  while (true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(115200);
  delay(800);

  esp_log_level_set("Wire", ESP_LOG_NONE);
  esp_log_level_set("i2c", ESP_LOG_NONE);

  Serial.println();
  Serial.println("BOOTING SMART QC STATION...");

  dataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED tidak terdeteksi.");
    Serial.println("Cek OLED SDA=21, SCL=22, VCC=3V3, GND=GND.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("OLED detected.");

  mpuReady = initMPU6050();

  connectWiFi();

  showBootScreen();
  delay(1200);

  if (mpuReady) {
    calibrateMPU6050();
  }

  setupHttpServer();

  Serial.println("====================================");
  Serial.println("SMART FRAGILE GOODS QC READY");
  Serial.print("ESP32 IP: ");
  Serial.println(espIpAddress);
  Serial.println("Ketik HELP lalu tekan ENTER.");
  Serial.println("====================================");

  printHelp();

  xTaskCreatePinnedToCore(
    taskSerial,
    "Task Serial",
    4096,
    NULL,
    4,
    &taskSerialHandle,
    0
  );

  xTaskCreatePinnedToCore(
    taskWiFi,
    "Task WiFi",
    4096,
    NULL,
    3,
    &taskWiFiHandle,
    0
  );

  xTaskCreatePinnedToCore(
    taskMPU,
    "Task MPU6050",
    4096,
    NULL,
    2,
    &taskMPUHandle,
    1
  );

  xTaskCreatePinnedToCore(
    taskOLED,
    "Task OLED",
    4096,
    NULL,
    1,
    &taskOLEDHandle,
    1
  );
}

// =========================
// LOOP
// =========================
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}