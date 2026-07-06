/*
 * XIAO ESP32-C3  +  MPU6050  ->  BLE orientation + position streamer
 * -------------------------------------------------------------------
 * Reads the MPU6050 over the XIAO's built-in I2C bus, runs a Madgwick
 * IMU filter for orientation, reads an LDR light sensor, and streams
 * both over BLE to a browser using the Web Bluetooth API.
 *
 * Board:   Seeed Studio XIAO ESP32-C3   (Bluetooth LE only)
 * Sensor:  MPU6050 (GY-521), I2C address 0x68
 *
 * Wiring (XIAO ESP32-C3 default I2C):
 *   MPU6050 VCC -> 3V3
 *   MPU6050 GND -> GND
 *   MPU6050 SDA -> D4  (GPIO6)
 *   MPU6050 SCL -> D5  (GPIO7)
 *
 * Recalibration button:
 *   Wire a momentary button from D9 (GPIO9) to GND. GPIO9 uses the
 *   internal pull-up, so it reads HIGH normally and LOW when pressed.
 *   Pressing it re-measures gyro/accel bias (keep the board still and
 *   level!) and saves the result to flash (NVS). Calibration is loaded
 *   automatically on every boot.
 *
 * LDR (light sensor):
 *   Analog input on D2 (GPIO4 / ADC1). Wire as a voltage divider:
 *     3V3 ---[ LDR ]---+---[ 10k ]--- GND
 *                      |
 *                     D2
 *   Reads brighter -> higher value. Streamed normalized to 0.0 .. 1.0.
 *
 * Power / deep-sleep button:
 *   Wire a momentary button from D1 (GPIO3) to GND (internal pull-up,
 *   active-low). Single button, several behaviors:
 *     - Awake, HOLD 5 s     : enter deep sleep (~uA current draw).
 *     - Awake, DOUBLE-CLICK : recalibrate the IMU after a 3 s delay
 *                             (time to place it on a flat, stable surface).
 *     - Asleep, single CLICK: wakes the chip (it restarts fresh).
 *   D1/GPIO3 is used because on the ESP32-C3 only GPIO0-5 can wake the
 *   chip from deep sleep, and GPIO3 (unlike GPIO2) is not a boot
 *   strapping pin, so a wake press can never disturb boot.
 *
 * Arduino IDE setup:
 *   - Boards Manager: install "esp32" by Espressif.
 *   - Select board:   "XIAO_ESP32C3".
 *   - No extra libraries required.
 *
 * Data sent to the browser (little-endian):
 *   21 bytes = 5 float32 (quaternion[w,x,y,z] + light[0..1]) + 1 byte
 *   single-click counter. The counter is bumped on each D1 single-click; the
 *   drone simulator uses it (LEFT board = arm/disarm, RIGHT board = liftoff/
 *   land). Viewers that read only the first 20 bytes are unaffected.
 */

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>   // memcpy for the BLE packet

// ---------- Controller identity (must match the website) ----------
// Both boards run this same firmware. Flash ONE as LEFT and ONE as RIGHT by
// setting CONTROLLER_SIDE below, so each advertises a unique name + UUIDs and
// the web app can target them individually.
#define SIDE_LEFT   0
#define SIDE_RIGHT  1
#define CONTROLLER_SIDE  SIDE_LEFT      // <-- set to SIDE_RIGHT for the other board

#if CONTROLLER_SIDE == SIDE_LEFT
  #define DEVICE_NAME         "XIAO-IMU-L"
  #define SERVICE_UUID        "12345678-1234-5678-1234-56789abcde10"
  #define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcde11"
#else
  #define DEVICE_NAME         "XIAO-IMU-R"
  #define SERVICE_UUID        "12345678-1234-5678-1234-56789abcde20"
  #define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcde21"
#endif

// ---------- Recalibration button ----------
#define RECAL_PIN 9          // D9 / GPIO9, active-low to GND

// ---------- Power / deep-sleep button (D1) ----------
// One switch, three gestures while awake:
//   - single click         : bump the click counter -> sends a click event
//                            (sim: LEFT board = arm/disarm, RIGHT = liftoff/land)
//   - double click         : recalibrate the IMU after RECAL_DELAY_MS
//   - hold POWER_HOLD_MS   : enter deep sleep
// (While asleep, any click wakes the chip via hardware.)
#define POWER_PIN        3      // D1 / GPIO3, active-low, deep-sleep-wake capable
#define POWER_HOLD_MS    5000   // hold this long to enter deep sleep
#define CLICK_MAX_MS     600    // a press shorter than this counts as a "click"
#define DOUBLE_PRESS_MS  700    // max gap between the two clicks of a double-press
#define RECAL_DELAY_MS   3000   // wait after a double-press, then calibrate

// ---------- I2C pins ----------
// Normal:  SDA=6 (D4), SCL=7 (D5).  To test a swapped bus, flip these to 7/6.
#define I2C_SDA 7
#define I2C_SCL 6

// ---------- LDR light sensor ----------
#define LDR_PIN 4            // D2 / GPIO4, ADC1 analog input

// Set to 1 to print live accel/gyro/quaternion to the Serial Monitor.
#define DEBUG_IMU 0

// ---------- MPU6050 registers ----------
// Valid MPU6050 addresses are ONLY 0x68 (AD0 low/floating) or 0x69 (AD0 high).
// This is auto-detected in setup(); the value here is just the starting guess.
static uint8_t       MPU_ADDR      = 0x68;
static bool          mpuOK         = false;   // true once the chip is talking
static const uint8_t REG_WHO_AM_I  = 0x75;
static const uint8_t REG_PWR_MGMT1 = 0x6B;
static const uint8_t REG_GYRO_CFG  = 0x1B;
static const uint8_t REG_ACCEL_CFG = 0x1C;
static const uint8_t REG_ACCEL_OUT = 0x3B;   // start of 14-byte burst

// Full-scale sensitivities for the default ranges (±2g, ±250 deg/s)
static const float ACCEL_LSB_PER_G  = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float DEG2RAD          = 0.017453292519943295f;

// ---------- Madgwick filter state ----------
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // orientation quaternion
static const float beta = 0.1f;                            // filter gain

// ---------- Calibration (persisted in NVS) ----------
static float gxBias = 0, gyBias = 0, gzBias = 0;   // gyro bias, deg/s
static float axBias = 0, ayBias = 0, azBias = 0;   // accel bias, g
Preferences prefs;

// ---------- Timing ----------
static uint32_t lastMicros = 0;

// ---------- BLE globals ----------
BLEServer*         pServer   = nullptr;
BLECharacteristic* pChar     = nullptr;
volatile bool      connected = false;
volatile uint8_t   clickSeq  = 0;   // ++ on each confirmed D1 single-click; sent in the packet

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override    { connected = true; }
  void onDisconnect(BLEServer* s) override {
    connected = false;
    BLEDevice::startAdvertising();          // allow the browser to reconnect
  }
};

// ---------------------------------------------------------------
//  I2C helpers
// ---------------------------------------------------------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Does something ACK at this address?
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Print every address that responds on the bus.
void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPing(a)) { Serial.printf("  found 0x%02X\n", a); found++; }
  }
  if (!found) Serial.println("  (nothing responded - check wiring/power)");
}

// Find the MPU6050 at 0x68 or 0x69 and wake it. Sets mpuOK.
void initMPU() {
  const uint8_t candidates[] = { 0x68, 0x69 };
  mpuOK = false;
  for (uint8_t a : candidates) {
    if (i2cPing(a)) { MPU_ADDR = a; mpuOK = true; break; }
  }

  if (!mpuOK) {
    Serial.println("!! No MPU6050 at 0x68/0x69. IMU disabled.");
    scanI2C();
    return;
  }

  uint8_t who = mpuReadByte(REG_WHO_AM_I);
  Serial.printf("MPU found at 0x%02X, WHO_AM_I=0x%02X\n", MPU_ADDR, who);

  mpuWrite(REG_PWR_MGMT1, 0x00);   // exit sleep
  delay(100);
  mpuWrite(REG_GYRO_CFG,  0x00);   // ±250 deg/s
  mpuWrite(REG_ACCEL_CFG, 0x00);   // ±2 g
  delay(100);
}

// Read accel (g) and gyro (deg/s) with calibration biases removed.
// Returns false if the sensor didn't return a full sample (all outputs are
// then left at a safe "flat and still" reading so the filter never sees NaN).
bool readIMU(float& ax, float& ay, float& az,
             float& gx, float& gy, float& gz) {
  ax = ay = 0; az = 1.0f;      // safe default: flat, +1g on Z
  gx = gy = gz = 0;

  if (!mpuOK) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_OUT);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // temperature (ignored)
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / ACCEL_LSB_PER_G - axBias;
  ay = rawAy / ACCEL_LSB_PER_G - ayBias;
  az = rawAz / ACCEL_LSB_PER_G - azBias;

  gx = rawGx / GYRO_LSB_PER_DPS - gxBias;
  gy = rawGy / GYRO_LSB_PER_DPS - gyBias;
  gz = rawGz / GYRO_LSB_PER_DPS - gzBias;
  return true;
}

// ---------------------------------------------------------------
//  Persistent calibration (NVS via Preferences)
// ---------------------------------------------------------------
void loadCalibration() {
  prefs.begin("imucal", true);              // read-only
  gxBias = prefs.getFloat("gx", 0);
  gyBias = prefs.getFloat("gy", 0);
  gzBias = prefs.getFloat("gz", 0);
  axBias = prefs.getFloat("ax", 0);
  ayBias = prefs.getFloat("ay", 0);
  azBias = prefs.getFloat("az", 0);
  prefs.end();
  Serial.printf("Loaded cal  gyro[%.3f %.3f %.3f]  accel[%.3f %.3f %.3f]\n",
                gxBias, gyBias, gzBias, axBias, ayBias, azBias);
}

void saveCalibration() {
  prefs.begin("imucal", false);             // read-write
  prefs.putFloat("gx", gxBias); prefs.putFloat("gy", gyBias); prefs.putFloat("gz", gzBias);
  prefs.putFloat("ax", axBias); prefs.putFloat("ay", ayBias); prefs.putFloat("az", azBias);
  prefs.end();
  Serial.println("Calibration saved to flash.");
}

// Measure biases with the board STILL and LEVEL, then persist them.
// Assumes gravity reads +1g on the Z axis (board flat).
void calibrate() {
  Serial.println("Calibrating - keep the board still and level...");
  gxBias = gyBias = gzBias = 0;             // read raw during calibration
  axBias = ayBias = azBias = 0;

  const int N = 600;
  double sgx = 0, sgy = 0, sgz = 0, sax = 0, say = 0, saz = 0;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sgx += gx; sgy += gy; sgz += gz;
    sax += ax; say += ay; saz += az;
    delay(3);
  }
  gxBias = sgx / N; gyBias = sgy / N; gzBias = sgz / N;
  axBias = sax / N; ayBias = say / N; azBias = saz / N - 1.0f;  // remove +1g

  // Zero the orientation: the current (flat) pose becomes the new reference,
  // so roll/pitch/yaw and the cube read 0 right after calibrating.
  q0 = 1.0f; q1 = q2 = q3 = 0.0f;

  saveCalibration();
  Serial.println("Calibration complete.");
}

// ---------------------------------------------------------------
//  Madgwick IMU update (gyro in rad/s, accel in any consistent unit)
// ---------------------------------------------------------------
static inline float invSqrt(float x) { return 1.0f / sqrtf(x); }

void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // Only apply the accel correction when the vector is a sane magnitude.
  // A near-zero norm (e.g. a dead/garbage read) would blow up invSqrt.
  float aNorm2 = ax * ax + ay * ay + az * az;
  if (aNorm2 > 0.01f) {
    recipNorm = invSqrt(aNorm2);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

  // If anything ever went non-finite, snap back to identity so the stream
  // recovers instead of latching NaN forever.
  if (isnan(q0) || isinf(q0)) { q0 = 1.0f; q1 = q2 = q3 = 0.0f; }
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(RECAL_PIN, INPUT_PULLUP);
  pinMode(POWER_PIN, INPUT_PULLUP);
  analogReadResolution(12);          // LDR on ADC1, 0..4095

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Woke from deep sleep (D1 press).");
  }

  // I2C on the XIAO's SDA/SCL (see I2C_SDA / I2C_SCL defines above).
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // Auto-detect and wake the MPU6050 (0x68 or 0x69).
  initMPU();

  loadCalibration();
  // If the sensor is present and nothing was ever stored, calibrate once.
  if (mpuOK && gxBias == 0 && gyBias == 0 && azBias == 0) {
    calibrate();
  }

  // --- BLE ---
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(64);   // allow >20-byte notifies (packet is now 21 bytes)
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pChar = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());       // enables notifications
  pService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as \"" DEVICE_NAME "\". Waiting for a browser...");
  Serial.println("Recalibrate: double-press D1 (3 s delay to set it down), or press D9.");
  Serial.println("Hold the D1 button 5 s to enter deep sleep.");
  lastMicros = micros();
}

// Debounced check of the D9 recalibration button.
void checkRecalButton() {
  if (digitalRead(RECAL_PIN) == LOW) {
    delay(30);                               // debounce
    if (digitalRead(RECAL_PIN) == LOW) {
      if (mpuOK) calibrate();
      else Serial.println("Recalibrate ignored - IMU not detected.");
      while (digitalRead(RECAL_PIN) == LOW) delay(10);  // wait for release
      lastMicros = micros();                 // discard the long dt from calibration
    }
  }
}

// Power down. Arms a GPIO wake on the D1 button (active-low) so the next
// click restarts the chip. Does not return.
void enterDeepSleep() {
  Serial.println("Entering deep sleep. Click D1 to wake.");
  Serial.flush();

  // Don't sleep while the button is still held, or we'd wake immediately.
  while (digitalRead(POWER_PIN) == LOW) delay(10);
  delay(50);

  // Keep the pull-up alive in the always-on domain so the pin idles HIGH
  // (floating would cause spurious wakes), then wake when it is pulled LOW.
  gpio_pullup_en((gpio_num_t)POWER_PIN);
  gpio_pulldown_dis((gpio_num_t)POWER_PIN);
  esp_deep_sleep_enable_gpio_wakeup(1ULL << POWER_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_deep_sleep_start();                    // <- chip stops here
}

// Non-blocking state machine for the D1 switch. Handles hold-to-sleep and
// double-click-to-recalibrate; streaming keeps running throughout (including
// during the 3 s pre-calibration delay, so the user can set the board down).
void checkPowerButton() {
  static bool     wasDown      = false;
  static uint32_t pressStart   = 0;
  static uint32_t lastClickAt  = 0;
  static uint8_t  clickCount   = 0;
  static bool     recalPending = false;
  static uint32_t recalAt      = 0;

  uint32_t now  = millis();
  bool     down = (digitalRead(POWER_PIN) == LOW);

  if (down && !wasDown) pressStart = now;                 // press edge

  // Held long enough -> deep sleep (never returns).
  if (down && (now - pressStart >= POWER_HOLD_MS)) enterDeepSleep();

  // Release edge: classify the press.
  if (!down && wasDown) {
    if (now - pressStart < CLICK_MAX_MS) {                // it was a click
      clickCount = (now - lastClickAt <= DOUBLE_PRESS_MS) ? clickCount + 1 : 1;
      lastClickAt = now;
      if (clickCount >= 2) {                              // double-press!
        clickCount = 0;
        recalPending = true;
        recalAt = now + RECAL_DELAY_MS;
        Serial.println("Double-press: recalibrating in 3 s - place on a flat, stable surface.");
      }
    }
  }
  wasDown = down;

  // A lone click with no second press inside the double-press window is a
  // single click -> bump clickSeq so the browser sees a one-shot click event.
  // (This fires ~DOUBLE_PRESS_MS after release, so it never overlaps a double-
  // click, which is still reserved for recalibration.)
  if (clickCount == 1 && (now - lastClickAt > DOUBLE_PRESS_MS)) {
    clickCount = 0;
    clickSeq++;
    Serial.println("Single click -> click event sent.");
  }

  // Fire the delayed recalibration once the 3 s window elapses.
  if (recalPending && (int32_t)(now - recalAt) >= 0) {
    recalPending = false;
    if (mpuOK) calibrate();
    else Serial.println("Recalibrate ignored - IMU not detected.");
    lastMicros = micros();                                // discard the long dt
  }
}

// ---------------------------------------------------------------
//  Loop  (~100 Hz filter update, ~50 Hz BLE notify)
// ---------------------------------------------------------------
void loop() {
  checkRecalButton();
  checkPowerButton();

  float ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);

  uint32_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;      // guard against glitches

  madgwickUpdate(gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD, ax, ay, az, dt);

  // Set DEBUG_IMU to 1 to print live readings (~4 Hz) to Serial Monitor.
#if DEBUG_IMU
  static uint32_t lastDbg = 0;
  if (now - lastDbg > 250000) {
    lastDbg = now;
    Serial.printf("A[% .2f % .2f % .2f]g  G[% .1f % .1f % .1f]dps  q[% .2f % .2f % .2f % .2f]\n",
                  ax, ay, az, gx, gy, gz, q0, q1, q2, q3);
  }
#endif

  // Notify at ~50 Hz to keep BLE happy.
  static uint32_t lastSend = 0;
  if (connected && (now - lastSend) > 20000) {
    lastSend = now;
    float light = analogRead(LDR_PIN) / 4095.0f;       // 0.0 (dark) .. 1.0 (bright)
    // 21-byte packet: quaternion + light (20 B) then the single-click counter.
    // Old viewers that read only the first 20 bytes keep working unchanged.
    uint8_t packet[21];
    float f[5] = { q0, q1, q2, q3, light };
    memcpy(packet, f, 20);
    packet[20] = clickSeq;
    pChar->setValue(packet, sizeof(packet));
    pChar->notify();
  }

  delay(5);   // ~200 Hz sensor read ceiling
}
