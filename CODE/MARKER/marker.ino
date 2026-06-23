#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ================= OLED =================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= ADXL345 (I2C2) =================
#define ADXL_ADDR 0x53
TwoWire I2C_ADXL(PB11, PB10);   // SDA, SCL

// ================= PIN =================
#define LED_PIN       PB0
#define BUZZER_PIN    PB1
#define BUTTON_PIN    PA0
#define BATTERY_PIN   PA2

#define NRF_CE_PIN    PA3
#define NRF_CSN_PIN   PA4

// ================= RF24 =================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "MKR01";

// ================= CONFIG =================
const uint8_t MARKER_ID = 1;

uint8_t markerStatus = 0;

bool ledState = false;
bool radioAvailable = false;
bool lastSendResult = false;
bool adxlAvailable = false;
bool isSleeping = false;

unsigned long previousLedMillis = 0;
unsigned long previousDisplayMillis = 0;
unsigned long previousSendMillis = 0;
unsigned long previousBatteryMillis = 0;
unsigned long lastDebounceTime = 0;
unsigned long bootTime = 0;
unsigned long lastImpactMillis = 0;
unsigned long rescuedTime = 0;

bool lastButtonReading = HIGH;
bool buttonState = HIGH;

const unsigned long ledInterval = 500;
const unsigned long displayInterval = 200;
const unsigned long sendInterval = 1000;
const unsigned long batteryUpdateInterval = 250;
const unsigned long debounceDelay = 50;
const unsigned long impactCooldown = 1500;
const unsigned long rescuedToSleepDelay = 2000;

// double click
int clickCount = 0;
unsigned long lastClickTime = 0;
const unsigned long doubleClickWindow = 400;

// impact detect
int impactCount = 0;
const long impactDeltaThreshold = 520;
const int impactNeedCount = 4;

long baseMagnitude = 0;
bool baseReady = false;

// ================= BATTERY =================
float batteryVoltageShown = -1.0f;
float batteryVoltageRaw = -1.0f;
float batteryVoltageFiltered = -1.0f;

const float batteryCalibrationFactor = 0.001607f;

// chỉnh cho OLED cập nhật tụt áp nhanh hơn
const float batteryTinyDeadband = 0.002f;
const float maxBatteryDropPerUpdate = 0.080f;
const float maxBatteryRisePerUpdate = 0.020f;

// ================= BUZZER =================
const unsigned long buzzerCycle = 2000;

const unsigned long beep1Start = 0;
const unsigned long beep1End   = 150;

const unsigned long beep2Start = 300;
const unsigned long beep2End   = 450;

const unsigned long beep3Start = 600;
const unsigned long beep3End   = 750;

const int freq1 = 2500;
const int freq2 = 3000;
const int freq3 = 3500;

// ================= RF PAYLOAD =================
struct __attribute__((packed)) MarkerPacket {
  uint8_t id;
  uint8_t status;
  float battery;
  uint32_t counter;
};

MarkerPacket packet;
uint32_t packetCounter = 0;

// ================= ADXL DATA =================
int16_t ax = 0, ay = 0, az = 0;

// ================= FORWARD DECLARATIONS =================
void sendMarkerPacket(float batteryVoltage);
void enterSleepMode();
void wakeFromSleep();
bool isBuzzerActive(unsigned long currentMillis);

// ================= HELPER =================
const char* getStatusText(uint8_t st) {
  if (st == 0) return "WAITING";
  if (st == 1) return "NOT RESCUED";
  if (st == 2) return "RESCUED";
  return "UNKNOWN";
}

// ================= ADXL345 =================
void adxlWrite(uint8_t reg, uint8_t value) {
  I2C_ADXL.beginTransmission(ADXL_ADDR);
  I2C_ADXL.write(reg);
  I2C_ADXL.write(value);
  I2C_ADXL.endTransmission();
}

bool adxlReadXYZ(int16_t &x, int16_t &y, int16_t &z) {
  I2C_ADXL.beginTransmission(ADXL_ADDR);
  I2C_ADXL.write(0x32);
  if (I2C_ADXL.endTransmission(false) != 0) return false;

  I2C_ADXL.requestFrom(ADXL_ADDR, 6);
  if (I2C_ADXL.available() < 6) return false;

  x = I2C_ADXL.read() | (I2C_ADXL.read() << 8);
  y = I2C_ADXL.read() | (I2C_ADXL.read() << 8);
  z = I2C_ADXL.read() | (I2C_ADXL.read() << 8);

  return true;
}

bool initADXL345() {
  I2C_ADXL.begin();

  adxlWrite(0x2D, 0x08);
  adxlWrite(0x31, 0x0B);

  delay(20);

  int16_t tx, ty, tz;
  return adxlReadXYZ(tx, ty, tz);
}

void adxlSleep() {
  if (!adxlAvailable) return;
  adxlWrite(0x2D, 0x00);
}

void adxlWake() {
  if (!adxlAvailable) return;
  adxlWrite(0x2D, 0x08);
  delay(10);
}

// ================= BATTERY =================
float readBatteryBlockTrimmedMean(int sampleCount, int adcDelayMs) {
  if (sampleCount < 5) sampleCount = 5;

  int buf[80];
  if (sampleCount > 80) sampleCount = 80;

  for (int i = 0; i < sampleCount; i++) {
    buf[i] = analogRead(BATTERY_PIN);
    delay(adcDelayMs);
  }

  for (int i = 0; i < sampleCount - 1; i++) {
    for (int j = i + 1; j < sampleCount; j++) {
      if (buf[j] < buf[i]) {
        int t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }

  int cut = sampleCount / 5;
  if (cut < 1) cut = 1;
  if (cut * 2 >= sampleCount) cut = 1;

  long sum = 0;
  int count = 0;

  for (int i = cut; i < sampleCount - cut; i++) {
    sum += buf[i];
    count++;
  }

  float rawAvg = sum / (float)count;
  return rawAvg * batteryCalibrationFactor;
}

float readBatteryVoltageStableSnapshot() {
  const int rounds = 7;
  float values[rounds];

  for (int r = 0; r < rounds; r++) {
    values[r] = readBatteryBlockTrimmedMean(25, 2);
    delay(12);
  }

  for (int i = 0; i < rounds - 1; i++) {
    for (int j = i + 1; j < rounds; j++) {
      if (values[j] < values[i]) {
        float t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  return values[rounds / 2];
}

void initBatteryFilter() {
  float v = readBatteryVoltageStableSnapshot();
  batteryVoltageRaw = v;
  batteryVoltageFiltered = v;
  batteryVoltageShown = v;
}

bool isBuzzerActive(unsigned long currentMillis) {
  if (isSleeping || markerStatus != 1) return false;

  unsigned long phase = currentMillis % buzzerCycle;

  return ((phase >= beep1Start && phase < beep1End) ||
          (phase >= beep2Start && phase < beep2End) ||
          (phase >= beep3Start && phase < beep3End));
}

void updateBatteryShown(unsigned long now) {
  if (batteryVoltageShown < 0.0f) {
    initBatteryFilter();
    return;
  }

  if (now - previousBatteryMillis < batteryUpdateInterval) return;
  previousBatteryMillis = now;

  float newRaw = readBatteryBlockTrimmedMean(15, 1);
  batteryVoltageRaw = newRaw;

  float diff = newRaw - batteryVoltageFiltered;

  if (fabs(diff) <= batteryTinyDeadband) {
    return;
  }

  if (diff > 0) {
    float rise = diff * 0.25f;
    if (rise > maxBatteryRisePerUpdate) rise = maxBatteryRisePerUpdate;
    batteryVoltageFiltered += rise;
  } else {
    float drop = -diff;
    float appliedDrop = drop * 0.70f;
    if (appliedDrop > maxBatteryDropPerUpdate) appliedDrop = maxBatteryDropPerUpdate;
    batteryVoltageFiltered -= appliedDrop;
  }

  batteryVoltageShown = batteryVoltageFiltered;
}

// ================= DISPLAY =================
void updateDisplay(float batteryVoltage, long sumAbs, bool adxlOk) {
  if (isSleeping) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Marker: ");
  display.print(MARKER_ID);

  display.setCursor(0, 18);
  display.print("Status: ");
  display.print(getStatusText(markerStatus));

  display.setCursor(0, 30);
  display.print("BAT: ");
  display.print(batteryVoltage, 2);
  display.print(" V");

  display.setCursor(0, 42);
  display.print("RF: ");
  if (!radioAvailable) {
    display.print("Not Found");
  } else {
    display.print(lastSendResult ? "OK" : "NO ACK");
  }

  display.setCursor(0, 54);
  if (!adxlOk) {
    display.print("ADXL: ERROR");
  } else {
    display.print("SUM: ");
    display.print(sumAbs);
  }

  display.display();
}

void showSleepScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SLEEP MODE");
  display.setCursor(0, 18);
  display.println("Double click");
  display.setCursor(0, 30);
  display.println("to wake");
  display.display();
}

// ================= RF =================
void sendMarkerPacket(float batteryVoltage) {
  if (!radioAvailable || isSleeping) return;

  packet.id = MARKER_ID;
  packet.status = markerStatus;
  packet.battery = batteryVoltage;
  packet.counter = packetCounter++;

  lastSendResult = radio.write(&packet, sizeof(packet));
}

void initNRF24() {
  if (!radio.begin()) {
    radioAvailable = false;
    return;
  }

  radioAvailable = true;
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.openWritingPipe(address);
  radio.stopListening();
}

// ================= ALERT =================
void updateBuzzer(unsigned long currentMillis) {
  if (isSleeping || markerStatus != 1) {
    noTone(BUZZER_PIN);
    return;
  }

  unsigned long phase = currentMillis % buzzerCycle;

  if (phase >= beep1Start && phase < beep1End) {
    tone(BUZZER_PIN, freq1);
  }
  else if (phase >= beep2Start && phase < beep2End) {
    tone(BUZZER_PIN, freq2);
  }
  else if (phase >= beep3Start && phase < beep3End) {
    tone(BUZZER_PIN, freq3);
  }
  else {
    noTone(BUZZER_PIN);
  }
}

void updateLed(unsigned long currentMillis) {
  if (isSleeping) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
    return;
  }

  if (markerStatus == 1) {
    if (currentMillis - previousLedMillis >= ledInterval) {
      previousLedMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

// ================= BUTTON =================
void registerButtonClick() {
  unsigned long now = millis();

  if (now - lastClickTime <= doubleClickWindow) {
    clickCount++;
  } else {
    clickCount = 1;
  }

  lastClickTime = now;
}

void handleButtonRaw() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        registerButtonClick();
      }
    }
  }

  lastButtonReading = reading;
}

void processButtonClicks() {
  if (clickCount == 0) return;
  if (millis() - lastClickTime <= doubleClickWindow) return;

  if (isSleeping) {
    if (clickCount >= 2) {
      wakeFromSleep();
    }
    clickCount = 0;
    return;
  }

  if (markerStatus == 1) {
    if (clickCount == 1) {
      markerStatus = 2;
      rescuedTime = millis();
      sendMarkerPacket(batteryVoltageShown);
    }
  }

  clickCount = 0;
}

// ================= SLEEP =================
void enterSleepMode() {
  if (isSleeping) return;

  isSleeping = true;

  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  ledState = false;

  if (radioAvailable) {
    radio.powerDown();
  }

  adxlSleep();

  showSleepScreen();
  delay(800);

  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void wakeFromSleep() {
  if (!isSleeping) return;

  isSleeping = false;

  display.ssd1306_command(SSD1306_DISPLAYON);
  delay(50);

  if (radioAvailable) {
    radio.powerUp();
    delay(10);
    radio.stopListening();
  }

  adxlWake();

  markerStatus = 0;
  bootTime = millis();
  lastImpactMillis = 0;
  clickCount = 0;
  impactCount = 0;
  baseReady = false;
  baseMagnitude = 0;

  initBatteryFilter();

  sendMarkerPacket(batteryVoltageShown);
  updateDisplay(batteryVoltageShown, 0, adxlAvailable);
}

// ================= DROP DETECTION =================
long calcMagnitudeAbs(int16_t x, int16_t y, int16_t z) {
  return abs(x) + abs(y) + abs(z);
}

void updateDropDetection(unsigned long now, bool adxlOk, long sumAbs) {
  if (isSleeping) return;
  if (!adxlOk) return;

  // bỏ qua lúc mới bật nguồn
  if (now - bootTime < 2500) return;

  // chỉ phát hiện khi đang WAITING
  if (markerStatus != 0) return;

  // tránh kích hoạt liên tục
  if (now - lastImpactMillis < impactCooldown) return;

  // tạo giá trị nền ban đầu theo tư thế thực tế của marker
  if (!baseReady) {
    baseMagnitude = sumAbs;
    baseReady = true;
    impactCount = 0;
    return;
  }

  long delta = labs(sumAbs - baseMagnitude);

  // cập nhật nền chậm để thích nghi với tư thế nằm dọc/ngang
  baseMagnitude = (baseMagnitude * 95 + sumAbs * 5) / 100;

  if (delta > impactDeltaThreshold) {
    impactCount++;
  } else {
    if (impactCount > 0) impactCount--;
  }

  if (impactCount >= impactNeedCount) {
    markerStatus = 1;
    lastImpactMillis = now;
    impactCount = 0;

    sendMarkerPacket(batteryVoltageShown);
  }
}

// ================= SETUP =================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  analogReadResolution(12);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Init system...");
  display.display();

  initNRF24();
  adxlAvailable = initADXL345();

  markerStatus = 0;
  isSleeping = false;
  bootTime = millis();
  rescuedTime = 0;
  impactCount = 0;

  initBatteryFilter();
  delay((MARKER_ID - 1) * 300);
  previousSendMillis = millis();

  sendMarkerPacket(batteryVoltageShown);

  long sumAbs = 0;
  if (adxlAvailable && adxlReadXYZ(ax, ay, az)) {
    sumAbs = abs(ax) + abs(ay) + abs(az);
  }

  updateDisplay(batteryVoltageShown, sumAbs, adxlAvailable);
}

// ================= LOOP =================
void loop() {
  unsigned long currentMillis = millis();

  handleButtonRaw();
  processButtonClicks();

  if (isSleeping) {
    delay(30);
    return;
  }

  bool adxlOk = false;
  long sumAbs = 0;

  if (adxlAvailable) {
    adxlOk = adxlReadXYZ(ax, ay, az);
    if (adxlOk) {
      sumAbs = abs(ax) + abs(ay) + abs(az);
      updateDropDetection(currentMillis, adxlOk, sumAbs);
    }
  }

  updateLed(currentMillis);
  updateBuzzer(currentMillis);

  updateBatteryShown(currentMillis);

  if (markerStatus == 2 && rescuedTime > 0) {
    if (currentMillis - rescuedTime >= rescuedToSleepDelay) {
      enterSleepMode();
      delay(30);
      return;
    }
  }

  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;
    sendMarkerPacket(batteryVoltageShown);
  }

  if (currentMillis - previousDisplayMillis >= displayInterval) {
    previousDisplayMillis = currentMillis;
    updateDisplay(batteryVoltageShown, sumAbs, adxlOk);
  }

  delay(10);
}