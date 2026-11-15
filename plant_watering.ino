// ESP32 Automatic Plant Watering
// Soil moisture + L298N + pump, BLE UART for the VerdeFlow app (+ optional Telegram alerts)
// Water level sensor removed. Auto irrigation is the main mode, manual pump control is optional.

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ================= Pins (unchanged)
// NOTE: GPIO4 is an ADC2 pin. It works as long as WiFi is OFF (ENABLE_TELEGRAM = false).
// If you turn WiFi on, move the soil sensor wire to an ADC1 pin (32-39) and change SOIL_PIN.
const int SOIL_PIN = 4;    // soil moisture sensor AO
const int PUMP_PIN = 23;   // L298N IN1 (IN2 -> GND, ENA jumper kept on)

// ================= Mode
// true  = only print raw readings, pump never runs
// false = normal operation (auto irrigation)
const bool CALIBRATION_MODE = false;

// ================= Calibration defaults (from your readings)
// These are only the first-boot defaults. After that, the saved values are used.
// Change them from the Serial Monitor with the commands "dry" and "wet" (no re-upload needed).
const int DEFAULT_SOIL_DRY = 3650;  // raw reading in dry soil
const int DEFAULT_SOIL_WET = 2220;  // raw reading in wet soil
const int DEFAULT_THRESHOLD_PCT = 35;

// ================= Behavior
const unsigned long CHECK_INTERVAL_MS = 3000;   // measure every 3 seconds
const unsigned long PUMP_TIME_MS      = 5000;    // pump runs 5 seconds each time
const unsigned long SOAK_TIME_MS      = 60000;   // wait after watering so water reaches the sensor
const int MAX_PUMPS_IN_ROW            = 5;       // safety: stop if soil never gets wet (bad sensor?)
const unsigned long MANUAL_MAX_MS     = 60000;   // safety: manual pump turns itself off after 60 s
const int SENSOR_LOST_RAW             = 3900;    // at/above this the sensor is in air or unplugged (air = 4095)
const unsigned long BLE_NOTIFY_MS     = 2000;    // send data to the app every 2 seconds

// ================= WiFi / Telegram (optional)
const bool ENABLE_TELEGRAM = false;
const char* WIFI_SSID  = "YOUR_WIFI_NAME";
const char* WIFI_PASS  = "YOUR_WIFI_PASSWORD";
const char* BOT_TOKEN  = "YOUR_BOT_TOKEN";
const char* CHAT_ID    = "YOUR_CHAT_ID";

// ================= BLE (Nordic UART service, same as the app)
#define DEVICE_NAME  "VerdeFlow-ESP32"
#define DEVICE_ID    "ESP32_001"
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // app -> ESP32 (write)
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // ESP32 -> app (notify)

// ================= State
Preferences prefs;
int soilDry = DEFAULT_SOIL_DRY;
int soilWet = DEFAULT_SOIL_WET;
int soilThresholdPct = DEFAULT_THRESHOLD_PCT;
bool autoMode = true;

bool pumpOn = false;
unsigned long pumpStopAt = 0;
unsigned long lastCheck = 0;
unsigned long nextAutoAt = 0;
unsigned long lastNotify = 0;
int pumpsInRow = 0;
bool sensorAlertSent = false;
bool stoppedForSafety = false;
int lastRaw = 0;
int lastPct = 0;

BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;
String bleCmd = "";
volatile bool bleCmdReady = false;

// ================= WiFi / Telegram
void connectWiFi() {
  if (!ENABLE_TELEGRAM) return;
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " failed");
}

void sendTelegram(const String& msg) {
  Serial.println(msg);
  if (!ENABLE_TELEGRAM) return;
  connectWiFi();
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient https;
  String url = String("https://api.telegram.org/bot") + BOT_TOKEN + "/sendMessage";
  if (https.begin(client, url)) {
    https.addHeader("Content-Type", "application/json");
    String body = String("{\"chat_id\":\"") + CHAT_ID + "\",\"text\":\"" + msg + "\"}";
    https.POST(body);
    https.end();
  }
}

// ================= Sensor
int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / 20;
}

int soilPercent(int raw) {
  return constrain(map(raw, soilDry, soilWet, 0, 100), 0, 100);
}

// ================= Pump
void setPump(bool on, unsigned long durationMs = 0) {
  pumpOn = on;
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  if (on) pumpStopAt = millis() + durationMs;
}

// ================= Settings
void saveSettings() {
  prefs.putInt("dry", soilDry);
  prefs.putInt("wet", soilWet);
  prefs.putInt("thr", soilThresholdPct);
  prefs.putBool("auto", autoMode);
}

// ================= BLE
void bleSend(const String& s) {
  if (!bleConnected || txChar == nullptr) return;
  for (size_t i = 0; i < s.length(); i += 20) {
    String part = s.substring(i, i + 20);
    txChar->setValue((uint8_t*)part.c_str(), part.length());
    txChar->notify();
    delay(10);
  }
}

String statusJson() {
  String j = "{\"deviceId\":\"";
  j += DEVICE_ID;
  j += "\",\"soilMoisture\":";
  j += String(lastPct);
  j += ",\"soilRaw\":";
  j += String(lastRaw);
  j += ",\"pumpState\":\"";
  j += pumpOn ? "ON" : "OFF";
  j += "\",\"mode\":\"";
  j += autoMode ? "AUTO" : "MANUAL";
  j += "\"}\n";
  return j;
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) { bleConnected = true; }
  void onDisconnect(BLEServer* s) {
    bleConnected = false;
    BLEDevice::startAdvertising();   // keep advertising so the app can reconnect
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String v = c->getValue().c_str();
    v.trim();
    if (v.length() > 0 && !bleCmdReady) {
      bleCmd = v;
      bleCmdReady = true;
    }
  }
};

void setupBLE() {
  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService* service = server->createService(SERVICE_UUID);

  txChar = service->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());

  BLECharacteristic* rxChar = service->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rxChar->setCallbacks(new RxCB());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising as " DEVICE_NAME);
}

// ================= Commands (Serial Monitor + BLE app, plain text)
// dry | wet | thr 30 | auto | manual | on | off | reset | status
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "dry") {
    soilDry = readAvg(SOIL_PIN);
    saveSettings();
    Serial.printf("Saved DRY = %d\n", soilDry);
  } else if (cmd == "wet") {
    soilWet = readAvg(SOIL_PIN);
    saveSettings();
    Serial.printf("Saved WET = %d\n", soilWet);
  } else if (cmd.startsWith("thr ")) {
    int v = cmd.substring(4).toInt();
    if (v >= 5 && v <= 95) {
      soilThresholdPct = v;
      saveSettings();
      Serial.printf("Saved threshold = %d%%\n", v);
    } else {
      Serial.println("threshold must be between 5 and 95");
    }
  } else if (cmd == "auto") {
    autoMode = true;
    pumpsInRow = 0;
    stoppedForSafety = false;
    if (pumpOn) setPump(false);
    saveSettings();
    Serial.println("Mode: AUTO");
  } else if (cmd == "manual") {
    autoMode = false;
    saveSettings();
    Serial.println("Mode: MANUAL");
  } else if (cmd == "on") {
    autoMode = false;               // turning the pump on by hand switches to manual
    setPump(true, MANUAL_MAX_MS);
    Serial.println("Pump ON (manual, auto-off after 60 s)");
  } else if (cmd == "off") {
    setPump(false);
    Serial.println("Pump OFF");
  } else if (cmd == "reset") {
    pumpsInRow = 0;
    stoppedForSafety = false;
    Serial.println("Safety stop cleared");
  } else if (cmd == "status") {
    Serial.printf("raw=%d soil=%d%% dry=%d wet=%d thr=%d%% mode=%s pump=%s\n",
                  lastRaw, lastPct, soilDry, soilWet, soilThresholdPct,
                  autoMode ? "AUTO" : "MANUAL", pumpOn ? "ON" : "OFF");
  } else if (cmd.length() > 0) {
    Serial.println("Unknown command: " + cmd);
  }

  bleSend(statusJson());
}

// ================= Setup / Loop
void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  analogSetAttenuation(ADC_11db);

  prefs.begin("plant", false);
  soilDry = prefs.getInt("dry", DEFAULT_SOIL_DRY);
  soilWet = prefs.getInt("wet", DEFAULT_SOIL_WET);
  soilThresholdPct = prefs.getInt("thr", DEFAULT_THRESHOLD_PCT);
  autoMode = prefs.getBool("auto", true);

  connectWiFi();
  setupBLE();

  lastCheck = millis() - CHECK_INTERVAL_MS;   // measure right away on boot
  Serial.println(CALIBRATION_MODE ? "CALIBRATION MODE" : "Watering system started");
  Serial.println("Commands: dry | wet | thr 30 | auto | manual | on | off | reset | status");
}

void measureAndDecide() {
  int raw = readAvg(SOIL_PIN);
  lastRaw = raw;
  lastPct = soilPercent(raw);
  Serial.printf("Soil: %d%% (raw %d) | Pump: %s | Mode: %s\n",
                lastPct, raw, pumpOn ? "ON" : "OFF", autoMode ? "AUTO" : "MANUAL");

  // Sensor in air or unplugged -> never water
  if (raw >= SENSOR_LOST_RAW) {
    if (!sensorAlertSent) {
      sendTelegram("⚠️ حساس التربة مش ملامس التربة أو مفصول، البامب متوقف");
      sensorAlertSent = true;
    }
    return;
  }
  sensorAlertSent = false;

  if (!autoMode || pumpOn || stoppedForSafety) return;
  if ((long)(millis() - nextAutoAt) < 0) return;   // still soaking after the last watering

  if (lastPct >= soilThresholdPct) {
    pumpsInRow = 0;
    return;
  }

  // Soil is dry
  if (pumpsInRow >= MAX_PUMPS_IN_ROW) {
    sendTelegram("🛑 البامب اشتغل عدة مرات والتربة لسه ناشفة. اتوقف للأمان، افحص الحساس");
    stoppedForSafety = true;
    return;
  }

  sendTelegram("🌱 التربة ناشفة (" + String(lastPct) + "%)، بسقي الزرع");
  setPump(true, PUMP_TIME_MS);
  pumpsInRow++;
  nextAutoAt = millis() + PUMP_TIME_MS + SOAK_TIME_MS;
}

void loop() {
  if (CALIBRATION_MODE) {
    Serial.print("soil raw: ");
    Serial.println(readAvg(SOIL_PIN));
    delay(1000);
    return;
  }

  // Serial commands
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  // Commands from the app
  if (bleCmdReady) {
    String c = bleCmd;
    bleCmdReady = false;
    handleCommand(c);
  }

  // Pump timeout
  if (pumpOn && (long)(millis() - pumpStopAt) >= 0) {
    setPump(false);
    bleSend(statusJson());
  }

  // Measure
  if (millis() - lastCheck >= CHECK_INTERVAL_MS) {
    lastCheck = millis();
    measureAndDecide();
    bleSend(statusJson());
    lastNotify = millis();
  }

  // Keep the app updated
  if (bleConnected && millis() - lastNotify >= BLE_NOTIFY_MS) {
    lastNotify = millis();
    bleSend(statusJson());
  }
}
