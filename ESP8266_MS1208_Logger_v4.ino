/*
 * ============================================================
 *  MS1208-M1 8-Channel Temperature Logger v4.0
 *  ESP8266 + MAX485 RS-485 Modbus RTU → Google Sheets
 *  Realtime logging only — no offline storage
 * ============================================================
 *  Hardware:
 *    ESP8266 (NodeMCU)
 *    MAX485 TTL Module
 *      DI  → D2 (GPIO4)
 *      RO  → D1 (GPIO5)
 *      DE  → D5 (GPIO14)
 *      RE  → GND (permanent)
 *      A+  → MS1208-M1 B+
 *      B-  → MS1208-M1 A-
 *    MS1208-M1: Slave ID=1, Baud=9600, 8N1
 * ============================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>

// ─── Google Sheets Script URL ────────────────────────────────────────────────
const char* SHEET_URL = "https://script.google.com/macros/s/AKfycby9_AJmUtAUOllo6qohRulgetPwm0fTeQyoWV51ZRBdXCMl3Ym4nNnMmpoyH1_UTdcR/exec";

// ─── WiFi Networks (tries each in order until connected) ─────────────────────
struct WiFiCred { const char* ssid; const char* password; };
const WiFiCred wifiList[] = {
  { "PlyLab",    "sharon123#@" },
  { "SHARONSA",  "sharon123#@" },
  { "Mi A3",     "123123123"   },
  { "ciscosa",   "Abc123#@"    },

};
const int WIFI_COUNT = sizeof(wifiList) / sizeof(wifiList[0]);

// ─── MAX485 Pin Definitions ──────────────────────────────────────────────────
#define RX_PIN  D1    // GPIO5  → RO of MAX485
#define TX_PIN  D2    // GPIO4  → DI of MAX485
#define DE_PIN  D5    // GPIO14 → DE of MAX485

// ─── Modbus Configuration ────────────────────────────────────────────────────
#define SLAVE_ID      1
#define RS485_BAUD    9600
#define NUM_CHANNELS  8
#define START_REG     0x006E   // CH1 PV start register

// ─── Timing ──────────────────────────────────────────────────────────────────
#define READ_INTERVAL       10000    // Read sensors every 10 seconds
#define UPLOAD_INTERVAL     60000   // Upload to Google Sheets every 60 seconds
#define WIFI_RETRY_INTERVAL 30000   // Retry WiFi every 30 seconds

// ─── Global Variables ────────────────────────────────────────────────────────
SoftwareSerial rs485Serial(RX_PIN, TX_PIN);

float temperatures[NUM_CHANNELS];
bool  channelValid[NUM_CHANNELS];

unsigned long lastReadTime     = 0;
unsigned long lastUploadTime   = 0;
unsigned long lastWifiRetry    = 0;

bool wifiConnected   = false;
bool lastWifiState   = false;
bool hasValidReading = false;

// ─── CRC16 ───────────────────────────────────────────────────────────────────
uint16_t crc16(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}

// ─── Word-Swapped IEEE 754 Float ─────────────────────────────────────────────
float bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  uint32_t raw = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) |
                 ((uint32_t)b2 << 8)  |  (uint32_t)b3;
  float result;
  memcpy(&result, &raw, sizeof(float));
  return result;
}

// ─── Read MS1208-M1 All 8 Channels ───────────────────────────────────────────
bool readMS1208(float *results, bool *valid) {
  while (rs485Serial.available()) rs485Serial.read();  // flush

  uint8_t request[8];
  request[0] = SLAVE_ID;
  request[1] = 0x03;
  request[2] = (START_REG >> 8) & 0xFF;
  request[3] = START_REG & 0xFF;
  request[4] = 0x00;
  request[5] = 16;
  uint16_t crc = crc16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  // Transmit
  digitalWrite(DE_PIN, HIGH);
  delayMicroseconds(500);
  rs485Serial.write(request, 8);
  rs485Serial.flush();
  delayMicroseconds(500);
  digitalWrite(DE_PIN, LOW);

  // Discard 8 echoed TX bytes
  uint8_t echoCount = 0;
  unsigned long t = millis();
  while (millis() - t < 1000 && echoCount < 8) {
    if (rs485Serial.available()) { rs485Serial.read(); echoCount++; }
    yield();
  }
  if (echoCount < 8) { Serial.println("[Modbus] Echo timeout"); return false; }

  // Read 37-byte response
  const uint8_t responseLen = 37;
  uint8_t response[responseLen];
  uint8_t idx = 0;
  t = millis();
  while (millis() - t < 1000 && idx < responseLen) {
    if (rs485Serial.available()) response[idx++] = rs485Serial.read();
    yield();
  }
  if (idx < responseLen) {
    Serial.printf("[Modbus] Timeout: got %d of %d bytes\n", idx, responseLen);
    return false;
  }

  // Verify CRC
  uint16_t recvCRC = (response[responseLen-1] << 8) | response[responseLen-2];
  uint16_t calcCRC = crc16(response, responseLen-2);
  if (recvCRC != calcCRC) {
    Serial.printf("[Modbus] CRC error\n");
    return false;
  }

  // Parse 8 channels
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    uint8_t base = 3 + (i * 4);
    float val = bytesToFloat(response[base+2], response[base+3],
                             response[base],   response[base+1]);
    if (val > 2000.0f || val < -300.0f || isnan(val) || isinf(val)) {
      results[i] = 0.0f;
      valid[i]   = false;
    } else {
      results[i] = val;
      valid[i]   = true;
    }
  }
  return true;
}

// ─── WiFi: Try All Networks ───────────────────────────────────────────────────
bool connectWiFi() {
  for (int n = 0; n < WIFI_COUNT; n++) {
    Serial.printf("[WiFi] Trying: %s", wifiList[n].ssid);
    WiFi.begin(wifiList[n].ssid, wifiList[n].password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
      yield();
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[WiFi] Connected to %s | IP: %s\n",
                    wifiList[n].ssid, WiFi.localIP().toString().c_str());
      return true;
    }
    Serial.println("\n[WiFi] Failed — trying next...");
    WiFi.disconnect();
    delay(500);
  }
  Serial.println("[WiFi] All networks failed.");
  return false;
}

// ─── Upload to Google Sheets ──────────────────────────────────────────────────
bool uploadToSheets(float t1, float t2, float t3) {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String url = String(SHEET_URL);
  url += "?temp1=" + (isnan(t1) ? String("0") : String(t1, 2));
  url += "&temp2=" + (isnan(t2) ? String("0") : String(t2, 2));
  url += "&temp3=" + (isnan(t3) ? String("0") : String(t3, 2));

  http.begin(client, url);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();
  bool success = (httpCode == HTTP_CODE_OK || httpCode == 302);

  if (success) Serial.printf("[Upload] OK T1=%.2f T2=%.2f T3=%.2f\n", t1, t2, t3);
  else         Serial.printf("[Upload] Failed HTTP %d\n", httpCode);

  http.end();
  return success;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n============================================");
  Serial.println("  MS1208-M1 Temperature Logger v4.0");
  Serial.println("  Realtime logging | No offline storage");
  Serial.println("============================================");

  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  rs485Serial.begin(RS485_BAUD);

  // Connect WiFi
  wifiConnected = connectWiFi();
  lastWifiState = wifiConnected;

  // Initial sensor read
  Serial.println("[Setup] Initial sensor read...");
  delay(500);
  bool ok = readMS1208(temperatures, channelValid);
  if (ok) {
    hasValidReading = true;
    Serial.println("[Setup] Success:");
    for (int i = 0; i < NUM_CHANNELS; i++) {
      if (channelValid[i]) Serial.printf("  CH%d: %.2f°C\n", i+1, temperatures[i]);
      else                 Serial.printf("  CH%d: --- (open/error)\n", i+1);
    }
  } else {
    Serial.println("[Setup] Initial read failed — will retry in loop.");
    for (int i = 0; i < NUM_CHANNELS; i++) { temperatures[i] = 0.0f; channelValid[i] = false; }
  }

  lastReadTime   = millis();
  lastUploadTime = millis();
  Serial.println("[Setup] Ready.\n");
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // WiFi watchdog
  bool currentWifi = (WiFi.status() == WL_CONNECTED);
  if (currentWifi != lastWifiState) {
    lastWifiState = currentWifi;
    wifiConnected = currentWifi;
    if (currentWifi) Serial.println("[WiFi] Reconnected!");
    else             Serial.println("[WiFi] Disconnected!");
  }
  if (!wifiConnected && (now - lastWifiRetry >= WIFI_RETRY_INTERVAL)) {
    lastWifiRetry = now;
    wifiConnected = connectWiFi();
    lastWifiState = wifiConnected;
  }

  // Sensor read every READ_INTERVAL
  if (now - lastReadTime >= READ_INTERVAL) {
    lastReadTime = now;
    Serial.println("[Read] Reading MS1208-M1...");
    bool ok = readMS1208(temperatures, channelValid);
    if (ok) {
      hasValidReading = true;
      for (int i = 0; i < NUM_CHANNELS; i++) {
        if (channelValid[i]) Serial.printf("  CH%d: %.2f°C\n", i+1, temperatures[i]);
        else                 Serial.printf("  CH%d: --- (open)\n", i+1);
      }
    } else {
      Serial.println("[Read] Failed — sending 0.");
      for (int i = 0; i < NUM_CHANNELS; i++) { temperatures[i] = 0.0f; channelValid[i] = false; }
    }
  }

  // Upload every UPLOAD_INTERVAL
  if (now - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime = now;
    if (!hasValidReading) {
      Serial.println("[Upload] Skipped — no reading yet.");
    } else if (wifiConnected) {
      float t1 = isnan(temperatures[0]) ? 0.0f : temperatures[0];
      float t2 = isnan(temperatures[1]) ? 0.0f : temperatures[1];
      float t3 = isnan(temperatures[2]) ? 0.0f : temperatures[2];
      uploadToSheets(t1, t2, t3);
    } else {
      Serial.println("[Upload] Skipped — no WiFi.");
    }
    Serial.printf("[Status] Heap: %u B\n", ESP.getFreeHeap());
  }

  delay(10);
  yield();
}
