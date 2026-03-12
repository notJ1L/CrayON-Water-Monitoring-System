#include <esp_now.h>
#include <WiFi.h>

#include <FastLED.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#define DATA_PIN 13
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];

#define S0 25
#define S1 26
#define S2 16
#define S3 17
#define sensorOut 4

#define PH_PIN 34
#define PH_PUMP_1 22
#define PH_PUMP_2 14

#define TDS_PIN 35
#define TDS_PUMP 18

#define VOLTAGE 3.3
#define ADC_RESOLUTION 4095.0

#define GROUP_ID 5
#define GROUP_NAME "GROUP 5"
uint8_t receiverMAC[] = {0x80, 0xf3, 0xda, 0x54, 0x33, 0xfc};

#define ESPNOW_MAX_FIELDS 8

struct __attribute__((packed)) DataField {
  char    name[16];
  char    unit[6];
  float   value;
  uint8_t type;   // 0 = sensor, 1 = actuator
  uint8_t _pad;
};
 
typedef struct __attribute__((packed)) {
  uint8_t   groupId;
  char      groupName[20];
  uint8_t   fieldCount;
  DataField fields[ESPNOW_MAX_FIELDS];
} EspNowPacket;
 
esp_now_peer_info_t espNowPeer;
bool espNowReady = false;

const char* ssid = "stem";
const char* password = "coaa_s2a";
const char* serverUrl = "http://10.29.2.198:8090/CrayON/api.php";

WebServer server(80);

struct SensorData {
  int rFreq, gFreq, bFreq;
  String colorStatus;
  float tdsValue;
  String tdsStatus;
  float phValue;
  float phVolt;
  String phStatus;
  unsigned long timestamp;
};
SensorData currentData;

unsigned long lastSensorRead = 0;
unsigned long lastServerSend = 0;
const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long SERVER_INTERVAL = 10000;

#define CONFIRM_NEEDED 3
int phHighCount  = 0;
int phLowCount   = 0;
float lastStablePh = 7.0;
#define PH_JUMP_THRESHOLD 2.0

bool manualPump1   = false;
bool manualPump2   = false;
bool manualTdsPump = false;
bool manualLED     = false;

// ── Helper: read TCS3200 frequency as pulse width (microseconds) ─
// At 100% scaling, pulseIn reads the period directly.
// LOWER value = HIGHER frequency = MORE of that color absorbed.
// Returns 0 if no signal (sensor absent or no light).
unsigned long readChannel(int pin) {
  unsigned long total = 0;
  int samples = 5;
  for (int i = 0; i < samples; i++) {
    unsigned long p = pulseIn(pin, LOW, 50000); // 50ms max per pulse
    total += p;
  }
  return total / samples;
}

void dipPump(int pin) {
  for (int d = 0; d < 3; d++) {
    digitalWrite(pin, LOW);
    delay(150);
    digitalWrite(pin, HIGH);
    delay(150);
  }
}

static void addField(EspNowPacket& pkt, const char* name,
                     const char* unit, float value, uint8_t type) {
  if (pkt.fieldCount >= ESPNOW_MAX_FIELDS) return;
  int i = pkt.fieldCount++;
  strncpy(pkt.fields[i].name, name, 15);  pkt.fields[i].name[15] = '\0';
  strncpy(pkt.fields[i].unit, unit, 5);   pkt.fields[i].unit[5]  = '\0';
  pkt.fields[i].value = value;
  pkt.fields[i].type  = type;
  pkt.fields[i]._pad  = 0;
}
 
void OnEspNowSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.printf("[ESP-NOW] Delivery: %s\n",
    status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}
 
void espNowSendData() {
  if (!espNowReady) return;
  EspNowPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.groupId    = GROUP_ID;
  pkt.fieldCount = 0;
  strncpy(pkt.groupName, GROUP_NAME, 19);
    
    addField(pkt, "TDS",      "ppm", currentData.tdsValue, 0);
    addField(pkt, "pH",       "pH",  currentData.phValue,  0);
    addField(pkt, "R Freq",   "hz",  (float)currentData.rFreq, 0);
    addField(pkt, "G Freq",   "hz",  (float)currentData.gFreq, 0);
    addField(pkt, "B Freq",   "hz",  (float)currentData.bFreq, 0);
    // ----------------------------------------------------------------
    addField(pkt, "TDS Pump", "",    digitalRead(TDS_PUMP)  == LOW ? 1.0f : 0.0f, 1);
    addField(pkt, "pH Pump1", "",    digitalRead(PH_PUMP_1) == LOW ? 1.0f : 0.0f, 1);
    addField(pkt, "pH Pump2", "",    digitalRead(PH_PUMP_2) == LOW ? 1.0f : 0.0f, 1);
    
    Serial.printf("[ESP-NOW] pkt.fieldCount=%d groupName='%s'\n", pkt.fieldCount, pkt.groupName);
    esp_now_send(receiverMAC, (uint8_t*)&pkt, sizeof(pkt));

}

void setup() {
  Serial.begin(115200);
  delay(2000);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  // TCS3200 pins
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // S0=HIGH, S1=HIGH → 100% output frequency (fastest, most readable)
  digitalWrite(S0, HIGH); digitalWrite(S1, HIGH);

  // Relay pins — HIGH = relay OFF (active-low relay board)
  pinMode(PH_PUMP_1, OUTPUT); digitalWrite(PH_PUMP_1, HIGH);
  pinMode(PH_PUMP_2, OUTPUT); digitalWrite(PH_PUMP_2, HIGH);
  pinMode(TDS_PUMP,  OUTPUT); digitalWrite(TDS_PUMP,  HIGH);

  pinMode(PH_PIN,  INPUT);
  pinMode(TDS_PIN, INPUT);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); Serial.print("."); attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    if (MDNS.begin("coaa-sensor")) Serial.println("MDNS started");
    setupWebServer();
    server.begin();
    Serial.print("Dashboard: http://"); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed.");
  }

  Serial.println(">>> SYSTEM READY <<<");
}

void loop() {
  unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED) server.handleClient();

  if (manualLED) {
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    digitalWrite(S2, LOW);  digitalWrite(S3, LOW);
    unsigned long rVal = readChannel(sensorOut);

    digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
    unsigned long gVal = readChannel(sensorOut);

    digitalWrite(S2, LOW);  digitalWrite(S3, HIGH);
    unsigned long bVal = readChannel(sensorOut);

    int rCount = (int)rVal;
    int gCount = (int)gVal;
    int bCount = (int)bVal;

    String colorStatus = "STANDARD";
    bool isGreen = false;
    float greenScore = 0.0f;

    if (rVal == 0 && gVal == 0 && bVal == 0) {
      Serial.println("[COLOR] no sensor");
    } else {
      // Lower pulse width = more of that color
      // Green dominant = gVal is LOWER than both rVal and bVal
      float avgRB = ((float)rVal + (float)bVal) / 2.0f;
      // rawScore positive = green is lower (more green)
      float rawScore = (gVal > 0 && avgRB > 0) ? ((avgRB - (float)gVal) / avgRB) * 100.0f : 0.0f;
      greenScore = rawScore < 0.0f ? 0.0f : rawScore * 3.0f;
      isGreen = (gVal > 0 && greenScore >= 5.0f);
      Serial.printf("[COLOR] R:%lu G:%lu B:%lu | score:%.1f%%%s\n",
        rVal, gVal, bVal, greenScore, isGreen ? " << GREEN!" : "");
    }
    if (!manualLED) {
      if (isGreen) {
        colorStatus = "GREEN DETECTED";
        fill_solid(leds, NUM_LEDS, CRGB::Red);
      } else {
        fill_solid(leds, NUM_LEDS, CRGB::White);
      }
      FastLED.show();
    }

    currentData.rFreq = rCount;
    currentData.gFreq = gCount;
    currentData.bFreq = bCount;
    currentData.colorStatus = colorStatus;

    // ── TDS Sensor ────────────────────────────────────────────
    int tdsRaw = analogRead(TDS_PIN);
    float tdsVolt  = tdsRaw * (3.3 / 4095.0);
    float tdsValue = (133.42 * pow(tdsVolt, 3) - 255.86 * pow(tdsVolt, 2) + 857.39 * tdsVolt) * 0.5;

    String tdsStatus = "CLEAN";
    // Relay is active-LOW: LOW = ON, HIGH = OFF
    if (manualTdsPump || tdsValue >= 150.0) {
      digitalWrite(TDS_PUMP, LOW);   // turn relay ON
      tdsStatus = "DIRTY! Pump ON";
    } else {
      digitalWrite(TDS_PUMP, HIGH);  // turn relay OFF
    }
    manualTdsPump = false;

    Serial.printf("[TDS] raw:%d volt:%.3f ppm:%.1f pump:%s\n",
      tdsRaw, tdsVolt, tdsValue, tdsValue >= 150.0 ? "ON" : "OFF");

    currentData.tdsValue  = tdsValue;
    currentData.tdsStatus = tdsStatus;

    // ── pH Sensor ─────────────────────────────────────────────
    long phRaw = 0;
    for (int i = 0; i < 10; i++) phRaw += analogRead(PH_PIN);
    float phVolt  = (phRaw / 10.0) * (VOLTAGE / ADC_RESOLUTION);
    float phValue = 7.0 + ((2.5 - phVolt) / 0.18);

    String phStatus = "OK";
    bool phJump = (abs(phValue - lastStablePh) > PH_JUMP_THRESHOLD);

    if (phJump) {
      Serial.printf("[SPIKE] pH was %.2f now %.2f — confirming\n", lastStablePh, phValue);
      if (phValue < 4.5)      { phLowCount++;  phHighCount = 0; }
      else if (phValue > 8.5) { phHighCount++; phLowCount  = 0; }
      else                    { phLowCount = 0; phHighCount = 0; }
    } else {
      lastStablePh = phValue;
      if (phValue < 4.5)      { phLowCount++;  phHighCount = 0; }
      else if (phValue > 8.5) { phHighCount++; phLowCount  = 0; }
      else                    { phLowCount = 0; phHighCount = 0; }
    }

    if (phLowCount >= CONFIRM_NEEDED || manualPump2) {
      phStatus = "ACIDIC! Dosing Base";
      lastStablePh = phValue;
      dipPump(PH_PUMP_2);
      phLowCount = 0;
    } else if (phHighCount >= CONFIRM_NEEDED || manualPump1) {
      phStatus = "ALKALINE! Dosing Acid";
      lastStablePh = phValue;
      dipPump(PH_PUMP_1);
      phHighCount = 0;
    } else if (phValue < 4.5) {
      phStatus = "ACIDIC! Confirming (" + String(phLowCount) + "/3)";
    } else if (phValue > 8.5) {
      phStatus = "ALKALINE! Confirming (" + String(phHighCount) + "/3)";
    }

    manualPump1 = false;
    manualPump2 = false;

    currentData.phValue     = phValue;
    currentData.phVolt      = phVolt;
    currentData.phStatus    = phStatus;
    currentData.timestamp   = millis();

    Serial.printf("[pH] volt:%.3f pH:%.2f status:%s\n", phVolt, phValue, phStatus.c_str());

  espNowSendData();
  
  }

  if (WiFi.status() == WL_CONNECTED && now - lastServerSend >= SERVER_INTERVAL) {
    lastServerSend = now;
    sendSensorData(
      currentData.rFreq, currentData.gFreq, currentData.bFreq,
      currentData.colorStatus, currentData.tdsValue, currentData.tdsStatus,
      currentData.phValue, currentData.phVolt, currentData.phStatus
    );
  }
}

void sendSensorData(int rFreq, int gFreq, int bFreq, String colorStatus,
                    float tdsValue, String tdsStatus, float phValue,
                    float phVolt, String phStatus) {
  HTTPClient http;
  DynamicJsonDocument doc(1024);
  doc["r_frequency"]  = rFreq;
  doc["g_frequency"]  = gFreq;
  doc["b_frequency"]  = bFreq;
  doc["color_status"] = colorStatus;
  doc["tds_value"]    = tdsValue;
  doc["tds_status"]   = tdsStatus;
  doc["ph_value"]     = phValue;
  doc["ph_voltage"]   = phVolt;
  doc["ph_status"]    = phStatus;
  String jsonString;
  serializeJson(doc, jsonString);
  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(10000);
  int code = http.POST(jsonString);
  Serial.printf("[POST] %d\n", code);
  http.end();
}

void setupWebServer() {
  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/data",   HTTP_GET,  handleData);
  server.on("/manual", HTTP_POST, handleManual);
  server.onNotFound(handleNotFound);
}

void handleManual() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }
  DynamicJsonDocument doc(256);
  deserializeJson(doc, server.arg("plain"));
  if (doc.containsKey("pump1"))   manualPump1   = doc["pump1"].as<bool>();
  if (doc.containsKey("pump2"))   manualPump2   = doc["pump2"].as<bool>();
  if (doc.containsKey("tdspump")) manualTdsPump = doc["tdspump"].as<bool>();
  if (doc.containsKey("led")) {
    manualLED = doc["led"].as<bool>();
    if (!manualLED) { fill_solid(leds, NUM_LEDS, CRGB::White); FastLED.show(); }
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleData() {
  DynamicJsonDocument doc(1024);
  doc["r_frequency"]  = currentData.rFreq;
  doc["g_frequency"]  = currentData.gFreq;
  doc["b_frequency"]  = currentData.bFreq;
  doc["color_status"] = currentData.colorStatus;
  doc["tds_value"]    = currentData.tdsValue;
  doc["tds_status"]   = currentData.tdsStatus;
  doc["ph_value"]     = currentData.phValue;
  doc["ph_voltage"]   = currentData.phVolt;
  doc["ph_status"]    = currentData.phStatus;
  doc["timestamp"]    = currentData.timestamp;
  doc["manual_led"]   = manualLED;
  String jsonString;
  serializeJson(doc, jsonString);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", jsonString);
}

void handleRoot() {
  String html = "";
  html += "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>";
  html += "<title>CrayON</title>";
  html += "<link href='https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap' rel='stylesheet'>";
  html += "<style>";
  html += "*{margin:0;padding:0;box-sizing:border-box;}";
  html += "body{font-family:'Inter',sans-serif;background:#0a0f1e;min-height:100vh;color:#e2e8f0;}";
  html += "body::before{content:'';position:fixed;inset:0;background:radial-gradient(ellipse 80% 60% at 70% 10%,rgba(15,40,90,0.5) 0%,transparent 60%),radial-gradient(ellipse 60% 50% at 10% 80%,rgba(10,30,70,0.4) 0%,transparent 60%);pointer-events:none;z-index:0;}";
  // Topbar
  html += ".topbar{position:relative;z-index:1;display:flex;align-items:center;justify-content:space-between;padding:18px 28px;border-bottom:1px solid rgba(255,255,255,0.07);background:rgba(10,15,30,0.9);backdrop-filter:blur(12px);}";
  html += ".brand{font-size:1.25em;font-weight:700;color:#38bdf8;letter-spacing:0.01em;}";
  html += ".topbar-right{display:flex;align-items:center;gap:12px;}";
  html += ".conn-badge{display:flex;align-items:center;gap:7px;background:#111827;border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:7px 16px;font-size:0.78em;color:#94a3b8;}";
  html += ".conn-dot{width:8px;height:8px;border-radius:50%;background:#22c55e;box-shadow:0 0 8px #22c55e;}";
  html += ".conn-dot.err{background:#ef4444;box-shadow:0 0 8px #ef4444;}";
  html += ".time-badge{background:#111827;border:1px solid rgba(255,255,255,0.1);border-radius:20px;padding:7px 16px;font-size:0.78em;color:#94a3b8;}";
  // Layout
  html += ".wrap{position:relative;z-index:1;padding:28px;}";
  // Sensor grid
  html += ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:20px;margin-bottom:20px;}";
  // Cards
  html += ".card{background:#111827;border:1px solid rgba(255,255,255,0.08);border-radius:14px;padding:22px;transition:border-color 0.3s;}";
  html += ".card.alert-green{border-color:rgba(34,197,94,0.5);box-shadow:0 0 20px rgba(34,197,94,0.08);}";
  html += ".card.alert-red{border-color:rgba(239,68,68,0.5);box-shadow:0 0 20px rgba(239,68,68,0.08);}";
  html += ".card.alert-amber{border-color:rgba(245,158,11,0.5);box-shadow:0 0 20px rgba(245,158,11,0.08);}";
  html += ".card.alert-blue{border-color:rgba(56,189,248,0.5);box-shadow:0 0 20px rgba(56,189,248,0.08);}";
  // Card header
  html += ".card-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:20px;padding-bottom:16px;border-bottom:1px solid rgba(255,255,255,0.06);}";
  html += ".card-head-left{display:flex;align-items:center;gap:10px;}";
  html += ".chip{width:34px;height:34px;border-radius:8px;display:flex;align-items:center;justify-content:center;font-size:0.7em;font-weight:700;letter-spacing:0.03em;}";
  html += ".chip-rgb{background:linear-gradient(135deg,#7c3aed,#db2777);}";
  html += ".chip-tds{background:linear-gradient(135deg,#0ea5e9,#06b6d4);}";
  html += ".chip-ph{background:linear-gradient(135deg,#059669,#10b981);}";
  html += ".card-title{font-size:1.05em;font-weight:600;color:#f1f5f9;}";
  // Alert pill
  html += ".pill{font-size:0.68em;font-weight:700;letter-spacing:0.08em;text-transform:uppercase;padding:4px 10px;border-radius:6px;}";
  html += ".pill-green{background:rgba(34,197,94,0.15);color:#4ade80;border:1px solid rgba(34,197,94,0.3);}";
  html += ".pill-red{background:rgba(239,68,68,0.15);color:#f87171;border:1px solid rgba(239,68,68,0.3);}";
  html += ".pill-amber{background:rgba(245,158,11,0.15);color:#fbbf24;border:1px solid rgba(245,158,11,0.3);}";
  html += ".pill-blue{background:rgba(56,189,248,0.15);color:#38bdf8;border:1px solid rgba(56,189,248,0.3);}";
  html += ".pill-gray{background:rgba(148,163,184,0.1);color:#94a3b8;border:1px solid rgba(148,163,184,0.2);}";
  // Data rows
  html += ".data-row{display:flex;align-items:center;justify-content:space-between;padding:10px 0;border-bottom:1px solid rgba(255,255,255,0.04);}";
  html += ".data-row:last-child{border-bottom:none;}";
  html += ".data-label{font-size:0.82em;color:#64748b;}";
  html += ".data-value{font-size:1.4em;font-weight:700;color:#f1f5f9;letter-spacing:-0.02em;}";
  html += ".data-value .unit{font-size:0.45em;font-weight:500;color:#64748b;margin-left:3px;vertical-align:middle;}";
  // Progress bar
  html += ".bar-wrap{margin:14px 0 4px;}";
  html += ".bar-track{width:100%;height:3px;background:rgba(255,255,255,0.06);border-radius:2px;overflow:hidden;}";
  html += ".bar-fill{height:100%;border-radius:2px;transition:width 0.6s ease;}";
  html += ".bar-cyan{background:linear-gradient(90deg,#0ea5e9,#22d3ee);}";
  html += ".bar-red{background:linear-gradient(90deg,#ef4444,#f87171);}";
  html += ".bar-green{background:linear-gradient(90deg,#22c55e,#4ade80);}";
  html += ".bar-amber{background:linear-gradient(90deg,#f59e0b,#fbbf24);}";
  html += ".bar-blue{background:linear-gradient(90deg,#3b82f6,#60a5fa);}";
  // Output status section inside card
  html += ".out-box{margin-top:16px;background:rgba(0,0,0,0.2);border-radius:9px;padding:11px 14px;border:1px solid rgba(255,255,255,0.04);}";
  html += ".out-box-title{font-size:0.65em;font-weight:600;letter-spacing:0.12em;text-transform:uppercase;color:#334155;margin-bottom:9px;}";
  html += ".out-row{display:flex;align-items:center;justify-content:space-between;padding:4px 0;}";
  html += ".out-label{font-size:0.75em;color:#475569;}";
  html += ".out-tag{font-size:0.68em;font-weight:600;padding:2px 8px;border-radius:5px;}";
  html += ".out-idle{background:rgba(100,116,139,0.15);color:#64748b;}";
  html += ".out-active{background:rgba(239,68,68,0.15);color:#f87171;}";
  html += ".out-running{background:rgba(34,197,94,0.12);color:#4ade80;}";
  html += ".out-warn{background:rgba(245,158,11,0.12);color:#fbbf24;}";
  // Hardware status section
  html += ".hw-card{background:#111827;border:1px solid rgba(255,255,255,0.08);border-radius:14px;padding:22px;}";
  html += ".hw-title{font-size:0.95em;font-weight:600;color:#f1f5f9;margin-bottom:18px;padding-bottom:14px;border-bottom:1px solid rgba(255,255,255,0.06);}";
  html += ".hw-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;}";
  html += ".hw-item{background:rgba(0,0,0,0.2);border-radius:10px;padding:16px;text-align:center;border:1px solid rgba(255,255,255,0.04);}";
  html += ".hw-circle{width:56px;height:56px;border-radius:50%;margin:0 auto 10px;display:flex;align-items:center;justify-content:center;font-size:0.7em;font-weight:700;letter-spacing:0.05em;transition:all 0.3s;}";
  html += ".hw-idle{background:#1e293b;color:#475569;border:2px solid #1e293b;}";
  html += ".hw-active{background:rgba(34,197,94,0.15);color:#4ade80;border:2px solid rgba(34,197,94,0.4);box-shadow:0 0 16px rgba(34,197,94,0.2);}";
  html += ".hw-alert{background:rgba(239,68,68,0.15);color:#f87171;border:2px solid rgba(239,68,68,0.4);box-shadow:0 0 16px rgba(239,68,68,0.2);}";
  html += ".hw-warn{background:rgba(245,158,11,0.12);color:#fbbf24;border:2px solid rgba(245,158,11,0.35);box-shadow:0 0 12px rgba(245,158,11,0.15);}";
  html += ".hw-name{font-size:0.8em;font-weight:600;color:#94a3b8;margin-bottom:2px;}";
  html += ".hw-desc{font-size:0.68em;color:#334155;}";
  // Manual override
  html += ".manual-card{background:#111827;border:1px solid rgba(239,68,68,0.15);border-radius:14px;padding:22px;margin-bottom:20px;}";
  html += ".manual-title{font-size:0.95em;font-weight:600;color:#f87171;margin-bottom:16px;padding-bottom:12px;border-bottom:1px solid rgba(239,68,68,0.1);}";
  html += ".manual-btns{display:flex;flex-wrap:wrap;gap:10px;}";
  html += ".btn{border:none;padding:9px 20px;border-radius:8px;font-size:0.8em;font-weight:600;cursor:pointer;letter-spacing:0.03em;transition:all 0.2s;font-family:'Inter',sans-serif;}";
  html += ".btn:hover{filter:brightness(1.15);transform:translateY(-1px);}";
  html += ".btn-base{background:rgba(16,185,129,0.15);color:#34d399;border:1px solid rgba(16,185,129,0.3);}";
  html += ".btn-acid{background:rgba(245,158,11,0.15);color:#fbbf24;border:1px solid rgba(245,158,11,0.3);}";
  html += ".btn-tds{background:rgba(56,189,248,0.15);color:#38bdf8;border:1px solid rgba(56,189,248,0.3);}";
  html += ".btn-led{background:rgba(148,163,184,0.08);color:#64748b;border:1px solid rgba(148,163,184,0.15);}";
  html += ".btn-led.on{background:rgba(250,204,21,0.12);color:#facc15;border-color:rgba(250,204,21,0.3);}";
  html += ".btn-refresh{background:rgba(56,189,248,0.1);color:#38bdf8;border:1px solid rgba(56,189,248,0.2);margin-top:6px;}";
  html += ".footer{text-align:center;color:#1e293b;font-size:0.72em;margin-top:20px;letter-spacing:0.06em;}";
  html += "</style></head><body>";

  // ── TOPBAR
  html += "<div class='topbar'>";
  html += "<div class='brand'>CrayON &mdash; Automated Water Quality Management System</div>";
  html += "<div class='topbar-right'>";
  html += "<div class='conn-badge'><span class='conn-dot' id='connDot'></span><span id='connStatus'>Connecting...</span></div>";
  html += "<div class='time-badge'>Last Updated: <span id='lastTime'>--:--:--</span></div>";
  html += "</div></div>";

  html += "<div class='wrap'>";
  html += "<div class='grid'>";

  // ── COLOR SENSOR CARD
  html += "<div class='card' id='colorCard'>";
  html += "<div class='card-head'>";
  html += "<div class='card-head-left'><div class='chip chip-rgb'>RGB</div><span class='card-title'>Color Sensor</span></div>";
  html += "<span id='colorPill' class='pill pill-gray'>STANDBY</span>";
  html += "</div>";
  html += "<div class='data-row'><span class='data-label'>Red Frequency</span><span class='data-value' id='rFreq'>-</span></div>";
  html += "<div class='data-row'><span class='data-label'>Green Frequency</span><span class='data-value' id='gFreq'>-</span></div>";
  html += "<div class='data-row'><span class='data-label'>Blue Frequency</span><span class='data-value' id='bFreq'>-</span></div>";
  html += "<div class='bar-wrap'><div class='bar-track'><div class='bar-fill bar-red' id='rBar' style='width:50%'></div></div></div>";
  html += "<div class='bar-wrap'><div class='bar-track'><div class='bar-fill bar-blue' id='bBar' style='width:50%'></div></div></div>";
  html += "<div class='bar-wrap'><div class='bar-track'><div class='bar-fill bar-green' id='gBar' style='width:50%'></div></div></div>";
  html += "<div class='out-box'><div class='out-box-title'>Output Status</div>";
  html += "<div class='out-row'><span class='out-label'>LED Strip</span><span class='out-tag out-idle' id='ledState'>WHITE &mdash; STANDBY</span></div>";
  html += "</div></div>";

  // ── TDS SENSOR CARD
  html += "<div class='card' id='tdsCard'>";
  html += "<div class='card-head'>";
  html += "<div class='card-head-left'><div class='chip chip-tds'>TDS</div><span class='card-title'>Water Quality</span></div>";
  html += "<span id='tdsPill' class='pill pill-gray'>STANDBY</span>";
  html += "</div>";
  html += "<div class='data-row'><span class='data-label'>TDS Level</span><span class='data-value' id='tdsValue'>-<span class='unit'>ppm</span></span></div>";
  html += "<div class='bar-wrap' style='margin-top:20px'><div class='bar-track' style='height:48px;border-radius:8px;background:rgba(14,165,233,0.06);border:1px solid rgba(14,165,233,0.12);'>";
  html += "<div class='bar-fill bar-cyan' id='tdsBar' style='width:0%;height:100%;border-radius:8px;opacity:0.7;'></div></div></div>";
  html += "<div class='out-box'><div class='out-box-title'>Output Status</div>";
  html += "<div class='out-row'><span class='out-label'>Diaphragm Pump (Filter)</span><span class='out-tag out-idle' id='tdsState'>STANDBY</span></div>";
  html += "</div></div>";

  // ── pH SENSOR CARD
  html += "<div class='card' id='phCard'>";
  html += "<div class='card-head'>";
  html += "<div class='card-head-left'><div class='chip chip-ph'>pH</div><span class='card-title'>pH Sensor</span></div>";
  html += "<span id='phPill' class='pill pill-gray'>STANDBY</span>";
  html += "</div>";
  html += "<div class='data-row'><span class='data-label'>pH Level</span><span class='data-value' id='phValue'>-</span></div>";
  html += "<div class='data-row'><span class='data-label'>Voltage</span><span class='data-value' id='phVolt'>-<span class='unit'>V</span></span></div>";
  html += "<div class='bar-wrap' style='margin-top:20px'><div class='bar-track' style='height:48px;border-radius:8px;background:rgba(16,185,129,0.06);border:1px solid rgba(16,185,129,0.12);'>";
  html += "<div class='bar-fill bar-green' id='phBar' style='width:0%;height:100%;border-radius:8px;opacity:0.7;'></div></div></div>";
  html += "<div class='out-box'><div class='out-box-title'>Output Status</div>";
  html += "<div class='out-row'><span class='out-label'>Peristaltic Pump 1 &mdash; Acid</span><span class='out-tag out-idle' id='pump1State'>IDLE</span></div>";
  html += "<div class='out-row'><span class='out-label'>Peristaltic Pump 2 &mdash; Base</span><span class='out-tag out-idle' id='pump2State'>IDLE</span></div>";
  html += "</div></div>";

  html += "</div>"; // end .grid

  // ── MANUAL OVERRIDE
  html += "<div class='manual-card'>";
  html += "<div class='manual-title'>Manual Override Controls</div>";
  html += "<div class='manual-btns'>";
  html += "<button class='btn btn-base' onclick='triggerManual(\"pump2\")'>Dose Base &mdash; Pump 2</button>";
  html += "<button class='btn btn-acid' onclick='triggerManual(\"pump1\")'>Dose Acid &mdash; Pump 1</button>";
  html += "<button class='btn btn-tds'  onclick='triggerManual(\"tdspump\")'>Run Diaphragm Pump</button>";
  html += "<button class='btn btn-led'  id='ledBtn' onclick='toggleLED()'>LED Strip: AUTO</button>";
  html += "<button class='btn btn-refresh' onclick='updateData()'>Refresh Now</button>";
  html += "</div></div>";

  // ── HARDWARE STATUS
  html += "<div class='hw-card'>";
  html += "<div class='hw-title'>System Hardware Status</div>";
  html += "<div class='hw-grid'>";
  html += "<div class='hw-item'><div class='hw-circle hw-idle' id='hwP1'>IDLE</div><div class='hw-name'>Acid Pump</div><div class='hw-desc' id='hwP1Desc'>Peristaltic &mdash; pH+</div></div>";
  html += "<div class='hw-item'><div class='hw-circle hw-idle' id='hwP2'>IDLE</div><div class='hw-name'>Base Pump</div><div class='hw-desc' id='hwP2Desc'>Peristaltic &mdash; pH&minus;</div></div>";
  html += "<div class='hw-item'><div class='hw-circle hw-idle' id='hwTDS'>IDLE</div><div class='hw-name'>Filter Pump</div><div class='hw-desc' id='hwTDSDesc'>Diaphragm &mdash; TDS</div></div>";
  html += "<div class='hw-item'><div class='hw-circle hw-idle' id='hwLED'>OFF</div><div class='hw-name'>LED Strip</div><div class='hw-desc' id='hwLEDDesc'>WS2812B &mdash; 60 LEDs</div></div>";
  html += "</div></div>";

  html += "<div class='footer'>CrayON v1.0 &mdash; Automated Water Quality Management System</div>";
  html += "</div>"; // end .wrap

  // ── SCRIPT
  html += "<script>";
  html += "var ledOn=false;";

  html += "function clamp(v,mn,mx){return Math.min(Math.max(v,mn),mx);}";

  html += "function setCard(id,cls){var c=document.getElementById(id);c.className='card '+cls;}";
  html += "function setPill(id,cls,txt){var p=document.getElementById(id);p.className='pill '+cls;p.textContent=txt;}";
  html += "function setOutTag(id,cls,txt){var t=document.getElementById(id);t.className='out-tag '+cls;t.textContent=txt;}";
  html += "function setHW(id,cls,txt){var h=document.getElementById(id);h.className='hw-circle '+cls;h.textContent=txt;}";

  html += "function triggerManual(w){var b={};b[w]=true;fetch('/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)}).catch(function(e){console.error(e);});}";

  html += "function toggleLED(){ledOn=!ledOn;var btn=document.getElementById('ledBtn');btn.textContent=ledOn?'LED Strip: FORCED ON':'LED Strip: AUTO';btn.className=ledOn?'btn btn-led on':'btn btn-led';fetch('/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({led:ledOn})}).catch(function(e){console.error(e);});}";

  html += "function updateData(){fetch('/data').then(function(r){return r.json();}).then(function(d){";

  // Connection
  html += "document.getElementById('connStatus').textContent='Connected';";
  html += "document.getElementById('connDot').className='conn-dot';";
  html += "document.getElementById('lastTime').textContent=new Date().toLocaleTimeString();";

  // ── Color sensor
  html += "var r=d.r_frequency||0,g=d.g_frequency||0,b=d.b_frequency||0;";
  html += "document.getElementById('rFreq').innerHTML=r;";
  html += "document.getElementById('gFreq').innerHTML=g;";
  html += "document.getElementById('bFreq').innerHTML=b;";
  html += "var mx=Math.max(r,g,b,1);";
  html += "document.getElementById('rBar').style.width=clamp(r/mx*100,2,100)+'%';";
  html += "document.getElementById('gBar').style.width=clamp(g/mx*100,2,100)+'%';";
  html += "document.getElementById('bBar').style.width=clamp(b/mx*100,2,100)+'%';";
  html += "if(d.color_status&&d.color_status.indexOf('GREEN')>=0){";
  html += "  setCard('colorCard','alert-red');setPill('colorPill','pill-red','GREEN DETECTED');";
  html += "  setOutTag('ledState','out-active','RED ALERT');setHW('hwLED','hw-alert','ALERT');";
  html += "  document.getElementById('hwLEDDesc').textContent='Algae detected!';";
  html += "}else{";
  html += "  setCard('colorCard','');setPill('colorPill','pill-green','CLEAR');";
  html += "  if(!ledOn){setOutTag('ledState','out-running','WHITE — NORMAL');setHW('hwLED','hw-active','ON');document.getElementById('hwLEDDesc').textContent='WS2812B — White';}";
  html += "  else{setOutTag('ledState','out-warn','FORCED ON');setHW('hwLED','hw-warn','ON');document.getElementById('hwLEDDesc').textContent='Manual override';}";
  html += "}";

  // ── TDS sensor
  html += "var tv=d.tds_value||0;";
  html += "document.getElementById('tdsValue').innerHTML=tv.toFixed(1)+' ppm';";
  html += "document.getElementById('tdsBar').style.width=clamp(tv/300*100,2,100)+'%';";
  html += "if(d.tds_status&&d.tds_status.indexOf('DIRTY')>=0){";
  html += "  setCard('tdsCard','alert-amber');setPill('tdsPill','pill-amber','DIRTY');";
  html += "  setOutTag('tdsState','out-active','FILTERING');setHW('hwTDS','hw-active','ACTIVE');";
  html += "  document.getElementById('hwTDSDesc').textContent='High TDS — filtering';";
  html += "}else{";
  html += "  setCard('tdsCard','');setPill('tdsPill','pill-green','CLEAN');";
  html += "  setOutTag('tdsState','out-idle','STANDBY');setHW('hwTDS','hw-idle','IDLE');";
  html += "  document.getElementById('hwTDSDesc').textContent='Diaphragm — TDS';";
  html += "}";

  // ── pH sensor
  html += "var pv=d.ph_value||7;";
  html += "document.getElementById('phValue').innerHTML=pv.toFixed(2);";
  html += "document.getElementById('phVolt').innerHTML=(d.ph_voltage||0).toFixed(3)+' V';";
  html += "document.getElementById('phBar').style.width=clamp(pv/14*100,2,100)+'%';";
  // pH bar color based on value
  html += "var phBarEl=document.getElementById('phBar');";
  html += "if(pv<4.5){phBarEl.className='bar-fill bar-red';}else if(pv>8.5){phBarEl.className='bar-fill bar-amber';}else{phBarEl.className='bar-fill bar-green';}";
  // pH status
  html += "setOutTag('pump1State','out-idle','IDLE');setOutTag('pump2State','out-idle','IDLE');";
  html += "setHW('hwP1','hw-idle','IDLE');setHW('hwP2','hw-idle','IDLE');";
  html += "document.getElementById('hwP1Desc').textContent='Peristaltic — pH+';";
  html += "document.getElementById('hwP2Desc').textContent='Peristaltic — pH−';";
  html += "if(d.ph_status&&d.ph_status.indexOf('ACIDIC')>=0&&d.ph_status.indexOf('Confirm')<0){";
  html += "  setCard('phCard','alert-green');setPill('phPill','pill-amber','ACIDIC');";
  html += "  setOutTag('pump2State','out-active','DOSING BASE');setHW('hwP2','hw-active','ACTIVE');";
  html += "  document.getElementById('hwP2Desc').textContent='Dosing base solution';";
  html += "}else if(d.ph_status&&d.ph_status.indexOf('ALKALINE')>=0&&d.ph_status.indexOf('Confirm')<0){";
  html += "  setCard('phCard','alert-amber');setPill('phPill','pill-red','ALKALINE');";
  html += "  setOutTag('pump1State','out-active','DOSING ACID');setHW('hwP1','hw-active','ACTIVE');";
  html += "  document.getElementById('hwP1Desc').textContent='Dosing acid solution';";
  html += "}else if(d.ph_status&&d.ph_status.indexOf('Confirm')>=0){";
  html += "  setCard('phCard','alert-blue');setPill('phPill','pill-blue','VERIFYING');";
  html += "  setOutTag('pump1State','out-warn','VERIFYING');setOutTag('pump2State','out-warn','VERIFYING');";
  html += "}else{";
  html += "  setCard('phCard','');setPill('phPill','pill-green','BALANCED');";
  html += "}";

  // LED sync
  html += "if(d.manual_led!==undefined){ledOn=d.manual_led;var btn=document.getElementById('ledBtn');btn.textContent=ledOn?'LED Strip: FORCED ON':'LED Strip: AUTO';btn.className=ledOn?'btn btn-led on':'btn btn-led';}";

  html += "}).catch(function(){";
  html += "document.getElementById('connStatus').textContent='Offline';";
  html += "document.getElementById('connDot').className='conn-dot err';";
  html += "});}";
  html += "setInterval(updateData,5000);updateData();";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}
void handleNotFound() {
  String message = "File Not Found\n\nURI: " + server.uri() + "\n";
  message += "Method: " + String(server.method() == HTTP_GET ? "GET" : "POST") + "\n";
  server.send(404, "text/plain", message);
}
