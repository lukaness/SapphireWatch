// src/main.cpp
// Smartwatch Full Firmware - Single-file main.cpp
// Features:
// - ESP32-CAM (OV2640) capture -> microSD (SPI)
// - TFT_eSPI UI: clock + app launcher + Camera/Msgs/Contacts/Files
// - Local account/contacts storage on SD
// - BLE GATT phone sync (JSON commands)
// - ESP-NOW auto-discovery + peer to peer messaging
// - Internet fallback via HTTP POST
// - SD storage warning, message queue & delivery logic
//
// Adjust constants / pins at top to match your hardware.

// ---------- includes ----------
#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include "esp_camera.h"
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>

// ---------- HARDWARE / CONFIG ----------
#define SD_CS 5                 // SD card CS (change if needed)
#define BUZZER_PIN 14
#define VIB_PIN 15
#define LED_PIN 2
#define BUTTON_A 0              // launcher
#define BUTTON_B 35             // back/select

// Camera pins for AI-Thinker / ESP32-CAM (OV2640)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// BLE UUIDs
#define SERVICE_UUID        "f0000001-0451-4000-b000-000000000000"
#define CHAR_WRITE_UUID     "f0000002-0451-4000-b000-000000000001"
#define CHAR_NOTIFY_UUID    "f0000003-0451-4000-b000-000000000002"

// Internet server (placeholder)
const char* INTERNET_SERVER_HOST = "example.com"; // change to your server
const char* INTERNET_API_SEND = "/api/send_message";
const char* INTERNET_API_PEERS = "/api/get_online_peers";

// Storage & persistence
const int STORAGE_WARNING_PERCENT = 90;
const char* ACCOUNT_PATH = "/account.json";
const char* CONTACTS_PATH = "/contacts.json";
const char* MESSAGE_QUEUE_PATH = "/msg_queue.json";

TFT_eSPI tft = TFT_eSPI();
WebServer server(80);

// ---------- GLOBALS ----------
bool wifiConnected = false;
BLECharacteristic *pWriteChar = nullptr;
BLECharacteristic *pNotifyChar = nullptr;

// Account structure
struct Account {
  String username;
  String deviceId;
  String token;
};
Account localAccount;

// Contact structure (includes MAC + flags)
struct Contact {
  String name;
  String watchId;           // unique id (could be username)
  bool isWatch = false;     // true if it's one of your watches
  uint8_t mac[6] = {0};     // peer MAC if known via ESP-NOW
  bool isOnlineESPNow = false;
  bool isOnlineInternet = false;
};

// Message structure
struct Message {
  String from;
  String to;
  String text;
  unsigned long timestamp;
  bool sent;
};

// In-memory lists
std::vector<Contact> contacts;
std::vector<Message> msgQueue;
std::map<String, std::array<uint8_t,6>> peerMacs; // watchId -> mac

unsigned long lastClockUpdate = 0;
unsigned long lastStorageCheck = 0;
unsigned long lastHelloSent = 0;
unsigned long lastInternetDiscovery = 0;

#define CHUNK_SIZE 512

// ---------- UTILITIES ----------
void uiLog(const String &s) {
  Serial.println(s);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // keep log short on screen
  String shortS = s;
  if (shortS.length() > 24) shortS = shortS.substring(0,24);
  tft.drawString(shortS, 4, tft.height() - 12);
}

String nowStr() {
  unsigned long s = millis()/1000;
  unsigned long hh = (s/3600)%24;
  unsigned long mm = (s/60)%60;
  unsigned long ss = s%60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hh, mm, ss);
  return String(buf);
}

// ---------- CAMERA ----------
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

String savePhotoToSD() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) { uiLog("Cam capture failed"); return ""; }
  String path = "/photo_" + String(millis()) + ".jpg";
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f) { uiLog("SD open for photo failed"); esp_camera_fb_return(fb); return ""; }
  f.write(fb->buf, fb->len);
  f.close();
  esp_camera_fb_return(fb);
  uiLog("Saved " + path);
  return path;
}

// ---------- SD ----------
bool initSD() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD begin failed");
    return false;
  }
  uint64_t total = SD.totalBytes();
  uiLog("SD total MB=" + String(total / (1024 * 1024)));
  return true;
}

void checkStorageAndWarn() {
  if (!SD.begin(SD_CS)) return;
  uint64_t total = SD.totalBytes();
  uint64_t used = SD.usedBytes();
  if (total == 0) return;
  int percent = (int)((used * 100) / total);
  Serial.printf("SD used %llu/%llu (%d%%)\n", used, total, percent);
  if (percent >= STORAGE_WARNING_PERCENT) {
    tft.setTextColor(TFT_RED);
    tft.drawString("Storage Nearly Full!", 6, tft.height() - 30);
    // vibration & buzzer
    digitalWrite(VIB_PIN, HIGH);
    delay(200);
    digitalWrite(VIB_PIN, LOW);
    tone(BUZZER_PIN, 2000, 200);
  }
}

// ---------- PERSISTENCE: contacts / account / queue ----------
void saveContactsToSD() {
  StaticJsonDocument<2048> doc;
  JsonArray arr = doc.createNestedArray("contacts");
  for (auto &c: contacts) {
    JsonObject o = arr.createNestedObject();
    o["name"] = c.name;
    o["watchId"] = c.watchId;
    o["isWatch"] = c.isWatch;
    // store mac as hex string if available
    char macs[32] = {0};
    if (c.mac[0] || c.mac[1] || c.mac[2]) {
      snprintf(macs, sizeof(macs), "%02X:%02X:%02X:%02X:%02X:%02X",
               c.mac[0], c.mac[1], c.mac[2], c.mac[3], c.mac[4], c.mac[5]);
      o["mac"] = macs;
    }
  }
  File f = SD.open(CONTACTS_PATH, FILE_WRITE);
  if (!f) { uiLog("Failed save contacts"); return; }
  serializeJson(doc, f);
  f.close();
  uiLog("Contacts saved");
}

void loadContactsFromSD() {
  contacts.clear();
  if (!SD.exists(CONTACTS_PATH)) return;
  File f = SD.open(CONTACTS_PATH);
  if (!f) return;
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { uiLog("Contacts parse err"); return; }
  JsonArray arr = doc["contacts"].as<JsonArray>();
  for (JsonObject o: arr) {
    Contact c;
    c.name = String((const char*)o["name"]);
    c.watchId = String((const char*)o["watchId"]);
    c.isWatch = o["isWatch"];
    if (o.containsKey("mac")) {
      const char* macs = o["mac"];
      int a[6];
      if (sscanf(macs, "%x:%x:%x:%x:%x:%x", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 6) {
        for (int i=0;i<6;i++) c.mac[i] = (uint8_t)a[i];
      }
    }
    contacts.push_back(c);
  }
  uiLog("Loaded contacts:" + String(contacts.size()));
}

void saveAccountToSD() {
  StaticJsonDocument<256> doc;
  doc["username"] = localAccount.username;
  doc["deviceId"] = localAccount.deviceId;
  doc["token"] = localAccount.token;
  File f = SD.open(ACCOUNT_PATH, FILE_WRITE);
  if (!f) { uiLog("Failed save account"); return; }
  serializeJson(doc, f);
  f.close();
  uiLog("Account saved");
}

bool loadAccountFromSD() {
  if (!SD.exists(ACCOUNT_PATH)) return false;
  File f = SD.open(ACCOUNT_PATH);
  if (!f) return false;
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  localAccount.username = String((const char*)doc["username"]);
  localAccount.deviceId = String((const char*)doc["deviceId"]);
  localAccount.token = String((const char*)doc["token"]);
  return true;
}

void saveMsgQueueToSD() {
  StaticJsonDocument<4096> doc;
  JsonArray arr = doc.createNestedArray("queue");
  for (auto &m: msgQueue) {
    JsonObject o = arr.createNestedObject();
    o["from"] = m.from;
    o["to"] = m.to;
    o["text"] = m.text;
    o["ts"] = m.timestamp;
    o["sent"] = m.sent;
  }
  File f = SD.open(MESSAGE_QUEUE_PATH, FILE_WRITE);
  if (!f) { uiLog("Failed save queue"); return; }
  serializeJson(doc, f);
  f.close();
}

void loadMsgQueueFromSD() {
  msgQueue.clear();
  if (!SD.exists(MESSAGE_QUEUE_PATH)) return;
  File f = SD.open(MESSAGE_QUEUE_PATH);
  if (!f) return;
  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;
  JsonArray arr = doc["queue"].as<JsonArray>();
  for (JsonObject o: arr) {
    Message m;
    m.from = String((const char*)o["from"]);
    m.to = String((const char*)o["to"]);
    m.text = String((const char*)o["text"]);
    m.timestamp = o["ts"];
    m.sent = o["sent"];
    msgQueue.push_back(m);
  }
}

// ---------- ESP-NOW (peer discovery & messaging) ----------
// on receive callback will parse "hello" or "msg" payloads
void onEspNowRecvCB(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // print MAC
  char buf[32];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("ESPNOW from %s\n", buf);

  // parse JSON safely
  String s = String((const char*)incomingData).substring(0, len);
  uiLog("ESPNOW RX: " + s);
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, s);
  if (err) { uiLog("ESPNOW JSON err"); return; }

  const char* t = doc["type"];
  if (!t) return;
  String type = String(t);

  if (type == "hello") {
    String peerId = String((const char*)doc["watchId"]);
    // copy mac into peerMacs and contacts
    std::array<uint8_t,6> macArr;
    for (int i=0;i<6;i++) macArr[i] = mac[i];
    peerMacs[peerId] = macArr;

    // update or add contact
    bool found=false;
    for (auto &c : contacts) {
      if (c.watchId == peerId) {
        memcpy(c.mac, macArr.data(), 6);
        c.isOnlineESPNow = true;
        found = true;
        break;
      }
    }
    if (!found) {
      Contact nc;
      nc.name = peerId;
      nc.watchId = peerId;
      nc.isWatch = true;
      memcpy(nc.mac, macArr.data(), 6);
      nc.isOnlineESPNow = true;
      contacts.push_back(nc);
    }
    saveContactsToSD();
    uiLog("Discovered peer " + peerId);
  }
  else if (type == "msg") {
    String from = String((const char*)doc["from"]);
    String text = String((const char*)doc["text"]);
    Message m; m.from = from; m.to = localAccount.username; m.text = text; m.timestamp = millis()/1000; m.sent = true;
    msgQueue.push_back(m);
    saveMsgQueueToSD();
    uiLog("Msg from " + from + ": " + text);
  }
}

void initEspNow() {
  if (esp_now_init() != ESP_OK) {
    uiLog("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecvCB);
  uiLog("ESP-NOW inited");
}

void broadcastHello() {
  StaticJsonDocument<256> doc;
  doc["type"] = "hello";
  doc["watchId"] = localAccount.username.length() ? localAccount.username : String(WiFi.macAddress());
  String payload;
  serializeJson(doc, payload);
  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_err_t r = esp_now_send(bcast, (uint8_t*)payload.c_str(), payload.length()+1);
  if (r==ESP_OK) uiLog("Hello broadcast");
  else uiLog("Hello failed");
}

bool sendMessageEspNowByWatchId(const String &watchId, const String &text) {
  if (peerMacs.find(watchId) == peerMacs.end()) return false;
  auto macArr = peerMacs[watchId];
  uint8_t dest[6];
  for (int i=0;i<6;i++) dest[i] = macArr[i];
  StaticJsonDocument<256> doc;
  doc["type"] = "msg";
  doc["from"] = localAccount.username.length()?localAccount.username:WiFi.macAddress();
  doc["text"] = text;
  String payload;
  serializeJson(doc, payload);
  esp_err_t r = esp_now_send(dest, (uint8_t*)payload.c_str(), payload.length()+1);
  return (r==ESP_OK);
}

// ---------- INTERNET MODE ----------
// simple discovery via HTTP GET (server returns JSON array of {name,watchId})
void discoverPeersInternet() {
  if (!wifiConnected) return;
  HTTPClient http;
  String url = String("http://") + INTERNET_SERVER_HOST + INTERNET_API_PEERS;
  http.begin(url);
  int code = http.GET();
  if (code != 200) { uiLog("Peer discovery HTTP failed: " + String(code)); http.end(); return; }
  String resp = http.getString();
  http.end();
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, resp);
  if (err) { uiLog("Discover parse err"); return; }
  JsonArray arr = doc.as<JsonArray>();
  for (JsonObject o: arr) {
    String name = String((const char*)o["name"]);
    String id   = String((const char*)o["watchId"]);
    bool found=false;
    for (auto &c: contacts) {
      if (c.watchId == id) { c.isOnlineInternet = true; found=true; break; }
    }
    if (!found) {
      Contact nc; nc.name = name; nc.watchId = id; nc.isWatch = true; nc.isOnlineInternet = true;
      contacts.push_back(nc);
    }
  }
  saveContactsToSD();
  uiLog("Internet peers updated");
}

bool sendMessageInternetHTTP(const Message &m) {
  if (!wifiConnected) return false;
  HTTPClient http;
  String url = String("http://") + INTERNET_SERVER_HOST + INTERNET_API_SEND;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  StaticJsonDocument<512> doc;
  doc["from"] = m.from;
  doc["to"] = m.to;
  doc["text"] = m.text;
  String body; serializeJson(doc, body);
  int code = http.POST(body);
  String resp = http.getString();
  http.end();
  uiLog("HTTP send code=" + String(code));
  return (code==200);
}

// ---------- MESSAGE DELIVERY ----------
void tryDeliverQueuedMessages() {
  for (auto &m : msgQueue) {
    if (m.sent) continue;
    bool delivered = false;
    // find target contact by name or watchId
    for (auto &c : contacts) {
      if (c.name == m.to || c.watchId == m.to) {
        // try ESP-NOW if peer known
        if (c.isOnlineESPNow && peerMacs.find(c.watchId) != peerMacs.end()) {
          if (sendMessageEspNowByWatchId(c.watchId, m.text)) { delivered = true; }
        }
        // else try internet
        if (!delivered && wifiConnected && c.isOnlineInternet) {
          if (sendMessageInternetHTTP(m)) { delivered = true; }
        }
        break;
      }
    }
    if (delivered) m.sent = true;
  }
  saveMsgQueueToSD();
}

// ---------- BLE PHONE SYNC ----------
class PhoneWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string val = pChar->getValue();
    if (val.length()==0) return;
    String s = String(val.c_str());
    uiLog("BLE RX: " + s);
    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, s);
    if (err) { uiLog("BLE JSON err"); return; }
    const char* action = doc["action"];
    if (!action) return;
    String act = String(action);

    if (act == "sync_contacts") {
      // load contacts array
      JsonArray arr = doc["contacts"].as<JsonArray>();
      contacts.clear();
      for (JsonObject c: arr) {
        Contact cc;
        cc.name = String((const char*)c["name"]);
        cc.watchId = String((const char*)c["watchId"]);
        cc.isWatch = c["isWatch"];
        // optional mac as string
        if (c.containsKey("mac")) {
          const char* macs = c["mac"];
          int a[6];
          if (sscanf(macs, "%x:%x:%x:%x:%x:%x", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) == 6) {
            for (int i=0;i<6;i++) cc.mac[i] = (uint8_t)a[i];
          }
        }
        contacts.push_back(cc);
      }
      saveContactsToSD();
      StaticJsonDocument<128> r; r["status"]="ok"; r["action"]="sync_contacts";
      String out; serializeJson(r, out);
      pNotifyChar->setValue(out.c_str()); pNotifyChar->notify();
    }
    else if (act == "send_message") {
      String to = String((const char*)doc["to"]);
      String text = String((const char*)doc["text"]);
      Message m; m.from = localAccount.username.length()?localAccount.username:WiFi.macAddress(); m.to = to; m.text = text; m.timestamp = millis()/1000; m.sent = false;
      msgQueue.push_back(m); saveMsgQueueToSD();
      tryDeliverQueuedMessages();
      StaticJsonDocument<128> r; r["status"]="queued"; r["action"]="send_message";
      String out; serializeJson(r,out); pNotifyChar->setValue(out.c_str()); pNotifyChar->notify();
    }
    else if (act == "take_photo") {
      String path = savePhotoToSD();
      StaticJsonDocument<256> r; r["action"]="photo_saved"; r["path"] = path;
      String out; serializeJson(r,out); pNotifyChar->setValue(out.c_str()); pNotifyChar->notify();
    }
    else if (act == "set_account") {
      localAccount.username = String((const char*)doc["username"]);
      localAccount.deviceId = String((const char*)doc["deviceId"]);
      localAccount.token = String((const char*)doc["token"]);
      saveAccountToSD();
      StaticJsonDocument<128> r; r["status"]="ok"; r["action"]="set_account";
      String out; serializeJson(r,out); pNotifyChar->setValue(out.c_str()); pNotifyChar->notify();
    }
  }
};

void initBle() {
  BLEDevice::init("MyWatch");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pWriteChar = pService->createCharacteristic(CHAR_WRITE_UUID, BLECharacteristic::PROPERTY_WRITE);
  pWriteChar->setCallbacks(new PhoneWriteCB());
  pNotifyChar = pService->createCharacteristic(CHAR_NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pNotifyChar->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->start();
  uiLog("BLE started");
}

// ---------- WIFI helpers & HTTP endpoint ----------
void initWiFi(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  uiLog("WiFi connecting...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) delay(200);
  if (WiFi.status() == WL_CONNECTED) { wifiConnected = true; uiLog("WiFi connected"); }
  else { wifiConnected = false; uiLog("WiFi failed"); }
}

// HTTP handler for incoming messages (internet mode, optional)
void handleIncomingMessage() {
  if (server.method() != HTTP_POST) { server.send(405); return; }
  String body = server.arg(0);
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) { server.send(400); return; }
  Message m; m.from = String((const char*)doc["from"]); m.to = String((const char*)doc["to"]); m.text = String((const char*)doc["text"]); m.timestamp = millis()/1000; m.sent=true;
  msgQueue.push_back(m); saveMsgQueueToSD();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// ---------- UI (simple) ----------
void drawClockScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(nowStr(), tft.width() / 2, tft.height() / 2 - 10);
  tft.setTextSize(1);
  tft.drawString("A:Apps B:Back", 6, tft.height() - 12);
}

void drawLauncher() {
  tft.fillScreen(TFT_NAVY);
  tft.setTextSize(2); tft.setTextColor(TFT_WHITE);
  tft.drawString("Apps", 6, 4);
  const char* labels[] = {"Clock","Camera","Messages","Contacts","Files","WiFi"};
  int idx=0;
  for (int r=0;r<3;r++) for (int c=0;c<2;c++) {
    int x = 10 + c*110; int y=30 + r*60;
    tft.fillRoundRect(x,y,100,50,8,TFT_BLACK);
    tft.setTextSize(1); tft.drawString(labels[idx], x+8, y+18);
    idx++; if (idx>=6) break;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(50);
  pinMode(BUZZER_PIN, OUTPUT); pinMode(VIB_PIN, OUTPUT); pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_A, INPUT_PULLUP); pinMode(BUTTON_B, INPUT_PULLUP);
  tft.init(); tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  uiLog("Booting...");
  if (!initSD()) uiLog("SD not ready");
  if (!initCamera()) uiLog("Camera not ready");
  // load persistent state
  loadAccountFromSD(); loadContactsFromSD(); loadMsgQueueFromSD();
  // networking & comms
  initEspNow();              // ESP-NOW for offline discovery/messaging
  initBle();                 // BLE for phone sync
  // webserver endpoints
  server.on("/incoming_message", HTTP_POST, handleIncomingMessage);
  server.begin();
  drawClockScreen();
}

// ---------- MAIN LOOP ----------
void loop() {
  unsigned long now = millis();
  if (now - lastClockUpdate >= 1000) { lastClockUpdate = now; drawClockScreen(); }
  if (now - lastStorageCheck >= 30000) { lastStorageCheck = now; checkStorageAndWarn(); }
  // button simple handling
  if (digitalRead(BUTTON_A) == LOW) { drawLauncher(); delay(300); }
  // message delivery attempts
  static unsigned long lastDeliver = 0;
  if (millis() - lastDeliver > 10000) { lastDeliver = millis(); tryDeliverQueuedMessages(); }
  // esp-now discovery broadcast every 10s
  if (millis() - lastHelloSent > 10000) { lastHelloSent = millis(); broadcastHello(); }
  // internet discovery if wifi connected
  if (wifiConnected && (millis() - lastInternetDiscovery > 30000)) { lastInternetDiscovery = millis(); discoverPeersInternet(); }
  // handle web server
  if (wifiConnected) server.handleClient();
}

// ---------- End of file ----------
