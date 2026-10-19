// GT-WT-01 Sensor-Nachbau fuer ESP32
// Sender: originales 433MHz ASK-Modul (VCC / GND / DATA)
//
// Saleae-Mitschnitt des Originals (digital.csv, 24.2C / 74%):
//   Puls 601us, 0 = 2065us, 1 = 4115us
//   37 Bit pro Paket, Payload A040F29560+0 -> GT-WT-02-Layout, Checksumme ok (44)
//   Pro Paket: Sync-Puls, Leading-Puls, dann 37x (Gap=Bit, Puls)
//   Sendungsbeginn: Sync-Puls ohne vorherigen Footer-Puls
//   Paketgrenze: Luecke 9040, Puls, Luecke 7650, Puls, Luecke 9035
//   Nachrichten-Ende: Luecke 9033, Puls, Luecke 3892, Puls
//   6 Pakete pro Sendung
//
// Frame (37 Bit, MSB zuerst):
//   ID(8) | Battery(1,0=ok) | Button(1) | Kanal(2) | Temp(12, Zweierkomplement x10)
//   | Feuchte(7) | Checksumme(6)
// Checksumme = Nibble-Summe (b0..b3, letztes Nibble b3&0xE) mod 64
//
// Datenquelle waehlbar: BME280 (I2C) oder MQTT-Topic mit JSON-Payload
// Laufzeit-Konfiguration ueber Serial-Menue (115200)
//
// PlatformIO lib_deps:
//   adafruit/Adafruit BME280 Library
//   knolleary/PubSubClient
//   bblanchon/ArduinoJson

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <cstring>
#include <cstddef>

void onMqttMessage(char *topic, byte *payload, unsigned int length);
void printMenu();

#define TX_PIN          25
#define SEND_INTERVAL_MS 45000UL  // Sendeintervall
#define TX_REPETITIONS  6

// Timing in Mikrosekunden, gemessen am Originalmitschnitt
#define PULSE_US    600
#define ZERO_US     2065
#define ONE_US      4115
#define GAP_END_US  9040          // letzte Daten-Puls -> Footer-Puls
#define GAP_MID_US  7650          // Footer-Puls -> Sync-Puls
#define GAP_PRE_US  9035          // Sync-Puls -> erster Daten-Puls
#define TAIL_US     3900          // Ende der Nachricht

char wifiSsid[33]   = "DEIN_SSID";
char wifiPass[65]   = "DEIN_PASSWORD";
char mqttServer[64] = "192.168.100.1";
#define MQTT_PORT   1883

#define SOURCE_BME280 0
#define SOURCE_MQTT   1

// Laufzeit-Konfiguration (ueber Serial-Menue aenderbar)
uint8_t sensorId = 0xA0;          // gueltig: 0..255
uint8_t channel = 1;              // gueltig: 1..3
bool buttonPressed = true;        // erste Sendung mit Button-Bit=1 -> Anlernen
uint8_t dataSource = SOURCE_BME280;
char mqttTopic[64] = "wetter/gt-wt01";
char jsonKeyTemp[48] = "temperature";
char jsonKeyHum[48] = "humidity";

// Persistente Konfiguration (EEPROM/Flash) mit Integritaets-Checksumme
#define EEPROM_SIZE   512
#define PERSIST_MAGIC   0x47575431u  // "GWT1"
#define PERSIST_VERSION 2

struct PersistData {
  uint32_t magic;
  uint8_t  version;
  uint8_t  dataSource;
  uint8_t  channel;
  uint8_t  sensorId;
  char     mqttTopic[64];
  char     jsonKeyTemp[48];
  char     jsonKeyHum[48];
  char     wifiSsid[33];
  char     wifiPass[65];
  char     mqttServer[64];
  uint32_t checksum;   // FNV-1a ueber alle Felder davor
};

uint32_t persistChecksum(const PersistData &p) {
  const uint8_t *b = reinterpret_cast<const uint8_t *>(&p);
  size_t n = offsetof(PersistData, checksum);
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 16777619u; }
  return h;
}

void saveConfig() {
  PersistData p;
  memset(&p, 0, sizeof(p));
  p.magic = PERSIST_MAGIC;
  p.version = PERSIST_VERSION;
  p.dataSource = dataSource;
  p.channel = channel;
  p.sensorId = sensorId;
  strncpy(p.mqttTopic, mqttTopic, sizeof(p.mqttTopic) - 1);
  strncpy(p.jsonKeyTemp, jsonKeyTemp, sizeof(p.jsonKeyTemp) - 1);
  strncpy(p.jsonKeyHum, jsonKeyHum, sizeof(p.jsonKeyHum) - 1);
  strncpy(p.wifiSsid, wifiSsid, sizeof(p.wifiSsid) - 1);
  strncpy(p.wifiPass, wifiPass, sizeof(p.wifiPass) - 1);
  strncpy(p.mqttServer, mqttServer, sizeof(p.mqttServer) - 1);
  p.checksum = persistChecksum(p);
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, p);
  bool ok = EEPROM.commit();
  EEPROM.end();
  Serial.println(ok ? F("Konfiguration gespeichert") : F("Speichern fehlgeschlagen"));
}

bool loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  PersistData p;
  EEPROM.get(0, p);
  EEPROM.end();
  if (p.magic != PERSIST_MAGIC || p.version != PERSIST_VERSION) return false;
  if (p.checksum != persistChecksum(p)) return false;
  if (p.dataSource > SOURCE_MQTT) return false;
  if (p.channel < 1 || p.channel > 3) return false;
  p.mqttTopic[sizeof(p.mqttTopic) - 1] = 0;
  p.jsonKeyTemp[sizeof(p.jsonKeyTemp) - 1] = 0;
  p.jsonKeyHum[sizeof(p.jsonKeyHum) - 1] = 0;
  p.wifiSsid[sizeof(p.wifiSsid) - 1] = 0;
  p.wifiPass[sizeof(p.wifiPass) - 1] = 0;
  p.mqttServer[sizeof(p.mqttServer) - 1] = 0;
  if (p.mqttTopic[0] == 0 || p.jsonKeyTemp[0] == 0 || p.jsonKeyHum[0] == 0) return false;
  if (p.wifiSsid[0] == 0 || p.mqttServer[0] == 0) return false;
  dataSource = p.dataSource;
  channel = p.channel;
  sensorId = p.sensorId;
  strncpy(mqttTopic, p.mqttTopic, sizeof(mqttTopic) - 1);
  strncpy(jsonKeyTemp, p.jsonKeyTemp, sizeof(jsonKeyTemp) - 1);
  strncpy(jsonKeyHum, p.jsonKeyHum, sizeof(jsonKeyHum) - 1);
  strncpy(wifiSsid, p.wifiSsid, sizeof(wifiSsid) - 1);
  strncpy(wifiPass, p.wifiPass, sizeof(wifiPass) - 1);
  strncpy(mqttServer, p.mqttServer, sizeof(mqttServer) - 1);
  return true;
}

WiFiClient espClient;
PubSubClient mqtt(espClient);
Adafruit_BME280 bme;

float mqttTemp = NAN;
float mqttHum = NAN;
unsigned long mqttLastMsg = 0;
unsigned long lastSend = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
bool wifiConnected = false;
bool wifiConnecting = false;
bool bmeFound = false;

void setup() {
  Serial.begin(115200);
  btStop();
  randomSeed(esp_random());
  if (loadConfig()) {
    Serial.println(F("Konfiguration aus Flash geladen"));
  } else {
    Serial.println(F("Keine gueltige Konfiguration im Flash - Defaults verwendet"));
    sensorId = random(1, 255);
  }
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);

  Wire.begin();
  bmeFound = bme.begin(0x76) || bme.begin(0x77);

  mqtt.setServer(mqttServer, MQTT_PORT);
  mqtt.setBufferSize(2048);
  mqtt.setCallback(onMqttMessage);

  printMenu();
}

void pulse() {
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(TX_PIN, LOW);
}

uint8_t calcChecksum(const uint8_t *b) {
  int sum = (b[0] >> 4) + (b[0] & 0xF)
          + (b[1] >> 4) + (b[1] & 0xF)
          + (b[2] >> 4) + (b[2] & 0xF)
          + (b[3] >> 4) + (b[3] & 0xE);
  return sum & 0x3F;
}

void sendPacket(const uint8_t *msg) {
  // Leading-Puls; die Bits stecken in den Luecken DANACH
  pulse();
  for (int b = 0; b < 5; b++) {
    for (int i = 7; i >= 0; i--) {
      if (b == 4 && i < 3) break;  // nur 37 Bit
      delayMicroseconds(((msg[b] >> i) & 0x01) ? ONE_US : ZERO_US);
      pulse();
    }
  }
}

// Komplette Aussendung: 6 Pakete, Timing identisch zum Originalmitschnitt
void sendTransmission(float tempC, float humidity) {
  int16_t t = constrain((int16_t)roundf(tempC * 10.0f), -200, 600);
  uint16_t tempRaw = (uint16_t)t;
  uint8_t hum = (uint8_t)constrain((int)roundf(humidity), 0, 99);

  uint8_t msg[5] = {0};
  msg[0] = sensorId;
  msg[1] = ((channel - 1) & 0x03) << 4;
  if (buttonPressed) msg[1] |= 0x40;
  msg[1] |= (tempRaw >> 8) & 0x0F;
  msg[2] = tempRaw & 0xFF;
  msg[3] = (hum << 1) & 0xFE;

  uint8_t chk = calcChecksum(msg);
  msg[3] |= (chk >> 5) & 0x01;
  msg[4] = (chk & 0x1F) << 3;

  for (int r = 0; r < TX_REPETITIONS; r++) {
    noInterrupts();  // Jitter waehrend der Bitfolge vermeiden
    if (r > 0) {
      delayMicroseconds(GAP_END_US);
      pulse();
      delayMicroseconds(GAP_MID_US);
    }
    pulse();
    delayMicroseconds(GAP_PRE_US);
    sendPacket(msg);
    interrupts();
  }

  // Nachrichten-Ende wie im Original
  noInterrupts();
  delayMicroseconds(GAP_PRE_US);
  pulse();
  delayMicroseconds(TAIL_US);
  pulse();
  interrupts();

  buttonPressed = false;  // nach dem Anlernen normal senden
}

// ---------------- Datenquellen ----------------

bool readSource(float &tempC, float &hum) {
  if (dataSource == SOURCE_BME280) {
    if (!bmeFound) return false;
    tempC = bme.readTemperature();
    hum = bme.readHumidity();
    return !isnan(tempC) && !isnan(hum);
  }
  // MQTT: nur verwenden, wenn die letzte Nachricht nicht aelter als 15 Minuten ist
  if (millis() - mqttLastMsg > 900000UL) return false;
  tempC = mqttTemp;
  hum = mqttHum;
  return !isnan(tempC) && !isnan(hum);
}

// Punkt-Pfad: '.' trennt Ebenen, '\.' escapet einen Punkt im Attributnamen
// z.B. "sensor.tempC" oder "Temp\.Aussen"
bool jsonPath(JsonVariantConst root, const char *path, float &out) {
  JsonVariantConst v = root;
  const char *p = path;
  while (true) {
    char key[48];
    size_t len = 0;
    while (*p) {
      if (*p == '\\' && p[1]) { key[len++] = p[1]; p += 2; }
      else if (*p == '.') { p++; break; }
      else key[len++] = *p++;
      if (len >= sizeof(key)) return false;
    }
    if (len == 0) return false;
    key[len] = 0;
    v = v[key];
    if (v.isNull()) return false;
    if (!*p) break;
  }
  if (!v.is<float>()) {
    if (v.is<const char*>()) {
      const char *s = v.as<const char*>();
      char *e = nullptr;
      float f = strtof(s, &e);
      if (e != s) { out = f; return true; }
    }
    return false;
  }
  out = v.as<float>();
  return true;
}

void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("MQTT JSON-Fehler (%s): %.*s\n", err.c_str(), (int)min(length, 200u), (char *)payload);
    return;
  }
  float t, h;
  bool okT = jsonPath(doc.as<JsonVariantConst>(), jsonKeyTemp, t);
  bool okH = jsonPath(doc.as<JsonVariantConst>(), jsonKeyHum, h);
  Serial.printf("MQTT %s: Temp-Filter '%s' %s, Hum-Filter '%s' %s\n",
                topic, jsonKeyTemp, okT ? "ok" : "FEHLER",
                jsonKeyHum, okH ? "ok" : "FEHLER");
  if (!okT || !okH)
    Serial.printf("Payload: %.*s\n", (int)min(length, 200u), (char *)payload);
  if (okT) mqttTemp = t;
  if (okH) mqttHum = h;
  if (okT || okH) mqttLastMsg = millis();
}

void maintainMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) { wifiConnected = false; Serial.println(F("WLAN getrennt")); }
    // begin() nur einmal pro Versuch; Retry erst nach 30s
    if (!wifiConnecting || millis() - lastWifiAttempt > 30000UL) {
      wifiConnecting = true;
      lastWifiAttempt = millis();
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSsid, wifiPass);
      Serial.printf("Verbinde WLAN (STA), SSID: %s\n", wifiSsid);
    }
    return;
  }
  wifiConnecting = false;
  if (!wifiConnected) {
    wifiConnected = true;
    Serial.printf("WLAN verbunden, IP: %s Gateway: %s Maske: %s\n",
                  WiFi.localIP().toString().c_str(),
                  WiFi.gatewayIP().toString().c_str(),
                  WiFi.subnetMask().toString().c_str());
  }
  if (!mqtt.connected()) {
    if (millis() - lastMqttAttempt > 5000UL) {
      lastMqttAttempt = millis();
      mqtt.setServer(mqttServer, MQTT_PORT);
      if (mqtt.connect("gtwt01-esp32")) {
        Serial.printf("MQTT verbunden, abonniere %s\n", mqttTopic);
        mqtt.subscribe(mqttTopic);
      } else {
        Serial.printf("MQTT nicht erreichbar: %s:%d (rc=%d)\n", mqttServer, MQTT_PORT, mqtt.state());
      }
    }
    return;
  }
  mqtt.loop();
}

// ---------------- Serial-Menue ----------------

void printMenu() {
  Serial.println(F("\n=== GT-WT-01 ESP32 ==="));
  Serial.printf("Quelle: %s | Kanal: %u | ID: %u | Topic: %s\n",
                dataSource == SOURCE_BME280 ? "BME280" : "MQTT",
                channel, sensorId, mqttTopic);
  Serial.printf("JSON Temp: %s | JSON Hum: %s\n", jsonKeyTemp, jsonKeyHum);
  Serial.println(F("1 = Datenquelle BME280"));
  Serial.println(F("2 = Datenquelle MQTT"));
  Serial.println(F("3 <topic> = MQTT-Topic setzen"));
  Serial.println(F("4 <pfad> = JSON-Filter Temperatur ('.'=Ebene, '\\.'=Punkt im Namen)"));
  Serial.println(F("5 <pfad> = JSON-Filter Feuchte (z.B. sensor.hum oder Temp\\.Aussen)"));
  Serial.println(F("6 <1|2|3> = Kanal wechseln"));
  Serial.println(F("7 <0-255> = Sende-ID setzen"));
  Serial.println(F("8 = Button-Bit setzen und sofort senden"));
  Serial.println(F("9 = Konfiguration persistent speichern"));
  Serial.println(F("w <ssid> = WLAN-SSID setzen"));
  Serial.println(F("p <password> = WLAN-Passwort setzen"));
  Serial.println(F("b <ip/host> = MQTT-Broker setzen"));
  Serial.println(F("m = Menue anzeigen"));
}

void processCommand(String line) {
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  String arg = line.substring(1);
  arg.trim();

  switch (cmd) {
    case '1':
      dataSource = SOURCE_BME280;
      wifiConnecting = false;
      wifiConnected = false;
      WiFi.disconnect(false);
      WiFi.mode(WIFI_OFF);
      Serial.println(F("Quelle: BME280"));
      break;
    case '2':
      dataSource = SOURCE_MQTT;
      Serial.println(F("Quelle: MQTT"));
      break;
    case '3':
      if (arg.length() > 0 && arg.length() < sizeof(mqttTopic)) {
        arg.toCharArray(mqttTopic, sizeof(mqttTopic));
        if (mqtt.connected()) { mqtt.unsubscribe(mqttTopic); mqtt.subscribe(mqttTopic); }
        Serial.printf("Topic: %s\n", mqttTopic);
      } else Serial.println(F("Topic leer/zu lang"));
      break;
    case '4':
      if (arg.length() > 0 && arg.length() < sizeof(jsonKeyTemp)) {
        arg.toCharArray(jsonKeyTemp, sizeof(jsonKeyTemp));
        Serial.printf("JSON-Filter Temp: %s\n", jsonKeyTemp);
      } else Serial.println(F("Filter leer/zu lang"));
      break;
    case '5':
      if (arg.length() > 0 && arg.length() < sizeof(jsonKeyHum)) {
        arg.toCharArray(jsonKeyHum, sizeof(jsonKeyHum));
        Serial.printf("JSON-Filter Hum: %s\n", jsonKeyHum);
      } else Serial.println(F("Filter leer/zu lang"));
      break;
    case '6':
      if (arg == "1" || arg == "2" || arg == "3") {
        channel = arg.toInt();
        Serial.printf("Kanal: %u\n", channel);
      } else Serial.println(F("Kanal muss 1, 2 oder 3 sein"));
      break;
    case '7': {
      long id = arg.toInt();
      if (arg.length() > 0 && id >= 0 && id <= 255) {
        sensorId = (uint8_t)id;
        Serial.printf("ID: %u\n", sensorId);
      } else Serial.println(F("ID muss 0-255 sein"));
      break;
    }
    case '8': {
      float t, h;
      if (!readSource(t, h)) {
        Serial.println(F("Keine gueltigen Messwerte - Sendung abgebrochen"));
        break;
      }
      buttonPressed = true;
      Serial.printf("Sofortsendung: %.1f C / %.0f %%\n", t, h);
      sendTransmission(t, h);
      break;
    }
    case '9':
      saveConfig();
      break;
    case 'w':
      if (arg.length() > 0 && arg.length() < sizeof(wifiSsid)) {
        arg.toCharArray(wifiSsid, sizeof(wifiSsid));
        WiFi.disconnect(false);
        wifiConnecting = false;
        wifiConnected = false;
        Serial.printf("WLAN-SSID: %s (erneuter Verbindungsaufbau)\n", wifiSsid);
      } else Serial.println(F("SSID leer/zu lang"));
      break;
    case 'p':
      if (arg.length() < sizeof(wifiPass)) {
        arg.toCharArray(wifiPass, sizeof(wifiPass));
        WiFi.disconnect(false);
        wifiConnecting = false;
        wifiConnected = false;
        Serial.println(F("WLAN-Passwort gesetzt (erneuter Verbindungsaufbau)"));
      } else Serial.println(F("Passwort zu lang"));
      break;
    case 'b':
      if (arg.length() > 0 && arg.length() < sizeof(mqttServer)) {
        arg.toCharArray(mqttServer, sizeof(mqttServer));
        mqtt.disconnect();
        Serial.printf("MQTT-Broker: %s:%d\n", mqttServer, MQTT_PORT);
      } else Serial.println(F("Broker leer/zu lang"));
      break;
    case 'm':
      printMenu();
      break;
    default:
      Serial.println(F("Unbekanntes Kommando, 'm' fuer Menue"));
  }
  printMenu();
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') { processCommand(line); line = ""; }
    else if (c != '\r') line += c;
  }
}

void loop() {
  handleSerial();
  if (dataSource == SOURCE_MQTT) maintainMqtt();

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    float t, h;
    if (readSource(t, h)) sendTransmission(t, h);
    else Serial.println(F("Datenquelle liefert keine gueltigen Werte"));
  }
}

