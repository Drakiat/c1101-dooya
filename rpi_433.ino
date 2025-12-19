/*
  WiFi + MQTT (Adafruit IO 
  - Connects to WiFi
  - Connects to io.adafruit.com MQTT
  - Subscribes to: <AIO_USERNAME>/feeds/tv-mount
  - On message "UP"/"DOWN": toggles LED and calls tv_up_routine()/tv_down_routine()

  Libraries:
    - WiFi (ESP32 / Pico W core)
    - PubSubClient by Nick O'Leary
*/
#include <RadioLib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// -------------------------
// HARDWARE
// -------------------------
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define SCK_PIN   18
#define MISO_PIN  16
#define MOSI_PIN  19
#define SS_PIN    17

// -------------------------
// USER SETTINGS TO CHANGE
// -------------------------
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD!";

const char* AIO_USERNAME  = "ADAFRUIT USERNAME";
const char* AIO_KEY       = "ADAFRUIT KEY";

// Adafruit IO MQTT
const char* MQTT_HOST = "io.adafruit.com";
const uint16_t MQTT_PORT = 1883;

// Feed name in your Python: io.get_feed("tv-mount")
const char* FEED_NAME = "FEED NAME";

// client id can be whatever
const char* MQTT_CLIENT_ID = "rpi-pico-2w-arduino";

static const uint8_t GDO0_PIN = 20;

// CC1101: CS=17, GDO0=20, RST=NC, GDO2=21 (unused here)
CC1101 radio = new Module(SS_PIN, GDO0_PIN, RADIOLIB_NC, 21);

// ---------------- RAW capture buffer ----------------
static const uint16_t MAX_EDGES = 12000;
volatile int32_t capTimings[MAX_EDGES];   // +HIGH us, -LOW us
volatile uint16_t capCount = 0;

volatile uint32_t lastEdgeUs = 0;
volatile bool lastLevelHigh = false;
volatile bool capturing = false;

// ---------------- PASTE RAW HERE TO TRANSMIT ----------------

//TV DOWN SIGNAL
const int32_t RAW[] = {
  687434,  -1513,  347,  -751,  708,  -359,  342,  -771

};
const size_t RAW_LEN = sizeof(RAW)/sizeof(RAW[0]);
//TV UP
const int32_t RAW2[] = {
  1010043,  -1505,  348,  -768,  697,  -348,  356,  -767
  
};
const size_t RAW2_LEN = sizeof(RAW)/sizeof(RAW[0]);



// -------------------------
void transmitRawTimings(const int32_t* raw, size_t rawLen, uint8_t repeats = 1, uint32_t gapUs = 8000);
void tv_up_routine(){
  if (RAW2_LEN > 0) {
      Serial.print(F("Transmitting pasted RAW2[] (len="));
      Serial.print(RAW2_LEN);
      Serial.println(F(") ..."));
      transmitRawTimings(RAW2, RAW2_LEN, 1, 8000);
      Serial.println(F("Done."));
    } else {
      Serial.println(F("RAW2 is empty. Paste RAW2[] above."));
    }
  }

void tv_down_routine(){if (RAW_LEN > 0) {
      Serial.print(F("Transmitting pasted RAW2[] (len="));
      Serial.print(RAW_LEN);
      Serial.println(F(") ..."));
      transmitRawTimings(RAW, RAW_LEN, 1, 8000);
      Serial.println(F("Done."));
    } else {
      Serial.println(F("RAW is empty. Paste RAW[] above."));
    }
  }

// -------------------------
// GLOBALS
// -------------------------
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastBlinkMs = 0;
bool blinkState = false;

// Topic format for Adafruit IO MQTT:
//   <username>/feeds/<feed>
String mqttTopic;

// -------------------------
// WIFI
// -------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// -------------------------
// MQTT CALLBACK
// -------------------------
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Convert payload to String
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("Topic ");
  Serial.print(topic);
  Serial.print(" received message: ");
  Serial.println(msg);

  // Match behavior of your Python:
  if (msg == "UP") {
    digitalWrite(LED_BUILTIN, HIGH);
    tv_up_routine();
  } else if (msg == "DOWN") {
    digitalWrite(LED_BUILTIN, LOW);
    tv_down_routine();
  }
}

// -------------------------
// MQTT CONNECT / RECONNECT
// -------------------------
void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  Serial.print("Connecting to MQTT broker...");
  while (!mqtt.connected()) {
    // Adafruit IO uses username + key as password
    if (mqtt.connect(MQTT_CLIENT_ID, AIO_USERNAME, AIO_KEY)) {
      Serial.println(" connected!");
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 2s...");
      delay(2000);
    }
  }

  // Subscribe
  Serial.print("Subscribing to: ");
  Serial.println(mqttTopic);
  mqtt.subscribe(mqttTopic.c_str());
}


// ---------------- GDO0 ISR stuff ----------------
void onGdo0Change() {
  if(!capturing) return;

  uint32_t now = micros();
  uint32_t dt  = now - lastEdgeUs;
  lastEdgeUs = now;

  if(capCount >= MAX_EDGES) {
    capturing = false;
    return;
  }

  capTimings[capCount++] = lastLevelHigh ? (int32_t)dt : -(int32_t)dt;
  lastLevelHigh = digitalRead(GDO0_PIN);
}

// ---------------- Helpers ----------------
void printCapturedAsArray() {
  noInterrupts();
  uint16_t n = capCount;
  interrupts();

  Serial.println();
  Serial.print(F("Captured edges: "));
  Serial.println(n);

  Serial.println(F("const int32_t RAW[] = {"));
  for(uint16_t i = 0; i < n; i++) {
    int32_t v;
    noInterrupts();
    v = capTimings[i];
    interrupts();

    Serial.print(F("  "));
    Serial.print(v);
    if(i + 1 < n) Serial.print(',');
    if((i % 8) == 7) Serial.println();
  }
  Serial.println();
  Serial.println(F("};"));
  Serial.println(F("const size_t RAW_LEN = sizeof(RAW)/sizeof(RAW[0]);"));
  Serial.println();
}

// Capture OOK edges function
void captureRaw(uint32_t captureMs) {
  int16_t st = radio.receiveDirectAsync();
  if(st != RADIOLIB_ERR_NONE) {
    Serial.print(F("receiveDirectAsync() failed, code "));
    Serial.println(st);
    return;
  }

  capCount = 0;
  lastEdgeUs = micros();
  lastLevelHigh = digitalRead(GDO0_PIN);
  capturing = true;

  attachInterrupt(digitalPinToInterrupt(GDO0_PIN), onGdo0Change, CHANGE);

  uint32_t t0 = millis();
  while(capturing && (millis() - t0 < captureMs)) {
    yield();
  }

  capturing = false;
  detachInterrupt(digitalPinToInterrupt(GDO0_PIN));
  radio.standby();

  // Append a tail duration for the last level (nice for replays)
  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  if(capCount < MAX_EDGES) {
    capTimings[capCount++] = lastLevelHigh ? (int32_t)dt : -(int32_t)dt;
  }
}

// Transmit a RAW timing array using CC1101 async direct TX.
// raw[i] > 0 => HIGH for raw[i] us
// raw[i] < 0 => LOW  for -raw[i] us
void transmitRawTimings(const int32_t* raw, size_t rawLen, uint8_t repeats, uint32_t gapUs) {

  if(raw == nullptr || rawLen == 0) {
    Serial.println(F("transmitRawTimings: empty RAW"));
    return;
  }

  for(uint8_t r = 0; r < repeats; r++) {
    int16_t st = radio.transmitDirectAsync();
    if(st != RADIOLIB_ERR_NONE) {
      Serial.print(F("transmitDirectAsync() failed, code "));
      Serial.println(st);
      return;
    }

    // drive the line connected to CC1101 GDO0
    pinMode(GDO0_PIN, OUTPUT);

    for(size_t i = 0; i < rawLen; i++) {
      int32_t v = raw[i];
      bool level = (v > 0);
      uint32_t us = (v > 0) ? (uint32_t)v : (uint32_t)(-v);

      digitalWrite(GDO0_PIN, level ? HIGH : LOW);

      // Split long delays into safe chunks
      while(us > 0) {
        uint32_t chunk = (us > 16000) ? 16000 : us;
        delayMicroseconds(chunk);
        us -= chunk;
        yield();
      }
    }

    // low
    digitalWrite(GDO0_PIN, LOW);

    radio.standby();
    pinMode(GDO0_PIN, INPUT);

    // Inter-frame gap
    uint32_t g = gapUs;
    while(g > 0) {
      uint32_t chunk = (g > 16000) ? 16000 : g;
      delayMicroseconds(chunk);
      g -= chunk;
      yield();
    }
  }
}

void setup() {
  Serial.begin(9600);
  while(!Serial && millis() < 10000UL) {}
  Serial.println("started");

#ifdef ARDUINO_ARCH_RP2040
  SPI.setRX(MISO_PIN);
  SPI.setCS(SS_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(MOSI_PIN);
#endif

  pinMode(GDO0_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  mqttTopic = String(AIO_USERNAME) + "/feeds/" + FEED_NAME;

  connectWiFi();
  connectMQTT();
  delay(2000);

  Serial.print(F("[CC1101] Initializing ... "));
  // begin(freq MHz, bit rate kbps, freqDev kHz, rxBW kHz, power dBm, preamble bits)
  int16_t state = radio.begin(433.92, 4.8, 5.0, 650.0, 10, 16);
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print(F("fail, code "));
    Serial.println(state);
    while(true) delay(1000);
  }
  Serial.println(F("ok"));

  state = radio.setOOK(true);
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print(F("setOOK(true) failed, code "));
    Serial.println(state);
  }

  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  c -> capture 5 seconds and print RAW[]"));
  Serial.println(F("  d -> transmit (pasted RAW[] if present, else last capture)"));
    Serial.println(F("  u -> transmit (pasted RAW2[] if present, else last capture)"));
}

void loop() {

   // Keep MQTT alive
  if (!mqtt.connected()) {
    connectMQTT();
  }
  mqtt.loop();

  unsigned long now = millis();
  if (now - lastBlinkMs >= 500) {
    lastBlinkMs = now;
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState ? HIGH : LOW);
  }
  if (!Serial.available()) return;

  char ch = (char)Serial.read();

  if (ch == 'c') {
    Serial.println(F("Capturing 5s... press/hold the remote now."));
    captureRaw(5000);
    printCapturedAsArray();
    Serial.println(F("Done."));
  }
  else if (ch == 'u') {
    if (RAW2_LEN > 0) {
      Serial.print(F("Transmitting pasted RAW2[] (len="));
      Serial.print(RAW2_LEN);
      Serial.println(F(") ..."));
      transmitRawTimings(RAW2, RAW2_LEN, 1, 8000);
      Serial.println(F("Done."));
    } else {
      Serial.println(F("RAW2 is empty. Paste RAW2[] above."));
    }
  }
  else if (ch == 'd') {
    if (RAW_LEN > 0) {
      Serial.print(F("Transmitting pasted RAW[] (len="));
      Serial.print(RAW_LEN);
      Serial.println(F(") ..."));
      transmitRawTimings(RAW, RAW_LEN, 1, 8000);
      Serial.println(F("Done."));
    }
    else if (capCount > 0) {
      noInterrupts();
      uint16_t n = capCount;
      interrupts();

      Serial.print(F("Transmitting last capture (len="));
      Serial.print(n);
      Serial.println(F(") ..."));

      static int32_t tmp[MAX_EDGES];
      noInterrupts();
      for (uint16_t i = 0; i < n; i++) tmp[i] = capTimings[i];
      interrupts();

      transmitRawTimings(tmp, n, 1, 8000);
      Serial.println(F("Done."));
    }
    else {
      Serial.println(F("Nothing to transmit. Paste RAW[] above or press 'c' first."));
    }
  }
}

