/*
 * VERKEERSLICHT B - SIMPELE VERSIE
 * Gewoon timer reset wanneer A groen wordt
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi instellingen
const char* WIFI_SSID = "WiFi-2.4-5800";
const char* WIFI_PASSWORD = "wj6ahm5m2d9xn";

// MQTT Broker
const char* MQTT_BROKER = "test.mosquitto.org";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "TrafficLightB_67890";

// Verkeerslicht ID
const char* LIGHT_ID = "B";

// Timing (seconden)
const int TIME_A_TO_B = 30;
const int TIME_B_TO_A = 45;
const int WAIT_TIME = 5;
const int YELLOW_TIME = 3;

// Communicatie
const int HEARTBEAT_INTERVAL = 5;
const int HEARTBEAT_TIMEOUT = 10;
const int ERROR_BLINK_INTERVAL = 1;

// Hardware pinnen
const int PIN_RED = 21;
const int PIN_YELLOW = 22;
const int PIN_GREEN = 23;

// MQTT Topics
const char* TOPIC_STATUS_A = "traffic/lightA/status";
const char* TOPIC_STATUS_B = "traffic/lightB/status";
const char* TOPIC_HEARTBEAT_A = "traffic/lightA/heartbeat";
const char* TOPIC_HEARTBEAT_B = "traffic/lightB/heartbeat";

// State Machine
enum State {
  STATE_GREEN,
  STATE_YELLOW,
  STATE_RED,
  STATE_ERROR
};

State currentState = STATE_RED;
State previousState = STATE_RED;

// Globale variabelen
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastStateChange = 0;
unsigned long lastHeartbeatSent = 0;
unsigned long lastHeartbeatReceived = 0;
unsigned long lastBlinkToggle = 0;

bool otherLightOnline = false;
bool errorBlinkState = false;
bool systemReady = false;

// Functie declaraties
void setupWiFi();
void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus(State state);
void publishHeartbeat();
void updateLEDs();
void handleStateMachine();
void switchState(State newState);
void checkHeartbeatTimeout();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=================================");
  Serial.println("VERKEERSLICHT B - SYSTEEM START");
  Serial.println("=================================\n");
  
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  
  digitalWrite(PIN_RED, LOW);
  digitalWrite(PIN_YELLOW, LOW);
  digitalWrite(PIN_GREEN, LOW);
  
  Serial.println("LED pinnen geconfigureerd:");
  Serial.println("  GPIO 21 = ROOD");
  Serial.println("  GPIO 22 = GEEL");
  Serial.println("  GPIO 23 = GROEN\n");
  
  setupWiFi();
  setupMQTT();
  
  lastHeartbeatReceived = millis();
  lastStateChange = millis();
  
  Serial.println("Verkeerslicht B start als SLAVE (RED)");
  switchState(STATE_RED);
  
  systemReady = true;
  Serial.println("\n✓ Systeem klaar!\n");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠ WiFi verbinding verloren!");
    setupWiFi();
  }
  
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  handleStateMachine();
  updateLEDs();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatSent >= HEARTBEAT_INTERVAL * 1000) {
    publishHeartbeat();
    lastHeartbeatSent = currentMillis;
  }
  
  checkHeartbeatTimeout();
  
  delay(50);
}

void setupWiFi() {
  Serial.print("Verbinden met WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi verbonden!");
    Serial.print("IP adres: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi verbinding mislukt!");
    switchState(STATE_ERROR);
  }
}

void setupMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  
  reconnectMQTT();
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Verbinden met MQTT broker...");
    
    String lastWillPayload = "{\"state\":\"OFFLINE\",\"timestamp\":" + 
                            String(millis()) + ",\"light_id\":\"B\"}";
    
    if (mqttClient.connect(MQTT_CLIENT_ID, 
                          TOPIC_STATUS_B,
                          1,
                          true,
                          lastWillPayload.c_str())) {
      
      Serial.println(" ✓ Verbonden!");
      
      mqttClient.subscribe(TOPIC_STATUS_A, 1);
      mqttClient.subscribe(TOPIC_HEARTBEAT_A, 1);
      
      Serial.println("Geabonneerd op:");
      Serial.print("  - ");
      Serial.println(TOPIC_STATUS_A);
      Serial.print("  - ");
      Serial.println(TOPIC_HEARTBEAT_A);
      
      publishStatus(currentState);
      
    } else {
      Serial.print(" ✗ Mislukt! Error: ");
      Serial.println(mqttClient.state());
      switchState(STATE_ERROR);
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("\n--- Bericht ontvangen ---");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Bericht: ");
  Serial.println(message);
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  if (strcmp(topic, TOPIC_HEARTBEAT_A) == 0) {
    lastHeartbeatReceived = millis();
    otherLightOnline = true;
    
    Serial.println("♥ Heartbeat van A ontvangen - OK");
    
    if (currentState == STATE_ERROR && systemReady) {
      Serial.println("✓ Communicatie hersteld!");
      switchState(STATE_RED);
      delay(2000);
    }
  }
  
  else if (strcmp(topic, TOPIC_STATUS_A) == 0) {
    const char* state = doc["state"];
    
    Serial.print("Status van A: ");
    Serial.println(state);
    
    if (strcmp(state, "OFFLINE") == 0) {
      Serial.println("⚠ A is OFFLINE! -> ERROR");
      switchState(STATE_ERROR);
      otherLightOnline = false;
    }
    else if (strcmp(state, "ERROR") == 0) {
      Serial.println("⚠ A is in ERROR! -> ERROR");
      switchState(STATE_ERROR);
    }
    // SIMPELE FIX: Als A groen wordt EN B is rood, reset timer
    else if (strcmp(state, "GREEN") == 0 && currentState == STATE_RED) {
      Serial.println("✓ A is groen - B reset timer");
      lastStateChange = millis();  // DIT IS DE FIX!
    }
  }
  
  Serial.println("-------------------------\n");
}

void publishStatus(State state) {
  JsonDocument doc;
  
  const char* stateStr;
  switch(state) {
    case STATE_GREEN:  stateStr = "GREEN";  break;
    case STATE_YELLOW: stateStr = "YELLOW"; break;
    case STATE_RED:    stateStr = "RED";    break;
    case STATE_ERROR:  stateStr = "ERROR";  break;
    default:           stateStr = "UNKNOWN"; break;
  }
  
  doc["state"] = stateStr;
  doc["timestamp"] = millis();
  doc["light_id"] = LIGHT_ID;
  
  String output;
  serializeJson(doc, output);
  
  if (mqttClient.publish(TOPIC_STATUS_B, output.c_str(), true)) {
    Serial.print("📤 Status gepubliceerd: ");
    Serial.println(output);
  }
}

void publishHeartbeat() {
  JsonDocument doc;
  
  doc["alive"] = true;
  doc["timestamp"] = millis();
  doc["light_id"] = LIGHT_ID;
  
  String output;
  serializeJson(doc, output);
  
  if (mqttClient.publish(TOPIC_HEARTBEAT_B, output.c_str())) {
    Serial.print("♥ Heartbeat verzonden: ");
    Serial.println(output);
  }
}

void checkHeartbeatTimeout() {
  unsigned long currentMillis = millis();
  
  if (currentState != STATE_ERROR && systemReady) {
    if (currentMillis - lastHeartbeatReceived > HEARTBEAT_TIMEOUT * 1000) {
      Serial.println("\n⚠ HEARTBEAT TIMEOUT!");
      switchState(STATE_ERROR);
      otherLightOnline = false;
    }
  }
}

void handleStateMachine() {
  if (currentState == STATE_ERROR) {
    return;
  }
  
  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - lastStateChange;
  
  switch(currentState) {
    case STATE_RED:
      // B wacht 38 seconden: A groen (30s) + A geel (3s) + wacht (5s)
      if (elapsed >= (TIME_A_TO_B + YELLOW_TIME + WAIT_TIME) * 1000) {
        switchState(STATE_GREEN);
      }
      break;
      
    case STATE_GREEN:
      // B groen voor 45 seconden
      if (elapsed >= TIME_B_TO_A * 1000) {
        switchState(STATE_YELLOW);
      }
      break;
      
    case STATE_YELLOW:
      // B geel voor 3 seconden
      if (elapsed >= YELLOW_TIME * 1000) {
        switchState(STATE_RED);
      }
      break;
      
    case STATE_ERROR:
      break;
  }
}

void switchState(State newState) {
  if (currentState == newState) {
    return;
  }
  
  previousState = currentState;
  currentState = newState;
  lastStateChange = millis();
  
  Serial.print("\n>>> STATE CHANGE: ");
  switch(previousState) {
    case STATE_GREEN:  Serial.print("GREEN");  break;
    case STATE_YELLOW: Serial.print("YELLOW"); break;
    case STATE_RED:    Serial.print("RED");    break;
    case STATE_ERROR:  Serial.print("ERROR");  break;
  }
  Serial.print(" -> ");
  switch(currentState) {
    case STATE_GREEN:  Serial.println("GREEN");  break;
    case STATE_YELLOW: Serial.println("YELLOW"); break;
    case STATE_RED:    Serial.println("RED");    break;
    case STATE_ERROR:  Serial.println("ERROR");  break;
  }
  
  publishStatus(currentState);
}

void updateLEDs() {
  switch(currentState) {
    case STATE_GREEN:
      digitalWrite(PIN_RED, LOW);
      digitalWrite(PIN_YELLOW, LOW);
      digitalWrite(PIN_GREEN, HIGH);
      break;
      
    case STATE_YELLOW:
      digitalWrite(PIN_RED, LOW);
      digitalWrite(PIN_YELLOW, HIGH);
      digitalWrite(PIN_GREEN, LOW);
      break;
      
    case STATE_RED:
      digitalWrite(PIN_RED, HIGH);
      digitalWrite(PIN_YELLOW, LOW);
      digitalWrite(PIN_GREEN, LOW);
      break;
      
    case STATE_ERROR:
      unsigned long currentMillis = millis();
      if (currentMillis - lastBlinkToggle >= ERROR_BLINK_INTERVAL * 1000) {
        errorBlinkState = !errorBlinkState;
        lastBlinkToggle = currentMillis;
      }
      
      digitalWrite(PIN_RED, errorBlinkState ? HIGH : LOW);
      digitalWrite(PIN_YELLOW, errorBlinkState ? HIGH : LOW);
      digitalWrite(PIN_GREEN, LOW);
      break;
  }
}
