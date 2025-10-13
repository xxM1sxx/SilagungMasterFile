#include <Arduino.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// MQTT broker details
const char* mqtt_broker = "164d4421be27493fac52acabe1391e0f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "Controller";
const char* mqtt_password = "Controller123!";

// MQTT topics
const char* topic_sensor = "silagung/sensor";
const char* topic_control = "silagung/controll";
const char* topic_status = "silagung/system";

// Konfigurasi Relay
#define RELAY1 1
#define RELAY2 2
#define RELAY3 41
#define RELAY4 42
#define RELAY5 45
#define RELAY6 46

// Konfigurasi I2C untuk sensor
#define SDA_PIN 8
#define SCL_PIN 9

// Debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 250;

// Tambahkan definisi pin RST
#define RST_PIN 0  // Pin RST pada ESP32

// Variabel untuk sensor (contoh)
float temperature = 0;
float waterFlow = 0;
float pressure = 0;
float ec = 0;
float ph = 0;
float nitrogen = 0;
float phosphorus = 0;
float potassium = 0;

// Variabel untuk MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

bool previousRelayStates[6] = {false, false, false, false, false, false};
bool relayStateChanged = false;

// Interval pengiriman data sensor
unsigned long previousMillis = 0;
const long interval = 5000;  // interval 5 seconds

void reconnectMQTT();
void callback(char* topic, byte* payload, unsigned int length);
void publishRelayStatus();
void publishSensorData();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("System starting...");
  
  pinMode(RST_PIN, INPUT_PULLUP);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);
  
  // Setup MQTT
  espClient.setInsecure(); // Bypass SSL certificate validation
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  
  // Connect to WiFi and MQTT
  WiFiManager wm;
  
  // Set timeout untuk portal konfigurasi (0 = tidak ada timeout)
  wm.setConfigPortalTimeout(0);
  
  // Tambahkan debug info
  wm.setDebugOutput(true);
  
  bool res = wm.autoConnect("Silagung", "admin123");
  
  if (res) {
    Serial.println("WiFi connected successfully");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Connect to MQTT
    reconnectMQTT();
  } else {
    Serial.println("WiFi connection failed");
    Serial.println("Portal konfigurasi akan tetap aktif...");
  }
}

void loop() {
  // Cek tombol RST
  if (digitalRead(RST_PIN) == LOW) {
    if ((millis() - lastDebounceTime) > debounceDelay) {
      Serial.println("Tombol RST ditekan!");
      Serial.println("Menghapus konfigurasi WiFi...");
      
      WiFiManager wm;
      wm.resetSettings();
      
      Serial.println("Konfigurasi WiFi berhasil dihapus");
      Serial.println("Sistem akan restart dalam 3 detik...");
      
      delay(3000);
      ESP.restart();
    }
    lastDebounceTime = millis();
  }

  // Cek koneksi WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi terputus, mencoba menghubungkan kembali...");
    WiFiManager wm;
    
    // Set timeout untuk portal konfigurasi (3 menit)
    wm.setConfigPortalTimeout(180);
    
    // Tambahkan debug output
    wm.setDebugOutput(true);
    
    bool res = wm.autoConnect("Silagung", "admin123");
    
    if (res) {
      Serial.println("WiFi connected successfully");
    } else {
      Serial.println("WiFi connection failed - Portal timeout reached");
      Serial.println("Akan mencoba lagi dalam 30 detik...");
      delay(30000);
    }
  }

  // Make sure MQTT is connected
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Kirim data sensor secara berkala
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Baca data sensor (simulasi)
    // Dalam implementasi nyata, baca dari sensor yang sebenarnya
    temperature = random(2500, 3500) / 100.0; // 25.00-35.00 C
    waterFlow = random(1000, 2000) / 100.0;   // 10.00-20.00 L/min
    pressure = random(200, 300) / 100.0;      // 2.00-3.00 bar
    ec = random(100, 300) / 100.0;            // 1.00-3.00 mS/cm
    ph = random(650, 750) / 100.0;            // 6.50-7.50
    nitrogen = random(30, 60);                // 30-60 ppm
    phosphorus = random(5, 20);               // 5-20 ppm
    potassium = random(20, 50);               // 20-50 ppm
    
    // Kirim data sensor
    publishSensorData();
  }
}

// Fungsi untuk mengirim data sensor ke MQTT
void publishSensorData() {
  StaticJsonDocument<256> doc;
  
  doc["waterFlow"] = waterFlow;
  doc["pressure"] = pressure;
  doc["ec"] = ec;
  doc["ph"] = ph;
  doc["nitrogen"] = nitrogen;
  doc["phosphorus"] = phosphorus;
  doc["potassium"] = potassium;
  doc["temperature"] = temperature;
  
  char buffer[256];
  serializeJson(doc, buffer);
  
  if (client.publish(topic_sensor, buffer)) {
    Serial.println("Sensor data published successfully");
    Serial.println(buffer);
  } else {
    Serial.println("Failed to publish sensor data");
  }
}

// Fungsi untuk reconnect ke MQTT
void reconnectMQTT() {
    int attempts = 0;
    while (!client.connected() && attempts < 3) {
        Serial.println("Attempting MQTT connection...");
        String client_id = "Controller-1";
        
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
            // Subscribe to control topic
            client.subscribe(topic_control);
            return;
        } else {
            Serial.print("Failed with state ");
            Serial.println(client.state());
            attempts++;
            delay(5000);
        }
    }
}

// Callback untuk menerima pesan MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    
    // Convert payload to string
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Message: " + message);

    // Parse JSON message
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.f_str());
        return;
    }

    // Handle relay control
    for(int i = 1; i <= 6; i++) {
        String relayKey = "relay" + String(i);
        if(doc.containsKey(relayKey)) {
            String state = doc[relayKey];
            bool relayState = (state == "on");
            
            Serial.print("Setting relay ");
            Serial.print(i);
            Serial.print(" to ");
            Serial.println(relayState ? "ON" : "OFF");
            
            switch(i) {
                case 1: digitalWrite(RELAY1, relayState); break;
                case 2: digitalWrite(RELAY2, relayState); break;
                case 3: digitalWrite(RELAY3, relayState); break;
                case 4: digitalWrite(RELAY4, relayState); break;
                case 5: digitalWrite(RELAY5, relayState); break;
                case 6: digitalWrite(RELAY6, relayState); break;
            }
        }
    }
    
    // Publish current status after any change
    publishRelayStatus();
}

// Fungsi untuk mengirim status relay
void publishRelayStatus() {
    // Check if any relay state has changed
    bool currentStates[6];
    currentStates[0] = digitalRead(RELAY1);
    currentStates[1] = digitalRead(RELAY2);
    currentStates[2] = digitalRead(RELAY3);
    currentStates[3] = digitalRead(RELAY4);
    currentStates[4] = digitalRead(RELAY5);
    currentStates[5] = digitalRead(RELAY6);
    
    // Check for any changes
    relayStateChanged = false;
    for(int i = 0; i < 6; i++) {
        if(currentStates[i] != previousRelayStates[i]) {
            relayStateChanged = true;
            previousRelayStates[i] = currentStates[i];
        }
    }
    
    // Only publish if there were changes
    if(relayStateChanged) {
        StaticJsonDocument<200> doc;
        
        // Add all relay states in one message
        doc["relay1"] = currentStates[0] ? "on" : "off";
        doc["relay2"] = currentStates[1] ? "on" : "off";
        doc["relay3"] = currentStates[2] ? "on" : "off";
        doc["relay4"] = currentStates[3] ? "on" : "off";
        doc["relay5"] = currentStates[4] ? "on" : "off";
        doc["relay6"] = currentStates[5] ? "on" : "off";
        
        char buffer[200];
        serializeJson(doc, buffer);
        client.publish(topic_status, buffer);
        
        Serial.println("Published relay states: " + String(buffer));
    }
}

