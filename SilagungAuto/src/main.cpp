#include <Arduino.h> //irigasi 3 lahan belum bener
#include <WiFiManager.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <ModbusMaster.h>

// MQTT broker details
const char* mqtt_broker = "164d4421be27493fac52acabe1391e0f.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "Controller";
const char* mqtt_password = "Controller123!";

// MQTT topics
const char* topic_sensor = "silagung/sensor";
const char* topic_control = "silagung/controll";
const char* topic_status = "silagung/system";
const char* topic_irrigation_config = "silagung/irrigation/config";

// Konfigurasi I2C untuk sensor
#define SDA_PIN 8
#define SCL_PIN 9

// Konfigurasi RS485 untuk Water Flow Sensor
#define RXD2 18
#define TXD2 17

// Konfigurasi Relay untuk Irigasi
#define RELAY1 1   // Valve 1 (Nutrisi)
#define RELAY2 2   // Valve 2 (Air)
#define RELAY3 41  // Lahan 1
#define RELAY4 42  // Lahan 2
#define RELAY5 45  // Lahan 3
#define RELAY6 46  // Pompa

//pressure
#define CurrentSensorPin  13
#define VREF 3300 // ADC's reference voltage on ESP32-S3: 3300mV
#define SAMPLES 20   // Increased samples for better averaging
#define FILTER_ALPHA 0.1  // Low-pass filter coefficient (0.1 = heavy filtering)

// Pressure sensor specifications: 0-10 Bar, 4-20mA
#define MIN_CURRENT 4.0   // 4mA = 0 Bar
#define MAX_CURRENT 20.0  // 20mA = 10 Bar
#define MAX_PRESSURE 10.0 // 10 Bar

//pressure Variable
unsigned long voltageSum = 0; // Sum for averaging
unsigned int voltage; //unit:mV
float current;  //unit:mA
float filteredCurrent = 0.0; // Filtered current value

// Irrigation Control Variables
struct ActiveIrrigation {
    int configId;
    String landName;
    String startTime;
    float waterNeeded;
    float waterDelivered;
    String irrigationType;
    bool isActive;
    unsigned long startMillis;
    int landRelay;
};

ActiveIrrigation currentIrrigation;
unsigned long lastScheduleCheck = 0;
const unsigned long SCHEDULE_CHECK_INTERVAL = 60000; // Check every minute

// RTC dan NTP Configuration
RTC_DS3231 rtc;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // GMT+7 (25200 seconds), update every 60 seconds

// Time sync variables
bool rtcInitialized = false;
unsigned long lastNTPSync = 0;
const unsigned long NTP_SYNC_INTERVAL = 3600000; // Sync every hour (3600000 ms)

// Automatic display variables
unsigned long lastTimeDisplay = 0;
unsigned long lastStatusDisplay = 0;
const unsigned long TIME_DISPLAY_INTERVAL = 30000; // Display time every 30 seconds
const unsigned long STATUS_DISPLAY_INTERVAL = 60000; // Display status every 60 seconds

// Irrigation Configuration Storage
Preferences preferences;
struct IrrigationSchedule {
    String time;
    bool isActive;
};

struct IrrigationConfig {
    int configId;                     // Unique identifier for each config
    String landName;
    String phaseName;
    float waterRequirement;
    float waterPerSchedule;
    float targetEC;
    String irrigationType;
    IrrigationSchedule schedules[10]; // Maximum 10 schedules
    int scheduleCount;
    bool isValid;
};

// Support for multiple configurations
#define MAX_CONFIGS 5
IrrigationConfig configs[MAX_CONFIGS];
int totalConfigs = 0;
// All valid configurations are considered active simultaneously

// Irrigation Queue System
struct IrrigationQueue {
    int configIndex;
    String scheduleTime;
    unsigned long queuedTime;
    bool isProcessed;
};

// Queue variables
#define MAX_QUEUE_SIZE 10
IrrigationQueue irrigationQueue[MAX_QUEUE_SIZE];
int queueCount = 0;
int currentQueueIndex = 0;

// Debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 250;

// Sensor variables
float temperature = 0;
float waterFlow = 0;
float pressure = 0;
float ec = 0;
float ph = 0;
float nitrogen = 0;
float phosphorus = 0;
float potassium = 0;

// Relay state tracking for manual control
bool previousRelayStates[6] = {false, false, false, false, false, false};
bool relayStateChanged = false;

// Interval pengiriman data sensor
unsigned long previousMillis = 0;
const long interval = 5000;  // interval 5 seconds

// Variabel untuk MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Modbus Master untuk Water Flow Sensor
ModbusMaster node;

// Set MQTT buffer size untuk menangani pesan JSON yang besar
void setupMQTTBufferSize() {
    client.setBufferSize(4096); // Increased buffer size for batch payloads
}

void reconnectMQTT();
void callback(char* topic, byte* payload, unsigned int length);
void checkSerialCommands();
void initializeRTC();
void syncTimeWithNTP();
void displayCurrentTime();
String formatDateTime(DateTime dt);
void saveIrrigationConfigs();
void loadIrrigationConfigs();
void saveIrrigationConfig(int configIndex);
void loadIrrigationConfig(int configIndex);
int findConfigById(int configId);
void addOrUpdateConfig(IrrigationConfig newConfig);
void displayAllConfigs();
void displayConfig(int configIndex);
void parseIrrigationConfig(String jsonString);
void displayIrrigationConfig();
void stopIrrigation();

// Manual control and sensor functions
void publishSensorData();
void publishRelayStatus();
float readWaterFlowSensor();
float readPressureSensor();
void read6in1Sensor();

// Queue management functions
void addToQueue(int configIndex, String scheduleTime);
void processQueue();
void clearQueue();
bool isInQueue(int configIndex, String scheduleTime);

// Save all irrigation configurations to preferences
void saveIrrigationConfigs() {
    preferences.putInt("totalConfigs", totalConfigs);
    
    for (int i = 0; i < totalConfigs; i++) {
        saveIrrigationConfig(i);
    }
}

// Load all irrigation configurations from preferences
void loadIrrigationConfigs() {
    totalConfigs = preferences.getInt("totalConfigs", 0);
    
    if (totalConfigs == 0) {
        return;
    }
    
    for (int i = 0; i < totalConfigs && i < MAX_CONFIGS; i++) {
        loadIrrigationConfig(i);
    }
}

// Save single irrigation configuration to preferences
void saveIrrigationConfig(int configIndex) {
    if (configIndex < 0 || configIndex >= MAX_CONFIGS || !configs[configIndex].isValid) {
        return;
    }
    
    String prefix = "cfg" + String(configIndex) + "_";
    
    preferences.putInt((prefix + "id").c_str(), configs[configIndex].configId);
    preferences.putString((prefix + "landName").c_str(), configs[configIndex].landName);
    preferences.putString((prefix + "phaseName").c_str(), configs[configIndex].phaseName);
    preferences.putFloat((prefix + "waterReq").c_str(), configs[configIndex].waterRequirement);
    preferences.putFloat((prefix + "waterPerSch").c_str(), configs[configIndex].waterPerSchedule);
    preferences.putFloat((prefix + "targetEC").c_str(), configs[configIndex].targetEC);
    preferences.putString((prefix + "irrigType").c_str(), configs[configIndex].irrigationType);
    preferences.putInt((prefix + "schedCount").c_str(), configs[configIndex].scheduleCount);
    preferences.putBool((prefix + "valid").c_str(), configs[configIndex].isValid);
    
    // Save schedules
    for (int j = 0; j < configs[configIndex].scheduleCount && j < 10; j++) {
        String timeKey = prefix + "time" + String(j);
        String activeKey = prefix + "active" + String(j);
        preferences.putString(timeKey.c_str(), configs[configIndex].schedules[j].time);
        preferences.putBool(activeKey.c_str(), configs[configIndex].schedules[j].isActive);
    }
}

// Load single irrigation configuration from preferences
void loadIrrigationConfig(int configIndex) {
    if (configIndex < 0 || configIndex >= MAX_CONFIGS) {
        return;
    }
    
    String prefix = "cfg" + String(configIndex) + "_";
    
    configs[configIndex].isValid = preferences.getBool((prefix + "valid").c_str(), false);
    
    if (!configs[configIndex].isValid) {
        return;
    }
    
    configs[configIndex].configId = preferences.getInt((prefix + "id").c_str(), 0);
    configs[configIndex].landName = preferences.getString((prefix + "landName").c_str(), "");
    configs[configIndex].phaseName = preferences.getString((prefix + "phaseName").c_str(), "");
    configs[configIndex].waterRequirement = preferences.getFloat((prefix + "waterReq").c_str(), 0.0);
    configs[configIndex].waterPerSchedule = preferences.getFloat((prefix + "waterPerSch").c_str(), 0.0);
    configs[configIndex].targetEC = preferences.getFloat((prefix + "targetEC").c_str(), 0.0);
    configs[configIndex].irrigationType = preferences.getString((prefix + "irrigType").c_str(), "");
    configs[configIndex].scheduleCount = preferences.getInt((prefix + "schedCount").c_str(), 0);
    
    // Load schedules
    for (int j = 0; j < configs[configIndex].scheduleCount && j < 10; j++) {
        String timeKey = prefix + "time" + String(j);
        String activeKey = prefix + "active" + String(j);
        configs[configIndex].schedules[j].time = preferences.getString(timeKey.c_str(), "");
        configs[configIndex].schedules[j].isActive = preferences.getBool(activeKey.c_str(), false);
    }
}

// Find configuration by ID
int findConfigById(int configId) {
    for (int i = 0; i < totalConfigs; i++) {
        if (configs[i].isValid && configs[i].configId == configId) {
            return i;
        }
    }
    return -1; // Not found
}

// Add or update configuration
void addOrUpdateConfig(IrrigationConfig newConfig) {
    int existingIndex = findConfigById(newConfig.configId);
    
    if (existingIndex != -1) {
        // Update existing configuration
        configs[existingIndex] = newConfig;
        
        // Save only the updated configuration
        saveIrrigationConfig(existingIndex);
        preferences.putInt("totalConfigs", totalConfigs);
    } else {
        // Add new configuration
        if (totalConfigs < MAX_CONFIGS) {
            configs[totalConfigs] = newConfig;
            totalConfigs++;
            
            // Save the new configuration and update total count
            saveIrrigationConfig(totalConfigs - 1);
            preferences.putInt("totalConfigs", totalConfigs);
        } else {
            return;
        }
    }
}

void parseIrrigationConfig(String jsonString) {
    // Use larger JSON document for irrigation config (increased size for batch payloads)
    StaticJsonDocument<4096> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
        Serial.print("❌ JSON parsing failed: ");
        Serial.println(error.f_str());
        return;
    }
    
    // Check if this is a batch payload (contains "configs" array)
    if (doc.containsKey("configs")) {
        Serial.println("📥 Processing batch configs...");
        
        // Parse metadata
        String timestamp = doc["timestamp"].as<String>();
        int totalCount = doc["totalConfigs"] | 0;
        
        // Parse configurations array
        JsonArray configurations = doc["configs"];
        int processedCount = 0;
        int successCount = 0;
        
        for (JsonObject config : configurations) {
            processedCount++;
            
            // Create temporary config
            IrrigationConfig tempConfig;
            
            // Parse configuration fields
            tempConfig.configId = config["configId"] | processedCount;
            tempConfig.landName = config["landName"].as<String>();
            tempConfig.phaseName = config["phaseName"].as<String>();
            tempConfig.waterRequirement = config["waterRequirement"];
            tempConfig.waterPerSchedule = config["waterPerSchedule"];
            tempConfig.targetEC = config["targetEC"];
            tempConfig.irrigationType = config["irrigationType"].as<String>();
            
            // Parse schedules array
            JsonArray schedules = config["schedules"];
            tempConfig.scheduleCount = min((int)schedules.size(), 10); // Limit to 10 schedules
            
            for (int i = 0; i < tempConfig.scheduleCount; i++) {
                tempConfig.schedules[i].time = schedules[i]["time"].as<String>();
                tempConfig.schedules[i].isActive = schedules[i]["isActive"];
            }
            
            // Validate configuration
            if (tempConfig.landName.length() > 0 && 
                tempConfig.phaseName.length() > 0 &&
                tempConfig.waterRequirement > 0 &&
                tempConfig.waterPerSchedule > 0) {
                tempConfig.isValid = true;
                
                // Add or update configuration
                addOrUpdateConfig(tempConfig);
                successCount++;
            }
        }
        
        Serial.println("✅ Batch processed: " + String(successCount) + "/" + String(totalCount) + " configs");
        
    } else {
        // Handle single configuration (backward compatibility)
        Serial.println("📥 Processing single config...");
        
        // Create temporary config
        IrrigationConfig tempConfig;
        
        // Parse configuration fields
        tempConfig.configId = doc["configId"] | 1; // Default to 1 if not provided
        tempConfig.landName = doc["landName"].as<String>();
        tempConfig.phaseName = doc["phaseName"].as<String>();
        tempConfig.waterRequirement = doc["waterRequirement"];
        tempConfig.waterPerSchedule = doc["waterPerSchedule"];
        tempConfig.targetEC = doc["targetEC"];
        tempConfig.irrigationType = doc["irrigationType"].as<String>();
        
        // Parse schedules array
        JsonArray schedules = doc["schedules"];
        tempConfig.scheduleCount = min((int)schedules.size(), 10); // Limit to 10 schedules
        
        for (int i = 0; i < tempConfig.scheduleCount; i++) {
            tempConfig.schedules[i].time = schedules[i]["time"].as<String>();
            tempConfig.schedules[i].isActive = schedules[i]["isActive"];
        }
        
        // Validate configuration
        if (tempConfig.landName.length() > 0 && 
            tempConfig.phaseName.length() > 0 &&
            tempConfig.waterRequirement > 0 &&
            tempConfig.waterPerSchedule > 0) {
            tempConfig.isValid = true;
            
            // Add or update configuration
            addOrUpdateConfig(tempConfig);
            
            Serial.print("✅ Single configuration (ID: ");
            Serial.print(tempConfig.configId);
            Serial.print(", Land: ");
            Serial.print(tempConfig.landName);
            Serial.println(") processed successfully");
        } else {
            Serial.println("❌ Single configuration validation failed");
        }
    }
}

void displayIrrigationConfig() {
    if (totalConfigs == 0) {
        Serial.println("📋 No configurations");
        return;
    }
    
    Serial.println("📋 Configurations (" + String(totalConfigs) + "):");
    for (int i = 0; i < totalConfigs; i++) {
        if (configs[i].isValid) {
            Serial.println("  " + String(i + 1) + ". " + configs[i].landName + " - " + configs[i].irrigationType + " (" + String(configs[i].scheduleCount) + " schedules)");
        }
    }
}

// Irrigation Control Functions
int getLandRelay(String landName) {
    landName.toLowerCase();
    if (landName.indexOf("1") != -1 || landName.indexOf("satu") != -1) {
        return RELAY3; // Lahan 1
    } else if (landName.indexOf("2") != -1 || landName.indexOf("dua") != -1) {
        return RELAY4; // Lahan 2
    } else if (landName.indexOf("3") != -1 || landName.indexOf("tiga") != -1) {
        return RELAY5; // Lahan 3
    }
    return RELAY3; // Default to Lahan 1
}

void startIrrigation(int configIndex, String scheduleTime) {
    if (currentIrrigation.isActive) {
        Serial.println("⚠️ Irrigation already active - stopping current irrigation first");
        stopIrrigation();
    }
    
    IrrigationConfig& config = configs[configIndex];
    
    // Setup current irrigation
    currentIrrigation.configId = config.configId;
    currentIrrigation.landName = config.landName;
    currentIrrigation.startTime = scheduleTime;
    currentIrrigation.waterNeeded = config.waterPerSchedule;
    currentIrrigation.waterDelivered = 0;
    currentIrrigation.irrigationType = config.irrigationType;
    currentIrrigation.isActive = true;
    currentIrrigation.startMillis = millis();
    currentIrrigation.landRelay = getLandRelay(config.landName);
    
    Serial.println("🚿 Starting: " + config.landName + " (" + config.irrigationType + ") - " + String(config.waterPerSchedule) + "L");
    
    // Control valves based on irrigation type
    String irrigationType = config.irrigationType;
    irrigationType.toLowerCase();
    if (irrigationType == "air") {
        // Water only: Close nutrient valve, open water valve
        digitalWrite(RELAY1, LOW);  // Valve 1 (Nutrisi) - CLOSED
        digitalWrite(RELAY2, HIGH); // Valve 2 (Air) - OPEN
    } else {
        // Water + Nutrient: Open both valves
        digitalWrite(RELAY1, HIGH); // Valve 1 (Nutrisi) - OPEN
        digitalWrite(RELAY2, HIGH); // Valve 2 (Air) - OPEN
    }
    
    // Open land valve
    digitalWrite(currentIrrigation.landRelay, HIGH);
    
    // Start pump
    digitalWrite(RELAY6, HIGH);
}

void stopIrrigation() {
    if (!currentIrrigation.isActive) {
        return;
    }
    
    Serial.println("🛑 Stopped: " + currentIrrigation.landName + " - " + String(currentIrrigation.waterDelivered) + "L in " + String((millis() - currentIrrigation.startMillis) / 1000) + "s");
    
    // Turn off all relays
    digitalWrite(RELAY1, LOW);  // Valve 1 (Nutrisi)
    digitalWrite(RELAY2, LOW);  // Valve 2 (Air)
    digitalWrite(RELAY3, LOW);  // Lahan 1
    digitalWrite(RELAY4, LOW);  // Lahan 2
    digitalWrite(RELAY5, LOW);  // Lahan 3
    digitalWrite(RELAY6, LOW);  // Pompa
    
    // Reset irrigation state
    currentIrrigation.isActive = false;
    currentIrrigation.waterDelivered = 0;
    
    // Process next item in queue
    processQueue();
}

void checkIrrigationSchedules() {
    if (!rtcInitialized) {
        return;
    }
    
    DateTime now = rtc.now();
    String currentTime = String(now.hour()) + ":" + (now.minute() < 10 ? "0" : "") + String(now.minute());
    
    // Check all configurations for matching schedules
    for (int i = 0; i < totalConfigs; i++) {
        if (!configs[i].isValid) continue;
        
        for (int j = 0; j < configs[i].scheduleCount; j++) {
            if (!configs[i].schedules[j].isActive) continue;
            
            String scheduleTime = configs[i].schedules[j].time;
            
            // Extract HH:MM from schedule time (remove seconds if present)
            String scheduleTimeHHMM = scheduleTime;
            if (scheduleTime.length() > 5) {
                scheduleTimeHHMM = scheduleTime.substring(0, 5); // Get only HH:MM part
            }
            
            // Check if current time matches schedule time
            if (currentTime == scheduleTimeHHMM) {
                // Add to queue if not already queued
                if (!isInQueue(i, scheduleTimeHHMM)) {
                    addToQueue(i, scheduleTimeHHMM);
                    Serial.println("📋 Added to queue: " + configs[i].landName + " at " + currentTime);
                }
            }
        }
    }
    
    // Process queue
    processQueue();
}

void monitorIrrigation() {
    if (!currentIrrigation.isActive) {
        return;
    }
    
    // Calculate elapsed time
    unsigned long elapsedTime = millis() - currentIrrigation.startMillis;
    
    // Read actual water flow from RS485 sensor
    static unsigned long lastFlowReading = 0;
    static float totalWaterDelivered = 0;
    
    // Read flow sensor every 1 second
    if (millis() - lastFlowReading >= 1000) {
        float currentFlow = readWaterFlowSensor(); // L/min
        
        // Calculate water delivered in the last second
        float waterInLastSecond = (currentFlow / 60.0); // Convert L/min to L/sec
        totalWaterDelivered += waterInLastSecond;
        
        currentIrrigation.waterDelivered = totalWaterDelivered;
        
        Serial.printf("💧 Current Flow: %.2f L/min, Total Delivered: %.2f L, Target: %.2f L\n", 
                     currentFlow, totalWaterDelivered, currentIrrigation.waterNeeded);
        
        lastFlowReading = millis();
    }
    
    // Check if water requirement is met
    if (currentIrrigation.waterDelivered >= currentIrrigation.waterNeeded) {
        Serial.println("💧 Water requirement met - stopping irrigation");
        totalWaterDelivered = 0; // Reset for next irrigation
        stopIrrigation();
    }
    
    // Safety timeout (maximum 30 minutes per irrigation)
    if (elapsedTime > 1800000) { // 30 minutes
        Serial.println("⚠️ Safety timeout reached - stopping irrigation");
        totalWaterDelivered = 0; // Reset for next irrigation
        stopIrrigation();
    }
}

float currentToPressure(float currentmA) {
    // Convert 4-20mA to 0-10 Bar
    if(currentmA < MIN_CURRENT) {
        return 0.0; // Below 4mA = fault condition
    }
    
    // Linear conversion: (current - 4mA) / (20mA - 4mA) * 10 Bar
    float pressureBar = ((currentmA - MIN_CURRENT) / (MAX_CURRENT - MIN_CURRENT)) * MAX_PRESSURE;
    
    // Clamp to valid range
    if(pressureBar < 0) pressureBar = 0;
    if(pressureBar > MAX_PRESSURE) pressureBar = MAX_PRESSURE;
    
    return pressureBar;
}


void setup() {
  Serial.begin(115200);
  delay(2000);  // Increased delay for ESP32-S3
  Serial.println("System starting...");
  Serial.println("ESP32-S3 Irrigation Controller");
  Serial.println("Commands available:");
  Serial.println("  'reset' - Reset WiFi settings");
  
  Wire.begin(SDA_PIN, SCL_PIN);        // join i2c bus as master with specific pins
  Serial.println("I2C initialized");
  
  // Initialize RS485 for Water Flow Sensor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(1, Serial2); // Modbus ID 1
  Serial.println("RS485 Water Flow Sensor initialized");
  
  // Initialize relay pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  pinMode(RELAY6, OUTPUT);
  
  // Set all relays to OFF initially
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);
  digitalWrite(RELAY5, LOW);
  digitalWrite(RELAY6, LOW);
  Serial.println("Relay pins initialized - All relays OFF");
  
//pressure 
 // Configure ADC for ESP32-S3
   analogReadResolution(12); // 12-bit resolution
   analogSetAttenuation(ADC_11db); // 0-3.3V range

  // Initialize irrigation control
  currentIrrigation.isActive = false;
  currentIrrigation.waterDelivered = 0;
  Serial.println("Irrigation control initialized");
  
  // Initialize RTC (works offline)
  initializeRTC();
  
  // Initialize irrigation configuration (works offline)
  preferences.begin("irrigation", false);
  loadIrrigationConfigs();
  Serial.println("Irrigation configurations loaded from memory");
  
  // Setup MQTT (prepare for when WiFi connects)
  espClient.setInsecure(); // Bypass SSL certificate validation
  setupMQTTBufferSize();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  Serial.println("MQTT client configured");
  
  // Try to connect to WiFi (non-blocking)
  Serial.println("Attempting WiFi connection...");
  WiFiManager wm;
  
  // Set shorter timeout for initial connection attempt
  wm.setConfigPortalTimeout(30); // 30 seconds timeout
  
  bool wifiConnected = wm.autoConnect("Silagung", "admin123");
  
  if (wifiConnected) {
    Serial.println("✅ WiFi connected successfully");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Connect to MQTT
    reconnectMQTT();
    
    // Initialize NTP client and sync time
    timeClient.begin();
    syncTimeWithNTP();
  } else {
    Serial.println("⚠️ WiFi connection failed or timeout reached");
    Serial.println("🔄 System will continue in OFFLINE MODE");
    Serial.println("📅 Irrigation schedules will run based on RTC time");
    Serial.println("🌐 WiFi connection will be retried periodically");
  }
  
  Serial.println("🚀 System initialization complete - Ready for operation");
}

// RTC and Time Functions
void initializeRTC() {
    Serial.println("Initializing RTC...");
    
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC module!");
        rtcInitialized = false;
        return;
    }
    
    rtcInitialized = true;
    Serial.println("RTC initialized successfully");
    
    // Check if RTC lost power and needs to be set
    if (rtc.lostPower()) {
        Serial.println("RTC lost power, will sync with NTP when WiFi connects");
    }
    
    displayCurrentTime();
}

void syncTimeWithNTP() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - cannot sync time");
        return;
    }
    
    Serial.println("Syncing time with NTP server...");
    
    // Force NTP update
    timeClient.forceUpdate();
    
    if (rtcInitialized) {
        // Get time from NTP
        unsigned long epochTime = timeClient.getEpochTime();
        
        // Set RTC time
        rtc.adjust(DateTime(epochTime));
        
        Serial.println("RTC time updated from NTP");
        lastNTPSync = millis();
        
        displayCurrentTime();
    } else {
        Serial.println("RTC not initialized - cannot set time");
    }
}

void displayCurrentTime() {
    if (!rtcInitialized) {
        Serial.println("RTC not initialized");
        return;
    }
    
    DateTime now = rtc.now();
    
    Serial.print("RTC Time: ");
    Serial.println(formatDateTime(now));
    
    // Also show NTP time if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("NTP Time: ");
        Serial.println(timeClient.getFormattedTime());
    }
}

String formatDateTime(DateTime dt) {
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", 
            dt.year(), dt.month(), dt.day(),
            dt.hour(), dt.minute(), dt.second());
    return String(buffer);
}

void loop() {
  // Check for serial commands
  checkSerialCommands();

  // Check irrigation schedules every minute
  if (millis() - lastScheduleCheck > SCHEDULE_CHECK_INTERVAL) {
    checkIrrigationSchedules();
    lastScheduleCheck = millis();
  }
  
  // Monitor active irrigation
  monitorIrrigation();

  // Automatic time display
  if (millis() - lastTimeDisplay > TIME_DISPLAY_INTERVAL) {
    Serial.println("=== Current Time ===");
    displayCurrentTime();
    
    // Also display irrigation status
    if (currentIrrigation.isActive) {
      Serial.println("=== Active Irrigation ===");
      Serial.println("Land: " + currentIrrigation.landName);
      Serial.println("Type: " + currentIrrigation.irrigationType);
      Serial.println("Water delivered: " + String(currentIrrigation.waterDelivered) + " L");
      Serial.println("Water needed: " + String(currentIrrigation.waterNeeded) + " L");
      Serial.println("Duration: " + String((millis() - currentIrrigation.startMillis) / 1000) + " seconds");
    }
    
    lastTimeDisplay = millis();
  }

  // Automatic status display
  if (millis() - lastStatusDisplay > STATUS_DISPLAY_INTERVAL) {
    Serial.println("=== System Status ===");
    Serial.print("WiFi Status: ");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());
    } else {
      Serial.println("Disconnected");
    }
    
    Serial.print("MQTT Status: ");
    if (client.connected()) {
      Serial.println("Connected");
    } else {
      Serial.println("Disconnected");
    }
    
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    
    Serial.print("RTC Status: ");
    if (rtcInitialized) {
      Serial.println("Initialized");
    } else {
      Serial.println("Not initialized");
    }
    
    Serial.println("=== Irrigation Configuration ===");
    displayAllConfigs();
    Serial.println("--- All Active Configurations ---");
    displayIrrigationConfig();
    
    lastStatusDisplay = millis();
  }

  // Check WiFi connection (non-blocking)
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 30000) { // Check every 30 seconds
    lastWiFiCheck = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ WiFi disconnected - System running in OFFLINE MODE");
      Serial.println("📅 Irrigation schedules continue based on RTC time");
      
      // Try to reconnect (non-blocking with short timeout)
      WiFiManager wm;
      wm.setConfigPortalTimeout(10); // Only 10 seconds timeout
      
      Serial.println("🔄 Attempting WiFi reconnection...");
      bool reconnected = wm.autoConnect("Silagung", "admin123");
      
      if (reconnected) {
        Serial.println("✅ WiFi reconnected successfully");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        
        // Reinitialize NTP and sync time
        timeClient.begin();
        syncTimeWithNTP();
      } else {
        Serial.println("⚠️ WiFi reconnection failed - Continuing offline");
      }
    }
  }

  // MQTT connection (only if WiFi is connected)
  if (WiFi.status() == WL_CONNECTED && !client.connected()) {
    reconnectMQTT();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    client.loop();
  }
  
  // Send sensor data periodically
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Read sensor data using separate functions (works offline)
    waterFlow = readWaterFlowSensor();    // Water Flow Sensor
    pressure = readPressureSensor();      // Pressure Sensor
    read6in1Sensor();                     // 6-in-1 Sensor (pH, N, P, K, EC, Temperature)
    
    // Publish sensor data (only if WiFi and MQTT are connected)
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
      publishSensorData();
    } else {
      // Log sensor data locally when offline
      Serial.println("📊 Sensor Data (Offline Mode):");
      Serial.printf("  Water Flow: %.2f L/min\n", waterFlow);
      Serial.printf("  Pressure: %.2f bar\n", pressure);
      Serial.printf("  pH: %.2f\n", ph);
      Serial.printf("  Nitrogen: %.2f mg/L\n", nitrogen);
      Serial.printf("  Phosphorus: %.2f mg/L\n", phosphorus);
      Serial.printf("  Potassium: %.2f mg/L\n", potassium);
      Serial.printf("  EC: %.2f µS/cm\n", ec);
      Serial.printf("  Temperature: %.2f°C\n", temperature);
    }
  }
  
  // Update NTP time periodically (only if WiFi connected)
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    
    // Sync RTC with NTP periodically
    if (millis() - lastNTPSync > NTP_SYNC_INTERVAL) {
      syncTimeWithNTP();
    }
  }
}

// Fungsi untuk reconnect ke MQTT
void reconnectMQTT() {
    int attempts = 0;
    while (!client.connected() && attempts < 3) {
        Serial.println("Attempting MQTT connection...");
        String client_id = "Controller-1";
        
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("✅ Connected to MQTT broker");
            // Subscribe to control topic
            client.subscribe(topic_control);
            Serial.print("✓ Subscribed to control topic: ");
            Serial.println(topic_control);
            
            // Subscribe to irrigation config topic
            client.subscribe(topic_irrigation_config);
            Serial.print("✓ Subscribed to irrigation config topic: ");
            Serial.println(topic_irrigation_config);
            
            Serial.println("All MQTT subscriptions completed");
            return;
        } else {
            Serial.print("❌ MQTT connection failed with state ");
            Serial.println(client.state());
            attempts++;
            delay(5000);
        }
    }
    
    if (!client.connected()) {
        Serial.println("❌ Failed to connect to MQTT after 3 attempts");
    }
}

// Callback untuk menerima pesan MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    // Check if message is too large for buffer
    if (length >= client.getBufferSize()) {
        Serial.println("⚠️ MQTT message too large - truncated");
        return;
    }
    
    // Convert payload to string
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    // Check if this is irrigation configuration topic
    if (strcmp(topic, topic_irrigation_config) == 0) {
        Serial.println("📥 Config received (" + String(length) + " bytes)");
        parseIrrigationConfig(message);
        Serial.println("✅ Config processed");
        return;
    }
    
    // Check if this is control topic
    if (strcmp(topic, topic_control) == 0) {
        Serial.println("🎮 Control: " + message);
        
        // Try to parse as JSON for control commands
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, message);
        
        if (error) {
            Serial.println("📝 Plain text command");
        } else {
            // Handle relay control for manual operation
            for(int i = 1; i <= 6; i++) {
                String relayKey = "relay" + String(i);
                if(doc.containsKey(relayKey)) {
                    String state = doc[relayKey];
                    bool relayState = (state == "on");
                    
                    Serial.print("🔌 Setting relay ");
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
        return;
    }

    // Handle any other topics or plain text messages
    Serial.println("📝 Message: " + message);
    
    // If it's a simple greeting, respond via serial
    if (message.equalsIgnoreCase("hello") || message.equalsIgnoreCase("test")) {
        Serial.println("👋 Hello from ESP32!");
    }
}

// Function to check for serial commands
void checkSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
        
        if (command == "reset") {
            Serial.println("=== WiFi Reset Command Received ===");
            Serial.println("Menghapus konfigurasi WiFi...");
            
            WiFiManager wm;
            wm.resetSettings();
            
            Serial.println("Konfigurasi WiFi berhasil dihapus");
            Serial.println("Sistem akan restart dalam 3 detik...");
            
            delay(3000);
            ESP.restart();
            
        } else if (command == "stop") {
            Serial.println("=== Manual Stop Irrigation Command ===");
            if (currentIrrigation.isActive) {
                stopIrrigation();
                Serial.println("✅ Irrigation stopped manually");
            } else {
                Serial.println("ℹ️ No active irrigation to stop");
            }
            
        } else if (command.startsWith("start ")) {
            Serial.println("=== Manual Start Irrigation Command ===");
            String landName = command.substring(6); // Remove "start "
            landName.trim();
            
            // Find configuration for the specified land
            bool found = false;
            for (int i = 0; i < totalConfigs; i++) {
                String configLandName = configs[i].landName;
                configLandName.toLowerCase();
                if (configs[i].isValid && configLandName.indexOf(landName) != -1) {
                    if (currentIrrigation.isActive) {
                        Serial.println("⚠️ Stopping current irrigation first");
                        stopIrrigation();
                    }
                    startIrrigation(i, "Manual");
                    Serial.println("✅ Manual irrigation started for " + configs[i].landName);
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                Serial.println("❌ Land configuration not found: " + landName);
                Serial.println("Available lands:");
                for (int i = 0; i < totalConfigs; i++) {
                    if (configs[i].isValid) {
                        Serial.println("  - " + configs[i].landName);
                    }
                }
            }
            
        } else if (command == "status") {
            Serial.println("=== Irrigation System Status ===");
            if (currentIrrigation.isActive) {
                Serial.println("🚿 IRRIGATION ACTIVE");
                Serial.println("Land: " + currentIrrigation.landName);
                Serial.println("Type: " + currentIrrigation.irrigationType);
                Serial.println("Water delivered: " + String(currentIrrigation.waterDelivered) + " L");
                Serial.println("Water needed: " + String(currentIrrigation.waterNeeded) + " L");
                Serial.println("Duration: " + String((millis() - currentIrrigation.startMillis) / 1000) + " seconds");
                Serial.println("Start time: " + currentIrrigation.startTime);
            } else {
                Serial.println("💤 No active irrigation");
            }
            
        } else if (command == "configs") {
            displayIrrigationConfig();
            
        } else if (command.length() > 0) {
            Serial.println("Unknown command: " + command);
            Serial.println("Available commands:");
            Serial.println("  reset - Reset WiFi settings");
            Serial.println("  stop - Stop current irrigation");
            Serial.println("  start <land> - Start irrigation for specific land");
            Serial.println("  status - Show irrigation status");
            Serial.println("  configs - Show all configurations");
        }
    }
}

// Display functions for configurations
void displayConfig(int configIndex) {
    if (configIndex < 0 || configIndex >= totalConfigs) {
        Serial.println("❌ Invalid config index");
        return;
    }
    
    IrrigationConfig config = configs[configIndex];
    
    Serial.println("=== Irrigation Configuration ===");
    Serial.print("Config ID: ");
    Serial.println(config.configId);
    Serial.print("Land Name: ");
    Serial.println(config.landName);
    Serial.print("Phase Name: ");
    Serial.println(config.phaseName);
    Serial.print("Water Requirement: ");
    Serial.print(config.waterRequirement);
    Serial.println(" L");
    Serial.print("Water per Schedule: ");
    Serial.print(config.waterPerSchedule);
    Serial.println(" L");
    Serial.print("Target EC: ");
    Serial.println(config.targetEC);
    Serial.print("Irrigation Type: ");
    Serial.println(config.irrigationType);
    Serial.print("Valid: ");
    Serial.println(config.isValid ? "Yes" : "No");
    
    Serial.println("--- Schedules ---");
    for (int i = 0; i < config.scheduleCount; i++) {
        Serial.print("Schedule ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(config.schedules[i].time);
        Serial.print(" (Active: ");
        Serial.print(config.schedules[i].isActive ? "Yes" : "No");
        Serial.println(")");
    }
    Serial.println("=== End Configuration ===");
}

void displayAllConfigs() {
    if (totalConfigs == 0) {
        Serial.println("📋 No configurations stored");
        return;
    }
    
    Serial.println("📋 All Configurations (" + String(totalConfigs) + "):");
    for (int i = 0; i < totalConfigs; i++) {
        Serial.println("  " + String(i + 1) + ". " + configs[i].landName + " - " + String(configs[i].waterPerSchedule) + "L (" + String(configs[i].scheduleCount) + " schedules)");
    }
}

// ========== IRRIGATION QUEUE MANAGEMENT FUNCTIONS ==========

void addToQueue(int configIndex, String scheduleTime) {
    if (queueCount >= MAX_QUEUE_SIZE) {
        Serial.println("⚠️ Queue full - cannot add more schedules");
        return;
    }
    
    irrigationQueue[queueCount].configIndex = configIndex;
    irrigationQueue[queueCount].scheduleTime = scheduleTime;
    irrigationQueue[queueCount].queuedTime = millis();
    irrigationQueue[queueCount].isProcessed = false;
    queueCount++;
    
    Serial.println("📋 Queue size: " + String(queueCount));
}

void processQueue() {
    if (queueCount == 0) return;
    
    // If no irrigation is active, start the next one in queue
    if (!currentIrrigation.isActive) {
        for (int i = 0; i < queueCount; i++) {
            if (!irrigationQueue[i].isProcessed) {
                Serial.println("🚀 Processing queue item: " + configs[irrigationQueue[i].configIndex].landName);
                startIrrigation(irrigationQueue[i].configIndex, irrigationQueue[i].scheduleTime);
                irrigationQueue[i].isProcessed = true;
                currentQueueIndex = i;
                return;
            }
        }
        
        // All items processed, clear queue
        clearQueue();
    }
}

void clearQueue() {
    queueCount = 0;
    currentQueueIndex = 0;
    Serial.println("🧹 Queue cleared");
}

bool isInQueue(int configIndex, String scheduleTime) {
    for (int i = 0; i < queueCount; i++) {
        if (irrigationQueue[i].configIndex == configIndex && 
            irrigationQueue[i].scheduleTime == scheduleTime &&
            !irrigationQueue[i].isProcessed) {
            return true;
        }
    }
    return false;
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
        Serial.println("📊 Sensor data published successfully");
        Serial.println(buffer);
    } else {
        Serial.println("❌ Failed to publish sensor data");
    }
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
        
        Serial.println("🔌 Published relay states: " + String(buffer));
    }
}

// Helper function untuk konversi register ke float (untuk Modbus)
float registersToFloat(uint16_t reg1, uint16_t reg2) {
    uint32_t combined = ((uint32_t)reg1 << 16) | reg2;
    float val;
    memcpy(&val, &combined, 4);
    return val;
}

// Fungsi untuk membaca sensor Water Flow menggunakan RS485 Modbus
float readWaterFlowSensor() {
    uint8_t result = node.readHoldingRegisters(0x0006, 2); // baca 2 register
    
    if (result == node.ku8MBSuccess) {
        uint16_t r6 = node.getResponseBuffer(0);
        uint16_t r7 = node.getResponseBuffer(1);
        float flow = registersToFloat(r6, r7);
        
        Serial.print("💧 Water Flow: ");
        Serial.print(flow, 2);
        Serial.println(" L/min");
        
        return flow;
    } else {
        Serial.print("❌ Water Flow Sensor Error: ");
        Serial.println(result);
        return 0.0; // Return 0 if sensor reading fails
    }
}

// Fungsi untuk membaca sensor Pressure
float readPressureSensor() {
    // Simulasi pembacaan sensor tekanan
    voltageSum = 0;
    
    // Read multiple samples with better timing
    for (int i = 0; i < SAMPLES; i++) {
        voltageSum += analogRead(CurrentSensorPin);
        delay(5); // Increased delay for better sample independence
    }
    
    // Calculate average voltage
    voltage = (voltageSum / SAMPLES) / 4096.0 * VREF;
    
    // Convert to current
    current = voltage / 120.0;  // Sense Resistor: 120ohm
    
    // Apply exponential moving average (low-pass filter)
    if (filteredCurrent == 0.0) {
        filteredCurrent = current; // Initialize on first reading
    } else {
        filteredCurrent = (FILTER_ALPHA * current) + ((1.0 - FILTER_ALPHA) * filteredCurrent);
    }
    

    float pressure = currentToPressure(filteredCurrent);
    
    Serial.println("📊 Pressure: " + String(pressure) + " bar");

    
    // Convert filtered current to pressure

    return pressure;
}

// Fungsi untuk membaca sensor 6-in-1 (pH, N, P, K, EC, Temperature)
void read6in1Sensor() {
    // Simulasi pembacaan sensor 6-in-1
    // Dalam implementasi nyata, komunikasi dengan sensor 6-in-1 melalui RS485/Modbus
    // Sensor seperti: Soil NPK pH Moisture EC Temperature 6-in-1 sensor
    
    Serial.println("🌱 Reading 6-in-1 sensor...");
    
    // Baca semua parameter dari sensor 6-in-1
    ec = random(100, 300) / 100.0;            // 1.00-3.00 mS/cm (Electrical Conductivity)
    ph = random(650, 750) / 100.0;            // 6.50-7.50 (pH level)
    nitrogen = random(30, 60);                // 30-60 ppm (Nitrogen)
    phosphorus = random(5, 20);               // 5-20 ppm (Phosphorus)
    potassium = random(20, 50);               // 20-50 ppm (Potassium)
    temperature = random(2500, 3500) / 100.0; // 25.00-35.00 C (Temperature)
    
    // Log hasil pembacaan
    Serial.println("  📈 EC: " + String(ec) + " mS/cm");
    Serial.println("  🧪 pH: " + String(ph));
    Serial.println("  🌿 Nitrogen: " + String(nitrogen) + " ppm");
    Serial.println("  🌾 Phosphorus: " + String(phosphorus) + " ppm");
    Serial.println("  🌽 Potassium: " + String(potassium) + " ppm");
    Serial.println("  🌡️ Temperature: " + String(temperature) + " °C");
}