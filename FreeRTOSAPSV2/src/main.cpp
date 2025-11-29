#include <Arduino.h>
#include <Wire.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
#include <RTClib.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>

// MQTT Config
const char* mqtt_broker = "72350f0b16bb43f2af1b3b453ac66c34.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Controller";
const char* mqtt_pass = "Controller123!";
const char* topic_sensor = "silagung/sensor";
const char* topic_status = "silagung/system";
const char* topic_control = "silagung/control";
const char* topic_control_feedback = "silagung/control/feedback";
const char* topic_irrigation_config = "silagung/irrigation/config";
const char* topic_irrigation_ack = "silagung/irrigation/ack";

// Hardware Pins
#define RS485_RX 17
#define RS485_TX 18
#define RS485_DE -1  // Jika pakai DE/RE, ganti dengan pin nyata
#define BAUD_RATE 9600

// Logging Config
#define LOG_ENABLED 1
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 3
#define LOG_LEVEL LOG_LEVEL_INFO

// Logging Macros
#if LOG_ENABLED
  #define LOG_INFO(fmt, ...)   Serial.printf("[INFO]  [%lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_WARN(fmt, ...)   Serial.printf("[WARN]  [%lu] " fmt "\n", millis(), ##__VA_ARGS__)
  #define LOG_ERROR(fmt, ...)  Serial.printf("[ERROR] [%lu] " fmt "\n", millis(), ##__VA_ARGS__)
#else
  #define LOG_INFO(fmt, ...)
  #define LOG_WARN(fmt, ...)
  #define LOG_ERROR(fmt, ...)
#endif

// Modbus Slave ID
#define HMI_ID 2
#define RELAY_ID 6
#define FLOW_ID 3
#define EC_ID 1
#define US1_ID 4
#define US2_ID 5

// Valve & HMI
#define VALVE_COUNT 5
#define HMI_BTN_ADDR 0
#define HMI_LAMP_ADDR 10
#define PUMP_BTN_INDEX 5
#define PUMP_LAMP_INDEX 5
#define MODE_SWITCH_INDEX 6
#define NUTRI_TANK_EMPTY_CM 120
#define NUTRI_TANK_FULL_CM 25
#define WATER_TANK_EMPTY_CM 120
#define WATER_TANK_FULL_CM 25

// Pressure Sensor Configuration (4-20mA)
#define PRESSURE_SENSOR_PIN 13    // ADC pin for pressure sensor
#define VREF 3300                 // ADC reference voltage in mV
#define ADC_SAMPLES 10            // Number of samples for averaging
#define MIN_CURRENT 4.0           // Minimum current for 4-20mA sensor
#define MAX_CURRENT 20.0          // Maximum current for 4-20mA sensor
#define MAX_PRESSURE 10.0         // Maximum pressure in Bar (0-10 Bar)

// Utility function to convert Modbus registers to float
float registersToFloat(uint16_t r1, uint16_t r2) {
  uint32_t combined = ((uint32_t)r1 << 16) | r2;
  float f;
  memcpy(&f, &combined, 4);
  return f;
}

float regsToFloatHL(uint16_t high, uint16_t low) {
  uint32_t raw = ((uint32_t)high << 16) | low;
  float val;
  memcpy(&val, &raw, sizeof(val));
  return val;
}
float regsToFloatLH(uint16_t low, uint16_t high) {
  uint32_t raw = ((uint32_t)high << 16) | low;
  float val;
  memcpy(&val, &raw, sizeof(val));
  return val;
}

bool readFloat32Holding(ModbusMaster& node, uint16_t addr, float &out) {
  uint8_t res = node.readHoldingRegisters(addr, 2);
  if (res != node.ku8MBSuccess) return false;
  uint16_t w0 = node.getResponseBuffer(0);
  uint16_t w1 = node.getResponseBuffer(1);
  float a = regsToFloatHL(w0, w1);
  if (isfinite(a)) { out = a; return true; }
  float b = regsToFloatLH(w0, w1);
  if (isfinite(b)) { out = b; return true; }
  return false;
}

// FreeRTOS Handles
QueueHandle_t xSensorQueue;
QueueHandle_t xCmdQueue;
QueueHandle_t xIrrigationQueue;
SemaphoreHandle_t xStateMutex;
TaskHandle_t xMQTTTask, xModbusTask, xMQTTRecvTask;
TaskHandle_t xSchedulerTask;
struct ControlCmd {
  int idx;
  bool open;
};

// Global State
bool valveState[VALVE_COUNT] = {false};
bool lastValveState[VALVE_COUNT] = {false};
bool webButtonChange[VALVE_COUNT+1] = {false}; // include pump at index VALVE_COUNT
float waterFlow = 0.0;
float ec = 0.0;
uint16_t us1 = 0, us2 = 0;
float pressure = 0.0;  // Add pressure variable
bool pumpState = false;
bool lastPumpBtnState = false;
bool autoMode = false;

// Logging state tracking
bool lastDisplayEmpty = false;
unsigned long lastSensorFail[5] = {0}; // flow, ec, us1, us2, pressure
unsigned long lastRelayLog = 0;
#define RELAY_LOG_INTERVAL 10000  // 10 detik
#define SENSOR_FAIL_LOG_INTERVAL 5000  // 5 detik
bool focusedLogging = false;

// Consecutive error tracking for sensor failure tolerance
uint8_t consecutiveSensorErrors[5] = {0}; // flow, ec, us1, us2, pressure
const uint8_t MAX_CONSECUTIVE_ERRORS = 10; // Ignore errors until this threshold

// Objects
HardwareSerial rs485(1);
ModbusMaster hmiNode, relayNode, flowNode, ecNode, us1Node, us2Node;
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

// ====== RTC, NTP, and Preferences for Scheduling ======
RTC_DS3231 rtc;
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000);
Preferences preferences;
bool rtcInitialized = false;
unsigned long lastNTPSync = 0;
const unsigned long NTP_SYNC_INTERVAL = 60000;
unsigned long lastTimeLog = 0;
const unsigned long TIME_LOG_INTERVAL = 5000;
const float MIN_FLOW_THRESHOLD = 0.10f;
const unsigned int NO_FLOW_STALL_SECONDS = 5; //berapa attemp ini maksudnya
const unsigned int NO_PROGRESS_STALL_SECONDS = 10;
#define ACTIVATION_SETTLE_MS 1500
#define PUMP_RELAY_COIL 10

struct IrrigationScheduleItem {
  String time;
  bool isActive;
};

struct IrrigationConfigItem {
  int configId;
  String landName;
  String phaseName;
  float waterRequirement;
  float waterPerSchedule;
  float targetEC;
  String irrigationType; // "air" | "air_nutrisi"
  IrrigationScheduleItem schedules[10];
  int scheduleCount;
  bool isValid;
};

struct IrrigationJob {
  int configIndex;
  int scheduleIndex;
};

#define MAX_CONFIGS 5
IrrigationConfigItem configs[MAX_CONFIGS];
int totalConfigs = 0;

struct ActiveIrrigationState {
  int configId;
  String landName;
  String startTime;
  float waterNeeded;
  float waterDelivered;
  String irrigationType;
  bool isActive;
  bool activationReady;
  unsigned long startMillis;
  int landRelay; // 3..5
  int configIndex;
  int scheduleIndex;
};

ActiveIrrigationState currentIrrigation;
unsigned long lastScheduleCheck = 0;
const unsigned long SCHEDULE_CHECK_INTERVAL = 60000;

static void publishScheduleFeedback(int valveIdx, const char* action, const char* status, int configId, const char* source) {
  StaticJsonDocument<256> doc;
  doc["valve"] = valveIdx;
  doc["action"] = action;
  doc["status"] = status;
  doc["source"] = source;
  doc["configId"] = configId;
  doc["timestamp"] = millis();
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) {
    mqtt.publish(topic_control_feedback, buffer);
  }
}

// ===== SENSOR READING FUNCTIONS =====

// Function to read ultrasonic sensor with error handling and multiple register attempts
bool readUltrasonicSensor(ModbusMaster& node, uint16_t& distance, const char* sensorName) {
    // Try multiple register addresses (common ultrasonic sensor registers)
    const uint16_t registerCandidates[] = {0x0000, 0x0001, 0x0100, 0x0101, 0x0010};
    bool success = false;
    uint8_t lastError = 0;
    
    for (uint8_t i = 0; i < sizeof(registerCandidates)/sizeof(registerCandidates[0]) && !success; i++) {
        // Add small delay to avoid bus collision
        delay(10);
        uint8_t result = node.readHoldingRegisters(registerCandidates[i], 1);
        if (result == node.ku8MBSuccess) {
            uint16_t raw = node.getResponseBuffer(0);
            uint16_t valCm = raw / 10;
            if (valCm >= 5 && valCm <= 750) {
                distance = valCm;
                consecutiveSensorErrors[2] = 0;
                return true;
            }
        } else {
            lastError = result;
        }
    }
    
    // Increment consecutive error counter
    consecutiveSensorErrors[2]++;
    
    // Only log error if we've exceeded the threshold (to ignore intermittent failures)
    if (consecutiveSensorErrors[2] >= MAX_CONSECUTIVE_ERRORS) {
        if (millis() - lastSensorFail[2] > SENSOR_FAIL_LOG_INTERVAL) {
            LOG_ERROR("%s read failure after multiple attempts - error code: %u", sensorName, lastError);
            lastSensorFail[2] = millis();
        }
    }
    
    distance = 0;
    return false;
}

// Function to read water flow sensor with error handling
bool readWaterFlowSensor(ModbusMaster& node, float& flow, const char* sensorName) {
    bool ok = readFloat32Holding(node, 0x0000, flow);
    if (ok) {
        consecutiveSensorErrors[0] = 0;
        return true;
    } else {
        consecutiveSensorErrors[0]++;
        if (consecutiveSensorErrors[0] >= MAX_CONSECUTIVE_ERRORS) {
            if (millis() - lastSensorFail[0] > SENSOR_FAIL_LOG_INTERVAL) {
                LOG_ERROR("%s read failure", sensorName);
                lastSensorFail[0] = millis();
            }
        }
        flow = 0.0;
        return false;
    }
}

// HMI Write Functions for Sensor Display
void writeUltrasonicToHMI() {
    hmiNode.writeSingleRegister(0x0000, us1);
    delay(10);
    hmiNode.writeSingleRegister(0x0001, us2);
    int nutrPct = (int)((NUTRI_TANK_EMPTY_CM - us1) * 100 / (NUTRI_TANK_EMPTY_CM - NUTRI_TANK_FULL_CM));
    if (nutrPct < 0) nutrPct = 0;
    if (nutrPct > 100) nutrPct = 100;
    hmiNode.writeSingleRegister(0x0005, nutrPct);
    delay(10);
    int airPct = (int)((WATER_TANK_EMPTY_CM - us2) * 100 / (WATER_TANK_EMPTY_CM - WATER_TANK_FULL_CM));
    if (airPct < 0) airPct = 0;
    if (airPct > 100) airPct = 100;
    hmiNode.writeSingleRegister(0x0006, airPct);
}

void writeWaterFlowToHMI() {
    // Write water flow to HMI register LW3 as integer with 2 decimal places (x100)
    uint16_t wf = (uint16_t)(waterFlow * 100); // L/min with 2 decimals
    hmiNode.writeSingleRegister(0x0003, wf); // LW3 - Water Flow
    delayMicroseconds(2000);
}

void writeECSensorToHMI() {
    // Write EC sensor to HMI register LW4
    uint16_t ecVal = (uint16_t)ec;
    hmiNode.writeSingleRegister(0x0004, ecVal); // LW4 - EC
    delayMicroseconds(2000);
}

void writePressureToHMI() {
    // Write pressure to HMI register LW2 as integer with 2 decimal places (x100)
    uint16_t p = (uint16_t)(pressure * 100); // Bar with 2 decimals
    hmiNode.writeSingleRegister(0x0002, p); // LW2 - Pressure
}

bool writeCoilWithRetry(uint16_t coil, bool state) {
  uint8_t lastErr = 0;
  for (int attempt = 0; attempt < 5; attempt++) {
    uint8_t r = relayNode.writeSingleCoil(coil, state ? 1 : 0);
    if (r == relayNode.ku8MBSuccess) return true;
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  LOG_ERROR("Relay write failure coil:%u state:%u code:%u", coil, state?1:0, lastErr);
  return false;
}

bool controlRelayValve(int idx, bool open) {
  int openCoil = idx * 2;
  int closeCoil = openCoil + 1;
  bool s1 = writeCoilWithRetry(openCoil, false);
  bool s2 = writeCoilWithRetry(closeCoil, false);
  vTaskDelay(pdMS_TO_TICKS(300));
  bool ok = false;
  if (open) {
    bool a = writeCoilWithRetry(closeCoil, false);
    bool b = writeCoilWithRetry(openCoil, true);
    vTaskDelay(pdMS_TO_TICKS(300));
    ok = a && b;
  } else {
    bool a = writeCoilWithRetry(openCoil, false);
    bool b = writeCoilWithRetry(closeCoil, true);
    vTaskDelay(pdMS_TO_TICKS(300));
    ok = a && b;
  }
  uint8_t res = relayNode.readCoils(openCoil, 2);
  if (res == relayNode.ku8MBSuccess) {
    uint16_t word = relayNode.getResponseBuffer(0);
    bool o = word & 0x01;
    bool c = word & 0x02;
    if (o && c) {
      writeCoilWithRetry(openCoil, false);
      writeCoilWithRetry(closeCoil, false);
      LOG_ERROR("Relay conflict valve:%d", idx+1);
      return false;
    }
    if (open && (!o || c)) {
      writeCoilWithRetry(openCoil, true);
      vTaskDelay(pdMS_TO_TICKS(200));
      LOG_WARN("Relay verify open mismatch valve:%d", idx+1);
    }
    if (!open && (!c || o)) {
      writeCoilWithRetry(closeCoil, true);
      vTaskDelay(pdMS_TO_TICKS(200));
      LOG_WARN("Relay verify close mismatch valve:%d", idx+1);
    }
  } else {
    LOG_WARN("Relay read failure valve:%d code:%u", idx+1, res);
  }
  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  valveState[idx] = open;
  xSemaphoreGive(xStateMutex);
  return ok;
}

bool controlPumpRelay(bool on) {
  bool ok = writeCoilWithRetry(PUMP_RELAY_COIL, on);
  vTaskDelay(pdMS_TO_TICKS(300));
  uint8_t res = relayNode.readCoils(PUMP_RELAY_COIL, 1);
  if (res == relayNode.ku8MBSuccess) {
    uint16_t word = relayNode.getResponseBuffer(0);
    bool bit = word & 0x01;
    if ((on && !bit) || (!on && bit)) {
      LOG_WARN("Pump relay verify mismatch coil:%u", PUMP_RELAY_COIL);
    }
  } else {
    LOG_WARN("Pump relay read failure code:%u", res);
  }
  if (!ok) {
    LOG_ERROR("Pump relay write failure coil:%u state:%u", PUMP_RELAY_COIL, on?1:0);
    return false;
  }
  pumpState = on;
  hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, on ? 1 : 0);
  return true;
}

// Function to read EC sensor with error handling
bool readECSensor(ModbusMaster& node, float& ecValue, const char* sensorName) {
    uint8_t result = node.readHoldingRegisters(0x0002, 1);
    if (result == node.ku8MBSuccess) {
        ecValue = node.getResponseBuffer(0);
        consecutiveSensorErrors[1] = 0; // Reset error counter on success
        return true;
    } else {
        consecutiveSensorErrors[1]++;
        
        // Only log error if we've exceeded the threshold (to ignore intermittent failures)
        if (consecutiveSensorErrors[1] >= MAX_CONSECUTIVE_ERRORS) {
            if (millis() - lastSensorFail[1] > SENSOR_FAIL_LOG_INTERVAL) {
                LOG_ERROR("%s read failure - error code: %u", sensorName, result);
                lastSensorFail[1] = millis();
            }
        }
        ecValue = 0.0;
        return false;
    }
}

// Function to read pressure sensor (4-20mA ADC) with EMA filtering
bool readPressureSensor(float& pressureValue, const char* sensorName) {
    static float filteredCurrent = 0.0; // EMA filtered current value
    static bool firstReading = true;
    
    unsigned long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PRESSURE_SENSOR_PIN);
        delayMicroseconds(2000); // 2ms delay
    }
    
    // Calculate average voltage (mV)
    float voltageMv = (sum / (float)ADC_SAMPLES) / 4096.0 * VREF;
    // Convert to current mA (assuming 120 ohm resistor)
    float currentmA = voltageMv / 120.0;
    
    // Apply EMA filtering to reduce noise
    if (firstReading) {
        filteredCurrent = currentmA;
        firstReading = false;
    } else {
        // EMA filter: filtered = alpha * new + (1-alpha) * old
        const float FILTER_ALPHA = 0.1; // Heavy filtering for stable readings
        filteredCurrent = FILTER_ALPHA * currentmA + (1.0 - FILTER_ALPHA) * filteredCurrent;
    }
    
    // Validate current range using filtered value
    if (filteredCurrent >= MIN_CURRENT && filteredCurrent <= MAX_CURRENT) {
        // Convert current to pressure (0-10 Bar range)
        pressureValue = ((filteredCurrent - MIN_CURRENT) / (MAX_CURRENT - MIN_CURRENT)) * MAX_PRESSURE;
        consecutiveSensorErrors[3] = 0; // Reset error counter on success
        return true;
    } else {
        consecutiveSensorErrors[3]++;
        
        // Only log error if we've exceeded the threshold (to ignore intermittent failures)
        if (consecutiveSensorErrors[3] >= MAX_CONSECUTIVE_ERRORS) {
            if (millis() - lastSensorFail[3] > SENSOR_FAIL_LOG_INTERVAL) {
                LOG_ERROR("%s current out of range: %.2f mA (expected 4-20mA), filtered: %.2f mA", sensorName, currentmA, filteredCurrent);
                lastSensorFail[3] = millis();
            }
        }
        pressureValue = 0.0;
        return false;
    }
}

// MQTT Publish
void publishSensorData() {
  StaticJsonDocument<256> doc;
  doc["waterFlow"] = waterFlow;
  doc["ec"] = ec;
  doc["ultrasonic1"] = us1;
  doc["ultrasonic2"] = us2;
  doc["pressure"] = pressure;
  int nutrPct = (int)((NUTRI_TANK_EMPTY_CM - us1) * 100 / (NUTRI_TANK_EMPTY_CM - NUTRI_TANK_FULL_CM));
  if (nutrPct < 0) nutrPct = 0;
  if (nutrPct > 100) nutrPct = 100;
  int airPct = (int)((WATER_TANK_EMPTY_CM - us2) * 100 / (WATER_TANK_EMPTY_CM - WATER_TANK_FULL_CM));
  if (airPct < 0) airPct = 0;
  if (airPct > 100) airPct = 100;
  doc["nutrisiPercent"] = nutrPct;
  doc["airPercent"] = airPct;
  doc["deviceId"] = "esp32-rtos-simple";
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) mqtt.publish(topic_sensor, buffer);
  
  // Log empty display detection
  bool currentEmpty = (waterFlow == 0.0 && ec == 0.0 && us1 == 0 && us2 == 0 && pressure == 0.0);
  if (currentEmpty && !lastDisplayEmpty) {
    LOG_WARN("Display showing empty data - all sensors zero");
  } else if (!currentEmpty && lastDisplayEmpty) {
    LOG_INFO("Display data restored - sensors active");
  }
  lastDisplayEmpty = currentEmpty;
}

void publishValveStatus() {
  StaticJsonDocument<256> doc;
  doc["valve1"] = valveState[0] ? "open" : "close";
  doc["valve2"] = valveState[1] ? "open" : "close";
  doc["valve3"] = valveState[2] ? "open" : "close";
  doc["valve4"] = valveState[3] ? "open" : "close";
  doc["valve5"] = valveState[4] ? "open" : "close";
  doc["pump"] = pumpState ? "on" : "off";
  doc["mode"] = autoMode ? "auto" : "manual";
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) mqtt.publish(topic_status, buffer);
  
  // Log relay safety status periodically
  if (!focusedLogging && millis() - lastRelayLog >= RELAY_LOG_INTERVAL) {
    LOG_INFO("Relay Safety Status: V1=%s V2=%s V3=%s V4=%s V5=%s Pump=%s",
             valveState[0]?"OPEN":"CLOSED",
             valveState[1]?"OPEN":"CLOSED", 
             valveState[2]?"OPEN":"CLOSED",
             valveState[3]?"OPEN":"CLOSED",
             valveState[4]?"OPEN":"CLOSED",
             pumpState?"ON":"OFF");
    lastRelayLog = millis();
  }
}

static void publishIrrigationAckBatch(int successCount, int totalCount);
static void publishIrrigationAckSingle(const IrrigationConfigItem& item);
static void logConfigSummary(const IrrigationConfigItem& item) {
  LOG_INFO("Config ID:%d Land:%s Phase:%s Type:%s WR:%.2f WPS:%.2f Schedules:%d",
           item.configId,
           item.landName.c_str(),
           item.phaseName.c_str(),
           item.irrigationType.c_str(),
           item.waterRequirement,
           item.waterPerSchedule,
           item.scheduleCount);
  for (int i = 0; i < item.scheduleCount; i++) {
    LOG_INFO("  - %s %s", item.schedules[i].time.c_str(), item.schedules[i].isActive ? "active" : "inactive");
  }
}

// MQTT Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];

  if (strcmp(topic, topic_irrigation_config) == 0) {
    StaticJsonDocument<4096> cfgDoc;
    DeserializationError cfgErr = deserializeJson(cfgDoc, message);
    if (cfgErr) {
      LOG_ERROR("Irrigation config JSON parse error: %s", cfgErr.c_str());
      return;
    }
    if (cfgDoc.containsKey("configs")) {
      JsonArray configurations = cfgDoc["configs"].as<JsonArray>();
      int successCount = 0;
      for (JsonObject config : configurations) {
        IrrigationConfigItem item;
        item.configId = config["configId"] | 0;
        item.landName = String(config["landName"].as<const char*>());
        item.phaseName = String(config["phaseName"].as<const char*>());
        item.waterRequirement = config["waterRequirement"] | 0.0;
        item.waterPerSchedule = config["waterPerSchedule"] | 0.0;
        item.targetEC = config["targetEC"] | 0.0;
        item.irrigationType = String(config["irrigationType"].as<const char*>());
        JsonArray schedules = config["schedules"].as<JsonArray>();
        item.scheduleCount = (int)schedules.size();
        if (item.scheduleCount > 10) item.scheduleCount = 10;
        for (int i = 0; i < item.scheduleCount; i++) {
          item.schedules[i].time = String(schedules[i]["time"].as<const char*>());
          item.schedules[i].isActive = schedules[i]["isActive"] | false;
        }
        item.isValid = (item.landName.length() > 0 && item.phaseName.length() > 0 && item.waterRequirement > 0 && item.waterPerSchedule > 0);
        if (item.isValid) {
          int existingIndex = -1;
          for (int k = 0; k < totalConfigs; k++) {
            if (configs[k].isValid && configs[k].configId == item.configId) { existingIndex = k; break; }
          }
          if (existingIndex != -1) {
            configs[existingIndex] = item;
            logConfigSummary(item);
          } else if (totalConfigs < MAX_CONFIGS) {
            configs[totalConfigs] = item;
            logConfigSummary(item);
            totalConfigs++;
          }
          successCount++;
        }
      }
      // Persist to NVS Preferences
      preferences.putInt("totalConfigs", totalConfigs);
      for (int i = 0; i < totalConfigs; i++) {
        String prefix = String("cfg") + String(i) + String("_");
        preferences.putInt((prefix + "id").c_str(), configs[i].configId);
        preferences.putString((prefix + "landName").c_str(), configs[i].landName);
        preferences.putString((prefix + "phaseName").c_str(), configs[i].phaseName);
        preferences.putFloat((prefix + "waterReq").c_str(), configs[i].waterRequirement);
        preferences.putFloat((prefix + "wps").c_str(), configs[i].waterPerSchedule);
        preferences.putFloat((prefix + "targetEC").c_str(), configs[i].targetEC);
        preferences.putString((prefix + "irrigType").c_str(), configs[i].irrigationType);
        preferences.putInt((prefix + "schedCount").c_str(), configs[i].scheduleCount);
        preferences.putBool((prefix + "valid").c_str(), configs[i].isValid);
        for (int j = 0; j < configs[i].scheduleCount && j < 10; j++) {
          preferences.putString((prefix + String("time") + String(j)).c_str(), configs[i].schedules[j].time);
          preferences.putBool((prefix + String("active") + String(j)).c_str(), configs[i].schedules[j].isActive);
        }
      }
      LOG_INFO("Irrigation batch configs processed: %d", successCount);
      publishIrrigationAckBatch(successCount, configurations.size());
      return;
    } else {
      IrrigationConfigItem item;
      item.configId = cfgDoc["configId"] | 0;
      item.landName = String(cfgDoc["landName"].as<const char*>());
      item.phaseName = String(cfgDoc["phaseName"].as<const char*>());
      item.waterRequirement = cfgDoc["waterRequirement"] | 0.0;
      item.waterPerSchedule = cfgDoc["waterPerSchedule"] | 0.0;
      item.targetEC = cfgDoc["targetEC"] | 0.0;
      item.irrigationType = String(cfgDoc["irrigationType"].as<const char*>());
      JsonArray schedules = cfgDoc["schedules"].as<JsonArray>();
      item.scheduleCount = (int)schedules.size();
      if (item.scheduleCount > 10) item.scheduleCount = 10;
      for (int i = 0; i < item.scheduleCount; i++) {
        item.schedules[i].time = String(schedules[i]["time"].as<const char*>());
        item.schedules[i].isActive = schedules[i]["isActive"] | false;
      }
      item.isValid = (item.landName.length() > 0 && item.phaseName.length() > 0 && item.waterRequirement > 0 && item.waterPerSchedule > 0);
      if (item.isValid) {
        int existingIndex = -1;
        for (int k = 0; k < totalConfigs; k++) {
          if (configs[k].isValid && configs[k].configId == item.configId) { existingIndex = k; break; }
        }
        if (existingIndex != -1) {
          configs[existingIndex] = item;
        } else if (totalConfigs < MAX_CONFIGS) {
          configs[totalConfigs] = item;
          totalConfigs++;
        }
        String prefix = String("cfg") + String(existingIndex == -1 ? (totalConfigs - 1) : existingIndex) + String("_");
        preferences.putInt((prefix + "id").c_str(), item.configId);
        preferences.putString((prefix + "landName").c_str(), item.landName);
        preferences.putString((prefix + "phaseName").c_str(), item.phaseName);
        preferences.putFloat((prefix + "waterReq").c_str(), item.waterRequirement);
        preferences.putFloat((prefix + "wps").c_str(), item.waterPerSchedule);
        preferences.putFloat((prefix + "targetEC").c_str(), item.targetEC);
        preferences.putString((prefix + "irrigType").c_str(), item.irrigationType);
        preferences.putInt((prefix + "schedCount").c_str(), item.scheduleCount);
        preferences.putBool((prefix + "valid").c_str(), item.isValid);
        for (int j = 0; j < item.scheduleCount && j < 10; j++) {
          preferences.putString((prefix + String("time") + String(j)).c_str(), item.schedules[j].time);
          preferences.putBool((prefix + String("active") + String(j)).c_str(), item.schedules[j].isActive);
        }
        preferences.putInt("totalConfigs", totalConfigs);
        LOG_INFO("Irrigation single config processed: ID %d", item.configId);
        logConfigSummary(item);
        publishIrrigationAckSingle(item);
      }
      return;
    }
  }

  if (strcmp(topic, topic_control) == 0) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, message);
    if (err) {
      LOG_ERROR("MQTT JSON parse error: %s", err.c_str());
      return;
    }
    if (doc.containsKey("mode")) {
      String m = String(doc["mode"].as<const char*>());
      m.toLowerCase();
      bool wantAuto = (m == "auto") || (doc["mode"] == true);
      hmiNode.writeSingleCoil(HMI_BTN_ADDR + MODE_SWITCH_INDEX, wantAuto ? 1 : 0);
      LOG_INFO("Received mode command: %s -> HMI LB6 set %d", m.c_str(), wantAuto?1:0);
      return;
    }
    if (doc.containsKey("valve") && doc.containsKey("action")) {
      int idx = doc["valve"].as<int>() - 1;
      bool open = strcmp(doc["action"], "open") == 0;
      if (idx >= 0 && idx < VALVE_COUNT + 1) {
        LOG_INFO("Web control request - Valve %d %s", idx+1, open?"OPEN":"CLOSE");
        ControlCmd cmd{ idx, open };
        xQueueSend(xCmdQueue, &cmd, 0);
        LOG_INFO("MQTT target %d set to %s - HMI state will be updated", idx+1, open?"OPEN":"CLOSE");
      } else {
        LOG_WARN("Invalid valve index: %d", idx+1);
      }
    } else {
      LOG_WARN("MQTT message missing valve or action field");
    }
  }
}

// Task MQTT
void TaskMQTT(void* pv) {
  LOG_INFO("Starting MQTT Task");
  WiFiManager wm;
  wm.autoConnect("Silagung", "admin123");
  espClient.setInsecure();
  mqtt.setServer(mqtt_broker, mqtt_port);
  mqtt.setBufferSize(4096);
  mqtt.setCallback(mqttCallback);
  LOG_INFO("WiFi connected, starting MQTT connection");

  while (!mqtt.connected()) {
    LOG_INFO("Attempting MQTT connection...");
      if (mqtt.connect("ESP32-RTOS", mqtt_user, mqtt_pass)) {
        mqtt.subscribe(topic_control);
        mqtt.subscribe(topic_irrigation_config);
        LOG_INFO("MQTT connected successfully");
        timeClient.begin();
        if (autoMode && WiFi.status() == WL_CONNECTED) {
          timeClient.forceUpdate();
          unsigned long epochTime = timeClient.getEpochTime();
          rtcInitialized = rtc.begin();
          if (rtcInitialized) {
            DateTime before = rtc.now();
            LOG_INFO("RTC before NTP: %02d:%02d:%02d %02d-%02d-%04d", before.hour(), before.minute(), before.second(), before.day(), before.month(), before.year());
            LOG_INFO("NTP epoch: %lu", epochTime);
            rtc.adjust(DateTime(epochTime));
            DateTime after = rtc.now();
            LOG_INFO("RTC after NTP: %02d:%02d:%02d %02d-%02d-%04d", after.hour(), after.minute(), after.second(), after.day(), after.month(), after.year());
            lastNTPSync = millis();
            LOG_INFO("RTC time updated from NTP on connect");
          }
        }
        break;
    }
    LOG_ERROR("MQTT connection failed, retrying in 2s");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  unsigned long lastPub = 0;
  for (;;) {
    if (!mqtt.connected()) {
      LOG_WARN("MQTT disconnected, reconnecting...");
      if (mqtt.connect("ESP32-RTOS", mqtt_user, mqtt_pass)) {
        mqtt.subscribe(topic_control);
        mqtt.subscribe(topic_irrigation_config);
        LOG_INFO("MQTT reconnected");
      }
    }
    mqtt.loop();

    if (millis() - lastPub >= 5000) {
      if (!currentIrrigation.isActive || currentIrrigation.activationReady) {
        if (!focusedLogging) LOG_INFO("Publishing sensor and valve data");
        publishSensorData();
        publishValveStatus();
      }
      lastPub = millis();
    }

    if (autoMode && millis() - lastTimeLog >= TIME_LOG_INTERVAL) {
      if (!rtcInitialized) rtcInitialized = rtc.begin();
      if (rtcInitialized) {
        DateTime now = rtc.now();
        if (!focusedLogging) LOG_INFO("RTC current time: %02d:%02d:%02d %02d-%02d-%04d", now.hour(), now.minute(), now.second(), now.day(), now.month(), now.year());
      }
      lastTimeLog = millis();
    }

    if (autoMode && WiFi.status() == WL_CONNECTED) {
      timeClient.update();
      if (millis() - lastNTPSync > NTP_SYNC_INTERVAL) {
        timeClient.forceUpdate();
        unsigned long epochTime = timeClient.getEpochTime();
        if (!rtcInitialized) rtcInitialized = rtc.begin();
        if (rtcInitialized) {
          DateTime before = rtc.now();
          if (!focusedLogging) LOG_INFO("RTC before NTP: %02d:%02d:%02d %02d-%02d-%04d", before.hour(), before.minute(), before.second(), before.day(), before.month(), before.year());
          if (!focusedLogging) LOG_INFO("NTP epoch: %lu", epochTime);
          rtc.adjust(DateTime(epochTime));
          DateTime after = rtc.now();
          if (!focusedLogging) LOG_INFO("RTC after NTP: %02d:%02d:%02d %02d-%02d-%04d", after.hour(), after.minute(), after.second(), after.day(), after.month(), after.year());
          lastNTPSync = millis();
          if (!focusedLogging) LOG_INFO("RTC time re-synced from NTP");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task Modbus
void TaskModbus(void* pv) {
  LOG_INFO("Starting Modbus Task");
  rs485.begin(BAUD_RATE, SERIAL_8N1, RS485_RX, RS485_TX);
  hmiNode.begin(HMI_ID, rs485);
  relayNode.begin(RELAY_ID, rs485);
  flowNode.begin(FLOW_ID, rs485);
  ecNode.begin(EC_ID, rs485);
  us1Node.begin(US1_ID, rs485);
  us2Node.begin(US2_ID, rs485);

  // Initialize pressure sensor ADC pin
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  LOG_INFO("Pressure sensor ADC initialized on pin %d", PRESSURE_SENSOR_PIN);

  // Init known valve coils OFF (0..9)
  for (int i = 0; i < 10; i++) relayNode.writeSingleCoil(i, 0);
  for (int i = 0; i < VALVE_COUNT; i++) hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
  hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, 0);
  LOG_INFO("Modbus initialized - all coils OFF");

  for (int i = 0; i < VALVE_COUNT; i++) hmiNode.writeSingleCoil(HMI_BTN_ADDR + i, 0);
  hmiNode.writeSingleCoil(HMI_BTN_ADDR + PUMP_BTN_INDEX, 0);
  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  for (int i = 0; i < VALVE_COUNT; i++) {
    valveState[i] = false;
    lastValveState[i] = false;
    webButtonChange[i] = false;
  }
  webButtonChange[VALVE_COUNT] = false; // pump
  xSemaphoreGive(xStateMutex);
  pumpState = false;
  for (int i = 0; i < VALVE_COUNT; i++) {
    controlRelayValve(i, false);
  }
  // Ensure pump relay (coil 11) is OFF at startup
  controlPumpRelay(false);
  publishValveStatus();
  LOG_INFO("Startup reset: all valves OFF, HMI buttons and lamps cleared");

  for (;;) {
    if (!currentIrrigation.isActive || currentIrrigation.activationReady) {
      bool flowSuccess = readWaterFlowSensor(flowNode, waterFlow, "Flow Sensor");
      delay(10);
      bool ecSuccess = readECSensor(ecNode, ec, "EC Sensor");
      delay(10);
      bool us1Success = readUltrasonicSensor(us1Node, us1, "Ultrasonic1 Sensor");
      delay(10);
      bool us2Success = readUltrasonicSensor(us2Node, us2, "Ultrasonic2 Sensor");
      delay(10);
      bool pressureSuccess = readPressureSensor(pressure, "Pressure Sensor");
      focusedLogging = currentIrrigation.isActive;
      writeUltrasonicToHMI();
      writeWaterFlowToHMI();
      writeECSensorToHMI();
      writePressureToHMI();
    }
    
    // Log successful sensor readings periodically
    static unsigned long lastSensorLog = 0;
    if (!currentIrrigation.isActive && millis() - lastSensorLog >= 10000) {
      LOG_INFO("Sensor readings - Flow:%.2f EC:%.0f US1:%u US2:%u Pressure:%.2f",
               waterFlow, ec, us1, us2, pressure);
      lastSensorLog = millis();
    }

    if (currentIrrigation.isActive) {
      static unsigned long lastFlowAccum = 0;
      static int lowFlowSeconds = 0;
      static int noProgressSeconds = 0;
      static float lastDelivered = 0.0f;
      if (millis() - lastFlowAccum >= 1000) {
        float waterInLastSec = (waterFlow / 60.0f);
        currentIrrigation.waterDelivered += waterInLastSec;
        lastFlowAccum = millis();
        bool isAirOnly = currentIrrigation.irrigationType.equalsIgnoreCase("air");
        bool isAirNutrisi = currentIrrigation.irrigationType.equalsIgnoreCase("air_nutrisi") || currentIrrigation.irrigationType.equalsIgnoreCase("air+nutrisi");
        int landIdx = currentIrrigation.landRelay - 1;
        bool expectV0 = isAirNutrisi;
        bool expectV1 = true;
        bool expectLand = true;
        bool configReady = pumpState && valveState[1] && (!expectV0 || valveState[0]) && valveState[landIdx];
        if ((millis() - currentIrrigation.startMillis) < ACTIVATION_SETTLE_MS) configReady = false;
        unsigned long elapsedSec = (millis() - currentIrrigation.startMillis) / 1000;
        int etaSec = -1;
        if (waterFlow > 0.001f) {
          float remaining = currentIrrigation.waterNeeded - currentIrrigation.waterDelivered;
          if (remaining < 0) remaining = 0;
          etaSec = (int)(remaining / (waterFlow / 60.0f));
        }
        LOG_INFO("Irrigation progress - delivered: %.2f/%.2fL flow:%.2fL/min lowFlow:%d/%u noProgress:%d/%u elapsed:%lus eta:%ds",
                 currentIrrigation.waterDelivered, currentIrrigation.waterNeeded, waterFlow, lowFlowSeconds, NO_FLOW_STALL_SECONDS, noProgressSeconds, NO_PROGRESS_STALL_SECONDS, elapsedSec, etaSec);
        if (!configReady) {
          LOG_WARN("Irrigation activation not ready: pump:%s v0:%s v1:%s land:%s",
                   pumpState?"ON":"OFF",
                   valveState[0]?"ON":"OFF",
                   valveState[1]?"ON":"OFF",
                   valveState[landIdx]?"ON":"OFF");
        }
        if (waterFlow < MIN_FLOW_THRESHOLD) {
          if (configReady) lowFlowSeconds++;
        } else {
          lowFlowSeconds = 0;
        }
        if ((currentIrrigation.waterDelivered - lastDelivered) < 0.01f) {
          if (configReady) noProgressSeconds++;
        } else {
          noProgressSeconds = 0;
          lastDelivered = currentIrrigation.waterDelivered;
        }
        static unsigned long lastPumpEnsure = 0;
        if (currentIrrigation.activationReady && millis() - lastPumpEnsure >= 1000) {
          uint8_t pr = relayNode.readCoils(PUMP_RELAY_COIL, 1);
          bool pon = false;
          if (pr == relayNode.ku8MBSuccess) {
            uint16_t w = relayNode.getResponseBuffer(0);
            pon = w & 0x01;
          }
          if (!pon) {
            bool sc = controlPumpRelay(true);
            if (!sc) {
              LOG_ERROR("Auto pump enforce failure");
            } else {
              LOG_INFO("Auto pump enforced ON");
            }
            publishValveStatus();
          }
          lastPumpEnsure = millis();
        }
        if (lowFlowSeconds >= (int)NO_FLOW_STALL_SECONDS || noProgressSeconds >= (int)NO_PROGRESS_STALL_SECONDS) {
          LOG_WARN("Irrigation stalled, advancing to next schedule if available");
          for (int i = 0; i < VALVE_COUNT; i++) {
            controlRelayValve(i, false);
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
          }
          controlPumpRelay(false);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, 0);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "fail_flow", currentIrrigation.configId, "schedule");
          int ci = currentIrrigation.configIndex;
          int si = currentIrrigation.scheduleIndex;
          if (ci >= 0 && si + 1 < configs[ci].scheduleCount) {
            IrrigationJob next; next.configIndex = ci; next.scheduleIndex = si + 1;
            xQueueSend(xIrrigationQueue, &next, 0);
          }
          lowFlowSeconds = 0;
          noProgressSeconds = 0;
          lastDelivered = 0.0f;
        }
        if (currentIrrigation.waterDelivered >= currentIrrigation.waterNeeded) {
          LOG_INFO("Target volume met, stopping irrigation");
          for (int i = 0; i < VALVE_COUNT; i++) {
            controlRelayValve(i, false);
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
          }
          controlPumpRelay(false);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, 0);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "success", currentIrrigation.configId, "schedule");
        }
        if ((millis() - currentIrrigation.startMillis) > 1800000) {
          LOG_WARN("Irrigation safety timeout, stopping");
          for (int i = 0; i < VALVE_COUNT; i++) {
            controlRelayValve(i, false);
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
          }
          controlPumpRelay(false);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, 0);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "timeout", currentIrrigation.configId, "schedule");
        }
      }
    }

    // Read HMI buttons
    uint8_t res = hmiNode.readCoils(HMI_BTN_ADDR, VALVE_COUNT + 2);
    if (res == hmiNode.ku8MBSuccess) {
      uint16_t word = hmiNode.getResponseBuffer(0);
      bool modeBit = (word >> MODE_SWITCH_INDEX) & 0x01;
      if (autoMode != modeBit) {
        autoMode = modeBit;
        LOG_INFO("Mode changed via HMI LB6: %s", autoMode?"AUTO":"MANUAL");
        for (int i = 0; i < VALVE_COUNT; i++) {
          controlRelayValve(i, false);
          vTaskDelay(pdMS_TO_TICKS(200));
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
        }
        controlPumpRelay(false);
        vTaskDelay(pdMS_TO_TICKS(200));
        hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, 0);
        publishValveStatus();
      }
      for (int i = 0; i < VALVE_COUNT; i++) {
        bool btn = (word >> i) & 0x01;
        if (btn != lastValveState[i]) {
          lastValveState[i] = btn;
          xSemaphoreTake(xStateMutex, portMAX_DELAY);
          valveState[i] = btn;
          xSemaphoreGive(xStateMutex);
          if (autoMode) {
            continue;
          }
          if (webButtonChange[i]) {
            bool success = controlRelayValve(i, btn);
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, btn ? 1 : 0);
            if (!success) {
              LOG_ERROR("WEB control relay failure valve:%d", i+1);
            }
            webButtonChange[i] = false;
            publishValveStatus();
            if (!focusedLogging) LOG_INFO("WEB control completed for valve %d", i+1);
          } else {
            bool success = controlRelayValve(i, btn);
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, btn ? 1 : 0);
            if (!success) {
              LOG_ERROR("HMI control relay failure valve:%d", i+1);
            }
            publishValveStatus();
            if (!focusedLogging) LOG_INFO("HMI change synced to web for valve %d", i+1);
          }
        }
      }
      // Pump button at LB5
      bool pumpBtn = (word >> PUMP_BTN_INDEX) & 0x01;
      if (pumpBtn != lastPumpBtnState) {
        lastPumpBtnState = pumpBtn;
        if (autoMode) {
          
        } else if (webButtonChange[VALVE_COUNT]) {
          bool success = controlPumpRelay(pumpBtn);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, pumpBtn ? 1 : 0);
          if (!success) {
            LOG_ERROR("WEB control pump relay failure");
          }
          webButtonChange[VALVE_COUNT] = false;
          publishValveStatus();
          if (!focusedLogging) LOG_INFO("WEB control completed for pump %s", pumpBtn?"ON":"OFF");
        } else {
          bool success = controlPumpRelay(pumpBtn);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, pumpBtn ? 1 : 0);
          if (!success) {
            LOG_ERROR("HMI control pump relay failure");
          }
          publishValveStatus();
          if (!focusedLogging) LOG_INFO("HMI change synced to web for pump %s", pumpBtn?"ON":"OFF");
        }
      }
    } else {
      LOG_WARN("HMI button read failure - error code: %u", res);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static int mapLandToValve(const String& landName) {
  String ln = landName;
  ln.toLowerCase();
  if (ln.indexOf("3") != -1 || ln.indexOf("tiga") != -1) return 5;
  if (ln.indexOf("2") != -1 || ln.indexOf("dua") != -1) return 4;
  return 3;
}

void TaskScheduler(void* pv) {
  LOG_INFO("Starting Scheduler Task");
  for (;;) {
    if (rtcInitialized && autoMode) {
      if (millis() - lastScheduleCheck >= SCHEDULE_CHECK_INTERVAL) {
        DateTime now = rtc.now();
        char hhmm[6];
        sprintf(hhmm, "%02d:%02d", now.hour(), now.minute());
        String currentTime = String(hhmm);
        for (int i = 0; i < totalConfigs; i++) {
          if (!configs[i].isValid) continue;
          for (int j = 0; j < configs[i].scheduleCount; j++) {
            if (!configs[i].schedules[j].isActive) continue;
            String sched = configs[i].schedules[j].time;
            String schedHHMM = sched.length() > 5 ? sched.substring(0,5) : sched;
            if (schedHHMM == currentTime) {
              IrrigationJob job; job.configIndex = i; job.scheduleIndex = j;
              xQueueSend(xIrrigationQueue, &job, 0);
              LOG_INFO("Enqueued irrigation for %s at %s", configs[i].landName.c_str(), currentTime.c_str());
            }
          }
        }
        lastScheduleCheck = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task MQTT Recv (real-time control)
void TaskMQTTRecv(void* pv) {
  LOG_INFO("Starting MQTT Receive Task");
  for (;;) {
    ControlCmd cmd;
    if (xQueueReceive(xCmdQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (autoMode) {
        continue;
      }
      if (cmd.idx < 0 || cmd.idx > VALVE_COUNT) {
        LOG_ERROR("Invalid WEB control index: %d", cmd.idx);
        continue;
      }
      LOG_INFO("Processing WEB control - Valve %d %s", cmd.idx+1, cmd.open?"OPEN":"CLOSE");
      webButtonChange[cmd.idx] = true;
      hmiNode.writeSingleCoil(HMI_BTN_ADDR + cmd.idx, cmd.open ? 1 : 0);
      LOG_INFO("WEB control - HMI button %d set to %s, waiting for TaskModbus to execute", cmd.idx+1, cmd.open?"ON":"OFF");
    }
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  LOG_INFO("System starting - FreeRTOS Irrigation Controller");

  xSensorQueue = xQueueCreate(8, sizeof(float[3]));
  xCmdQueue = xQueueCreate(8, sizeof(ControlCmd));
  xIrrigationQueue = xQueueCreate(8, sizeof(IrrigationJob));
  xStateMutex = xSemaphoreCreateMutex();

  Wire.begin(SDA_PIN, SCL_PIN);

  preferences.begin("irrigation", false);
  totalConfigs = preferences.getInt("totalConfigs", 0);
  for (int i = 0; i < totalConfigs && i < MAX_CONFIGS; i++) {
    String prefix = String("cfg") + String(i) + String("_");
    configs[i].isValid = preferences.getBool((prefix + "valid").c_str(), false);
    if (!configs[i].isValid) continue;
    configs[i].configId = preferences.getInt((prefix + "id").c_str(), 0);
    configs[i].landName = preferences.getString((prefix + "landName").c_str(), "");
    configs[i].phaseName = preferences.getString((prefix + "phaseName").c_str(), "");
    configs[i].waterRequirement = preferences.getFloat((prefix + "waterReq").c_str(), 0.0);
    configs[i].waterPerSchedule = preferences.getFloat((prefix + "wps").c_str(), 0.0);
    configs[i].targetEC = preferences.getFloat((prefix + "targetEC").c_str(), 0.0);
    configs[i].irrigationType = preferences.getString((prefix + "irrigType").c_str(), "");
    configs[i].scheduleCount = preferences.getInt((prefix + "schedCount").c_str(), 0);
    for (int j = 0; j < configs[i].scheduleCount && j < 10; j++) {
      configs[i].schedules[j].time = preferences.getString((prefix + String("time") + String(j)).c_str(), "");
      configs[i].schedules[j].isActive = preferences.getBool((prefix + String("active") + String(j)).c_str(), false);
    }
  }

  // Watchdog 10 detik
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  rtcInitialized = rtc.begin();

  LOG_INFO("Creating FreeRTOS tasks");
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 8192, NULL, 2, &xMQTTTask, 0);
  xTaskCreatePinnedToCore(TaskModbus, "Modbus", 8192, NULL, 3, &xModbusTask, 1);
  xTaskCreatePinnedToCore(TaskMQTTRecv, "MQTTRecv", 4096, NULL, 2, &xMQTTRecvTask, 0);
  xTaskCreatePinnedToCore(TaskScheduler, "Scheduler", 4096, NULL, 2, &xSchedulerTask, 1);
  xTaskCreatePinnedToCore([](void* pv){
    for(;;){
      IrrigationJob job;
      if (!currentIrrigation.isActive && xQueueReceive(xIrrigationQueue, &job, pdMS_TO_TICKS(200)) == pdTRUE) {
        int i = job.configIndex;
        if (i < 0 || i >= MAX_CONFIGS) continue;
        if (!configs[i].isValid) continue;
        currentIrrigation.isActive = true;
        currentIrrigation.activationReady = false;
        currentIrrigation.configId = configs[i].configId;
        currentIrrigation.landName = configs[i].landName;
        currentIrrigation.irrigationType = configs[i].irrigationType;
        currentIrrigation.waterNeeded = configs[i].waterPerSchedule;
        currentIrrigation.waterDelivered = 0;
        currentIrrigation.startMillis = millis();
        currentIrrigation.landRelay = mapLandToValve(configs[i].landName);
        currentIrrigation.configIndex = i;
        currentIrrigation.scheduleIndex = job.scheduleIndex;
        for (int k = 0; k < VALVE_COUNT; k++) {
          controlRelayValve(k, false);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        controlPumpRelay(false);
        vTaskDelay(pdMS_TO_TICKS(200));
        for (int k = 0; k < VALVE_COUNT; k++) {
          controlRelayValve(k, false);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        controlPumpRelay(false);
        vTaskDelay(pdMS_TO_TICKS(ACTIVATION_SETTLE_MS));
        bool isAirOnly = currentIrrigation.irrigationType.equalsIgnoreCase("air");
        bool isAirNutrisi = currentIrrigation.irrigationType.equalsIgnoreCase("air_nutrisi") || currentIrrigation.irrigationType.equalsIgnoreCase("air+nutrisi");
        bool allOk = true;
        if (isAirOnly) {
          bool ok0 = controlRelayValve(0, false);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 0, 0);
          vTaskDelay(pdMS_TO_TICKS(300));
          bool ok1 = controlRelayValve(1, true);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 1, 1);
          allOk = ok0 && ok1;
        } else if (isAirNutrisi) {
          bool ok0 = controlRelayValve(0, true);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 0, 1);
          vTaskDelay(pdMS_TO_TICKS(300));
          bool ok1 = controlRelayValve(1, true);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 1, 1);
          allOk = ok0 && ok1;
        } else {
          bool ok0 = controlRelayValve(0, false);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 0, 0);
          vTaskDelay(pdMS_TO_TICKS(300));
          bool ok1 = controlRelayValve(1, true);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + 1, 1);
          allOk = ok0 && ok1;
        }
        vTaskDelay(pdMS_TO_TICKS(300));
        for (int v = 3; v <= 5; v++) {
          int idx = v - 1;
          bool on = (v == currentIrrigation.landRelay);
          bool okv = controlRelayValve(idx, on);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + idx, on ? 1 : 0);
          vTaskDelay(pdMS_TO_TICKS(300));
          allOk = allOk && okv;
        }
        {
          int landIdx = currentIrrigation.landRelay - 1;
          bool okLand = controlRelayValve(landIdx, true);
          hmiNode.writeSingleCoil(HMI_LAMP_ADDR + landIdx, 1);
          vTaskDelay(pdMS_TO_TICKS(300));
          allOk = allOk && okLand;
        }
        if (allOk) {
          vTaskDelay(pdMS_TO_TICKS(300));
          controlPumpRelay(true);
          currentIrrigation.activationReady = true;
          LOG_INFO("Irrigation started for %s schedule:%d target:%.2fL", currentIrrigation.landName.c_str(), currentIrrigation.scheduleIndex+1, currentIrrigation.waterNeeded);
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "start", "success", currentIrrigation.configId, "schedule");
        } else {
          LOG_ERROR("Valve activation failed, aborting irrigation start");
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "start", "fail_config", currentIrrigation.configId, "schedule");
        }
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }, "IrrigationCtrl", 4096, NULL, 2, NULL, 1);
  
  LOG_INFO("System initialization complete");
}

void loop() {
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Background monitoring log (optional)
  static unsigned long lastBgLog = 0;
  if (millis() - lastBgLog > 60000) {
    if (!focusedLogging) LOG_INFO("Background check - System running normally");
    lastBgLog = millis();
  }
}
static void publishIrrigationAckBatch(int successCount, int totalCount) {
  StaticJsonDocument<256> doc;
  doc["type"] = "irrigation_config_ack";
  doc["mode"] = "batch";
  doc["success"] = successCount;
  doc["total"] = totalCount;
  doc["status"] = "stored";
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) mqtt.publish(topic_irrigation_ack, buffer, true);
}

static void publishIrrigationAckSingle(const IrrigationConfigItem& item) {
  StaticJsonDocument<256> doc;
  doc["type"] = "irrigation_config_ack";
  doc["mode"] = "single";
  doc["configId"] = item.configId;
  doc["landName"] = item.landName;
  doc["status"] = "stored";
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) mqtt.publish(topic_irrigation_ack, buffer, true);
}
