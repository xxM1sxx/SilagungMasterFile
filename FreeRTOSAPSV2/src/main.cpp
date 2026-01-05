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

static inline void lockModbus();
static inline void unlockModbus();
static bool writeHmiCoilWithRetry(uint16_t coil, bool state);
static bool writeHmiRegisterWithRetry(uint16_t reg, uint16_t value);

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
const char* topic_irrigation_log = "silagung/irrigation/log";

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
#define VFD_ID 7

// Valve & HMI
#define VALVE_COUNT 5
#define HMI_BTN_ADDR 0
#define HMI_LAMP_ADDR 10
#define PUMP_BTN_INDEX 5
#define PUMP_LAMP_INDEX 5
#define MODE_SWITCH_INDEX 6
#define WATER_SUPPLY_VALVE_IDX 0
#define NUTRI_SUPPLY_VALVE_IDX 1
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

static constexpr float VFD_MAX_ALLOWED_HZ = 50.0f;
static constexpr uint16_t VFD_REG_CONTROL_COMMAND = 0x2000;
static constexpr uint16_t VFD_REG_COMM_SET_FREQUENCY = 0x2001;
static constexpr uint16_t VFD_REG_RUN_STATUS = 0x2101;
static constexpr uint16_t VFD_CMD_STOP = 0x0001;
static constexpr uint16_t VFD_CMD_RUN_FWD = 0x0012;

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
  esp_task_wdt_reset();
  lockModbus();
  uint8_t res = node.readHoldingRegisters(addr, 2);
  esp_task_wdt_reset();
  if (res != node.ku8MBSuccess) { unlockModbus(); return false; }
  uint16_t w0 = node.getResponseBuffer(0);
  uint16_t w1 = node.getResponseBuffer(1);
  unlockModbus();
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
SemaphoreHandle_t xModbusMutex;
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
bool lastPumpLampState = true;
bool autoMode = false;
uint16_t lastHmiVfdFreqRaw = 0xFFFF;
unsigned long lastHmiVfdFreqPoll = 0;
float vfdFrequencyHz = 0.0f;

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
ModbusMaster hmiNode, relayNode, flowNode, ecNode, us1Node, us2Node, vfdNode;
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
const unsigned int NO_PROGRESS_STALL_SECONDS = 5;
const float EC_CONTROL_HYSTERESIS_US = 50.0f;
const unsigned long EC_CONTROL_INTERVAL_MS = 1500;
const unsigned long EC_CONTROL_MIN_SWITCH_MS = 3000;
unsigned long NUTRI_MOTOR_DRIVE_MS = 5000;
unsigned long NUTRI_MOTOR_REDRIVE_INTERVAL_MS = 15000;
const uint8_t NUTRI_MOTOR_REDRIVE_MAX = 3;
#define ACTIVATION_SETTLE_MS 1500
#define PUMP_RELAY_COIL 10
unsigned long HMI_POLL_INTERVAL_AUTO_MS = 500;
unsigned long HMI_POLL_INTERVAL_MANUAL_MS = 5;
unsigned long MODBUS_RETRY_DELAY_MS = 1;
int MODBUS_WRITE_RETRY_COUNT = 2;
unsigned long RELAY_CONTROL_PREPARE_MS = 2;
unsigned long RELAY_CONTROL_SETTLE_MS = 2;
unsigned long RELAY_VERIFY_DELAY_MS = 2;
unsigned long PUMP_VERIFY_DELAY_MS = 2;
unsigned long SENSOR_POLL_GAP_MS = 1;
unsigned long ULTRASONIC_READ_RETRY_GAP_MS = 15; //15 maksimal
unsigned long HMI_MODE_CHANGE_RELAY_OFF_DELAY_MS = 1;
unsigned long HMI_WRITE_DELAY_MS = 2;
unsigned long EC_RETRY_DELAY_MS = 15; //15 maksimal
unsigned long PRESSURE_ADC_DELAY_US = 1000;
unsigned long IRRIGATION_STEP_DELAY_MS = 200;
unsigned long IRRIGATION_VALVE_DELAY_MS = 300;
unsigned long lastHMIProcess = 0;
static const uint8_t READ_RETRY = 3;
static const unsigned long SENSOR_POLL_INTERVAL_MANUAL_IDLE_MS = 200;
static const unsigned long RS485_FLUSH_MAX_MS = 20;

unsigned long MODBUS_RETRY_DELAY_AUTO_MS = 180;
int MODBUS_WRITE_RETRY_COUNT_AUTO = 10;
unsigned long RELAY_CONTROL_SETTLE_AUTO_MS = 150;
unsigned long RELAY_VERIFY_DELAY_AUTO_MS = 150;
unsigned long SENSOR_POLL_GAP_AUTO_MS = 30;
unsigned long ULTRASONIC_READ_RETRY_GAP_AUTO_MS = 120;
unsigned long HMI_WRITE_DELAY_AUTO_MS = 120;
unsigned long EC_RETRY_DELAY_AUTO_MS = 200;
unsigned long IRRIGATION_STEP_DELAY_AUTO_MS = 500;
unsigned long IRRIGATION_VALVE_DELAY_AUTO_MS = 800;
unsigned long PUMP_START_CONFIRM_DELAY_MS = 800;

static inline bool isAutoSensitiveTiming();
static inline unsigned long modbusRetryDelayMs();
static inline int modbusWriteRetryCount();
static inline unsigned long relayControlSettleMs();
static inline unsigned long relayVerifyDelayMs();
static inline unsigned long sensorPollGapMs();
static inline unsigned long ultrasonicRetryGapMs();
static inline unsigned long hmiWriteDelayMs();
static inline unsigned long ecRetryDelayMs();
static inline unsigned long irrigationStepDelayMs();
static inline unsigned long irrigationValveDelayMs();

    
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
  float targetEC;
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
const unsigned long SCHEDULE_CHECK_INTERVAL = 5000;
int32_t lastScheduleEnqueueStamp[MAX_CONFIGS][10];

static inline bool isAutoSensitiveTiming() { return autoMode || currentIrrigation.isActive; }
static inline unsigned long modbusRetryDelayMs() { return isAutoSensitiveTiming() ? MODBUS_RETRY_DELAY_AUTO_MS : MODBUS_RETRY_DELAY_MS; }
static inline int modbusWriteRetryCount() { return isAutoSensitiveTiming() ? MODBUS_WRITE_RETRY_COUNT_AUTO : MODBUS_WRITE_RETRY_COUNT; }
static inline unsigned long relayControlSettleMs() { return isAutoSensitiveTiming() ? RELAY_CONTROL_SETTLE_AUTO_MS : RELAY_CONTROL_SETTLE_MS; }
static inline unsigned long relayVerifyDelayMs() { return isAutoSensitiveTiming() ? RELAY_VERIFY_DELAY_AUTO_MS : RELAY_VERIFY_DELAY_MS; }
static inline unsigned long sensorPollGapMs() { return isAutoSensitiveTiming() ? SENSOR_POLL_GAP_AUTO_MS : SENSOR_POLL_GAP_MS; }
static inline unsigned long ultrasonicRetryGapMs() { return isAutoSensitiveTiming() ? ULTRASONIC_READ_RETRY_GAP_AUTO_MS : ULTRASONIC_READ_RETRY_GAP_MS; }
static inline unsigned long hmiWriteDelayMs() { return isAutoSensitiveTiming() ? HMI_WRITE_DELAY_AUTO_MS : HMI_WRITE_DELAY_MS; }
static inline unsigned long ecRetryDelayMs() { return isAutoSensitiveTiming() ? EC_RETRY_DELAY_AUTO_MS : EC_RETRY_DELAY_MS; }
static inline unsigned long irrigationStepDelayMs() { return isAutoSensitiveTiming() ? IRRIGATION_STEP_DELAY_AUTO_MS : IRRIGATION_STEP_DELAY_MS; }
static inline unsigned long irrigationValveDelayMs() { return isAutoSensitiveTiming() ? IRRIGATION_VALVE_DELAY_AUTO_MS : IRRIGATION_VALVE_DELAY_MS; }

static bool isEpochPlausible(unsigned long epochTime) {
  return epochTime >= 1609459200UL && epochTime <= 1893456000UL;
}

static bool parseScheduleMinute(const String& s, int& minuteOfDay) {
  int hh = -1, mm = -1;
  if (sscanf(s.c_str(), "%d:%d", &hh, &mm) == 2) {
  } else if (sscanf(s.c_str(), "%d.%d", &hh, &mm) == 2) {
  } else {
    return false;
  }
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return false;
  minuteOfDay = (hh * 60) + mm;
  return true;
}

static bool minuteInWindow(int startExclusive, int endInclusive, int target) {
  if (startExclusive == endInclusive) return target == endInclusive;
  if (startExclusive < endInclusive) return target > startExclusive && target <= endInclusive;
  return target > startExclusive || target <= endInclusive;
}

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

static void publishIrrigationLog(const ActiveIrrigationState& state,
                                 const IrrigationConfigItem* config,
                                 const IrrigationScheduleItem* sched,
                                 const char* resultStatus,
                                 float waterDelivered) {
  StaticJsonDocument<256> doc;
  doc["landName"] = state.landName;
  if (sched) {
    doc["scheduleTime"] = sched->time;
  }
  doc["irrigationType"] = state.irrigationType;
  float targetEcVal = state.targetEC;
  float waterPlanVal = state.waterNeeded;
  if (config) {
    targetEcVal = config->targetEC;
    waterPlanVal = config->waterPerSchedule;
  }
  doc["targetEC"] = targetEcVal;
  doc["waterPlan"] = waterPlanVal;
  doc["waterDelivered"] = waterDelivered;
  doc["result"] = resultStatus;
  if (rtcInitialized) {
    DateTime now = rtc.now();
    char hhmmss[9];
    sprintf(hhmmss, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    doc["finishTime"] = hhmmss;
  }
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) {
    mqtt.publish(topic_irrigation_log, buffer);
  }
}

// ===== SENSOR READING FUNCTIONS =====

// Function to read ultrasonic sensor with error handling (Prioritize 0x0100, then 0x0101, fallback 0x0001)
bool readUltrasonicSensor(ModbusMaster& node, uint16_t& distance, const char* sensorName, int errorIndex) {
    // Candidates: 0x0100 (Std), 0x0101 (Alt), 0x0001 (User fallback)
    const uint16_t candidates[] = {0x0100};
    uint8_t result = node.ku8MBResponseTimedOut;

    for (uint16_t reg : candidates) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(ultrasonicRetryGapMs()));
        lockModbus();
        node.clearResponseBuffer();
        result = node.readHoldingRegisters(reg, 1);
        esp_task_wdt_reset();
        
        if (result == node.ku8MBSuccess) {
            uint16_t raw = node.getResponseBuffer(0);
            unlockModbus();
            // Convert to cm (assuming raw is mm)
            uint16_t valCm = raw / 10;
            
            if (valCm >= 5 && valCm <= 750) {
                distance = valCm;
                consecutiveSensorErrors[errorIndex] = 0;
                return true;
            }
        } else {
            unlockModbus();
        }
    }
    
    // Increment consecutive error counter
    consecutiveSensorErrors[errorIndex]++;
    
    // Only log error if we've exceeded the threshold
    if (consecutiveSensorErrors[errorIndex] >= MAX_CONSECUTIVE_ERRORS) {
        if (millis() - lastSensorFail[errorIndex] > SENSOR_FAIL_LOG_INTERVAL) {
            LOG_ERROR("%s read failure - last error: 0x%02X", sensorName, result);
            lastSensorFail[errorIndex] = millis();
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
static bool writeHmiRegisterWithRetry(uint16_t reg, uint16_t value);

void writeUltrasonicToHMI() {
    int nutrPct = (int)((NUTRI_TANK_EMPTY_CM - us1) * 100 / (NUTRI_TANK_EMPTY_CM - NUTRI_TANK_FULL_CM));
    if (nutrPct < 0) nutrPct = 0;
    if (nutrPct > 100) nutrPct = 100;
    writeHmiRegisterWithRetry(0x0000, (uint16_t)nutrPct);
    vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
    writeHmiRegisterWithRetry(0x0005, (uint16_t)nutrPct);
    vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
    int airPct = (int)((WATER_TANK_EMPTY_CM - us2) * 100 / (WATER_TANK_EMPTY_CM - WATER_TANK_FULL_CM));
    if (airPct < 0) airPct = 0;
    if (airPct > 100) airPct = 100;
    writeHmiRegisterWithRetry(0x0001, (uint16_t)airPct);
    vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
    writeHmiRegisterWithRetry(0x0006, (uint16_t)airPct);
}

void writeWaterFlowToHMI() {
    // Write water flow to HMI register LW3 as integer with 2 decimal places (x100)
    uint16_t wf = (uint16_t)(waterFlow * 100); // L/min with 2 decimals
    writeHmiRegisterWithRetry(0x0003, wf); // LW3 - Water Flow
    vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
}

void writeECSensorToHMI() {
    // Write EC sensor to HMI register LW4
    uint16_t ecVal = (uint16_t)ec;
    writeHmiRegisterWithRetry(0x0004, ecVal); // LW4 - EC
    vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
}

void writePressureToHMI() {
    // Write pressure to HMI register LW2 as integer with 2 decimal places (x100)
    uint16_t p = (uint16_t)(pressure * 100); // Bar with 2 decimals
    writeHmiRegisterWithRetry(0x0002, p); // LW2 - Pressure
}

static inline void lockModbus() { xSemaphoreTake(xModbusMutex, portMAX_DELAY); }
static inline void unlockModbus() { xSemaphoreGive(xModbusMutex); }

bool writeCoilWithRetry(uint16_t coil, bool state) {
  uint8_t lastErr = 0;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    uint8_t r = relayNode.writeSingleCoil(coil, state ? 1 : 0);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == relayNode.ku8MBSuccess) return true;
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs() * (unsigned long)(attempt + 1)));
  }
  LOG_ERROR("Relay write failure coil:%u state:%u code:%u", coil, state?1:0, lastErr);
  return false;
}

static bool writeMultipleCoilsWithRetry(ModbusMaster& node, const char* tag, uint16_t startCoil, const bool* states, uint16_t qty, bool postDelay, unsigned long postDelayMs) {
  uint8_t lastErr = 0;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    node.clearTransmitBuffer();
    node.beginTransmission(startCoil);
    for (uint16_t i = 0; i < qty; i++) {
      node.sendBit(states[i]);
    }
    uint8_t r = node.writeMultipleCoils(startCoil, qty);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == node.ku8MBSuccess) {
      if (postDelay && postDelayMs > 0) vTaskDelay(pdMS_TO_TICKS(postDelayMs));
      return true;
    }
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs() * (unsigned long)(attempt + 1)));
  }
  LOG_ERROR("%s multi-coil write failure start:%u qty:%u code:%u", tag, startCoil, qty, lastErr);
  return false;
}

static bool writeRelayCoilsBatchWithRetry(uint16_t startCoil, const bool* states, uint16_t qty) {
  return writeMultipleCoilsWithRetry(relayNode, "Relay", startCoil, states, qty, true, relayControlSettleMs());
}

static bool writeHmiCoilsBatchWithRetry(uint16_t startCoil, const bool* states, uint16_t qty) {
  return writeMultipleCoilsWithRetry(hmiNode, "HMI", startCoil, states, qty, true, hmiWriteDelayMs());
}

static bool writeHmiCoilWithRetry(uint16_t coil, bool state) {
  uint8_t lastErr = 0;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    uint8_t r = hmiNode.writeSingleCoil(coil, state ? 1 : 0);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == hmiNode.ku8MBSuccess) {
      vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
      return true;
    }
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs() * (unsigned long)(attempt + 1)));
  }
  LOG_ERROR("HMI coil write failure coil:%u state:%u code:%u", coil, state ? 1 : 0, lastErr);
  return false;
}

static bool writeHmiRegisterWithRetry(uint16_t reg, uint16_t value) {
  uint8_t lastErr = 0xFF;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    uint8_t r = hmiNode.writeSingleRegister(reg, value);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == hmiNode.ku8MBSuccess) {
      vTaskDelay(pdMS_TO_TICKS(hmiWriteDelayMs()));
      return true;
    }
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs() * (unsigned long)(attempt + 1)));
  }
  LOG_ERROR("HMI register write failure reg:%u value:%u code:%u", reg, value, lastErr);
  return false;
}

static void setPumpLamp(bool on) {
  if (lastPumpLampState == on) return;
  bool ok = writeHmiCoilWithRetry(HMI_LAMP_ADDR + PUMP_LAMP_INDEX, on);
  if (ok) lastPumpLampState = on;
}

static bool controlRelayValvesBatch(const bool desiredOpen[VALVE_COUNT]) {
  bool coilBits[VALVE_COUNT * 2];
  for (int idx = 0; idx < VALVE_COUNT; idx++) {
    coilBits[idx * 2] = desiredOpen[idx];
    coilBits[idx * 2 + 1] = !desiredOpen[idx];
  }
  bool ok = writeRelayCoilsBatchWithRetry(0, coilBits, VALVE_COUNT * 2);
  if (!ok) return false;

  if (isAutoSensitiveTiming()) {
    lockModbus();
    uint8_t res = relayNode.readCoils(0, VALVE_COUNT * 2);
    if (res == relayNode.ku8MBSuccess) {
      uint16_t word = relayNode.getResponseBuffer(0);
      unlockModbus();
      for (int idx = 0; idx < VALVE_COUNT; idx++) {
        bool o = (word >> (idx * 2)) & 0x01;
        bool c = (word >> (idx * 2 + 1)) & 0x01;
        if (o && c) {
          LOG_ERROR("Relay conflict valve:%d", idx + 1);
          return false;
        }
      }
    } else {
      unlockModbus();
      LOG_WARN("Relay read failure (batch) code:%u", res);
    }
  }

  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  for (int i = 0; i < VALVE_COUNT; i++) {
    valveState[i] = desiredOpen[i];
  }
  xSemaphoreGive(xStateMutex);
  return true;
}

static bool controlRelayValveForceVerify(int idx, bool open) {
  int openCoil = idx * 2;
  int closeCoil = openCoil + 1;
  for (int attempt = 0; attempt < 2; attempt++) {
    if (open) {
      bool a = writeCoilWithRetry(closeCoil, false);
      vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
      bool b = writeCoilWithRetry(openCoil, true);
      vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
      if (!(a && b)) continue;
    } else {
      bool a = writeCoilWithRetry(openCoil, false);
      vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
      bool b = writeCoilWithRetry(closeCoil, true);
      vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
      if (!(a && b)) continue;
    }

    uint16_t word = 0;
    uint8_t res = 0xFF;
    lockModbus();
    res = relayNode.readCoils(openCoil, 2);
    if (res == relayNode.ku8MBSuccess) {
      word = relayNode.getResponseBuffer(0);
    }
    unlockModbus();
    if (res != relayNode.ku8MBSuccess) continue;

    bool o = (word & 0x01) != 0;
    bool c = (word & 0x02) != 0;
    if (o && c) {
      writeCoilWithRetry(openCoil, false);
      writeCoilWithRetry(closeCoil, false);
      return false;
    }
    if (open && o && !c) {
      xSemaphoreTake(xStateMutex, portMAX_DELAY);
      valveState[idx] = true;
      xSemaphoreGive(xStateMutex);
      return true;
    }
    if (!open && c && !o) {
      xSemaphoreTake(xStateMutex, portMAX_DELAY);
      valveState[idx] = false;
      xSemaphoreGive(xStateMutex);
      return true;
    }
  }
  return false;
}

static bool setNutrisiRelaysOff() {
  const int idx = NUTRI_SUPPLY_VALVE_IDX;
  const int openCoil = idx * 2;
  const int closeCoil = openCoil + 1;
  bool a = writeCoilWithRetry(openCoil, false);
  bool b = writeCoilWithRetry(closeCoil, false);
  return a && b;
}

static bool driveNutrisiMotorTo(bool open) {
  const int idx = NUTRI_SUPPLY_VALVE_IDX;
  const int openCoil = idx * 2;
  const int closeCoil = openCoil + 1;

  bool ok = false;
  if (open) {
    bool a = writeCoilWithRetry(closeCoil, false);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    bool b = writeCoilWithRetry(openCoil, true);
    ok = a && b;
  } else {
    bool a = writeCoilWithRetry(openCoil, false);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    bool b = writeCoilWithRetry(closeCoil, true);
    ok = a && b;
  }
  if (!ok) {
    setNutrisiRelaysOff();
    return false;
  }

  unsigned long driveMs = NUTRI_MOTOR_DRIVE_MS;
  if (driveMs > 0) {
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(driveMs));
    esp_task_wdt_reset();
  }

  return setNutrisiRelaysOff();
}

static bool applyRelayStateSafeBatch(const bool desiredOpen[VALVE_COUNT]) {
  bool allOff[VALVE_COUNT * 2];
  for (int i = 0; i < VALVE_COUNT * 2; i++) allOff[i] = false;
  if (!writeRelayCoilsBatchWithRetry(0, allOff, VALVE_COUNT * 2)) return false;

  bool finalBits[VALVE_COUNT * 2];
  for (int idx = 0; idx < VALVE_COUNT; idx++) {
    finalBits[idx * 2] = desiredOpen[idx];
    finalBits[idx * 2 + 1] = !desiredOpen[idx];
  }
  if (!writeRelayCoilsBatchWithRetry(0, finalBits, VALVE_COUNT * 2)) return false;

  bool verified = true;
  uint16_t word = 0;
  uint8_t res = 0xFF;
  lockModbus();
  res = relayNode.readCoils(0, VALVE_COUNT * 2);
  if (res == relayNode.ku8MBSuccess) {
    word = relayNode.getResponseBuffer(0);
  }
  unlockModbus();
  if (res != relayNode.ku8MBSuccess) {
    verified = false;
  } else {
    for (int idx = 0; idx < VALVE_COUNT; idx++) {
      bool o = (word >> (idx * 2)) & 0x01;
      bool c = (word >> (idx * 2 + 1)) & 0x01;
      if (o && c) { verified = false; break; }
      if (o != desiredOpen[idx]) { verified = false; break; }
      if (c != (!desiredOpen[idx])) { verified = false; break; }
    }
  }

  if (!verified) {
    bool allOk = true;
    for (int idx = 0; idx < VALVE_COUNT; idx++) {
      bool ok = controlRelayValveForceVerify(idx, desiredOpen[idx]);
      allOk = allOk && ok;
    }
    if (!allOk) return false;
  }

  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  for (int i = 0; i < VALVE_COUNT; i++) {
    valveState[i] = desiredOpen[i];
  }
  xSemaphoreGive(xStateMutex);
  return true;
}

static void setHmiValveLampsBatch(const bool lamps[VALVE_COUNT]) {
  writeHmiCoilsBatchWithRetry(HMI_LAMP_ADDR, lamps, VALVE_COUNT);
}

static void clearHmiValveButtonsBatch() {
  bool bits[VALVE_COUNT + 1];
  for (int i = 0; i < VALVE_COUNT + 1; i++) bits[i] = false;
  writeHmiCoilsBatchWithRetry(HMI_BTN_ADDR, bits, VALVE_COUNT + 1);
}

static void stopAllValvesAndResetHmiBatch() {
  bool desired[VALVE_COUNT];
  bool lamps[VALVE_COUNT];
  for (int i = 0; i < VALVE_COUNT; i++) { desired[i] = false; lamps[i] = false; }
  applyRelayStateSafeBatch(desired);
  if (NUTRI_MOTOR_DRIVE_MS > 0) vTaskDelay(pdMS_TO_TICKS(NUTRI_MOTOR_DRIVE_MS));
  setNutrisiRelaysOff();
  setHmiValveLampsBatch(lamps);
  clearHmiValveButtonsBatch();
}

static void setAllValvesOffFast() {
  bool desired[VALVE_COUNT];
  for (int i = 0; i < VALVE_COUNT; i++) desired[i] = false;
  applyRelayStateSafeBatch(desired);
}

bool controlRelayValve(int idx, bool open) {
  int openCoil = idx * 2;
  int closeCoil = openCoil + 1;
  bool ok = false;
  if (open) {
    bool a = writeCoilWithRetry(closeCoil, false);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    bool b = writeCoilWithRetry(openCoil, true);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    ok = a && b;
  } else {
    bool a = writeCoilWithRetry(openCoil, false);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    bool b = writeCoilWithRetry(closeCoil, true);
    vTaskDelay(pdMS_TO_TICKS(relayControlSettleMs()));
    ok = a && b;
  }
  if (autoMode) {
    lockModbus();
    uint8_t res = relayNode.readCoils(openCoil, 2);
    if (res == relayNode.ku8MBSuccess) {
      uint16_t word = relayNode.getResponseBuffer(0);
      unlockModbus();
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
        vTaskDelay(pdMS_TO_TICKS(relayVerifyDelayMs()));
        LOG_WARN("Relay verify open mismatch valve:%d", idx+1);
      }
      if (!open && (!c || o)) {
        writeCoilWithRetry(closeCoil, true);
        vTaskDelay(pdMS_TO_TICKS(relayVerifyDelayMs()));
        LOG_WARN("Relay verify close mismatch valve:%d", idx+1);
      }
    } else {
      unlockModbus();
      LOG_WARN("Relay read failure valve:%d code:%u", idx+1, res);
    }
  }
  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  valveState[idx] = open;
  xSemaphoreGive(xStateMutex);
  return ok;
}

bool controlPumpRelay(bool on) {
  uint8_t lastErr = 0xFF;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    uint8_t r = vfdNode.writeSingleRegister(VFD_REG_CONTROL_COMMAND, on ? VFD_CMD_RUN_FWD : VFD_CMD_STOP);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == vfdNode.ku8MBSuccess) {
      pumpState = on;
      setPumpLamp(on);
      return true;
    }
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs()));
  }
  LOG_ERROR("VFD command failure reg:%u cmd:0x%04X code:%u", VFD_REG_CONTROL_COMMAND, on ? VFD_CMD_RUN_FWD : VFD_CMD_STOP, lastErr);
  return false;
}

static bool vfdSetFrequencyHz(float hz) {
  float safeHz = hz;
  if (safeHz < 0.0f) safeHz = 0.0f;
  if (safeHz > VFD_MAX_ALLOWED_HZ) safeHz = VFD_MAX_ALLOWED_HZ;

  float pct = (safeHz / VFD_MAX_ALLOWED_HZ) * 100.0f;
  int32_t rawPct = (int32_t)lroundf(pct * 100.0f);
  if (rawPct < 0) rawPct = 0;
  if (rawPct > 10000) rawPct = 10000;

  uint8_t lastErr = 0xFF;
  int tries = modbusWriteRetryCount();
  for (int attempt = 0; attempt < tries; attempt++) {
    esp_task_wdt_reset();
    lockModbus();
    uint8_t r = vfdNode.writeSingleRegister(VFD_REG_COMM_SET_FREQUENCY, (uint16_t)rawPct);
    unlockModbus();
    esp_task_wdt_reset();
    if (r == vfdNode.ku8MBSuccess) return true;
    lastErr = r;
    vTaskDelay(pdMS_TO_TICKS(modbusRetryDelayMs()));
  }
  LOG_ERROR("VFD set frequency failure reg:%u hz:%.2f code:%u", VFD_REG_COMM_SET_FREQUENCY, safeHz, lastErr);
  return false;
}

static bool vfdIsRunning() {
  esp_task_wdt_reset();
  lockModbus();
  uint8_t res = vfdNode.readHoldingRegisters(VFD_REG_RUN_STATUS, 1);
  esp_task_wdt_reset();
  if (res != vfdNode.ku8MBSuccess) { unlockModbus(); return false; }
  uint16_t runStatus = vfdNode.getResponseBuffer(0);
  unlockModbus();
  return (runStatus & 0x0001) != 0;
}

static bool vfdReadIsRunning(bool& running) {
  esp_task_wdt_reset();
  lockModbus();
  uint8_t res = vfdNode.readHoldingRegisters(VFD_REG_RUN_STATUS, 1);
  esp_task_wdt_reset();
  if (res != vfdNode.ku8MBSuccess) { unlockModbus(); return false; }
  uint16_t runStatus = vfdNode.getResponseBuffer(0);
  unlockModbus();
  running = (runStatus & 0x0001) != 0;
  return true;
}

// Function to read EC sensor with error handling
bool readECSensor(ModbusMaster& node, float& ecValue, const char* sensorName) {
    uint8_t rc = 0xFF;

    for (uint8_t i = 0; i < READ_RETRY; i++) {
        esp_task_wdt_reset();
        lockModbus();
        unsigned long flushStart = millis();
        while (rs485.available() && (millis() - flushStart < RS485_FLUSH_MAX_MS)) {
            rs485.read();
        }

        rc = node.readHoldingRegisters(0x0002, 1);  // REG_EC = 0x0002
        esp_task_wdt_reset();
        if (rc == node.ku8MBSuccess) {
            ecValue = node.getResponseBuffer(0);     // nilai uint16, unit µS/cm
            unlockModbus();
            consecutiveSensorErrors[1] = 0;
            return true;
        }
        unlockModbus();
        vTaskDelay(pdMS_TO_TICKS(ecRetryDelayMs()));
    }

    // Gagal setelah retry
    consecutiveSensorErrors[1]++;
    if (consecutiveSensorErrors[1] >= MAX_CONSECUTIVE_ERRORS) {
        if (millis() - lastSensorFail[1] > SENSOR_FAIL_LOG_INTERVAL) {
            LOG_ERROR("%s read failure - error code: %u", sensorName, rc); // 226 = ResponseTimedOut
            lastSensorFail[1] = millis();
        }
    }
    ecValue = 0.0f;
    return false;
}

// Function to read pressure sensor (4-20mA ADC) with EMA filtering
bool readPressureSensor(float& pressureValue, const char* sensorName) {
    esp_task_wdt_reset();
    static float filteredCurrent = 0.0; // EMA filtered current value
    static bool firstReading = true;
    
    unsigned long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(PRESSURE_SENSOR_PIN);
        delayMicroseconds(PRESSURE_ADC_DELAY_US);
    }
    esp_task_wdt_reset();
    
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
        consecutiveSensorErrors[4] = 0; // Reset error counter on success
        return true;
    } else {
        consecutiveSensorErrors[4]++;
        
        // Only log error if we've exceeded the threshold (to ignore intermittent failures)
        if (consecutiveSensorErrors[4] >= MAX_CONSECUTIVE_ERRORS) {
            if (millis() - lastSensorFail[4] > SENSOR_FAIL_LOG_INTERVAL) {
                LOG_ERROR("%s current out of range: %.2f mA (expected 4-20mA), filtered: %.2f mA", sensorName, currentmA, filteredCurrent);
                lastSensorFail[4] = millis();
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
  StaticJsonDocument<320> doc;
  doc["valve1"] = valveState[0] ? "open" : "close";
  doc["valve2"] = valveState[1] ? "open" : "close";
  doc["valve3"] = valveState[2] ? "open" : "close";
  doc["valve4"] = valveState[3] ? "open" : "close";
  doc["valve5"] = valveState[4] ? "open" : "close";
  doc["pump"] = pumpState ? "on" : "off";
  doc["mode"] = autoMode ? "auto" : "manual";
  doc["vfdHz"] = vfdFrequencyHz;
  char buffer[320];
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
static bool writeHmiCoilWithRetry(uint16_t coil, bool state);
static bool writeHmiRegisterWithRetry(uint16_t reg, uint16_t value);
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
        item.irrigationType.trim();
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
          for (int k = 0; k < totalConfigs && k < MAX_CONFIGS; k++) {
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
      item.irrigationType.trim();
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
        for (int k = 0; k < totalConfigs && k < MAX_CONFIGS; k++) {
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
      writeHmiCoilWithRetry(HMI_BTN_ADDR + MODE_SWITCH_INDEX, wantAuto);
      LOG_INFO("Received mode command: %s -> HMI LB6 set %d", m.c_str(), wantAuto?1:0);
      return;
    }
    if (doc.containsKey("vfdHz") || doc.containsKey("vfd_hz") || doc.containsKey("vfdFrequency") || doc.containsKey("frequency") || doc.containsKey("pumpFrequency")) {
      if (autoMode) {
        LOG_WARN("Ignoring VFD frequency command in AUTO mode");
        return;
      }
      float hz = 0.0f;
      if (doc.containsKey("vfdHz")) hz = doc["vfdHz"].as<float>();
      else if (doc.containsKey("vfd_hz")) hz = doc["vfd_hz"].as<float>();
      else if (doc.containsKey("vfdFrequency")) hz = doc["vfdFrequency"].as<float>();
      else if (doc.containsKey("pumpFrequency")) hz = doc["pumpFrequency"].as<float>();
      else hz = doc["frequency"].as<float>();

      if (!isfinite(hz)) {
        LOG_WARN("Invalid VFD frequency value");
        return;
      }
      if (hz < 0.0f) hz = 0.0f;
      if (hz > VFD_MAX_ALLOWED_HZ) hz = VFD_MAX_ALLOWED_HZ;

      float roundedHz = roundf(hz);
      uint16_t raw = 0;
      if (fabsf(hz - roundedHz) < 0.001f && roundedHz <= VFD_MAX_ALLOWED_HZ) raw = (uint16_t)roundedHz;
      else raw = (uint16_t)roundf(hz * 100.0f);

      bool ok = writeHmiRegisterWithRetry(10, raw);
      if (ok) {
        LOG_INFO("Web VFD frequency request -> HMI LW10: raw=%u hz=%.2f", raw, hz);
      } else {
        LOG_ERROR("Web VFD frequency write failure raw:%u", raw);
      }
      return;
    }
    if (doc.containsKey("valve") && doc.containsKey("action")) {
      int idx = doc["valve"].as<int>() - 1;
      bool open = strcmp(doc["action"], "open") == 0;
      if (idx >= 0 && idx < VALVE_COUNT + 1) {
        LOG_INFO("Web control request - Valve %d %s", idx+1, open?"OPEN":"CLOSE");
        ControlCmd cmd{ idx, open };
        xQueueSend(xCmdQueue, &cmd, pdMS_TO_TICKS(100));
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
            if (isEpochPlausible(epochTime)) {
              rtc.adjust(DateTime(epochTime));
              DateTime after = rtc.now();
              LOG_INFO("RTC after NTP: %02d:%02d:%02d %02d-%02d-%04d", after.hour(), after.minute(), after.second(), after.day(), after.month(), after.year());
              lastNTPSync = millis();
              LOG_INFO("RTC time updated from NTP on connect");
            } else {
              LOG_WARN("Ignoring NTP epoch out of range: %lu", epochTime);
            }
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
          if (isEpochPlausible(epochTime)) {
            rtc.adjust(DateTime(epochTime));
            DateTime after = rtc.now();
            if (!focusedLogging) LOG_INFO("RTC after NTP: %02d:%02d:%02d %02d-%02d-%04d", after.hour(), after.minute(), after.second(), after.day(), after.month(), after.year());
            lastNTPSync = millis();
            if (!focusedLogging) LOG_INFO("RTC time re-synced from NTP");
          } else {
            if (!focusedLogging) LOG_WARN("Ignoring NTP epoch out of range: %lu", epochTime);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task Modbus
void TaskModbus(void* pv) {
  LOG_INFO("Starting Modbus Task");
  esp_task_wdt_add(NULL);
  rs485.begin(BAUD_RATE, SERIAL_8N1, RS485_RX, RS485_TX);
  hmiNode.begin(HMI_ID, rs485);
  relayNode.begin(RELAY_ID, rs485);
  flowNode.begin(FLOW_ID, rs485);
  ecNode.begin(EC_ID, rs485);
  us1Node.begin(US1_ID, rs485);
  us2Node.begin(US2_ID, rs485);
  vfdNode.begin(VFD_ID, rs485);
  
  // Initialize pressure sensor ADC pin
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  LOG_INFO("Pressure sensor ADC initialized on pin %d", PRESSURE_SENSOR_PIN);

  setAllValvesOffFast();
  bool allLampsOff[VALVE_COUNT];
  for (int i = 0; i < VALVE_COUNT; i++) allLampsOff[i] = false;
  setHmiValveLampsBatch(allLampsOff);
  lastPumpLampState = true;
  setPumpLamp(false);
  LOG_INFO("Modbus initialized - all coils OFF");

  clearHmiValveButtonsBatch();
  xSemaphoreTake(xStateMutex, portMAX_DELAY);
  for (int i = 0; i < VALVE_COUNT; i++) {
    valveState[i] = false;
    lastValveState[i] = false;
    webButtonChange[i] = false;
  }
  webButtonChange[VALVE_COUNT] = false; // pump
  xSemaphoreGive(xStateMutex);
  pumpState = false;
  // Ensure pump is OFF at startup
  controlPumpRelay(false);
  publishValveStatus();
  LOG_INFO("Startup reset: all valves OFF, HMI buttons and lamps cleared");

  for (;;) {
    esp_task_wdt_reset();
    bool hadManualButtonChange = false;
    {
      uint16_t word = 0;
      uint8_t res = 0xFF;
      lockModbus();
      res = hmiNode.readCoils(HMI_BTN_ADDR, VALVE_COUNT + 2);
      if (res == hmiNode.ku8MBSuccess) {
        word = hmiNode.getResponseBuffer(0);
      }
      unlockModbus();
      if (res == hmiNode.ku8MBSuccess) {
        bool needPublish = false;
        bool modeBit = (word >> MODE_SWITCH_INDEX) & 0x01;
        if (autoMode != modeBit) {
          autoMode = modeBit;
          hadManualButtonChange = !autoMode;
          setAllValvesOffFast();
          bool allLampsOff[VALVE_COUNT];
          for (int i = 0; i < VALVE_COUNT; i++) { allLampsOff[i] = false; lastValveState[i] = false; }
          setHmiValveLampsBatch(allLampsOff);
          clearHmiValveButtonsBatch();
          controlPumpRelay(false);
          setPumpLamp(false);
          lastPumpBtnState = false;
          needPublish = true;
        }
        for (int i = 0; i < VALVE_COUNT; i++) {
          bool btn = (word >> i) & 0x01;
          if (btn != lastValveState[i]) {
            lastValveState[i] = btn;
            xSemaphoreTake(xStateMutex, portMAX_DELAY);
            valveState[i] = btn;
            xSemaphoreGive(xStateMutex);
            if (!autoMode) {
              hadManualButtonChange = true;
              controlRelayValve(i, btn);
              writeHmiCoilWithRetry(HMI_LAMP_ADDR + i, btn);
              if (webButtonChange[i]) webButtonChange[i] = false;
              needPublish = true;
            }
          }
        }
        bool pumpBtn = (word >> PUMP_BTN_INDEX) & 0x01;
        if (pumpBtn != lastPumpBtnState) {
          lastPumpBtnState = pumpBtn;
          if (!autoMode) {
            hadManualButtonChange = true;
            controlPumpRelay(pumpBtn);
            setPumpLamp(pumpBtn);
            if (webButtonChange[VALVE_COUNT]) webButtonChange[VALVE_COUNT] = false;
            needPublish = true;
          }
        }
        if (needPublish) publishValveStatus();
        lastHMIProcess = millis();
      }
    }
    static unsigned long lastManualSensorPollMs = 0;
    bool shouldPollSensors = (!currentIrrigation.isActive || currentIrrigation.activationReady);
    if (!autoMode && hadManualButtonChange) {
      shouldPollSensors = false;
    }
    if (!autoMode && !currentIrrigation.isActive) {
      unsigned long now = millis();
      if (now - lastManualSensorPollMs < SENSOR_POLL_INTERVAL_MANUAL_IDLE_MS) {
        shouldPollSensors = false;
      } else {
        lastManualSensorPollMs = now;
      }
    }
    if (shouldPollSensors) {
      readWaterFlowSensor(flowNode, waterFlow, "Flow Sensor");
      vTaskDelay(pdMS_TO_TICKS(sensorPollGapMs()));
      readECSensor(ecNode, ec, "EC Sensor");
      vTaskDelay(pdMS_TO_TICKS(sensorPollGapMs()));
      readUltrasonicSensor(us1Node, us1, "Ultrasonic1 Sensor", 2);
      vTaskDelay(pdMS_TO_TICKS(sensorPollGapMs()));
      readUltrasonicSensor(us2Node, us2, "Ultrasonic2 Sensor", 3);
      vTaskDelay(pdMS_TO_TICKS(sensorPollGapMs()));
      readPressureSensor(pressure, "Pressure Sensor");
      focusedLogging = currentIrrigation.isActive;
      writeUltrasonicToHMI();
      writeWaterFlowToHMI();
      writeECSensorToHMI();
      writePressureToHMI();
    }
    
    // Log successful sensor readings periodically
    static unsigned long lastSensorLog = 0;
    if (!currentIrrigation.isActive && millis() - lastSensorLog >= 10000) {
      int nutrPct = (int)((NUTRI_TANK_EMPTY_CM - us1) * 100 / (NUTRI_TANK_EMPTY_CM - NUTRI_TANK_FULL_CM));
      if (nutrPct < 0) nutrPct = 0;
      if (nutrPct > 100) nutrPct = 100;
      int airPct = (int)((WATER_TANK_EMPTY_CM - us2) * 100 / (WATER_TANK_EMPTY_CM - WATER_TANK_FULL_CM));
      if (airPct < 0) airPct = 0;
      if (airPct > 100) airPct = 100;
      LOG_INFO("Sensor readings - Flow:%.2f L/min EC:%.0f uS/cm Pressure:%.2f Bar Nutri:%d%% Air:%d%%",
               waterFlow, ec, pressure, nutrPct, airPct);
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
        bool ecControlEnabled = isAirNutrisi && currentIrrigation.targetEC > 0.1f;
        int landIdx = currentIrrigation.landRelay - 1;
        bool configReady = pumpState && valveState[WATER_SUPPLY_VALVE_IDX] && valveState[landIdx];
        if (!ecControlEnabled && isAirNutrisi) configReady = configReady && valveState[NUTRI_SUPPLY_VALVE_IDX];
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
          LOG_WARN("Irrigation activation not ready: pump:%s nutri:%s water:%s land:%s",
                   pumpState?"ON":"OFF",
                   valveState[NUTRI_SUPPLY_VALVE_IDX]?"ON":"OFF",
                   valveState[WATER_SUPPLY_VALVE_IDX]?"ON":"OFF",
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
          bool running = false;
          bool okRead = vfdReadIsRunning(running);
          if (!okRead) {
            bool sc = controlPumpRelay(true);
            if (!sc) {
              LOG_ERROR("Auto pump enforce failure");
            } else {
              LOG_INFO("Auto pump enforced ON");
            }
            publishValveStatus();
          } else if (!running) {
            bool sc = controlPumpRelay(true);
            if (!sc) {
              LOG_ERROR("Auto pump enforce failure");
            } else {
              LOG_INFO("Auto pump enforced ON");
            }
            publishValveStatus();
          } else {
            pumpState = true;
            setPumpLamp(true);
          }
          lastPumpEnsure = millis();
        }

        static unsigned long lastEcControlMs = 0;
        static unsigned long lastNutrValveChangeMs = 0;
        static unsigned long lastNutrRelaysOffMs = 0;
        static unsigned long lastNutrRedriveMs = 0;
        static unsigned long lastEcCtrlIrrStartMs = 0;
        static uint8_t nutrRedriveCount = 0;
        if (currentIrrigation.activationReady && ecControlEnabled && configReady) {
          unsigned long now = millis();
          if (now - lastEcControlMs >= EC_CONTROL_INTERVAL_MS) {
            lastEcControlMs = now;
            float target = currentIrrigation.targetEC;
            if (ec > 1.0f && target > 0.1f) {
              if (lastEcCtrlIrrStartMs != currentIrrigation.startMillis) {
                lastEcCtrlIrrStartMs = currentIrrigation.startMillis;
                nutrRedriveCount = 0;
                lastNutrRedriveMs = 0;
              }

              bool currNutr = valveState[NUTRI_SUPPLY_VALVE_IDX];
              bool wantNutr = currNutr;
              float low = target - EC_CONTROL_HYSTERESIS_US;
              float high = target + EC_CONTROL_HYSTERESIS_US;
              if (ec < low) wantNutr = true;
              else if (ec > high) wantNutr = false;
              else {
                nutrRedriveCount = 0;
                if (now - lastNutrRelaysOffMs >= 1500) {
                  bool offOk = setNutrisiRelaysOff();
                  if (offOk) lastNutrRelaysOffMs = now;
                }
              }

              if (wantNutr != currNutr && (now - lastNutrValveChangeMs >= EC_CONTROL_MIN_SWITCH_MS)) {
                bool ok = driveNutrisiMotorTo(wantNutr);
                writeHmiCoilWithRetry(HMI_LAMP_ADDR + NUTRI_SUPPLY_VALVE_IDX, wantNutr);
                writeHmiCoilWithRetry(HMI_BTN_ADDR + NUTRI_SUPPLY_VALVE_IDX, false);
                if (ok) {
                  xSemaphoreTake(xStateMutex, portMAX_DELAY);
                  valveState[NUTRI_SUPPLY_VALVE_IDX] = wantNutr;
                  xSemaphoreGive(xStateMutex);
                  lastNutrValveChangeMs = millis();
                  lastNutrRelaysOffMs = millis();
                  lastNutrRedriveMs = millis();
                  nutrRedriveCount = 0;
                  publishValveStatus();
                  LOG_INFO("EC control switch: ec=%.0f target=%.0f nutr:%s", ec, target, wantNutr ? "ON" : "OFF");
                } else {
                  LOG_WARN("EC control switch failed: ec=%.0f target=%.0f nutr:%s", ec, target, wantNutr ? "ON" : "OFF");
                }
              } else if (wantNutr == currNutr && (ec < low || ec > high)) {
                if (nutrRedriveCount < NUTRI_MOTOR_REDRIVE_MAX && (now - lastNutrRedriveMs >= NUTRI_MOTOR_REDRIVE_INTERVAL_MS)) {
                  bool ok = driveNutrisiMotorTo(wantNutr);
                  if (ok) {
                    lastNutrRelaysOffMs = millis();
                    lastNutrRedriveMs = millis();
                    nutrRedriveCount++;
                  }
                }
              }
            }
          }
        }

        if (lowFlowSeconds >= (int)NO_FLOW_STALL_SECONDS || noProgressSeconds >= (int)NO_PROGRESS_STALL_SECONDS) {
          LOG_WARN("Irrigation stalled, advancing to next schedule if available");
          controlPumpRelay(false);
          setPumpLamp(false);
          stopAllValvesAndResetHmiBatch();
          vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));
          float delivered = currentIrrigation.waterDelivered;
          int ci = currentIrrigation.configIndex;
          int si = currentIrrigation.scheduleIndex;
          IrrigationConfigItem* cfg = nullptr;
          IrrigationScheduleItem* sched = nullptr;
          if (ci >= 0 && ci < totalConfigs && configs[ci].isValid) {
            cfg = &configs[ci];
            if (si >= 0 && si < configs[ci].scheduleCount) {
              sched = &configs[ci].schedules[si];
            }
          }
          publishIrrigationLog(currentIrrigation, cfg, sched, "fail_flow", delivered);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "fail_flow", currentIrrigation.configId, "schedule");
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
          controlPumpRelay(false);
          setPumpLamp(false);
          stopAllValvesAndResetHmiBatch();
          vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));
          float delivered = currentIrrigation.waterDelivered;
          int ci = currentIrrigation.configIndex;
          int si = currentIrrigation.scheduleIndex;
          IrrigationConfigItem* cfg = nullptr;
          IrrigationScheduleItem* sched = nullptr;
          if (ci >= 0 && ci < totalConfigs && configs[ci].isValid) {
            cfg = &configs[ci];
            if (si >= 0 && si < configs[ci].scheduleCount) {
              sched = &configs[ci].schedules[si];
            }
          }
          publishIrrigationLog(currentIrrigation, cfg, sched, "success", delivered);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "success", currentIrrigation.configId, "schedule");
        }
        if ((millis() - currentIrrigation.startMillis) > 1800000) {
          LOG_WARN("Irrigation safety timeout, stopping");
          controlPumpRelay(false);
          setPumpLamp(false);
          stopAllValvesAndResetHmiBatch();
          vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));
          float delivered = currentIrrigation.waterDelivered;
          int ci = currentIrrigation.configIndex;
          int si = currentIrrigation.scheduleIndex;
          IrrigationConfigItem* cfg = nullptr;
          IrrigationScheduleItem* sched = nullptr;
          if (ci >= 0 && ci < totalConfigs && configs[ci].isValid) {
            cfg = &configs[ci];
            if (si >= 0 && si < configs[ci].scheduleCount) {
              sched = &configs[ci].schedules[si];
            }
          }
          publishIrrigationLog(currentIrrigation, cfg, sched, "timeout", delivered);
          currentIrrigation.isActive = false;
          currentIrrigation.activationReady = false;
          currentIrrigation.waterDelivered = 0;
          publishValveStatus();
          publishScheduleFeedback(currentIrrigation.landRelay, "stop", "timeout", currentIrrigation.configId, "schedule");
        }
      }
    }

    unsigned long nowMs = millis();
    unsigned long pollMs = autoMode ? 500 : 200;
    if (nowMs - lastHmiVfdFreqPoll >= pollMs) {
      lastHmiVfdFreqPoll = nowMs;
      uint8_t fr = 0xFF;
      uint16_t raw = 0xFFFF;
      lockModbus();
      fr = hmiNode.readHoldingRegisters(10, 1);
      if (fr == hmiNode.ku8MBSuccess) {
        raw = hmiNode.getResponseBuffer(0);
      }
      unlockModbus();
      if (fr == hmiNode.ku8MBSuccess && raw != lastHmiVfdFreqRaw) {
        lastHmiVfdFreqRaw = raw;
        float hz = 0.0f;
        if (raw <= (uint16_t)VFD_MAX_ALLOWED_HZ) hz = (float)raw;
        else hz = raw / 100.0f;
        if (hz < 0.0f) hz = 0.0f;
        if (hz > VFD_MAX_ALLOWED_HZ) hz = VFD_MAX_ALLOWED_HZ;
        vfdFrequencyHz = hz;
        bool ok = vfdSetFrequencyHz(hz);
        publishValveStatus();
        if (ok) {
          LOG_INFO("VFD frequency set from HMI LW10: raw=%u hz=%.2f", raw, hz);
        } else {
          LOG_ERROR("VFD frequency set failed from HMI LW10: raw=%u hz=%.2f", raw, hz);
        }
      }
    }

    static unsigned long lastPumpSyncMs = 0;
    if ((autoMode || currentIrrigation.isActive) && (millis() - lastPumpSyncMs >= 1000)) {
      bool running = false;
      bool okRead = vfdReadIsRunning(running);
      if (okRead) {
        if (pumpState != running) {
          pumpState = running;
          publishValveStatus();
        }
        setPumpLamp(running);
      }
      lastPumpSyncMs = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(autoMode ? HMI_POLL_INTERVAL_AUTO_MS : HMI_POLL_INTERVAL_MANUAL_MS));
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
  static int lastCheckMinuteOfDay = -1;
  static int lastCheckYmd = 0;
  for (;;) {
    if (rtcInitialized && autoMode) {
      if (millis() - lastScheduleCheck >= SCHEDULE_CHECK_INTERVAL) {
        DateTime now = rtc.now();
        int ymd = (now.year() * 10000) + (now.month() * 100) + now.day();
        int currentMinuteOfDay = (now.hour() * 60) + now.minute();

        if (lastCheckMinuteOfDay < 0 || lastCheckYmd != ymd) {
          lastCheckMinuteOfDay = currentMinuteOfDay - 1;
          if (lastCheckMinuteOfDay < 0) lastCheckMinuteOfDay = 1439;
          lastCheckYmd = ymd;
        }

        for (int i = 0; i < totalConfigs && i < MAX_CONFIGS; i++) {
          if (!configs[i].isValid) continue;
          for (int j = 0; j < configs[i].scheduleCount && j < 10; j++) {
            if (!configs[i].schedules[j].isActive) continue;
            int targetMinuteOfDay = -1;
            if (!parseScheduleMinute(configs[i].schedules[j].time, targetMinuteOfDay)) continue;
            if (!minuteInWindow(lastCheckMinuteOfDay, currentMinuteOfDay, targetMinuteOfDay)) continue;

            int32_t stamp = (int32_t)(ymd * 1440) + targetMinuteOfDay;
            if (lastScheduleEnqueueStamp[i][j] == stamp) continue;

            IrrigationJob job;
            job.configIndex = i;
            job.scheduleIndex = j;
            xQueueSend(xIrrigationQueue, &job, 0);
            lastScheduleEnqueueStamp[i][j] = stamp;

            char hhmm[6];
            sprintf(hhmm, "%02d:%02d", targetMinuteOfDay / 60, targetMinuteOfDay % 60);
            LOG_INFO("Enqueued irrigation for %s at %s", configs[i].landName.c_str(), hhmm);
          }
        }
        lastCheckMinuteOfDay = currentMinuteOfDay;
        lastCheckYmd = ymd;
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
    if (xQueueReceive(xCmdQueue, &cmd, pdMS_TO_TICKS(HMI_POLL_INTERVAL_MANUAL_MS)) == pdTRUE) {
      if (autoMode) {
        continue;
      }
      if (cmd.idx < 0 || cmd.idx > VALVE_COUNT) {
        LOG_ERROR("Invalid WEB control index: %d", cmd.idx);
        continue;
      }
      LOG_INFO("Processing WEB control - Valve %d %s", cmd.idx+1, cmd.open?"OPEN":"CLOSE");
      webButtonChange[cmd.idx] = true;
      writeHmiCoilWithRetry(HMI_BTN_ADDR + cmd.idx, cmd.open);
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
  xModbusMutex = xSemaphoreCreateMutex();

  Wire.begin(SDA_PIN, SCL_PIN);

  preferences.begin("irrigation", false);
  totalConfigs = preferences.getInt("totalConfigs", 0);
  if (totalConfigs < 0) totalConfigs = 0;
  if (totalConfigs > MAX_CONFIGS) totalConfigs = MAX_CONFIGS;
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
    if (configs[i].scheduleCount < 0) configs[i].scheduleCount = 0;
    if (configs[i].scheduleCount > 10) configs[i].scheduleCount = 10;
    for (int j = 0; j < configs[i].scheduleCount; j++) {
      configs[i].schedules[j].time = preferences.getString((prefix + String("time") + String(j)).c_str(), "");
      configs[i].schedules[j].isActive = preferences.getBool((prefix + String("active") + String(j)).c_str(), false);
    }
  }

  // Watchdog 30 detik
  esp_task_wdt_init(30, true);
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
        currentIrrigation.targetEC = configs[i].targetEC;
        currentIrrigation.startMillis = millis();
        currentIrrigation.landRelay = mapLandToValve(configs[i].landName);
        currentIrrigation.configIndex = i;
        currentIrrigation.scheduleIndex = job.scheduleIndex;
        controlPumpRelay(false);
        setPumpLamp(false);
        stopAllValvesAndResetHmiBatch();
        publishValveStatus();
        vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));

        bool isAirNutrisi = currentIrrigation.irrigationType.equalsIgnoreCase("air_nutrisi") || currentIrrigation.irrigationType.equalsIgnoreCase("air+nutrisi");
        bool ecControlEnabled = isAirNutrisi && currentIrrigation.targetEC > 0.1f;
        int landIdx = currentIrrigation.landRelay - 1;
        bool desired[VALVE_COUNT] = {false, false, false, false, false};
        desired[WATER_SUPPLY_VALVE_IDX] = true;
        if (isAirNutrisi && !ecControlEnabled) desired[NUTRI_SUPPLY_VALVE_IDX] = true;
        if (landIdx >= 0 && landIdx < VALVE_COUNT) desired[landIdx] = true;

        bool allOk = applyRelayStateSafeBatch(desired);
        setHmiValveLampsBatch(desired);
        clearHmiValveButtonsBatch();
        vTaskDelay(pdMS_TO_TICKS(irrigationValveDelayMs()));

        if (allOk) {
          bool pumpOk = controlPumpRelay(true);
          vTaskDelay(pdMS_TO_TICKS(PUMP_START_CONFIRM_DELAY_MS));
          bool running = false;
          bool okRead = vfdReadIsRunning(running);
          if (!pumpOk || !okRead || !running) {
            for (int attempt = 0; attempt < 2; attempt++) {
              controlPumpRelay(true);
              vTaskDelay(pdMS_TO_TICKS(PUMP_START_CONFIRM_DELAY_MS));
              bool r2 = false;
              bool ok2 = vfdReadIsRunning(r2);
              if (ok2 && r2) { running = true; break; }
            }
          }
          if (running) {
            currentIrrigation.activationReady = true;
            LOG_INFO("Irrigation started for %s schedule:%d target:%.2fL", currentIrrigation.landName.c_str(), currentIrrigation.scheduleIndex+1, currentIrrigation.waterNeeded);
            publishValveStatus();
            publishScheduleFeedback(currentIrrigation.landRelay, "start", "success", currentIrrigation.configId, "schedule");
          } else {
            LOG_ERROR("Pump start failed, aborting irrigation start");
            controlPumpRelay(false);
            setPumpLamp(false);
            stopAllValvesAndResetHmiBatch();
            vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));
            currentIrrigation.isActive = false;
            currentIrrigation.activationReady = false;
            publishValveStatus();
            publishScheduleFeedback(currentIrrigation.landRelay, "start", "fail_pump", currentIrrigation.configId, "schedule");
          }
        } else {
          LOG_ERROR("Valve activation failed, aborting irrigation start");
          controlPumpRelay(false);
          setPumpLamp(false);
          stopAllValvesAndResetHmiBatch();
          vTaskDelay(pdMS_TO_TICKS(irrigationStepDelayMs()));
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
