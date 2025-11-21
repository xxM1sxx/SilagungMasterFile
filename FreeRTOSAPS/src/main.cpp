#include <Arduino.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusMaster.h>
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

// FreeRTOS Handles
QueueHandle_t xSensorQueue;
QueueHandle_t xCmdQueue;
SemaphoreHandle_t xStateMutex;
TaskHandle_t xMQTTTask, xModbusTask, xMQTTRecvTask;

// Global State
bool valveState[VALVE_COUNT] = {false};
bool lastValveState[VALVE_COUNT] = {false};
bool webButtonChange[VALVE_COUNT] = {false}; // Track web-initiated button changes
float waterFlow = 0.0;
float ec = 0.0;
uint16_t us1 = 0, us2 = 0;
float pressure = 0.0;  // Add pressure variable

// Logging state tracking
bool lastDisplayEmpty = false;
unsigned long lastSensorFail[5] = {0}; // flow, ec, us1, us2, pressure
unsigned long lastRelayLog = 0;
#define RELAY_LOG_INTERVAL 10000  // 10 detik
#define SENSOR_FAIL_LOG_INTERVAL 5000  // 5 detik

// Consecutive error tracking for sensor failure tolerance
uint8_t consecutiveSensorErrors[5] = {0}; // flow, ec, us1, us2, pressure
const uint8_t MAX_CONSECUTIVE_ERRORS = 10; // Ignore errors until this threshold

// Objects
HardwareSerial rs485(1);
ModbusMaster hmiNode, relayNode, flowNode, ecNode, us1Node, us2Node;
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

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
            // Some sensors return mm, convert to cm if > 1000
            uint16_t valCm = (raw > 1000) ? (raw / 10) : raw;
            // Validate distance range (5-500 cm)
            if (valCm >= 5 && valCm <= 500) {
                distance = valCm;
                consecutiveSensorErrors[2] = 0; // Reset error counter on success
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
    uint8_t result = node.readHoldingRegisters(0x0006, 2);
    if (result == node.ku8MBSuccess) {
        uint16_t r1 = node.getResponseBuffer(0);
        uint16_t r2 = node.getResponseBuffer(1);
        flow = registersToFloat(r1, r2);
        consecutiveSensorErrors[0] = 0; // Reset error counter on success
        return true;
    } else {
        consecutiveSensorErrors[0]++;
        
        // Only log error if we've exceeded the threshold (to ignore intermittent failures)
        if (consecutiveSensorErrors[0] >= MAX_CONSECUTIVE_ERRORS) {
            if (millis() - lastSensorFail[0] > SENSOR_FAIL_LOG_INTERVAL) {
                LOG_ERROR("%s read failure - error code: %u", sensorName, result);
                lastSensorFail[0] = millis();
            }
        }
        flow = 0.0;
        return false;
    }
}

// HMI Write Functions for Sensor Display
void writeUltrasonicToHMI() {
    // Write ultrasonic sensor values to HMI registers LW0 and LW1
    hmiNode.writeSingleRegister(0x0000, us1); // LW0 - Ultrasonic 1
    delay(10);
    hmiNode.writeSingleRegister(0x0001, us2); // LW1 - Ultrasonic 2
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
  doc["pump"] = valveState[5] ? "on" : "off";
  char buffer[256];
  serializeJson(doc, buffer);
  if (mqtt.connected()) mqtt.publish(topic_status, buffer);
  
  // Log relay safety status periodically
  if (millis() - lastRelayLog >= RELAY_LOG_INTERVAL) {
    LOG_INFO("Relay Safety Status: V1=%s V2=%s V3=%s V4=%s V5=%s Pump=%s",
             valveState[0]?"OPEN":"CLOSED",
             valveState[1]?"OPEN":"CLOSED", 
             valveState[2]?"OPEN":"CLOSED",
             valveState[3]?"OPEN":"CLOSED",
             valveState[4]?"OPEN":"CLOSED",
             valveState[5]?"ON":"OFF");
    lastRelayLog = millis();
  }
}

// MQTT Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    LOG_ERROR("MQTT JSON parse error: %s", err.c_str());
    return;
  }

  if (strcmp(topic, topic_control) == 0) {
    if (doc.containsKey("valve") && doc.containsKey("action")) {
      int idx = doc["valve"].as<int>() - 1;
      bool open = strcmp(doc["action"], "open") == 0;
      if (idx >= 0 && idx < VALVE_COUNT) {
        LOG_INFO("Web control request - Valve %d %s", idx+1, open?"OPEN":"CLOSE");
        xQueueSend(xCmdQueue, &idx, 0);
        xQueueSend(xCmdQueue, &open, 0);
        LOG_INFO("MQTT valve %d set to %s - HMI state will be updated", idx+1, open?"OPEN":"CLOSE");
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
  mqtt.setCallback(mqttCallback);
  LOG_INFO("WiFi connected, starting MQTT connection");

  while (!mqtt.connected()) {
    LOG_INFO("Attempting MQTT connection...");
    if (mqtt.connect("ESP32-RTOS", mqtt_user, mqtt_pass)) {
      mqtt.subscribe(topic_control);
      LOG_INFO("MQTT connected successfully");
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
        LOG_INFO("MQTT reconnected");
      }
    }
    mqtt.loop();

    if (millis() - lastPub >= 5000) {
      LOG_INFO("Publishing sensor and valve data");
      publishSensorData();
      publishValveStatus();
      lastPub = millis();
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

  // Init all coils OFF
  for (int i = 0; i < 10; i++) relayNode.writeSingleCoil(i, 0);
  for (int i = 0; i < VALVE_COUNT; i++) hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, 0);
  LOG_INFO("Modbus initialized - all coils OFF");

  for (;;) {
    // Read sensors using dedicated functions with timing delays
    bool flowSuccess = readWaterFlowSensor(flowNode, waterFlow, "Flow Sensor");
    delay(10); // Small delay between sensor reads to avoid bus collision
    
    bool ecSuccess = readECSensor(ecNode, ec, "EC Sensor");
    delay(10); // Small delay between sensor reads to avoid bus collision
    
    bool us1Success = readUltrasonicSensor(us1Node, us1, "Ultrasonic1 Sensor");
    delay(10); // Small delay between sensor reads to avoid bus collision
    
    bool us2Success = readUltrasonicSensor(us2Node, us2, "Ultrasonic2 Sensor");
    delay(10); // Small delay between sensor reads to avoid bus collision
    
    bool pressureSuccess = readPressureSensor(pressure, "Pressure Sensor");
    
    // Update HMI displays with sensor values
    writeUltrasonicToHMI();
    writeWaterFlowToHMI();
    writeECSensorToHMI();
    writePressureToHMI();
    
    // Log successful sensor readings periodically
    static unsigned long lastSensorLog = 0;
    if (millis() - lastSensorLog >= 10000) { // Log every 10 seconds
      LOG_INFO("Sensor readings - Flow:%.2f EC:%.0f US1:%u US2:%u Pressure:%.2f",
               waterFlow, ec, us1, us2, pressure);
      lastSensorLog = millis();
    }

    // Read HMI buttons
    uint8_t res = hmiNode.readCoils(HMI_BTN_ADDR, VALVE_COUNT);
    if (res == hmiNode.ku8MBSuccess) {
      uint16_t word = hmiNode.getResponseBuffer(0);
      for (int i = 0; i < VALVE_COUNT; i++) {
        bool btn = (word >> i) & 0x01;
        if (btn != lastValveState[i]) {
          lastValveState[i] = btn;
          xSemaphoreTake(xStateMutex, portMAX_DELAY);
          valveState[i] = btn;
          xSemaphoreGive(xStateMutex);

          // Check if this is a web-initiated change
          if (webButtonChange[i]) {
            // This change came from web - execute relay and update lamp
            int openCoil = i * 2;
            int closeCoil = openCoil + 1;
            relayNode.writeSingleCoil(closeCoil, 0);
            vTaskDelay(pdMS_TO_TICKS(1));
            relayNode.writeSingleCoil(openCoil, btn ? 1 : 0);

            // Update HMI lamp
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, btn ? 1 : 0);
            
            LOG_INFO("WEB-initiated control - Button %d changed to %s - Relay executed", i+1, btn?"ON":"OFF");
            
            // Clear the web change flag
            webButtonChange[i] = false;
            
            // Publish status update to web immediately
            publishValveStatus();
            LOG_INFO("WEB control completed for valve %d", i+1);
          } else {
            // This change came from physical HMI - execute normally
            int openCoil = i * 2;
            int closeCoil = openCoil + 1;
            relayNode.writeSingleCoil(closeCoil, 0);
            vTaskDelay(pdMS_TO_TICKS(1));
            relayNode.writeSingleCoil(openCoil, btn ? 1 : 0);

            // Update HMI lamp
            hmiNode.writeSingleCoil(HMI_LAMP_ADDR + i, btn ? 1 : 0);
            
            LOG_INFO("HMI control - Button %d changed to %s - Syncing to web", i+1, btn?"ON":"OFF");
            
            // Publish status update to web immediately after HMI change
            publishValveStatus();
            LOG_INFO("HMI change synced to web for valve %d", i+1);
          }
        }
      }
    } else {
      LOG_WARN("HMI button read failure - error code: %u", res);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task MQTT Recv (real-time control)
void TaskMQTTRecv(void* pv) {
  LOG_INFO("Starting MQTT Receive Task");
  int idx;
  bool open;
  for (;;) {
    if (xQueueReceive(xCmdQueue, &idx, pdMS_TO_TICKS(100)) == pdTRUE &&
        xQueueReceive(xCmdQueue, &open, pdMS_TO_TICKS(100)) == pdTRUE) {

      LOG_INFO("Processing WEB control - Valve %d %s", idx+1, open?"OPEN":"CLOSE");
      
      // Mark this as web-initiated change
      webButtonChange[idx] = true;
      
      // Update HMI button state only - relay control will be handled by TaskModbus
      // when it reads the button state change
      hmiNode.writeSingleCoil(HMI_BTN_ADDR + idx, open ? 1 : 0);
      
      LOG_INFO("WEB control - HMI button %d set to %s, waiting for TaskModbus to execute", idx+1, open?"ON":"OFF");
    }
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  LOG_INFO("System starting - FreeRTOS Irrigation Controller");

  xSensorQueue = xQueueCreate(8, sizeof(float[3]));
  xCmdQueue = xQueueCreate(8, sizeof(int) + sizeof(bool));
  xStateMutex = xSemaphoreCreateMutex();

  // Watchdog 10 detik
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  LOG_INFO("Creating FreeRTOS tasks");
  xTaskCreatePinnedToCore(TaskMQTT, "MQTT", 8192, NULL, 2, &xMQTTTask, 0);
  xTaskCreatePinnedToCore(TaskModbus, "Modbus", 8192, NULL, 3, &xModbusTask, 1);
  xTaskCreatePinnedToCore(TaskMQTTRecv, "MQTTRecv", 4096, NULL, 2, &xMQTTRecvTask, 0);
  
  LOG_INFO("System initialization complete");
}

void loop() {
  esp_task_wdt_reset();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Background monitoring log (optional)
  static unsigned long lastBgLog = 0;
  if (millis() - lastBgLog > 60000) { // Setiap 1 menit
    LOG_INFO("Background check - System running normally");
    lastBgLog = millis();
  }
}