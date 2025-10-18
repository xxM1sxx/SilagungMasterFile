#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>

// Modbus Master instance
ModbusMaster node;

// RS485 pins for ESP32-S3 (MAX485 module with RX/TX interface only)
#define RS485_RX_PIN 17  // Connect to RX pin of MAX485 module
#define RS485_TX_PIN 18  // Connect to TX pin of MAX485 module

// Waveshare Relay 16CH default settings
#define RELAY_SLAVE_ID 1    // Default slave address
#define RELAY_BAUD_RATE 9600
#define RELAY_COUNT 16

// Serial for RS485 communication
HardwareSerial rs485Serial(1);

// Testing variables
unsigned long lastTestTime = 0;
uint8_t currentTestRelay = 1;
bool testingActive = false;
bool relayState = false;

// Function declarations
void setupModbus();
bool setRelay(uint8_t relayNum, bool state);
bool setAllRelays(bool state);
bool getRelayStatus(uint8_t relayNum);
void printRelayStatus();
void processSerialCommands();
void startRelayTest();
void runRelayTest();

void setup() {
  // Initialize main serial for debugging
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32-S3 Waveshare 16CH Relay Controller");
  Serial.println("Using ModbusMaster Library - RX/TX Interface");
  Serial.println("============================================");
  
  // Setup Modbus RTU communication
  setupModbus();
  
  Serial.println("System ready!");
  Serial.println("Commands:");
  Serial.println("  on <1-16>   - Turn on relay (1-16)");
  Serial.println("  off <1-16>  - Turn off relay (1-16)");
  Serial.println("  allon       - Turn on all relays");
  Serial.println("  alloff      - Turn off all relays");
  Serial.println("  status      - Show all relay status");
  Serial.println("  status <1-16> - Show specific relay status");
  Serial.println("  test        - Start automatic relay testing");
  Serial.println("  stop        - Stop automatic testing");
  
  // Start automatic relay testing
  Serial.println("\nStarting automatic relay test...");
  startRelayTest();
}

void loop() {
  // Run automatic relay testing if active
  if (testingActive) {
    runRelayTest();
  }
  
  // Process serial commands
  processSerialCommands();
  
  delay(10);
}

void setupModbus() {
  // Initialize RS485 serial (no DE pin needed for RX/TX interface)
  rs485Serial.begin(RELAY_BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // Initialize Modbus communication
  node.begin(RELAY_SLAVE_ID, rs485Serial);
  
  Serial.println("Modbus RTU initialized (RX/TX interface)");
  Serial.printf("Slave ID: %d, Baud Rate: %d\n", RELAY_SLAVE_ID, RELAY_BAUD_RATE);
  Serial.printf("RX Pin: %d, TX Pin: %d\n", RS485_RX_PIN, RS485_TX_PIN);
}

bool setRelay(uint8_t relayNum, bool state) {
  if (relayNum < 1 || relayNum > RELAY_COUNT) {
    Serial.println("Error: Relay number must be between 1-16");
    return false;
  }
  
  // Modbus coil address (0-based, so relay 1 = coil 0)
  uint16_t coilAddress = relayNum - 1;
  
  // Send Modbus command to write single coil (Function Code 0x05)
  uint8_t result = node.writeSingleCoil(coilAddress, state);
  
  if (result == node.ku8MBSuccess) {
    Serial.printf("Relay %d %s\n", relayNum, state ? "ON" : "OFF");
    return true;
  } else {
    Serial.printf("Error: Failed to %s relay %d (Error: 0x%02X)\n", 
                  state ? "turn on" : "turn off", relayNum, result);
    return false;
  }
}

bool setAllRelays(bool state) {
  Serial.printf("Setting all relays %s...\n", state ? "ON" : "OFF");
  
  // For ModbusMaster library, we need to set the transmit buffer first
  // Set all 16 relays to the same state
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    node.setTransmitBuffer(i, state ? 1 : 0);
  }
  
  // Write multiple coils (Function Code 0x0F)
  uint8_t result = node.writeMultipleCoils(0, RELAY_COUNT);
  
  if (result == node.ku8MBSuccess) {
    Serial.printf("All relays set %s successfully\n", state ? "ON" : "OFF");
    return true;
  } else {
    Serial.printf("Error: Failed to set all relays (Error: 0x%02X)\n", result);
    return false;
  }
}

bool getRelayStatus(uint8_t relayNum) {
  if (relayNum < 1 || relayNum > RELAY_COUNT) {
    Serial.println("Error: Relay number must be between 1-16");
    return false;
  }
  
  // Modbus coil address (0-based)
  uint16_t coilAddress = relayNum - 1;
  
  // Read single coil status (Function Code 0x01)
  uint8_t result = node.readCoils(coilAddress, 1);
  
  if (result == node.ku8MBSuccess) {
    bool status = node.getResponseBuffer(0) & 0x01;
    Serial.printf("Relay %d: %s\n", relayNum, status ? "ON" : "OFF");
    return status;
  } else {
    Serial.printf("Error: Failed to read relay %d status (Error: 0x%02X)\n", relayNum, result);
    return false;
  }
}

void printRelayStatus() {
  Serial.println("Relay Status:");
  Serial.println("=============");
  
  // Read all coils at once (Function Code 0x01)
  uint8_t result = node.readCoils(0, RELAY_COUNT);
  
  if (result == node.ku8MBSuccess) {
    // Process response buffer
    uint16_t responseData = node.getResponseBuffer(0);
    
    for (uint8_t i = 1; i <= RELAY_COUNT; i++) {
      bool status = (responseData >> (i-1)) & 0x01;
      Serial.printf("Relay %2d: %s\n", i, status ? "ON" : "OFF");
    }
  } else {
    Serial.printf("Error: Failed to read relay status (Error: 0x%02X)\n", result);
  }
}

void startRelayTest() {
  testingActive = true;
  currentTestRelay = 1;
  relayState = false;
  lastTestTime = millis();
  Serial.println("=== RELAY TESTING STARTED ===");
  Serial.println("Testing each relay: ON for 2 seconds, OFF for 1 second");
}

void runRelayTest() {
  unsigned long currentTime = millis();
  
  // Check if it's time to change relay state
  if (currentTime - lastTestTime >= (relayState ? 2000 : 1000)) { // ON for 2s, OFF for 1s
    if (relayState) {
      // Turn OFF current relay
      setRelay(currentTestRelay, false);
      relayState = false;
      
      // Move to next relay
      currentTestRelay++;
      if (currentTestRelay > RELAY_COUNT) {
        currentTestRelay = 1; // Loop back to relay 1
        Serial.println("=== Test cycle completed, starting again ===");
      }
    } else {
      // Turn ON current relay
      Serial.printf("Testing Relay %d...\n", currentTestRelay);
      setRelay(currentTestRelay, true);
      relayState = true;
    }
    
    lastTestTime = currentTime;
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command.startsWith("on ")) {
      int relayNum = command.substring(3).toInt();
      testingActive = false; // Stop testing when manual command is used
      setRelay(relayNum, true);
    }
    else if (command.startsWith("off ")) {
      int relayNum = command.substring(4).toInt();
      testingActive = false; // Stop testing when manual command is used
      setRelay(relayNum, false);
    }
    else if (command == "allon") {
      testingActive = false;
      setAllRelays(true);
    }
    else if (command == "alloff") {
      testingActive = false;
      setAllRelays(false);
    }
    else if (command == "status") {
      printRelayStatus();
    }
    else if (command.startsWith("status ")) {
      int relayNum = command.substring(7).toInt();
      getRelayStatus(relayNum);
    }
    else if (command == "test") {
      startRelayTest();
    }
    else if (command == "stop") {
      testingActive = false;
      setAllRelays(false); // Turn off all relays when stopping
      Serial.println("Automatic testing stopped. All relays turned OFF.");
    }
    else if (command == "help") {
      Serial.println("Commands:");
      Serial.println("  on <1-16>   - Turn on relay");
      Serial.println("  off <1-16>  - Turn off relay");
      Serial.println("  allon       - Turn on all relays");
      Serial.println("  alloff      - Turn off all relays");
      Serial.println("  status      - Show all relay status");
      Serial.println("  status <1-16> - Show specific relay status");
      Serial.println("  test        - Start automatic relay testing");
      Serial.println("  stop        - Stop automatic testing");
    }
    else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}