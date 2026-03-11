#include <Arduino.h>
#include <ModbusMaster.h>

static constexpr int RS485_RX_PIN = 17;
static constexpr int RS485_TX_PIN = 18;
static constexpr int RS485_DE_RE_PIN = -1;

static constexpr float MAX_ALLOWED_HZ = 50.0f;

static constexpr uint16_t REG_CMD  = 0x31D9;  // CMD (command word)
static constexpr uint16_t REG_LFRD = 0x31DA;  // LFRD (reference speed)
static constexpr uint16_t REG_ETA  = 0x31C5;  // ETA (status word)
static constexpr uint16_t REG_RFRD = 0x31C6;  // RFRD (actual speed)

static constexpr uint16_t CMD_DISABLE_VOLTAGE      = 0x0000;
static constexpr uint16_t CMD_SHUTDOWN             = 0x0006;
static constexpr uint16_t CMD_SWITCH_ON            = 0x0007;
static constexpr uint16_t CMD_ENABLE_OPERATION_FWD = 0x000F;
static constexpr uint16_t CMD_ENABLE_OPERATION_REV = 0x080F;
static constexpr uint16_t CMD_QUICK_STOP           = 0x0002;
static constexpr uint16_t CMD_FAULT_RESET          = 0x0080;
static constexpr uint8_t RELAY_ID = 6;
static constexpr uint16_t PUMP_RELAY_COIL = 10;
static constexpr int RELAY_WRITE_RETRY_COUNT = 3;
static constexpr int RELAY_WRITE_RETRY_DELAY_MS = 50;

static HardwareSerial& RS485_SERIAL = Serial2;
static ModbusMaster node;
static ModbusMaster relayNode;

static uint8_t  vfdAddress    = 7;
static uint32_t rs485Baud     = 9600;
static uint32_t rs485Config   = SERIAL_8N1;
static float    maxFrequencyHz = MAX_ALLOWED_HZ;
static float    frequencyScale = 3.0f;
static uint32_t lastKeepAliveMs = 0;
static constexpr uint32_t KEEPALIVE_INTERVAL_MS = 10000;

enum class InputMode : uint8_t {
  Menu,
  SetFreq,
  SetAddr,
  SetBaud,
  SetMaxHz,
  SetScale,
  ScanStart,
  ScanEnd
};

static InputMode inputMode = InputMode::Menu;
static uint8_t   scanStartAddr = 1;

static void preTransmission() {
  if (RS485_DE_RE_PIN >= 0) {
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(100);
  }
}

static void postTransmission() {
  if (RS485_DE_RE_PIN >= 0) {
    delayMicroseconds(100);
    digitalWrite(RS485_DE_RE_PIN, LOW);
  }
}

static const char* modbusErrorToString(uint8_t code) {
  switch (code) {
    case ModbusMaster::ku8MBSuccess:           return "OK";
    case ModbusMaster::ku8MBIllegalFunction:   return "Illegal Function";
    case ModbusMaster::ku8MBIllegalDataAddress:return "Illegal Data Address";
    case ModbusMaster::ku8MBIllegalDataValue:  return "Illegal Data Value";
    case ModbusMaster::ku8MBSlaveDeviceFailure:return "Slave Device Failure";
    case ModbusMaster::ku8MBInvalidSlaveID:    return "Invalid Slave ID";
    case ModbusMaster::ku8MBInvalidFunction:   return "Invalid Function";
    case ModbusMaster::ku8MBResponseTimedOut:  return "Response Timed Out";
    case ModbusMaster::ku8MBInvalidCRC:        return "Invalid CRC";
    default:                                   return "Unknown";
  }
}

static void initModbus() {
  if (RS485_DE_RE_PIN >= 0) {
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);
  }
  RS485_SERIAL.begin(rs485Baud, rs485Config, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(vfdAddress, RS485_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  relayNode.begin(RELAY_ID, RS485_SERIAL);
  relayNode.preTransmission(preTransmission);
  relayNode.postTransmission(postTransmission);
}

static bool probeAddress(uint8_t address) {
  node.begin(address, RS485_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  uint8_t result = node.readHoldingRegisters(REG_ETA, 1);
  return result == node.ku8MBSuccess;
}

static void printMenu() {
  Serial.println();
  Serial.println("=== ATV310 Modbus RTU Control ===");
  Serial.printf("Addr: %u | Baud: %lu | MaxHz: %.2f\n",
                vfdAddress,
                (unsigned long)rs485Baud,
                maxFrequencyHz);
  Serial.printf("FreqScale: %.2f\n", frequencyScale);
  Serial.print("Serial: 8N1\n");
  if (RS485_DE_RE_PIN >= 0) {
    Serial.printf("DE/RE pin: %d\n", RS485_DE_RE_PIN);
  } else {
    Serial.println("DE/RE pin: (auto / tidak digunakan)");
  }
  Serial.println();
  Serial.println("1. RUN Forward");
  Serial.println("2. RUN Reverse");
  Serial.println("3. STOP");
  Serial.println("4. Set Frequency (Hz)");
  Serial.println("5. Status (ETA, freq)");
  Serial.println("6. Fault reset");
  Serial.println("7. Set Modbus Address");
  Serial.println("8. Set RS485 Baudrate");
  Serial.println("9. Set Max Frequency (Hz)");
  Serial.println("10. Scan/Ping inverter");
  Serial.println("11. Set Frequency Scale");
  Serial.println("0. Tampilkan menu");
  Serial.println();
  Serial.print("Pilih menu: ");
}

static bool writeRegister(uint16_t reg, uint16_t value) {
  uint8_t result = node.writeSingleRegister(reg, value);
  if (result == node.ku8MBSuccess) return true;
  Serial.printf("Modbus write gagal reg=0x%04X val=0x%04X err=0x%02X (%s)\n",
                reg,
                value,
                result,
                modbusErrorToString(result));
  return false;
}

static bool writeRelayCoilWithRetry(uint16_t coil, bool state) {
  uint8_t lastErr = 0xFF;
  for (int attempt = 0; attempt < RELAY_WRITE_RETRY_COUNT; attempt++) {
    uint8_t result = relayNode.writeSingleCoil(coil, state ? 1 : 0);
    if (result == relayNode.ku8MBSuccess) return true;
    lastErr = result;
    delay(RELAY_WRITE_RETRY_DELAY_MS);
  }
  Serial.printf("Relay write gagal coil=%u state=%u err=0x%02X (%s)\n",
                coil,
                state ? 1 : 0,
                lastErr,
                modbusErrorToString(lastErr));
  return false;
}

static bool readRelayCoilState(uint16_t coil, bool& state) {
  uint8_t result = relayNode.readCoils(coil, 1);
  if (result != relayNode.ku8MBSuccess) {
    Serial.printf("Relay read gagal coil=%u err=0x%02X (%s)\n",
                  coil,
                  result,
                  modbusErrorToString(result));
    return false;
  }
  uint16_t word = relayNode.getResponseBuffer(0);
  state = (word & 0x01) != 0;
  return true;
}

static bool controlPumpRelay(bool on) {
  bool ok = writeRelayCoilWithRetry(PUMP_RELAY_COIL, on);
  bool actual = on;
  if (ok) {
    bool readOk = readRelayCoilState(PUMP_RELAY_COIL, actual);
    if (readOk && actual != on) {
      Serial.printf("Relay verify mismatch coil=%u want=%u got=%u\n",
                    PUMP_RELAY_COIL,
                    on ? 1 : 0,
                    actual ? 1 : 0);
    }
  }
  Serial.printf("Pump relay %s\n", on ? "ON" : "OFF");
  return ok;
}

static bool readRegisters(uint16_t startReg, uint16_t count) {
  uint8_t result = node.readHoldingRegisters(startReg, count);
  if (result == node.ku8MBSuccess) return true;
  Serial.printf("Modbus read gagal start=0x%04X cnt=%u err=0x%02X (%s)\n",
                startReg,
                count,
                result,
                modbusErrorToString(result));
  return false;
}

static void sendCmd(uint16_t cmd) {
  if (writeRegister(REG_CMD, cmd)) {
    Serial.printf("OK CMD=0x%04X\n", cmd);
  }
}

static void sendRunSequence(uint16_t runCmd) {
  if (!writeRegister(REG_CMD, CMD_SHUTDOWN)) return;
  delay(50);
  if (!writeRegister(REG_CMD, CMD_SWITCH_ON)) return;
  delay(50);
  if (writeRegister(REG_CMD, runCmd)) {
    Serial.printf("OK CMD=0x%04X\n", runCmd);
  }
}

static void cmdRunForward() {
  controlPumpRelay(true);
}

static void cmdRunReverse() {
  controlPumpRelay(true);
}

static void cmdStop() {
  controlPumpRelay(false);
}

static void cmdQuickStop() {
  sendCmd(CMD_QUICK_STOP);
}

static void cmdFaultReset() {
  sendCmd(CMD_FAULT_RESET);
}

static void cmdSetFrequencyHz(float hz) {
  float safeHz = hz;
  if (safeHz < 0.0f) safeHz = 0.0f;
  float maxInputHz = MAX_ALLOWED_HZ;
  if (frequencyScale < 0.01f) frequencyScale = 0.01f;
  float maxByScale = 4000.0f / (10.0f * frequencyScale);
  if (maxByScale < maxInputHz) maxInputHz = maxByScale;
  if (safeHz > maxInputHz) safeHz = maxInputHz;
  if (maxFrequencyHz < 0.01f) maxFrequencyHz = 0.01f;
  if (maxFrequencyHz > MAX_ALLOWED_HZ) maxFrequencyHz = MAX_ALLOWED_HZ;
  if (safeHz > maxFrequencyHz) safeHz = maxFrequencyHz;

  int32_t raw = (int32_t)lroundf(safeHz * frequencyScale * 10.0f);
  if (raw < 0) raw = 0;
  if (raw > 4000) raw = 4000;

  if (writeRegister(REG_LFRD, (uint16_t)raw)) {
    Serial.printf("OK freq set: %.2f Hz -> reg 0x%04X = %u\n",
                  safeHz,
                  REG_LFRD,
                  (uint16_t)raw);
  }
}

static void cmdStatus() {
  if (!readRegisters(REG_ETA, 2)) return;
  uint16_t eta  = node.getResponseBuffer(0);
  uint16_t rfrd = node.getResponseBuffer(1);

  float freqHz = (float)((int16_t)rfrd) / 10.0f;

  Serial.printf("ETA: 0x%04X\n", eta);
  Serial.printf(
      "  ready_to_switch_on=%u switched_on=%u operation_enabled=%u "
      "fault=%u quick_stop=%u switch_on_disabled=%u\n",
      (eta >> 0) & 0x01,
      (eta >> 1) & 0x01,
      (eta >> 2) & 0x01,
      (eta >> 3) & 0x01,
      (eta >> 5) & 0x01,
      (eta >> 6) & 0x01);
  Serial.printf("Actual freq: %.2f Hz (raw=%d)\n",
                freqHz,
                (int16_t)rfrd);
}

static void handleLine(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  if (inputMode == InputMode::SetMaxHz) {
    float hz = s.toFloat();
    if (hz <= 0.0f || hz > MAX_ALLOWED_HZ) {
      Serial.printf("MaxHz tidak valid. Range: 0.01..%.2f\n", MAX_ALLOWED_HZ);
    } else {
      maxFrequencyHz = hz;
      Serial.printf("OK MaxHz=%.2f\n", maxFrequencyHz);
    }
    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (inputMode == InputMode::SetScale) {
    float scale = s.toFloat();
    if (scale < 0.1f || scale > 10.0f) {
      Serial.println("Scale tidak valid. Range: 0.1..10.0");
    } else {
      frequencyScale = scale;
      Serial.printf("OK FreqScale=%.2f\n", frequencyScale);
    }
    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (inputMode == InputMode::ScanStart) {
    int start = s.toInt();
    if (start == 0) {
      scanStartAddr = vfdAddress;
    } else if (start < 1 || start > 247) {
      Serial.println("Start address tidak valid. Range: 1..247 (atau 0=addr saat ini)");
      Serial.print("Start addr: ");
      return;
    } else {
      scanStartAddr = (uint8_t)start;
    }
    inputMode = InputMode::ScanEnd;
    Serial.println("Masukkan end address (1..247, 0=sama dengan start).");
    Serial.print("End addr: ");
    return;
  }

  if (inputMode == InputMode::ScanEnd) {
    int end = s.toInt();
    uint8_t scanEndAddr = scanStartAddr;
    if (end == 0) {
      scanEndAddr = scanStartAddr;
    } else if (end < 1 || end > 247) {
      Serial.println("End address tidak valid. Range: 1..247 (atau 0=sama dengan start)");
      Serial.print("End addr: ");
      return;
    } else {
      scanEndAddr = (uint8_t)end;
    }

    uint8_t start = scanStartAddr;
    uint8_t stop  = scanEndAddr;
    if (stop < start) {
      uint8_t tmp = start;
      start = stop;
      stop  = tmp;
    }

    const uint8_t maxRange = 30;
    if ((uint16_t)stop - (uint16_t)start + 1 > maxRange) {
      stop = start + maxRange - 1;
      Serial.printf("Range terlalu besar, dibatasi jadi %u..%u (maks %u alamat per scan)\n",
                    start, stop, maxRange);
    }

    Serial.printf("Scan address %u..%u (bisa lambat jika tidak ada response)\n",
                  start, stop);
    bool foundAny = false;
    for (uint8_t a = start; a <= stop; a++) {
      Serial.printf("  Ping %u ... ", a);
      bool ok = probeAddress(a);
      Serial.println(ok ? "OK" : "NO");
      if (ok) {
        foundAny = true;
        vfdAddress = a;
      }
      if (a == 247) break;
    }

    initModbus();
    if (foundAny) {
      Serial.printf("Selesai. Inverter terdeteksi. Addr aktif sekarang: %u\n",
                    vfdAddress);
    } else {
      Serial.println("Selesai. Tidak ada inverter terdeteksi pada range tersebut.");
    }

    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (inputMode == InputMode::SetFreq) {
    float hz = s.toFloat();
    cmdSetFrequencyHz(hz);
    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (inputMode == InputMode::SetAddr) {
    int a = s.toInt();
    if (a < 1 || a > 247) {
      Serial.println("Alamat tidak valid. Range: 1..247");
    } else {
      vfdAddress = (uint8_t)a;
      initModbus();
      Serial.printf("OK addr=%u\n", vfdAddress);
    }
    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (inputMode == InputMode::SetBaud) {
    uint32_t b = (uint32_t)s.toInt();
    if (b == 0) {
      Serial.println("Baud tidak valid.");
    } else {
      rs485Baud = b;
      initModbus();
      Serial.printf("OK baud=%lu\n", (unsigned long)rs485Baud);
    }
    inputMode = InputMode::Menu;
    printMenu();
    return;
  }

  if (s == "1") {
    cmdRunForward();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "2") {
    cmdRunReverse();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "3") {
    cmdStop();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "4") {
    inputMode = InputMode::SetFreq;
    Serial.println("Masukkan frekuensi (Hz), contoh: 10.50");
    Serial.print("Hz: ");
    return;
  }
  if (s == "5") {
    cmdStatus();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "6") {
    cmdFaultReset();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "7") {
    inputMode = InputMode::SetAddr;
    Serial.println("Masukkan Modbus address (1..247).");
    Serial.print("Addr: ");
    return;
  }
  if (s == "8") {
    inputMode = InputMode::SetBaud;
    Serial.println("Masukkan baudrate (contoh: 9600 / 19200 / 38400).");
    Serial.print("Baud: ");
    return;
  }
  if (s == "9") {
    inputMode = InputMode::SetMaxHz;
    Serial.println("Masukkan max frequency inverter (Hz), contoh: 50.00");
    Serial.print("MaxHz: ");
    return;
  }
  if (s == "10") {
    inputMode = InputMode::ScanStart;
    Serial.println("Masukkan start address (1..247, 0=addr saat ini).");
    Serial.print("Start addr: ");
    return;
  }
  if (s == "11") {
    inputMode = InputMode::SetScale;
    Serial.println("Masukkan freq scale (contoh: 1.0 / 3.0).");
    Serial.print("Scale: ");
    return;
  }
  if (s == "0") {
    printMenu();
    return;
  }

  Serial.println("Pilihan tidak dikenal.");
  printMenu();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  initModbus();
  Serial.printf("Ping inverter addr %u ... ", vfdAddress);
  Serial.println(probeAddress(vfdAddress) ? "OK" : "NO");
  initModbus();
  printMenu();
}

void loop() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastKeepAliveMs) >= KEEPALIVE_INTERVAL_MS) {
    lastKeepAliveMs = now;
    uint8_t result = node.readHoldingRegisters(REG_ETA, 1);
    if (result != node.ku8MBSuccess) {
      initModbus();
    }
  }
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
