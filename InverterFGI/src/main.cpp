#include <Arduino.h>
#include <ModbusMaster.h>

static constexpr int RS485_RX_PIN = 17;
static constexpr int RS485_TX_PIN = 18;
static constexpr int RS485_DE_RE_PIN = -1;

static constexpr float MAX_ALLOWED_HZ = 50.0f;

static constexpr uint16_t REG_CONTROL_COMMAND = 0x2000;
static constexpr uint16_t REG_COMM_SET_FREQUENCY = 0x2001;

static constexpr uint16_t REG_FAULT_CODE = 0x2100;
static constexpr uint16_t REG_RUN_STATUS = 0x2101;
static constexpr uint16_t REG_SET_FREQUENCY = 0x2102;
static constexpr uint16_t REG_OUTPUT_FREQUENCY = 0x2103;
static constexpr uint16_t REG_OUTPUT_CURRENT = 0x2104;
static constexpr uint16_t REG_BUS_VOLTAGE = 0x2105;
static constexpr uint16_t REG_OUTPUT_VOLTAGE = 0x2106;

static constexpr uint16_t CMD_STOP = 0x0001;
static constexpr uint16_t CMD_RUN_FWD = 0x0012;
static constexpr uint16_t CMD_JOG_FWD = 0x0013;
static constexpr uint16_t CMD_RUN_REV = 0x0022;
static constexpr uint16_t CMD_JOG_REV = 0x0023;

static HardwareSerial& RS485_SERIAL = Serial2;
static ModbusMaster node;

static uint8_t vfdAddress = 1;
static uint32_t rs485Baud = 9600;
static float maxFrequencyHz = MAX_ALLOWED_HZ;

enum class InputMode : uint8_t { Menu, SetFreq, SetAddr, SetBaud, SetMaxHz, ScanStart, ScanEnd };
static InputMode inputMode = InputMode::Menu;
static uint8_t scanStartAddr = 1;

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

static void initModbus() {
  if (RS485_DE_RE_PIN >= 0) {
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);
  }
  RS485_SERIAL.begin(rs485Baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(vfdAddress, RS485_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

static bool probeAddress(uint8_t address) {
  node.begin(address, RS485_SERIAL);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  uint8_t result = node.readHoldingRegisters(REG_SET_FREQUENCY, 1);
  return result == node.ku8MBSuccess;
}

static void printMenu() {
  Serial.println();
  Serial.println("=== FGI FD100M Modbus RTU Control ===");
  Serial.printf("Addr: %u | Baud: %lu | MaxHz: %.2f\n", vfdAddress, (unsigned long)rs485Baud, maxFrequencyHz);
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
  Serial.println("5. Read Freq (set & output)");
  Serial.println("6. Status (fault, run status, V/A/Hz)");
  Serial.println("7. Fault code");
  Serial.println("8. Set Modbus Address");
  Serial.println("9. Set RS485 Baudrate");
  Serial.println("10. Set Max Frequency (Hz)");
  Serial.println("11. Scan/Ping inverter");
  Serial.println("0. Tampilkan menu");
  Serial.println();
  Serial.print("Pilih menu: ");
}

static const char* modbusErrorToString(uint8_t code) {
  switch (code) {
    case ModbusMaster::ku8MBSuccess:
      return "OK";
    case ModbusMaster::ku8MBIllegalFunction:
      return "Illegal Function";
    case ModbusMaster::ku8MBIllegalDataAddress:
      return "Illegal Data Address";
    case ModbusMaster::ku8MBIllegalDataValue:
      return "Illegal Data Value";
    case ModbusMaster::ku8MBSlaveDeviceFailure:
      return "Slave Device Failure";
    case ModbusMaster::ku8MBInvalidSlaveID:
      return "Invalid Slave ID";
    case ModbusMaster::ku8MBInvalidFunction:
      return "Invalid Function";
    case ModbusMaster::ku8MBResponseTimedOut:
      return "Response Timed Out";
    case ModbusMaster::ku8MBInvalidCRC:
      return "Invalid CRC";
    default:
      return "Unknown";
  }
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

static void printFault(uint16_t code) {
  struct FaultMap {
    uint16_t code;
    const char* name;
  };
  static const FaultMap map[] = {
      {0x0000, "No abnormality"},
      {0x0001, "Module fault"},
      {0x0002, "Overvoltage"},
      {0x0003, "Temperature fault"},
      {0x0004, "Inverter overload"},
      {0x0005, "Motor overload"},
      {0x0006, "External fault"},
      {0x0010, "Overcurrent during acceleration"},
      {0x0011, "Overcurrent during deceleration"},
      {0x0012, "Overcurrent at constant speed"},
      {0x0014, "Under voltage"},
      {0x0016, "RS485 communication failure"},
      {0x0017, "Tube explosion fault"},
      {0x0019, "Dual CPU communication failure"},
  };

  const char* desc = "Unknown";
  for (const auto& it : map) {
    if (it.code == code) {
      desc = it.name;
      break;
    }
  }
  Serial.printf("Fault code: %u (0x%04X) - %s\n", code, code, desc);
}

static void cmdRun(uint16_t cmd) {
  if (writeRegister(REG_CONTROL_COMMAND, cmd)) {
    Serial.printf("OK write 0x%04X = 0x%04X\n", REG_CONTROL_COMMAND, cmd);
  }
}

static void cmdSetFrequencyHz(float hz) {
  float safeHz = hz;
  if (safeHz < 0.0f) safeHz = 0.0f;
  if (safeHz > MAX_ALLOWED_HZ) {
    safeHz = MAX_ALLOWED_HZ;
    Serial.printf("Limit: max %.2f Hz\n", MAX_ALLOWED_HZ);
  }
  if (maxFrequencyHz < 0.01f) maxFrequencyHz = 0.01f;
  if (maxFrequencyHz > MAX_ALLOWED_HZ) maxFrequencyHz = MAX_ALLOWED_HZ;
  if (safeHz > maxFrequencyHz) safeHz = maxFrequencyHz;

  float pct = (safeHz / maxFrequencyHz) * 100.0f;
  int32_t rawPct = (int32_t)lroundf(pct * 100.0f);
  if (rawPct < 0) rawPct = 0;
  if (rawPct > 10000) rawPct = 10000;

  if (writeRegister(REG_COMM_SET_FREQUENCY, (uint16_t)rawPct)) {
    Serial.printf("OK freq set: %.2f Hz (%.2f%% -> reg 0x%04X = %u)\n",
                  safeHz,
                  pct,
                  REG_COMM_SET_FREQUENCY,
                  (uint16_t)rawPct);
  }
}

static void cmdReadFreq() {
  if (!readRegisters(REG_SET_FREQUENCY, 2)) return;
  uint16_t setRaw = node.getResponseBuffer(0);
  uint16_t outRaw = node.getResponseBuffer(1);
  Serial.printf("SetFreq: %.2f Hz | OutFreq: %.2f Hz\n", setRaw / 100.0f, outRaw / 100.0f);
}

static void cmdStatus() {
  if (!readRegisters(REG_FAULT_CODE, 7)) return;
  uint16_t fault = node.getResponseBuffer(0);
  uint16_t runStatus = node.getResponseBuffer(1);
  uint16_t setFreq = node.getResponseBuffer(2);
  uint16_t outFreq = node.getResponseBuffer(3);
  uint16_t outCurrent = node.getResponseBuffer(4);
  uint16_t busVolt = node.getResponseBuffer(5);
  uint16_t outVolt = node.getResponseBuffer(6);

  Serial.printf("SetFreq: %.2f Hz | OutFreq: %.2f Hz\n", setFreq / 100.0f, outFreq / 100.0f);
  Serial.printf("OutCurrent: %.1f A | BusVolt: %.1f V | OutVolt: %.1f V\n",
                outCurrent / 10.0f,
                busVolt / 10.0f,
                outVolt / 10.0f);

  Serial.printf("RunStatus: 0x%04X\n", runStatus);
  Serial.printf("  running=%u shutdown=%u jog=%u fwd=%u rev=%u comm_cmd=%u\n",
                (runStatus >> 0) & 0x01,
                (runStatus >> 1) & 0x01,
                (runStatus >> 2) & 0x01,
                (runStatus >> 3) & 0x01,
                (runStatus >> 4) & 0x01,
                (runStatus >> 10) & 0x01);

  printFault(fault);
}

static void cmdFaultOnly() {
  if (!readRegisters(REG_FAULT_CODE, 1)) return;
  uint16_t fault = node.getResponseBuffer(0);
  printFault(fault);
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
    uint8_t stop = scanEndAddr;
    if (stop < start) {
      uint8_t tmp = start;
      start = stop;
      stop = tmp;
    }

    const uint8_t maxRange = 30;
    if ((uint16_t)stop - (uint16_t)start + 1 > maxRange) {
      stop = start + maxRange - 1;
      Serial.printf("Range terlalu besar, dibatasi jadi %u..%u (maks %u alamat per scan)\n", start, stop, maxRange);
    }

    Serial.printf("Scan address %u..%u (bisa lambat jika tidak ada response)\n", start, stop);
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
      Serial.printf("Selesai. Inverter terdeteksi. Addr aktif sekarang: %u\n", vfdAddress);
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
    cmdRun(CMD_RUN_FWD);
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "2") {
    cmdRun(CMD_RUN_REV);
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "3") {
    cmdRun(CMD_STOP);
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
    cmdReadFreq();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "6") {
    cmdStatus();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "7") {
    cmdFaultOnly();
    Serial.print("Pilih menu: ");
    return;
  }
  if (s == "8") {
    inputMode = InputMode::SetAddr;
    Serial.println("Masukkan Modbus address (1..247).");
    Serial.print("Addr: ");
    return;
  }
  if (s == "9") {
    inputMode = InputMode::SetBaud;
    Serial.println("Masukkan baudrate (contoh: 9600 / 19200 / 38400).");
    Serial.print("Baud: ");
    return;
  }
  if (s == "10") {
    inputMode = InputMode::SetMaxHz;
    Serial.println("Masukkan max frequency inverter (Hz), contoh: 50.00");
    Serial.print("MaxHz: ");
    return;
  }
  if (s == "11") {
    inputMode = InputMode::ScanStart;
    Serial.println("Masukkan start address (1..247, 0=addr saat ini).");
    Serial.print("Start addr: ");
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
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
