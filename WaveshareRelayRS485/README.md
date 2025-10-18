# ESP32-S3 Waveshare 16-Channel Relay Controller

This project implements ESP32-S3 code to control a Waveshare 16-channel relay module using Modbus RTU communication through a MAX485 module with RX/TX interface (no DE pin).

## Hardware Requirements

- ESP32-S3 development board
- Waveshare 16-channel relay module (RS485 interface)
- MAX485 module with RX/TX interface (no DE pin required)
- Jumper wires for connections

## Wiring Diagram

### MAX485 to ESP32-S3 Connection (RX/TX Interface)
```
MAX485 Module    ESP32-S3
-----------      --------
VCC         -->  3.3V or 5V
GND         -->  GND
RX          -->  GPIO 16 (RS485_RX_PIN)
TX          -->  GPIO 17 (RS485_TX_PIN)
```

### MAX485 to Waveshare Relay Connection
```
MAX485 Module    Waveshare Relay
-----------      ---------------
A+          -->  A+
B-          -->  B-
GND         -->  GND
VCC         -->  VCC (12V/24V as per relay requirements)
```

## Software Dependencies

The project uses the ModbusMaster library for Modbus RTU communication:

```ini
lib_deps = 4-20ma/ModbusMaster@^2.0.1
```

## Default Modbus Settings

- **Slave ID**: 1
- **Baud Rate**: 9600
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Function Codes**: 
  - 0x05 (Write Single Coil) - Individual relay control
  - 0x0F (Write Multiple Coils) - All relays control
  - 0x01 (Read Coils) - Relay status reading

## Pin Configuration

```cpp
#define RS485_RX_PIN 16  // Connect to RX pin of MAX485 module
#define RS485_TX_PIN 17  // Connect to TX pin of MAX485 module
```

## Usage

1. Upload the code to your ESP32-S3
2. Open Serial Monitor at 115200 baud rate
3. The system will automatically start relay testing upon startup
4. Use serial commands to control relays manually

### Serial Commands

- `on <1-16>` - Turn on specific relay (e.g., `on 5`)
- `off <1-16>` - Turn off specific relay (e.g., `off 3`)
- `allon` - Turn on all relays
- `alloff` - Turn off all relays
- `status` - Show status of all relays
- `status <1-16>` - Show status of specific relay
- `test` - Start automatic relay testing
- `stop` - Stop automatic testing and turn off all relays
- `help` - Show available commands

### Automatic Testing Feature

The system includes an automatic relay testing function that:
- Tests each relay sequentially (1 to 16)
- Turns each relay ON for 2 seconds
- Turns each relay OFF for 1 second
- Loops continuously until stopped
- Automatically starts on system boot
- Can be stopped/started using serial commands

## Features

- **Individual Relay Control**: Control each of the 16 relays independently
- **Bulk Operations**: Turn all relays on/off with single command
- **Status Monitoring**: Read current state of individual or all relays
- **Automatic Testing**: Built-in sequential relay testing for verification
- **User-friendly Interface**: Simple serial commands for easy control
- **Error Handling**: Comprehensive error reporting with Modbus error codes
- **No DE Pin Required**: Simplified wiring for MAX485 modules with RX/TX interface
- **ModbusMaster Library**: Reliable Modbus RTU communication

## Troubleshooting

1. **No response from relays**:
   - Check wiring connections (A+, B-, power)
   - Verify Modbus slave ID (default: 1)
   - Ensure correct baud rate (9600)
   - Check power supply to relay module

2. **Communication errors**:
   - Verify RX/TX pin connections
   - Check if MAX485 module is powered correctly
   - Ensure proper grounding between all devices
   - Try different baud rates if needed

3. **Relay testing not working**:
   - Use `stop` command to stop testing
   - Use `test` command to restart testing
   - Check individual relay control with `on`/`off` commands

## References

- [Waveshare 16-Channel Relay Wiki](https://www.waveshare.com/wiki/16-Channel_Relay_Module)
- [ModbusMaster Library](https://github.com/4-20ma/ModbusMaster)
- [ESP32-S3 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)

## License

This project is open source and available under the MIT License.