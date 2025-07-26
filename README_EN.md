# 🔧 Arduino Daily Project

**Collection of Practical and Educational Hardware Projects**

[![Arduino](https://img.shields.io/badge/Arduino-UNO%20R3-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![ESP32](https://img.shields.io/badge/ESP32-WiFi%2FBT-red?style=for-the-badge&logo=espressif&logoColor=white)](https://espressif.com/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)](LICENSE)
[![Version](https://img.shields.io/badge/Version-3.0-green?style=for-the-badge)](https://github.com/Jirehhyeon/Arduino-Daily-project)

[🇰🇷 한국어](README.md) | 🇺🇸 **English**

---

## 🌟 Project Lineup

### 🎯 **v3.0 - Smart IoT Fan Control System** (Latest)
**Next-Generation IoT-Based Complete Smart Home Fan Solution**
- **AI-Based Environment Detection**: Automatic analysis of temperature, humidity, light, and noise
- **Mobile App Integration**: WiFi/Bluetooth remote control
- **Advanced Safety System**: Multi-layer protection and real-time monitoring
- **[Detailed Guide](Smart_Fan_README.md)**

### 🔧 **v2.0 - HW-020 Motor Shield Control System**
**12V Fan Motor Control System Using HW-020 Motor Shield**
- **Safe Motor Control**: Safety design based on jumper wires
- **Real-time Monitoring**: Speed, temperature, current monitoring
- **EEPROM Settings Storage**: Settings persistence after restart

> Arduino project providing safe motor control through jumper wires and real-time monitoring functionality

---

## 📚 Documentation Guide

### 📖 **User Guides**
- **⚡ [Quick Start](QUICK_START_GUIDE.md)**: Get started in 10 minutes
- **📘 [Complete Manual](Smart_Fan_README.md)**: Detailed features and usage
- **🔧 [Troubleshooting](TROUBLESHOOTING.md)**: Comprehensive troubleshooting guide

### 🔬 **Technical Documentation**
- **🔌 [Circuit Diagrams](Smart_Fan_Circuit_Diagram.md)**: Detailed hardware connection guide
- **📐 [Detailed Circuits](CIRCUIT_DIAGRAMS.md)**: Step-by-step circuit assembly
- **🚀 [Development Roadmap](PROJECT_ROADMAP.md)**: Future development plans
- **📡 [API Documentation](API_DOCUMENTATION.md)**: Complete API reference
- **📊 [Performance Benchmarks](PERFORMANCE_BENCHMARKS.md)**: System performance analysis

### 🎓 **Learning Materials**
- **🎒 [Beginner Guide](BEGINNER_GUIDE.md)**: Complete learning path for beginners
- **🛠️ [Hands-on Tutorial](HANDS_ON_TUTORIAL.md)**: 7-day structured course
- **🛒 [Parts Guide](PARTS_GUIDE.md)**: Complete purchasing guide

---

## ✨ Key Features

### 🎛️ **Complete Motor Control**
- **PWM Speed Control**: Smooth 0-100% speed adjustment
- **Direction Control**: Clockwise/Counterclockwise switching
- **Smooth Acceleration/Deceleration**: Prevents sudden start/stop
- **Real-time Status Monitoring**: Speed, direction, temperature, current

### 🛡️ **Safety System**
- **Overcurrent Protection**: Automatic stop at 1.5A threshold
- **Temperature Monitoring**: Warning and deceleration above 60°C
- **Safety Mode**: Immediate motor stop upon danger detection
- **Electrical Safety**: 2.5x safety margin for jumper wire safety

### 💡 **Intelligent Features**
- **EEPROM Settings Storage**: Settings persist after restart
- **LED Status Display**: Power, direction, warning LEDs
- **Serial Commands**: Real-time control and monitoring from PC
- **Performance Reports**: Periodic status reporting

---

## 🔌 Hardware Configuration

### Essential Components

| Component | Quantity | Purpose |
|-----------|----------|---------|
| Arduino Uno R3 | 1 | Main controller |
| ESP32 DevKit v1 | 1 | WiFi/Bluetooth communication |
| HW-020 Motor Shield | 1 | L298P-based motor driver |
| 12V DC Fan Motor | 1 | Small, propeller-attachable |
| 12V 2A Adapter | 1 | External power supply |
| DHT22 Sensor | 1 | Temperature/humidity |
| ACS712 Current Sensor | 1 | Current monitoring |
| Servo Motor SG90 | 1 | Wind direction control |
| Jumper Wires 20AWG | 20 | Signal connections (Male-Female) |
| 10kΩ Potentiometer | 1 | Speed control |
| Tactile Switch | 2 | Direction control/Emergency stop |
| LEDs (Red/Green/Yellow) | 3 | Status display |
| 220Ω Resistors | 5 | LED current limiting |

### Optional Components
- **I2C LCD 16x2** - Status display
- **LDR Light Sensor** - Ambient light detection
- **Buzzer** - Audio notifications
- **Breadboards** - Additional circuit construction

---

## ⚡ Electrical Safety

### Current Calculation Results
```
12V Small Fan Motor: 0.3-0.8A (rated)
20AWG Jumper Wire Allowance: 2A (safe), 11A (maximum)
HW-020 Shield Allowance: 2A (per channel)

Safety Margin: 2.5x ✅ Sufficiently Safe
```

---

## 📋 Pin Assignment

| Arduino Pin | Connected To | Function | Type |
|-------------|--------------|----------|------|
| **D3** | HW-020 ENA | Motor PWM | PWM Output |
| **D7** | DHT22 Data | Temperature/Humidity | Digital I/O |
| **D11** | Servo Signal | Wind Direction | PWM Output |
| **D12** | HW-020 IN1 | Motor Direction | Digital Output |
| **A0** | Potentiometer | Speed Control | Analog Input |
| **A1** | ACS712 Output | Current Sensing | Analog Input |
| **A2** | LDR | Light Sensing | Analog Input |
| **A4** | I2C SDA | LCD Data | Digital Communication |
| **A5** | I2C SCL | LCD Clock | Digital Communication |

| ESP32 Pin | Connected To | Function | Type |
|-----------|--------------|----------|------|
| **TX0** | Arduino D0 (RX) | Serial Communication | UART |
| **RX0** | Arduino D1 (TX) | Serial Communication | UART |
| **GPIO2** | Status LED | ESP32 Status | Digital Output |
| **3.3V** | Arduino 3.3V | Power Supply | Power |
| **GND** | Arduino GND | Common Ground | Ground |

---

## 🚀 Quick Start

### 1. Hardware Assembly
```bash
1. Mount HW-020 shield on Arduino
2. Check 12V adapter polarity and connect
3. Connect motor to shield OUT1, OUT2
4. Connect control circuit (potentiometer, button, LED)
5. Connect signal lines with jumper wires
6. Connect ESP32 for WiFi communication
```

### 2. Software Upload
```bash
Arduino Code:
1. Open src/arduino/Smart_Fan_Controller.ino in Arduino IDE
2. Select board: "Arduino Uno"
3. Select port and upload ▶️

ESP32 Code:
1. Open src/esp32/ESP32_WiFi_Controller.ino in Arduino IDE
2. Select board: "ESP32 Dev Module"  
3. Select port and upload ▶️
```

### 3. WiFi Setup and Testing
```bash
WiFi Connection:
1. Connect smartphone to "SmartFan_Setup" network (password: 12345678)
2. Open browser to 192.168.4.1
3. Enter home WiFi information and click "Connect"

System Testing:
1. After WiFi connection, access http://smartfan.local
2. Click "🟢 Start" button
3. Test fan speed with speed slider
4. Check real-time sensor data
```

---

## 💻 Control Commands

### Serial Commands
| Command | Function | Example |
|---------|----------|---------|
| `START` | Start motor | `START` |
| `STOP` | Stop motor | `STOP` |
| `SPEED:<0-100>` | Set speed | `SPEED:75` |
| `DIR` | Change direction | `DIR` |
| `STATUS` | Check status | `STATUS` |
| `AUTO:ON/OFF` | Auto mode | `AUTO:ON` |
| `SERVO:<0-180>` | Wind direction | `SERVO:90` |

### Hardware Control
- **Potentiometer**: Real-time speed control (0-100%)
- **Direction Button**: Direction switching (with debouncing)
- **Emergency Stop**: Immediate safety stop (D2 interrupt)
- **LED Display**: 
  - Green: Power status
  - Yellow: Direction (blink rate indicates mode)
  - Red: Warning/safety mode

---

## 🌐 Web Interface Features

### Real-time Control
```
🟢 Start/🔴 Stop: Instant fan on/off
🤖 Auto Mode: AI automatic control 
🌱 Eco Mode: Energy saving mode
Speed Slider: 0-100% adjustment
🎛️ Wind Direction: 0-180° servo control
```

### Real-time Monitoring
```
🌡️ Temperature/Humidity: Real-time environmental data
⚡ Current: Power consumption monitoring  
🔄 Runtime: Cumulative operation time
🚨 Safety Status: System safety mode
💡 Light Level: Ambient light detection
```

### Advanced Features
```
⏰ Schedule: Time-based automatic operation
🌙 Night Mode: Quiet night operation
📱 Mobile Responsive: Optimized for smartphones
📡 WebSocket: Real-time data updates
🔔 Notifications: System alerts and warnings
```

---

## 🔧 Major Feature Details

### 1. Smooth Speed Control
```cpp
// Gradual speed change (ramping)
if (current_speed < target_speed) {
    current_speed = min(current_speed + 2, target_speed);
} else if (current_speed > target_speed) {
    current_speed = max(current_speed - 2, target_speed);
}
```

### 2. Safety Monitoring
```cpp
// Overcurrent/overheat check
if (motor.current_ma > SAFETY_CURRENT || motor.temperature > MAX_TEMP) {
    motor.safety_mode = true;
    Serial.println("SAFETY MODE ACTIVATED");
}
```

### 3. Direction Change Logic
```cpp
// Safe direction change
motor.enabled = false;    // Stop motor
delay(500);               // Wait 500ms
motor.direction = !motor.direction;  // Change direction
motor.enabled = true;     // Restart
```

### 4. EEPROM Settings Storage
```cpp
// Automatic settings save (every 30 seconds)
EEPROM.write(EEPROM_SPEED, motor.speed);
EEPROM.write(EEPROM_DIRECTION, motor.direction);
EEPROM.write(EEPROM_AUTO_MODE, motor.auto_mode);
```

---

## 📊 Performance Specifications

### System Performance
- **Control Frequency**: 50Hz (20ms interval)
- **Speed Resolution**: 256 steps (0-255 PWM)
- **Safety Check Period**: 1 second
- **Status Report**: Every 5 seconds
- **Settings Save**: Every 30 seconds

### Motor Performance
- **Speed Range**: 0-100% (smooth control)
- **Direction**: Bidirectional (CW/CCW)
- **Acceleration/Deceleration**: Gradual (impact prevention)
- **Precision**: ±2% (PWM-based)

### Communication Performance
- **Serial Communication**: 115200 bps
- **WiFi Response**: <500ms average
- **WebSocket Latency**: <50ms
- **HTTP API**: 15-25 requests/second
- **Concurrent Users**: Up to 4 simultaneous

---

## 🛡️ Safety Features

### Multi-layer Protection System
1. **Overcurrent Protection**: Immediate stop above 1.5A
2. **Overheat Protection**: Deceleration and warning above 55°C
3. **Voltage Monitoring**: Low voltage detection and warning
4. **Safety Mode**: Automatic response to dangerous situations
5. **Emergency Stop**: Immediate stop via serial command or hardware button

### Electrical Safety
- **Insulation**: All connections insulation verified
- **Polarity Protection**: Power connection polarity marking
- **Overload Prevention**: Operation within motor specifications
- **Short Circuit Protection**: Short circuit check before connection

---

## 🔍 Troubleshooting

### Common Issues

**Motor doesn't spin:**
1. Check 12V adapter connection and polarity
2. Verify PWM signal (D3) connection
3. Confirm `motor.enabled = true` setting
4. Test with serial command `START`

**Speed control not working:**
1. Check potentiometer A0 connection
2. Verify 5V power supply
3. Validate PWM mapping logic
4. Test with serial command `SPEED:50`

**Direction doesn't change:**
1. Check D12 signal line connection
2. Verify OUT1, OUT2 terminals
3. Debug direction logic
4. Test with serial command `DIR`

**WiFi connection fails:**
1. Verify 2.4GHz WiFi (5GHz not supported)
2. Check password accuracy
3. Confirm router distance (within 30m)
4. Restart ESP32

---

## 📈 Advanced Features

### 1. PWM Frequency Optimization
```cpp
// High-frequency PWM for smoother control
setPWMFrequency(3, 1);  // 31.25kHz
```

### 2. Performance Monitoring
```cpp
// Real-time performance data
Serial.print("Performance: Speed=");
Serial.print(map(motor.speed, 0, MAX_SPEED, 0, 100));
Serial.print("%, Current=");
Serial.println(motor.current_ma);
```

### 3. Predictive Safety Control
```cpp
// Temperature-based predictive control
if (motor.temperature > TEMP_THRESHOLD - 10) {
    // Pre-decelerate when approaching threshold
    target_speed = motor.speed * 0.8;
}
```

---

## 🔄 Expansion Plans

### Hardware Expansion
- [ ] **Encoder Addition**: Accurate RPM feedback
- [ ] **Temperature Sensor**: Actual motor temperature measurement
- [ ] **Vibration Sensor**: Imbalance detection
- [ ] **Remote Control**: ESP32 WiFi control

### Software Expansion  
- [ ] **PID Control**: Precise speed control
- [ ] **Data Logging**: Operation record to SD card
- [ ] **Web Interface**: Browser monitoring
- [ ] **Mobile App**: Smartphone control

---

## 🤝 Contributing

This project is open source. Contributions are welcome!

1. Fork this repository
2. Create a new feature branch (`git checkout -b feature/new-feature`)
3. Commit your changes (`git commit -am 'Add new feature'`)
4. Push to the branch (`git push origin feature/new-feature`)
5. Create a Pull Request

### Development Guidelines
- **Safety First**: All changes require safety review
- **Testing**: Submit code after hardware testing
- **Documentation**: Include documentation updates for new features
- **Compatibility**: Maintain Arduino IDE compatibility

---

## 📄 License

This project is distributed under the MIT License. See [LICENSE](LICENSE) file for details.

---

## 📞 Support and Contact

- **GitHub Issues**: [Project Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)
- **Discussions**: [Community Discussions](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)
- **Documentation**: Refer to this README file and code comments

---

## 🙏 Acknowledgments

- **Arduino Community**: Open source hardware platform
- **ESP32 Ecosystem**: Powerful IoT development platform
- **L298P Chip**: Reliable motor driver
- **All Contributors**: Everyone who participated in project improvement

---

## 🌟 Project Structure

```
Arduino-Daily-Project-Upload/
├── 📁 src/
│   ├── 📁 arduino/          # Arduino source codes
│   │   └── Smart_Fan_Controller.ino
│   ├── 📁 esp32/            # ESP32 source codes
│   │   └── ESP32_WiFi_Controller.ino
│   └── 📁 examples/         # Example codes
├── 📁 hardware/
│   ├── 📁 3d-models/        # 3D printing files
│   └── 📁 pcb/              # PCB design files
├── 📁 docs/
│   ├── 📁 images/           # Documentation images
│   └── 📁 api/              # API documentation
├── 📄 README.md             # Korean documentation
├── 📄 README_EN.md          # English documentation
├── 📄 BEGINNER_GUIDE.md     # Beginner's complete guide
├── 📄 HANDS_ON_TUTORIAL.md  # 7-day hands-on course
├── 📄 CIRCUIT_DIAGRAMS.md   # Detailed circuit diagrams
├── 📄 API_DOCUMENTATION.md  # Complete API reference
├── 📄 PERFORMANCE_BENCHMARKS.md # Performance analysis
├── 📄 PARTS_GUIDE.md        # Parts purchasing guide
├── 📄 QUICK_START_GUIDE.md  # 10-minute quick start
├── 📄 TROUBLESHOOTING.md    # Troubleshooting guide
└── 📄 PROJECT_ROADMAP.md    # Development roadmap
```

---

<div align="center">

**🔧 Arduino Daily Project - Start of Safe and Smart Motor Control! 🔧**

![Made with Arduino](https://img.shields.io/badge/Made%20with-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Safety First](https://img.shields.io/badge/Safety-First-green?style=for-the-badge&logo=shield&logoColor=white)
![Open Source](https://img.shields.io/badge/Open-Source-blue?style=for-the-badge&logo=opensource&logoColor=white)

⭐ **If this project helped you, please give it a star!** ⭐

</div>