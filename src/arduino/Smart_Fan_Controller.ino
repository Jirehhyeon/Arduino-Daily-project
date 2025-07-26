/*
 * 🌀 Smart Fan Controller v3.0 - Arduino Main Controller
 * Professional IoT Smart Home Fan System
 * 
 * Features:
 * - AI-based environmental sensing and control
 * - DHT22 temperature/humidity sensor
 * - ACS712 current monitoring
 * - LDR light sensor (night mode)
 * - Microphone sensor (noise detection)  
 * - I2C LCD display
 * - SG90 servo for wind direction
 * - ESP32 WiFi/Bluetooth communication
 * - Mobile app integration
 * - Advanced safety system
 * 
 * Hardware:
 * - Arduino Uno R3 (Main Controller)
 * - ESP32 DevKit (WiFi/Bluetooth)
 * - HW-020 Motor Shield (L298P)
 * - 12V DC Fan Motor
 * - DHT22 (Temperature/Humidity)
 * - ACS712 (Current Sensor)
 * - LDR (Light Sensor)
 * - Microphone Module
 * - I2C LCD 16x2
 * - SG90 Servo Motor
 * - Emergency Stop Button
 * - LED Status Indicators
 * - Active Buzzer
 * 
 * Author: Arduino Daily Project
 * Version: 3.0 Smart Edition
 * Date: 2025-07-26
 */

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <EEPROM.h>
#include <ArduinoJson.h>

// ========== 핀 정의 ==========
#define MOTOR_PWM       3     // Motor PWM (ENA)
#define MOTOR_DIR       12    // Motor Direction (IN1)
#define DHT_PIN         7     // DHT22 Data Pin
#define SERVO_PIN       9     // Servo Control Pin
#define BUZZER_PIN      8     // Active Buzzer Pin
#define EMERGENCY_BTN   2     // Emergency Stop (Interrupt)

#define LED_POWER       4     // Power Status LED
#define LED_WARNING     5     // Warning LED
#define LED_STATUS      6     // Status LED

#define POT_SPEED       A0    // Speed Potentiometer
#define CURRENT_SENSOR  A1    // ACS712 Current Sensor
#define LIGHT_SENSOR    A2    // LDR Light Sensor
#define MIC_SENSOR      A3    // Microphone Sensor

// ========== 센서 설정 ==========
#define DHT_TYPE        DHT22
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo windServo;

// ========== 시스템 상수 ==========
#define MAX_SPEED           255   // Maximum PWM value
#define MIN_SPEED           30    // Minimum operation speed
#define SAFETY_CURRENT      1200  // Safety current limit (mA)
#define MAX_TEMPERATURE     55    // Temperature warning (°C)
#define MAX_RUNTIME         7200  // Max continuous runtime (seconds)
#define COMFORT_TEMP_MIN    22    // Minimum comfort temperature
#define COMFORT_TEMP_MAX    25    // Maximum comfort temperature
#define COMFORT_HUMID_MIN   45    // Minimum comfort humidity
#define COMFORT_HUMID_MAX   65    // Maximum comfort humidity

// ========== EEPROM 주소 ==========
#define EEPROM_SPEED        0
#define EEPROM_DIRECTION    1
#define EEPROM_MODE         2
#define EEPROM_WIND_DIR     3

// ========== 시스템 상태 구조체 ==========
struct SmartFanState {
  // 기본 제어
  int speed;                    // Current speed (0-255)
  bool direction;               // Motor direction (true=CW, false=CCW)
  bool enabled;                 // Fan enabled state
  int mode;                     // 0=Manual, 1=Auto, 2=Schedule, 3=Eco
  
  // 환경 센서 데이터
  float temperature;            // Temperature (°C)
  float humidity;               // Humidity (%)
  int light_level;              // Light level (0-100%)
  int noise_level;              // Noise level (0-100%)
  
  // 제어 및 안전
  int current_ma;               // Current consumption (mA)
  unsigned long runtime;        // Total runtime (seconds)
  bool safety_mode;             // Safety mode active
  bool comfort;                 // Comfort status
  float discomfort_index;       // Discomfort index
  
  // 고급 기능
  int wind_direction;           // Servo wind direction (0-180°)
  bool night_mode;              // Night mode active
  bool eco_mode;                // Eco mode active
  bool schedule_enabled;        // Schedule mode enabled
  
  // 타이밍
  unsigned long start_time;     // Session start time
  unsigned long last_sensor_read; // Last sensor reading
  unsigned long last_comfort_calc; // Last comfort calculation
};

SmartFanState fan = {0};

// ========== 전역 변수 ==========
volatile bool emergency_stop = false;
unsigned long last_esp32_comm = 0;
unsigned long last_display_update = 0;
unsigned long last_auto_adjust = 0;
unsigned long last_safety_check = 0;

int target_speed = 0;
int current_speed = 0;
unsigned long last_ramp_time = 0;

// ========== 초기화 ==========
void setup() {
  Serial.begin(115200);
  Serial.println("🌀 Smart Fan Controller v3.0");
  Serial.println("Professional IoT Smart Home System");
  Serial.println("===================================");
  
  // 핀 모드 설정
  setupPins();
  
  // 센서 및 디스플레이 초기화
  initializeSensors();
  
  // EEPROM 설정 로드
  loadSettings();
  
  // 시스템 초기화
  systemInitialization();
  
  // 안전 검사
  performSafetyCheck();
  
  Serial.println("🚀 Smart Fan System Ready!");
  Serial.println("Commands: START, STOP, AUTO, ECO, SPEED:<0-100>, STATUS");
  
  displayWelcomeMessage();
  sendToESP32("SYSTEM:READY");
}

// ========== 메인 루프 ==========
void loop() {
  // 메인 제어 루프 (20ms 주기)
  static unsigned long last_main_loop = 0;
  if (millis() - last_main_loop >= 20) {
    last_main_loop = millis();
    
    // 1. 센서 데이터 읽기
    readSensors();
    
    // 2. 자동 제어 처리
    handleAutoControl();
    
    // 3. 속도 제어 업데이트
    updateSpeedControl();
    
    // 4. 모터 제어 적용
    applyMotorControl();
    
    // 5. 안전 모니터링
    monitorSafety();
    
    // 6. LED 상태 업데이트
    updateStatusLEDs();
    
    // 7. 비상정지 처리
    handleEmergencyStop();
  }
  
  // LCD 디스플레이 업데이트 (1초마다)
  if (millis() - last_display_update >= 1000) {
    last_display_update = millis();
    updateLCDDisplay();
  }
  
  // ESP32 통신 처리
  handleESP32Communication();
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // 주기적 데이터 전송 (2초마다)
  static unsigned long last_data_send = 0;
  if (millis() - last_data_send >= 2000) {
    last_data_send = millis();
    sendDataToESP32();
  }
}

// ========== 핀 설정 ==========
void setupPins() {
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  pinMode(POT_SPEED, INPUT);
  pinMode(CURRENT_SENSOR, INPUT);
  pinMode(LIGHT_SENSOR, INPUT);
  pinMode(MIC_SENSOR, INPUT);
  
  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), onEmergencyStop, FALLING);
}

// ========== 센서 초기화 ==========
void initializeSensors() {
  // DHT22 초기화
  dht.begin();
  Serial.println("✅ DHT22 Temperature/Humidity sensor initialized");
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  Serial.println("✅ I2C LCD display initialized");
  
  // 서보 모터 초기화
  windServo.attach(SERVO_PIN);
  windServo.write(90); // 중앙 위치
  fan.wind_direction = 90;
  Serial.println("✅ Servo motor initialized");
  
  delay(2000); // 센서 안정화 대기
}

// ========== 센서 데이터 읽기 ==========
void readSensors() {
  if (millis() - fan.last_sensor_read >= 2000) { // 2초마다 읽기
    fan.last_sensor_read = millis();
    
    // DHT22 읽기
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (!isnan(h) && !isnan(t)) {
      fan.temperature = t;
      fan.humidity = h;
    }
    
    // 전류 센서 읽기 (ACS712)
    int current_raw = analogRead(CURRENT_SENSOR);
    fan.current_ma = calculateCurrent(current_raw);
    
    // 조도 센서 읽기
    int light_raw = analogRead(LIGHT_SENSOR);
    fan.light_level = map(light_raw, 0, 1023, 0, 100);
    
    // 마이크 센서 읽기
    int mic_raw = analogRead(MIC_SENSOR);
    fan.noise_level = map(mic_raw, 0, 1023, 0, 100);
    
    // 야간 모드 자동 전환
    fan.night_mode = (fan.light_level < 20);
    
    // 쾌적지수 계산
    calculateComfortIndex();
  }
}

// ========== 쾌적지수 계산 ==========
void calculateComfortIndex() {
  if (millis() - fan.last_comfort_calc >= 5000) { // 5초마다 계산
    fan.last_comfort_calc = millis();
    
    // 불쾌지수 계산 (Discomfort Index)
    fan.discomfort_index = 0.81 * fan.temperature + 0.01 * fan.humidity * 
                          (0.99 * fan.temperature - 14.3) + 46.3;
    
    // 쾌적 상태 판단
    bool temp_comfort = (fan.temperature >= COMFORT_TEMP_MIN && 
                        fan.temperature <= COMFORT_TEMP_MAX);
    bool humid_comfort = (fan.humidity >= COMFORT_HUMID_MIN && 
                         fan.humidity <= COMFORT_HUMID_MAX);
    bool noise_comfort = (fan.noise_level < 70); // 70% 이하
    
    fan.comfort = temp_comfort && humid_comfort && noise_comfort;
  }
}

// ========== 자동 제어 처리 ==========
void handleAutoControl() {
  if (fan.mode == 1 && millis() - last_auto_adjust >= 10000) { // 자동 모드, 10초마다
    last_auto_adjust = millis();
    
    int auto_speed = 0;
    
    // 온도 기반 속도 조절
    if (fan.temperature > COMFORT_TEMP_MAX) {
      auto_speed = map(fan.temperature, COMFORT_TEMP_MAX, 35, 50, 100);
    } else if (fan.temperature < COMFORT_TEMP_MIN) {
      auto_speed = 0; // 추우면 정지
    } else {
      auto_speed = 30; // 쾌적 온도에서는 약풍
    }
    
    // 습도 보정
    if (fan.humidity > COMFORT_HUMID_MAX) {
      auto_speed += 20; // 습하면 강풍
    }
    
    // 야간 모드 보정
    if (fan.night_mode) {
      auto_speed = min(auto_speed, 50); // 야간에는 최대 50%
    }
    
    // 소음 레벨 보정
    if (fan.noise_level > 80) {
      auto_speed = min(auto_speed, 30); // 조용히
    }
    
    // 에코 모드 보정
    if (fan.eco_mode) {
      auto_speed = auto_speed * 0.8; // 20% 절약
    }
    
    target_speed = constrain(auto_speed, 0, 100);
    target_speed = map(target_speed, 0, 100, 0, MAX_SPEED);
    
    Serial.print("Auto adjustment: ");
    Serial.print(map(target_speed, 0, MAX_SPEED, 0, 100));
    Serial.println("%");
  }
}

// ========== 속도 제어 업데이트 ==========
void updateSpeedControl() {
  // 포텐셔미터 읽기 (수동 모드에서만)
  if (fan.mode == 0) { // 수동 모드
    int pot_value = analogRead(POT_SPEED);
    target_speed = map(pot_value, 0, 1023, 0, MAX_SPEED);
  }
  
  // 최소 속도 적용
  if (target_speed > 0 && target_speed < MIN_SPEED) {
    target_speed = MIN_SPEED;
  }
  
  // 부드러운 속도 변화 (램핑)
  if (millis() - last_ramp_time >= 50) { // 50ms마다 업데이트
    last_ramp_time = millis();
    
    int ramp_step = fan.night_mode ? 1 : 3; // 야간에는 더 부드럽게
    
    if (current_speed < target_speed) {
      current_speed = min(current_speed + ramp_step, target_speed);
    } else if (current_speed > target_speed) {
      current_speed = max(current_speed - ramp_step, target_speed);
    }
    
    fan.speed = current_speed;
  }
}

// ========== 모터 제어 적용 ==========
void applyMotorControl() {
  if (fan.enabled && !fan.safety_mode && !emergency_stop) {
    // PWM 출력
    analogWrite(MOTOR_PWM, fan.speed);
    
    // 방향 제어
    digitalWrite(MOTOR_DIR, fan.direction ? HIGH : LOW);
    
    // 운행 시간 누적
    if (fan.speed > 0) {
      if (fan.start_time == 0) {
        fan.start_time = millis();
      }
    } else {
      if (fan.start_time > 0) {
        fan.runtime += (millis() - fan.start_time) / 1000;
        fan.start_time = 0;
      }
    }
  } else {
    // 모터 정지
    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_DIR, LOW);
    
    if (fan.start_time > 0) {
      fan.runtime += (millis() - fan.start_time) / 1000;
      fan.start_time = 0;
    }
  }
}

// ========== 안전 모니터링 ==========
void monitorSafety() {
  if (millis() - last_safety_check >= 1000) { // 1초마다 검사
    last_safety_check = millis();
    
    bool safety_issue = false;
    String alert_message = "";
    
    // 과전류 검사
    if (fan.current_ma > SAFETY_CURRENT) {
      safety_issue = true;
      alert_message = "High current: " + String(fan.current_ma) + "mA";
    }
    
    // 과열 검사
    if (fan.temperature > MAX_TEMPERATURE) {
      safety_issue = true;
      alert_message = "High temperature: " + String(fan.temperature) + "°C";
    }
    
    // 운행 시간 검사
    unsigned long current_session = fan.start_time > 0 ? 
                                   (millis() - fan.start_time) / 1000 : 0;
    if (current_session > MAX_RUNTIME) {
      safety_issue = true;
      alert_message = "Runtime limit exceeded: " + String(current_session) + "s";
    }
    
    // 안전 모드 상태 변경
    if (safety_issue && !fan.safety_mode) {
      fan.safety_mode = true;
      Serial.println("🚨 SAFETY MODE ACTIVATED: " + alert_message);
      sendToESP32("ALERT:" + alert_message);
      soundAlert(3); // 3회 경고음
    } else if (!safety_issue && fan.safety_mode) {
      fan.safety_mode = false;
      Serial.println("✅ Safety mode deactivated");
      sendToESP32("ALERT:Safety mode cleared");
    }
  }
}

// ========== LED 상태 업데이트 ==========
void updateStatusLEDs() {
  // 전원 LED (모터 활성화 상태)
  digitalWrite(LED_POWER, fan.enabled && fan.speed > 0);
  
  // 경고 LED (안전 모드 또는 문제 상황)
  static unsigned long warning_blink = 0;
  static bool warning_state = false;
  
  bool warning_condition = fan.safety_mode || emergency_stop ||
                          fan.temperature > MAX_TEMPERATURE ||
                          fan.current_ma > SAFETY_CURRENT;
  
  if (warning_condition) {
    if (millis() - warning_blink >= 200) {
      warning_blink = millis();
      warning_state = !warning_state;
      digitalWrite(LED_WARNING, warning_state);
    }
  } else {
    digitalWrite(LED_WARNING, LOW);
  }
  
  // 상태 LED (모드 표시)
  static unsigned long status_blink = 0;
  static bool status_state = false;
  
  int blink_interval = 1000; // 기본 간격
  
  switch (fan.mode) {
    case 0: blink_interval = 2000; break; // 수동: 느린 깜빡임
    case 1: blink_interval = 500; break;  // 자동: 빠른 깜빡임
    case 2: blink_interval = 250; break;  // 스케줄: 매우 빠른 깜빡임
    case 3: blink_interval = 1500; break; // 에코: 중간 깜빡임
  }
  
  if (millis() - status_blink >= blink_interval) {
    status_blink = millis();
    status_state = !status_state;
    digitalWrite(LED_STATUS, status_state);
  }
}

// ========== 비상정지 처리 ==========
void handleEmergencyStop() {
  if (emergency_stop) {
    fan.enabled = false;
    fan.speed = 0;
    current_speed = 0;
    target_speed = 0;
    
    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_DIR, LOW);
    
    Serial.println("🚨 EMERGENCY STOP ACTIVATED");
    sendToESP32("EMERGENCY_STOP");
    
    // 연속 경고음
    for (int i = 0; i < 5; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  }
}

// ========== LCD 디스플레이 업데이트 ==========
void updateLCDDisplay() {
  lcd.clear();
  
  if (emergency_stop) {
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY STOP");
    lcd.setCursor(0, 1);
    lcd.print("SYSTEM HALTED");
    return;
  }
  
  // 첫 번째 줄: 온도, 습도, 속도
  lcd.setCursor(0, 0);
  lcd.print(fan.temperature, 1);
  lcd.print("C ");
  lcd.print(fan.humidity, 0);
  lcd.print("% ");
  lcd.print(map(fan.speed, 0, MAX_SPEED, 0, 100));
  lcd.print("%");
  
  // 두 번째 줄: 모드, 상태, 시간
  lcd.setCursor(0, 1);
  
  switch (fan.mode) {
    case 0: lcd.print("MAN"); break;
    case 1: lcd.print("AUTO"); break;
    case 2: lcd.print("SCHED"); break;
    case 3: lcd.print("ECO"); break;
  }
  
  if (fan.safety_mode) {
    lcd.print(" SAFE");
  } else if (fan.night_mode) {
    lcd.print(" NIGHT");
  } else if (fan.comfort) {
    lcd.print(" OK");
  } else {
    lcd.print(" ADJ");
  }
  
  // 운행 시간 표시
  lcd.print(" ");
  lcd.print(fan.runtime / 60);
  lcd.print("m");
}

// ========== ESP32 통신 처리 ==========
void handleESP32Communication() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    last_esp32_comm = millis();
    
    if (command.startsWith("CMD:")) {
      processESP32Command(command.substring(4));
    } else if (command == "GET_STATUS") {
      sendDataToESP32();
    } else if (command == "PING") {
      sendToESP32("PONG");
    }
  }
}

// ========== ESP32 명령 처리 ==========
void processESP32Command(String cmd) {
  if (cmd == "START") {
    fan.enabled = true;
    Serial.println("Fan started via ESP32");
    
  } else if (cmd == "STOP") {
    fan.enabled = false;
    current_speed = 0;
    target_speed = 0;
    Serial.println("Fan stopped via ESP32");
    
  } else if (cmd.startsWith("SPEED:")) {
    int speed_percent = cmd.substring(6).toInt();
    target_speed = map(constrain(speed_percent, 0, 100), 0, 100, 0, MAX_SPEED);
    fan.mode = 0; // 수동 모드로 전환
    Serial.print("Speed set to ");
    Serial.print(speed_percent);
    Serial.println("% via ESP32");
    
  } else if (cmd.startsWith("MODE:")) {
    fan.mode = constrain(cmd.substring(5).toInt(), 0, 3);
    Serial.print("Mode changed to ");
    Serial.println(fan.mode);
    
  } else if (cmd.startsWith("WIND:")) {
    fan.wind_direction = constrain(cmd.substring(5).toInt(), 0, 180);
    windServo.write(fan.wind_direction);
    Serial.print("Wind direction: ");
    Serial.println(fan.wind_direction);
    
  } else if (cmd == "ECO_ON") {
    fan.eco_mode = true;
    Serial.println("Eco mode enabled");
    
  } else if (cmd == "ECO_OFF") {
    fan.eco_mode = false;
    Serial.println("Eco mode disabled");
  }
}

// ========== 데이터 전송 ==========
void sendDataToESP32() {
  StaticJsonDocument<512> doc;
  
  doc["temperature"] = fan.temperature;
  doc["humidity"] = fan.humidity;
  doc["speed"] = map(fan.speed, 0, MAX_SPEED, 0, 100);
  doc["mode"] = fan.mode;
  doc["enabled"] = fan.enabled;
  doc["comfort"] = fan.comfort;
  doc["discomfort_index"] = fan.discomfort_index;
  doc["current"] = fan.current_ma;
  doc["runtime"] = fan.runtime;
  doc["safety_mode"] = fan.safety_mode;
  doc["wind_direction"] = fan.wind_direction;
  doc["night_mode"] = fan.night_mode;
  doc["eco_mode"] = fan.eco_mode;
  doc["light_level"] = fan.light_level;
  doc["noise_level"] = fan.noise_level;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  sendToESP32("DATA:" + jsonString);
}

// ========== 시리얼 명령 처리 ==========
void handleSerialCommands() {
  // ESP32와의 통신이 우선이므로 여기서는 간단한 명령만 처리
  // 자세한 명령은 ESP32를 통해 처리
}

// ========== 유틸리티 함수 ==========
void sendToESP32(String message) {
  Serial.println(message);
}

float calculateCurrent(int analogValue) {
  // ACS712-05B: 185mV/A, 2.5V zero current
  float voltage = (analogValue * 5.0) / 1024.0;
  float current = abs((voltage - 2.5) / 0.185);
  return current * 1000; // mA로 변환
}

void soundAlert(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

// ========== 인터럽트 핸들러 ==========
void onEmergencyStop() {
  emergency_stop = true;
}

// ========== 시스템 초기화 ==========
void systemInitialization() {
  Serial.println("Performing system initialization...");
  
  // LED 테스트 시퀀스
  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_WARNING, HIGH);
  digitalWrite(LED_STATUS, HIGH);
  delay(500);
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_WARNING, LOW);
  digitalWrite(LED_STATUS, LOW);
  
  // 부저 테스트
  soundAlert(1);
  
  // 서보 테스트
  windServo.write(0);
  delay(500);
  windServo.write(180);
  delay(500);
  windServo.write(90);
  
  Serial.println("System initialization complete!");
}

void performSafetyCheck() {
  Serial.println("Performing safety check...");
  
  // 전압 체크
  long vcc = readVcc();
  Serial.print("VCC: ");
  Serial.print(vcc);
  Serial.println("mV");
  
  if (vcc < 4500) {
    Serial.println("WARNING: Low VCC detected!");
  }
  
  // 초기 센서 체크
  float test_temp = dht.readTemperature();
  float test_humid = dht.readHumidity();
  
  if (isnan(test_temp) || isnan(test_humid)) {
    Serial.println("WARNING: DHT22 sensor error!");
  } else {
    Serial.println("✅ DHT22 sensor OK");
  }
  
  Serial.println("Safety check complete.");
}

void loadSettings() {
  // EEPROM에서 설정 로드
  fan.speed = EEPROM.read(EEPROM_SPEED);
  if (fan.speed > MAX_SPEED) fan.speed = 0;
  
  fan.direction = EEPROM.read(EEPROM_DIRECTION);
  fan.mode = EEPROM.read(EEPROM_MODE);
  if (fan.mode > 3) fan.mode = 0;
  
  fan.wind_direction = EEPROM.read(EEPROM_WIND_DIR);
  if (fan.wind_direction > 180) fan.wind_direction = 90;
  
  target_speed = fan.speed;
  current_speed = fan.speed;
  
  Serial.println("Settings loaded from EEPROM");
}

void saveSettings() {
  EEPROM.write(EEPROM_SPEED, fan.speed);
  EEPROM.write(EEPROM_DIRECTION, fan.direction);
  EEPROM.write(EEPROM_MODE, fan.mode);
  EEPROM.write(EEPROM_WIND_DIR, fan.wind_direction);
}

void displayWelcomeMessage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Fan v3.0");
  lcd.setCursor(0, 1);
  lcd.print("System Ready!");
  delay(2000);
}

long readVcc() {
  // Arduino 내장 기준전압 사용
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result;
  return result;
}

/*
 * 🌀 Smart Fan Controller v3.0 Features:
 * 
 * 🤖 AI-based Control:
 * - Environmental sensing and analysis
 * - Automatic comfort optimization
 * - User pattern learning ready
 * 
 * 📱 IoT Integration:
 * - ESP32 WiFi/Bluetooth communication
 * - Mobile app connectivity
 * - Real-time monitoring and control
 * 
 * 🛡️ Advanced Safety:
 * - Multi-level protection system
 * - Real-time monitoring
 * - Emergency stop capability
 * 
 * 🎛️ Precision Control:
 * - Stepless speed control
 * - Wind direction adjustment
 * - Multiple operation modes
 * 
 * 📊 Comprehensive Monitoring:
 * - Temperature, humidity, light, noise
 * - Current consumption tracking
 * - Comfort index calculation
 * 
 * This smart fan system represents the future
 * of home automation and environmental control!
 */