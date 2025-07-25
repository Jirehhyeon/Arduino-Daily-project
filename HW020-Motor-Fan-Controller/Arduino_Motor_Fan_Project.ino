/*
 * Arduino HW-020 Motor Shield + 12V Fan Motor Project
 * 점퍼선을 통한 안전한 모터 제어 시스템
 * 
 * Hardware:
 * - Arduino Uno R3
 * - HW-020 Motor Driver Shield (L298P 기반)
 * - 12V DC Fan Motor (Small propeller compatible)
 * - 12V External Power Supply (2A recommended)
 * - Jumper wires (Male to Female, 20AWG)
 * - Potentiometer 10kΩ (Speed control)
 * - Button (Direction control)
 * - LED indicators (2x)
 * 
 * 전류 계산:
 * - 소형 12V 선풍기 모터: 일반적으로 0.3-0.8A
 * - 점퍼선 허용 전류: 20AWG = 최대 11A (안전 마진 고려 시 2A)
 * - HW-020 쉴드 최대 전류: 2A per channel
 * - 안전성: 충분함 (모터 전류 << 점퍼선/쉴드 허용 전류)
 * 
 * Pin Configuration:
 * - Motor A: Pin 3 (PWM), Pin 12 (Direction)
 * - Motor B: Pin 11 (PWM), Pin 13 (Direction) 
 * - Speed Control: A0 (Potentiometer)
 * - Direction Button: Pin 2 (Interrupt)
 * - LED Status: Pin 4 (Direction), Pin 5 (Power)
 * - Current Sense: A1 (Optional current monitoring)
 */

#include <EEPROM.h>

// 핀 정의
#define MOTOR_A_PWM     3     // Motor A PWM (ENA)
#define MOTOR_A_DIR     12    // Motor A Direction (IN1)
#define MOTOR_B_PWM     11    // Motor B PWM (ENB) - 예비용
#define MOTOR_B_DIR     13    // Motor B Direction (IN2) - 예비용

#define SPEED_POT       A0    // 속도 제어 포텐셔미터
#define CURRENT_SENSE   A1    // 전류 감지 (선택사항)
#define DIRECTION_BTN   2     // 방향 제어 버튼 (인터럽트)

#define LED_DIRECTION   4     // 방향 표시 LED
#define LED_POWER       5     // 전원 표시 LED
#define LED_WARNING     6     // 경고 LED (과전류/과열)

// 시스템 상수
#define MAX_SPEED       255   // 최대 PWM 값
#define MIN_SPEED       50    // 최소 작동 속도
#define SAFETY_CURRENT  1500  // 안전 전류 한계 (mA)
#define TEMP_THRESHOLD  60    // 온도 경고 임계값 (°C)
#define EEPROM_SPEED    0     // EEPROM 속도 저장 주소
#define EEPROM_DIRECTION 1    // EEPROM 방향 저장 주소

// 모터 상태 구조체
struct MotorState {
  int speed;              // 현재 속도 (0-255)
  bool direction;         // 방향 (true=CW, false=CCW)
  bool enabled;           // 모터 활성화 상태
  unsigned long runtime;  // 누적 운행 시간
  int temperature;        // 추정 온도
  int current_ma;         // 현재 전류 (mA)
  bool safety_mode;       // 안전 모드 활성화
};

// 전역 변수
MotorState motor = {0, true, false, 0, 25, 0, false};
volatile bool direction_changed = false;
unsigned long last_button_time = 0;
unsigned long last_temp_check = 0;
unsigned long motor_start_time = 0;
unsigned long last_speed_save = 0;

// 속도 램핑을 위한 변수
int target_speed = 0;
int current_speed = 0;
unsigned long last_ramp_time = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Motor Fan Controller v2.0");
  Serial.println("HW-020 Shield + 12V Fan Motor");
  Serial.println("================================");
  
  // 핀 모드 설정
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  
  pinMode(LED_DIRECTION, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  
  pinMode(DIRECTION_BTN, INPUT_PULLUP);
  pinMode(SPEED_POT, INPUT);
  pinMode(CURRENT_SENSE, INPUT);
  
  // 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(DIRECTION_BTN), onDirectionButton, FALLING);
  
  // EEPROM에서 설정 복원
  loadSettings();
  
  // 초기화 시퀀스
  initializationSequence();
  
  // 안전 검사
  performSafetyCheck();
  
  Serial.println("Setup complete. Motor ready!");
  Serial.println("Commands: START, STOP, SPEED:<0-100>, DIR, STATUS, SAVE");
  
  displayStatus();
}

void loop() {
  // 메인 제어 루프 (50Hz = 20ms)
  static unsigned long last_main_loop = 0;
  if (millis() - last_main_loop >= 20) {
    last_main_loop = millis();
    
    // 1. 속도 제어 업데이트
    updateSpeedControl();
    
    // 2. 모터 제어 적용
    applyMotorControl();
    
    // 3. 안전 모니터링
    monitorSafety();
    
    // 4. LED 상태 업데이트
    updateLEDs();
    
    // 5. 방향 변경 처리
    handleDirectionChange();
  }
  
  // 시리얼 명령 처리
  handleSerialCommands();
  
  // 주기적 상태 리포트 (5초마다)
  static unsigned long last_status_report = 0;
  if (millis() - last_status_report >= 5000) {
    last_status_report = millis();
    reportPerformance();
  }
  
  // 설정 자동 저장 (30초마다)
  if (millis() - last_speed_save >= 30000) {
    last_speed_save = millis();
    saveSettings();
  }
}

void updateSpeedControl() {
  // 포텐셔미터에서 속도 읽기
  int pot_value = analogRead(SPEED_POT);
  target_speed = map(pot_value, 0, 1023, 0, MAX_SPEED);
  
  // 최소 속도 적용
  if (target_speed > 0 && target_speed < MIN_SPEED) {
    target_speed = MIN_SPEED;
  }
  
  // 부드러운 속도 변화 (램핑)
  if (millis() - last_ramp_time >= 10) { // 10ms마다 업데이트
    last_ramp_time = millis();
    
    if (current_speed < target_speed) {
      current_speed = min(current_speed + 2, target_speed);
    } else if (current_speed > target_speed) {
      current_speed = max(current_speed - 2, target_speed);
    }
    
    motor.speed = current_speed;
  }
}

void applyMotorControl() {
  if (motor.enabled && !motor.safety_mode) {
    // PWM 출력
    analogWrite(MOTOR_A_PWM, motor.speed);
    
    // 방향 제어
    digitalWrite(MOTOR_A_DIR, motor.direction ? HIGH : LOW);
    
    // 운행 시간 누적
    if (motor.speed > 0) {
      if (motor_start_time == 0) {
        motor_start_time = millis();
      }
    } else {
      if (motor_start_time > 0) {
        motor.runtime += (millis() - motor_start_time) / 1000;
        motor_start_time = 0;
      }
    }
  } else {
    // 모터 정지
    analogWrite(MOTOR_A_PWM, 0);
    digitalWrite(MOTOR_A_DIR, LOW);
    
    if (motor_start_time > 0) {
      motor.runtime += (millis() - motor_start_time) / 1000;
      motor_start_time = 0;
    }
  }
}

void monitorSafety() {
  // 전류 감지 (선택사항)
  if (millis() - last_temp_check >= 1000) { // 1초마다 체크
    last_temp_check = millis();
    
    // 전류 측정 (전류 센서가 있는 경우)
    int current_reading = analogRead(CURRENT_SENSE);
    motor.current_ma = map(current_reading, 0, 1023, 0, 3000); // 0-3A 범위
    
    // 온도 추정 (운행 시간과 부하 기반)
    if (motor.speed > 0) {
      float load_factor = motor.speed / 255.0;
      motor.temperature = 25 + (load_factor * 20) + (motor.runtime * 0.01);
    } else {
      motor.temperature = max(25, motor.temperature - 1); // 서서히 냉각
    }
    
    // 안전 검사
    bool safety_issue = false;
    
    // 과전류 검사
    if (motor.current_ma > SAFETY_CURRENT) {
      Serial.println("WARNING: High current detected!");
      safety_issue = true;
    }
    
    // 과열 검사
    if (motor.temperature > TEMP_THRESHOLD) {
      Serial.println("WARNING: High temperature detected!");
      safety_issue = true;
    }
    
    // 안전 모드 활성화/비활성화
    if (safety_issue && !motor.safety_mode) {
      motor.safety_mode = true;
      Serial.println("SAFETY MODE ACTIVATED - Motor stopped");
    } else if (!safety_issue && motor.safety_mode) {
      motor.safety_mode = false;
      Serial.println("Safety mode deactivated");
    }
  }
}

void updateLEDs() {
  // 전원 LED (모터 활성화 상태)
  digitalWrite(LED_POWER, motor.enabled && motor.speed > 0);
  
  // 방향 LED (깜빡임으로 방향 표시)
  static unsigned long led_blink_time = 0;
  static bool led_state = false;
  
  if (motor.enabled && motor.speed > 0) {
    // 방향에 따라 깜빡임 속도 변경
    int blink_interval = motor.direction ? 500 : 200; // CW: 느림, CCW: 빠름
    
    if (millis() - led_blink_time >= blink_interval) {
      led_blink_time = millis();
      led_state = !led_state;
      digitalWrite(LED_DIRECTION, led_state);
    }
  } else {
    digitalWrite(LED_DIRECTION, LOW);
  }
  
  // 경고 LED (안전 모드 또는 고온/과전류)
  bool warning_condition = motor.safety_mode || 
                          motor.temperature > TEMP_THRESHOLD || 
                          motor.current_ma > SAFETY_CURRENT;
  
  if (warning_condition) {
    // 빠른 깜빡임
    static unsigned long warning_blink = 0;
    static bool warning_state = false;
    
    if (millis() - warning_blink >= 100) {
      warning_blink = millis();
      warning_state = !warning_state;
      digitalWrite(LED_WARNING, warning_state);
    }
  } else {
    digitalWrite(LED_WARNING, LOW);
  }
}

void handleDirectionChange() {
  if (direction_changed) {
    direction_changed = false;
    
    // 디바운싱 체크
    if (millis() - last_button_time > 500) {
      last_button_time = millis();
      
      // 방향 변경 시 모터를 잠시 정지
      motor.enabled = false;
      analogWrite(MOTOR_A_PWM, 0);
      delay(200); // 200ms 정지
      
      // 방향 토글
      motor.direction = !motor.direction;
      motor.enabled = true;
      
      Serial.print("Direction changed to: ");
      Serial.println(motor.direction ? "Clockwise" : "Counter-clockwise");
      
      // 설정 저장
      EEPROM.write(EEPROM_DIRECTION, motor.direction);
    }
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command == "START") {
      motor.enabled = true;
      Serial.println("Motor started");
      
    } else if (command == "STOP") {
      motor.enabled = false;
      motor.speed = 0;
      current_speed = 0;
      target_speed = 0;
      Serial.println("Motor stopped");
      
    } else if (command.startsWith("SPEED:")) {
      int speed_percent = command.substring(6).toInt();
      speed_percent = constrain(speed_percent, 0, 100);
      target_speed = map(speed_percent, 0, 100, 0, MAX_SPEED);
      Serial.print("Target speed set to: ");
      Serial.print(speed_percent);
      Serial.println("%");
      
    } else if (command == "DIR") {
      motor.direction = !motor.direction;
      Serial.print("Direction: ");
      Serial.println(motor.direction ? "CW" : "CCW");
      
    } else if (command == "STATUS") {
      displayStatus();
      
    } else if (command == "SAVE") {
      saveSettings();
      Serial.println("Settings saved to EEPROM");
      
    } else if (command == "RESET") {
      resetMotor();
      Serial.println("Motor reset to default state");
      
    } else if (command == "HELP") {
      printHelp();
      
    } else {
      Serial.println("Unknown command. Type HELP for available commands.");
    }
  }
}

void onDirectionButton() {
  direction_changed = true;
}

void initializationSequence() {
  Serial.println("Performing initialization sequence...");
  
  // LED 테스트
  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_DIRECTION, HIGH);
  digitalWrite(LED_WARNING, HIGH);
  delay(500);
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_DIRECTION, LOW);
  digitalWrite(LED_WARNING, LOW);
  
  // 모터 시스템 테스트 (저속으로)
  Serial.println("Motor system test...");
  analogWrite(MOTOR_A_PWM, 80); // 낮은 속도로 테스트
  digitalWrite(MOTOR_A_DIR, HIGH);
  delay(1000);
  
  analogWrite(MOTOR_A_PWM, 0);
  digitalWrite(MOTOR_A_DIR, LOW);
  delay(500);
  
  Serial.println("Initialization complete!");
}

void performSafetyCheck() {
  Serial.println("Performing safety check...");
  
  // 전원 전압 체크 (아날로그 기준전압 사용)
  long vcc = readVcc();
  Serial.print("VCC: ");
  Serial.print(vcc);
  Serial.println("mV");
  
  if (vcc < 4500) { // 4.5V 미만
    Serial.println("WARNING: Low VCC detected!");
  }
  
  // 초기 전류 체크
  int initial_current = analogRead(CURRENT_SENSE);
  Serial.print("Initial current reading: ");
  Serial.println(initial_current);
  
  if (initial_current > 100) { // 높은 초기 전류
    Serial.println("WARNING: High initial current!");
    motor.safety_mode = true;
  }
  
  Serial.println("Safety check complete.");
}

void loadSettings() {
  // EEPROM에서 설정 읽기
  int saved_speed = EEPROM.read(EEPROM_SPEED);
  if (saved_speed <= MAX_SPEED) {
    target_speed = saved_speed;
    current_speed = saved_speed;
    motor.speed = saved_speed;
  }
  
  motor.direction = EEPROM.read(EEPROM_DIRECTION);
  
  Serial.print("Loaded settings - Speed: ");
  Serial.print(motor.speed);
  Serial.print(", Direction: ");
  Serial.println(motor.direction ? "CW" : "CCW");
}

void saveSettings() {
  EEPROM.write(EEPROM_SPEED, motor.speed);
  EEPROM.write(EEPROM_DIRECTION, motor.direction);
}

void resetMotor() {
  motor.speed = 0;
  motor.direction = true;
  motor.enabled = false;
  motor.safety_mode = false;
  current_speed = 0;
  target_speed = 0;
  
  analogWrite(MOTOR_A_PWM, 0);
  digitalWrite(MOTOR_A_DIR, LOW);
}

void displayStatus() {
  Serial.println("\n=== MOTOR STATUS ===");
  Serial.print("Enabled: ");
  Serial.println(motor.enabled ? "YES" : "NO");
  Serial.print("Speed: ");
  Serial.print(map(motor.speed, 0, MAX_SPEED, 0, 100));
  Serial.println("%");
  Serial.print("Direction: ");
  Serial.println(motor.direction ? "Clockwise" : "Counter-clockwise");
  Serial.print("Runtime: ");
  Serial.print(motor.runtime);
  Serial.println(" seconds");
  Serial.print("Temperature: ");
  Serial.print(motor.temperature);
  Serial.println("°C");
  Serial.print("Current: ");
  Serial.print(motor.current_ma);
  Serial.println("mA");
  Serial.print("Safety Mode: ");
  Serial.println(motor.safety_mode ? "ACTIVE" : "Normal");
  
  // 포텐셔미터 값
  int pot_reading = analogRead(SPEED_POT);
  Serial.print("Pot Reading: ");
  Serial.print(pot_reading);
  Serial.print(" (");
  Serial.print(map(pot_reading, 0, 1023, 0, 100));
  Serial.println("%)");
  
  Serial.println("==================");
}

void reportPerformance() {
  // 간단한 성능 리포트
  Serial.print("Performance: Speed=");
  Serial.print(map(motor.speed, 0, MAX_SPEED, 0, 100));
  Serial.print("%, Temp=");
  Serial.print(motor.temperature);
  Serial.print("°C, Current=");
  Serial.print(motor.current_ma);
  Serial.print("mA, Runtime=");
  Serial.print(motor.runtime);
  Serial.println("s");
}

void printHelp() {
  Serial.println("\n=== AVAILABLE COMMANDS ===");
  Serial.println("START       - Enable motor");
  Serial.println("STOP        - Disable motor");
  Serial.println("SPEED:<0-100> - Set speed percentage");
  Serial.println("DIR         - Toggle direction");
  Serial.println("STATUS      - Show detailed status");
  Serial.println("SAVE        - Save settings to EEPROM");
  Serial.println("RESET       - Reset to default state");
  Serial.println("HELP        - Show this help");
  Serial.println("\nHardware Controls:");
  Serial.println("Potentiometer - Speed control");
  Serial.println("Button        - Direction toggle");
  Serial.println("=========================");
}

// VCC 전압 측정 함수
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

// 전류 계산 함수 (ACS712 센서 사용 시)
float calculateCurrent(int analogValue) {
  // ACS712-05B: 185mV/A, 2.5V zero current
  float voltage = (analogValue * 5.0) / 1024.0;
  float current = (voltage - 2.5) / 0.185;
  return abs(current);
}

// PWM 주파수 변경 함수 (고급 제어용)
void setPWMFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}