# 🛠️ 실습 튜토리얼 - 단계별 조립 가이드

**손으로 직접 만들어보는 스마트 선풍기 제작 과정**

## 📚 학습 과정 개요

### 🎯 학습 목표
- Arduino와 ESP32를 활용한 IoT 시스템 구축 능력 습득
- 전자회로 설계 및 조립 경험
- 센서 데이터 처리 및 모터 제어 기술 학습
- 모바일 앱과 하드웨어 연동 시스템 이해

### ⏱️ 전체 소요 시간
- **1일차**: 기본 부품 준비 및 전원 회로 (2시간)
- **2일차**: Arduino 기본 프로그래밍 (3시간)
- **3일차**: 센서 연결 및 테스트 (3시간)
- **4일차**: 모터 제어 시스템 (4시간)
- **5일차**: ESP32 WiFi 연동 (3시간)
- **6일차**: 통합 테스트 및 디버깅 (2시간)
- **7일차**: 케이스 제작 및 마무리 (2시간)

---

## 📅 1일차: 부품 준비 및 기본 회로

### 🔍 1-1. 부품 검수 및 테스트 (30분)

**준비된 부품 체크리스트**:
```
□ Arduino Uno R3 (LED 점등 확인)
□ ESP32 DevKit v1 (USB 연결 확인)
□ HW-020 Motor Shield (핀 손상 없음)
□ 12V DC 모터 (축 회전 확인)
□ 12V 2A 어댑터 (출력 전압 측정)
□ DHT22 센서 (핀 번호 확인)
□ 브레드보드 2개 (830홀)
□ 점퍼선 세트 (M-M, M-F 각 20개)
□ 저항 세트 (220Ω × 5개, 10kΩ × 5개)
□ LED 세트 (빨강, 초록, 노랑 각 2개)
```

**부품 테스트 방법**:
```bash
1. Arduino 테스트:
   - USB 연결 → 전원 LED 확인
   - Blink 예제 업로드 → 내장 LED 깜빡임 확인

2. ESP32 테스트:
   - USB 연결 → 전원 LED 확인
   - WiFiScan 예제 업로드 → 시리얼 모니터에서 WiFi 목록 확인

3. 멀티미터 테스트:
   - 12V 어댑터 출력 전압 측정 (11.5V~12.5V 정상)
   - 저항값 측정 (220Ω ±5%, 10kΩ ±5% 정상)
```

### ⚡ 1-2. 전원 회로 구성 (60분)

**단계 1: HW-020 쉴드 장착**
```
조립 과정:
1. Arduino Uno의 모든 핀에 쉴드가 정확히 삽입되는지 확인
2. 쉴드의 핀이 구부러지지 않도록 천천히 삽입
3. 단단히 고정될 때까지 눌러서 장착

확인 방법:
✓ 쉴드가 Arduino에 평평하게 장착됨
✓ 모든 핀이 정확히 삽입됨
✓ 쉴드의 LED나 부품이 손상되지 않음
```

**단계 2: 전원 연결 및 테스트**
```
안전 수칙:
⚠️ 전원 연결 전 반드시 극성 확인
⚠️ 전압과 전류 규격 준수
⚠️ 연결 작업은 전원 OFF 상태에서 실시

연결 순서:
1. 12V 어댑터를 HW-020 전원 터미널에 연결
   - 빨간선(+) → +12V 터미널
   - 검은선(-) → GND 터미널

2. 전원 ON 후 LED 확인
   - HW-020 전원 LED 점등
   - Arduino 전원 LED 점등
   - 이상 시 즉시 전원 차단

3. 멀티미터로 전압 측정
   - Arduino VIN 핀: 12V ±0.5V
   - Arduino 5V 핀: 5V ±0.1V
   - Arduino 3.3V 핀: 3.3V ±0.1V
```

### 🔗 1-3. 기본 브레드보드 설정 (30분)

**브레드보드 전원 레일 구성**:
```
브레드보드 1 (센서용):
┌─────────────────────────────┐
│ + 레일: Arduino 5V 연결     │ ← 빨간 점퍼선
│ - 레일: Arduino GND 연결    │ ← 검은 점퍼선
└─────────────────────────────┘

브레드보드 2 (제어용):
┌─────────────────────────────┐
│ + 레일: Arduino 5V 연결     │ ← 빨간 점퍼선
│ - 레일: Arduino GND 연결    │ ← 검은 점퍼선
└─────────────────────────────┘

브레드보드 간 연결:
• 브레드보드1 GND ↔ 브레드보드2 GND (검은 점퍼선)
• 브레드보드1 5V ↔ 브레드보드2 5V (빨간 점퍼선)
```

**전원 테스트**:
```
멀티미터 측정:
1. 브레드보드1 + 레일: 5V 확인
2. 브레드보드1 - 레일: 0V 확인
3. 브레드보드2 + 레일: 5V 확인
4. 브레드보드2 - 레일: 0V 확인
5. 브레드보드 간 전압차: 0V 확인 (같은 전위)
```

---

## 📅 2일차: Arduino 기본 프로그래밍

### 💻 2-1. 개발 환경 설정 (45분)

**Arduino IDE 설정**:
```
1. Arduino IDE 1.8.19 다운로드 및 설치
   - 공식 사이트: https://www.arduino.cc/en/software

2. 보드 매니저 설정:
   - 파일 → 환경설정
   - 보드 매니저 URL: 
     https://dl.espressif.com/dl/package_esp32_index.json

3. ESP32 보드 패키지 설치:
   - 도구 → 보드 매니저
   - "ESP32" 검색 후 "ESP32 by Espressif Systems" 설치

4. 필수 라이브러리 설치:
   - 라이브러리 매니저에서 검색 및 설치:
     • "DHT sensor library" by Adafruit
     • "LiquidCrystal I2C"
     • "ArduinoJson"
     • "WebSockets" by Markus Sattler
     • "Servo" (내장 라이브러리)
```

**첫 번째 테스트 코드**:
```cpp
// LED_Test.ino - 기본 LED 제어 테스트
void setup() {
    Serial.begin(115200);
    
    // LED 핀 설정
    pinMode(4, OUTPUT);   // 초록 LED
    pinMode(5, OUTPUT);   // 빨강 LED  
    pinMode(8, OUTPUT);   // 노랑 LED
    pinMode(13, OUTPUT);  // 내장 LED
    
    Serial.println("LED Test Started");
}

void loop() {
    // 순차적으로 LED 켜기
    digitalWrite(4, HIGH);   // 초록 ON
    delay(500);
    digitalWrite(4, LOW);    // 초록 OFF
    
    digitalWrite(5, HIGH);   // 빨강 ON
    delay(500);
    digitalWrite(5, LOW);    // 빨강 OFF
    
    digitalWrite(8, HIGH);   // 노랑 ON
    delay(500);
    digitalWrite(8, LOW);    // 노랑 OFF
    
    digitalWrite(13, HIGH);  // 내장 LED ON
    delay(500);
    digitalWrite(13, LOW);   // 내장 LED OFF
    delay(500);
    
    Serial.println("LED Sequence Complete");
}
```

### 🎛️ 2-2. 기본 입출력 프로그래밍 (90분)

**아날로그 입력 테스트**:
```cpp
// Analog_Test.ino - 포텐셔미터와 센서 테스트
void setup() {
    Serial.begin(115200);
    Serial.println("Analog Input Test Started");
}

void loop() {
    // 포텐셔미터 읽기 (A0)
    int potValue = analogRead(A0);
    int percentage = map(potValue, 0, 1023, 0, 100);
    
    // 조도 센서 읽기 (A2)  
    int lightValue = analogRead(A2);
    int lightPercent = map(lightValue, 0, 1023, 0, 100);
    
    // 전류 센서 읽기 (A1)
    int currentRaw = analogRead(A1);
    float voltage = (currentRaw * 5.0) / 1024.0;
    float current = (voltage - 2.5) / 0.185;  // ACS712-05B
    
    // 시리얼 출력
    Serial.print("Pot: ");
    Serial.print(percentage);
    Serial.print("%, Light: ");
    Serial.print(lightPercent);
    Serial.print("%, Current: ");
    Serial.print(current, 2);
    Serial.println("A");
    
    delay(500);
}
```

**디지털 입력 테스트 (인터럽트)**:
```cpp
// Button_Test.ino - 버튼 인터럽트 테스트
volatile bool buttonPressed = false;
volatile unsigned long lastPressTime = 0;

void setup() {
    Serial.begin(115200);
    
    // 버튼 핀 설정 (내부 풀업 저항 사용)
    pinMode(2, INPUT_PULLUP);
    
    // 인터럽트 설정 (FALLING 엣지에서 발생)
    attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);
    
    Serial.println("Button Test Started - Press the button!");
}

void loop() {
    if (buttonPressed) {
        buttonPressed = false;
        Serial.println("Button Pressed!");
        
        // LED로 버튼 상태 표시
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
    }
    
    delay(10);
}

// 인터럽트 서비스 루틴 (ISR)
void buttonISR() {
    unsigned long currentTime = millis();
    
    // 디바운싱 (50ms 이내 중복 신호 무시)
    if (currentTime - lastPressTime > 50) {
        buttonPressed = true;
        lastPressTime = currentTime;
    }
}
```

### ⚙️ 2-3. PWM 및 서보 제어 (45분)

**PWM 기본 테스트**:
```cpp
// PWM_Test.ino - PWM 출력 테스트
void setup() {
    Serial.begin(115200);
    
    // PWM 핀 설정
    pinMode(3, OUTPUT);   // 모터 속도 제어
    pinMode(6, OUTPUT);   // 부저
    pinMode(11, OUTPUT);  // 서보 모터
    
    Serial.println("PWM Test Started");
}

void loop() {
    // 점진적 속도 증가 (0 → 255)
    for (int speed = 0; speed <= 255; speed += 5) {
        analogWrite(3, speed);
        
        Serial.print("PWM Value: ");
        Serial.print(speed);
        Serial.print(" (");
        Serial.print(map(speed, 0, 255, 0, 100));
        Serial.println("%)");
        
        delay(100);
    }
    
    // 점진적 속도 감소 (255 → 0)
    for (int speed = 255; speed >= 0; speed -= 5) {
        analogWrite(3, speed);
        delay(100);
    }
    
    Serial.println("PWM Cycle Complete");
    delay(1000);
}
```

**서보 모터 제어**:
```cpp
// Servo_Test.ino - 서보 모터 제어 테스트
#include <Servo.h>

Servo windServo;

void setup() {
    Serial.begin(115200);
    
    // 서보 모터 초기화
    windServo.attach(11);
    windServo.write(90);  // 중앙 위치
    
    Serial.println("Servo Test Started");
    delay(1000);
}

void loop() {
    // 좌측으로 이동 (0도)
    Serial.println("Moving to 0 degrees");
    windServo.write(0);
    delay(1000);
    
    // 중앙으로 이동 (90도)
    Serial.println("Moving to 90 degrees");
    windServo.write(90);
    delay(1000);
    
    // 우측으로 이동 (180도)
    Serial.println("Moving to 180 degrees");
    windServo.write(180);
    delay(1000);
    
    // 중앙으로 복귀
    Serial.println("Returning to center");
    windServo.write(90);
    delay(1000);
}
```

---

## 📅 3일차: 센서 연결 및 데이터 처리

### 🌡️ 3-1. DHT22 온습도 센서 (90분)

**하드웨어 연결**:
```
DHT22 연결도:
┌─────────┐
│ 1  2    │ 1: VCC (5V)
│ 3  4    │ 2: Data (D7)
└─────────┘ 3: NC (미사용)
            4: GND

추가 부품:
• 풀업 저항 10kΩ (Data-VCC 간)
• 디커플링 커패시터 100nF (VCC-GND 간)
```

**DHT22 테스트 코드**:
```cpp
// DHT22_Test.ino - 온습도 센서 테스트
#include <DHT.h>

#define DHT_PIN 7
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
    Serial.begin(115200);
    
    // DHT22 초기화
    dht.begin();
    
    Serial.println("DHT22 Test Started");
    Serial.println("Waiting for sensor to stabilize...");
    delay(2000);
}

void loop() {
    // 센서 읽기 (최소 2초 간격 필요)
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // 데이터 유효성 검사
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Error: Failed to read from DHT sensor!");
        delay(2000);
        return;
    }
    
    // 온도 관련 계산
    float heatIndex = dht.computeHeatIndex(temperature, humidity, false);
    float dewPoint = temperature - ((100 - humidity) / 5);
    
    // 시리얼 출력
    Serial.println("=== DHT22 Sensor Data ===");
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.println("°C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity, 1);
    Serial.println("%");
    
    Serial.print("Heat Index: ");
    Serial.print(heatIndex, 1);
    Serial.println("°C");
    
    Serial.print("Dew Point: ");
    Serial.print(dewPoint, 1);
    Serial.println("°C");
    
    // 편안함 지수 계산
    String comfort = "";
    if (temperature >= 20 && temperature <= 26 && humidity >= 40 && humidity <= 60) {
        comfort = "Comfortable";
    } else if (temperature > 26 || humidity > 60) {
        comfort = "Too Warm/Humid";
    } else {
        comfort = "Too Cool/Dry";
    }
    
    Serial.print("Comfort Level: ");
    Serial.println(comfort);
    Serial.println();
    
    delay(2000);
}
```

### 💡 3-2. 조도 센서 (LDR) (60분)

**LDR 전압 분배 회로**:
```
회로 구성:
5V ──┬── LDR ──┬── A2 (아날로그 입력)
     │         │
    (햇빛)     10kΩ
     │         │
     └─────────┴── GND

동작 원리:
• 밝을 때: LDR 저항 ↓ → A2 전압 ↑
• 어둘 때: LDR 저항 ↑ → A2 전압 ↓
```

**LDR 테스트 및 보정 코드**:
```cpp
// LDR_Test.ino - 조도 센서 테스트 및 보정
void setup() {
    Serial.begin(115200);
    Serial.println("LDR Light Sensor Test Started");
    Serial.println("Cover/uncover the sensor to test");
    delay(1000);
}

void loop() {
    // 아날로그 값 읽기
    int rawValue = analogRead(A2);
    float voltage = (rawValue * 5.0) / 1024.0;
    
    // 조도 레벨 계산 (0-100%)
    int lightLevel = map(rawValue, 0, 1023, 0, 100);
    
    // 조도 상태 분류
    String lightStatus = "";
    if (lightLevel < 20) {
        lightStatus = "Very Dark";
    } else if (lightLevel < 40) {
        lightStatus = "Dark";
    } else if (lightLevel < 60) {
        lightStatus = "Dim";
    } else if (lightLevel < 80) {
        lightStatus = "Bright";
    } else {
        lightStatus = "Very Bright";
    }
    
    // 시리얼 출력
    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(", Voltage: ");
    Serial.print(voltage, 2);
    Serial.print("V, Light: ");
    Serial.print(lightLevel);
    Serial.print("% (");
    Serial.print(lightStatus);
    Serial.println(")");
    
    delay(500);
}
```

### ⚡ 3-3. 전류 센서 (ACS712) (90분)

**ACS712 연결 및 보정**:
```
ACS712-05B 사양:
• 측정 범위: ±5A
• 감도: 185mV/A
• 제로점: 2.5V (전류 0A일 때)
• 전원: 5V
• 출력: 아날로그 (0-5V)

연결도:
IP+ ─── 모터 +12V 라인에 직렬 연결
IP- ─── HW-020 +12V 터미널로
VCC ─── Arduino 5V
OUT ─── Arduino A1
GND ─── Arduino GND
```

**ACS712 테스트 및 보정 코드**:
```cpp
// ACS712_Test.ino - 전류 센서 테스트 및 보정
// ACS712-05B 사양: 185mV/A, 2.5V 제로점

float zeroCurrentVoltage = 2.5;  // 보정값 (실측으로 조정)
float sensitivity = 0.185;       // 185mV/A

void setup() {
    Serial.begin(115200);
    Serial.println("ACS712 Current Sensor Test");
    Serial.println("Calibrating zero point...");
    
    // 무부하 상태에서 제로점 보정
    delay(1000);
    calibrateZeroPoint();
}

void loop() {
    // 전류 측정
    float current = readCurrent();
    float power = current * 12.0;  // 12V 기준 전력 계산
    
    // 시리얼 출력
    Serial.print("Current: ");
    Serial.print(current, 3);
    Serial.print("A, Power: ");
    Serial.print(power, 2);
    Serial.println("W");
    
    // 과전류 경고
    if (current > 1.5) {
        Serial.println("WARNING: Over current detected!");
    }
    
    delay(500);
}

void calibrateZeroPoint() {
    float total = 0;
    int samples = 100;
    
    // 100번 측정하여 평균값 계산
    for (int i = 0; i < samples; i++) {
        int rawValue = analogRead(A1);
        float voltage = (rawValue * 5.0) / 1024.0;
        total += voltage;
        delay(10);
    }
    
    zeroCurrentVoltage = total / samples;
    
    Serial.print("Zero point calibrated: ");
    Serial.print(zeroCurrentVoltage, 3);
    Serial.println("V");
    
    delay(1000);
}

float readCurrent() {
    // 여러 번 측정하여 노이즈 제거
    float total = 0;
    int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        int rawValue = analogRead(A1);
        float voltage = (rawValue * 5.0) / 1024.0;
        total += voltage;
        delay(1);
    }
    
    float avgVoltage = total / samples;
    float current = (avgVoltage - zeroCurrentVoltage) / sensitivity;
    
    return abs(current);  // 절댓값 반환
}
```

---

## 📅 4일차: 모터 제어 시스템

### 🔄 4-1. HW-020 모터 쉴드 기본 제어 (120분)

**기본 모터 제어 코드**:
```cpp
// Motor_Basic_Test.ino - 기본 모터 제어 테스트
// HW-020 Motor Shield 사용

// 핀 정의
#define MOTOR_PWM  3   // ENA - 속도 제어 (PWM)
#define MOTOR_DIR  12  // IN1 - 방향 제어
// IN2는 GND에 연결 (단방향 제어)

// 전역 변수
int motorSpeed = 0;        // 현재 모터 속도 (0-255)
bool motorDirection = true; // 모터 방향 (true=정방향)
bool motorEnabled = false;  // 모터 동작 상태

void setup() {
    Serial.begin(115200);
    
    // 핀 모드 설정
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    
    // 초기 상태 설정
    analogWrite(MOTOR_PWM, 0);      // 모터 정지
    digitalWrite(MOTOR_DIR, LOW);   // 방향 설정
    
    Serial.println("Motor Control Test Started");
    Serial.println("Commands:");
    Serial.println("  START - Start motor");
    Serial.println("  STOP  - Stop motor");
    Serial.println("  SPEED:<0-100> - Set speed percentage");
    Serial.println("  DIR   - Change direction");
    Serial.println("  STATUS - Show current status");
}

void loop() {
    // 시리얼 명령 처리
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        command.toUpperCase();
        
        processCommand(command);
    }
    
    // 상태 업데이트
    updateMotor();
    
    delay(50);
}

void processCommand(String cmd) {
    if (cmd == "START") {
        motorEnabled = true;
        if (motorSpeed == 0) {
            motorSpeed = 50;  // 기본 속도 50%
        }
        Serial.println("Motor started");
        
    } else if (cmd == "STOP") {
        motorEnabled = false;
        Serial.println("Motor stopped");
        
    } else if (cmd.startsWith("SPEED:")) {
        int speed = cmd.substring(6).toInt();
        if (speed >= 0 && speed <= 100) {
            motorSpeed = map(speed, 0, 100, 0, 255);
            Serial.print("Speed set to: ");
            Serial.print(speed);
            Serial.println("%");
        } else {
            Serial.println("Error: Speed must be 0-100");
        }
        
    } else if (cmd == "DIR") {
        changeDirection();
        
    } else if (cmd == "STATUS") {
        showStatus();
        
    } else {
        Serial.println("Unknown command");
    }
}

void updateMotor() {
    if (motorEnabled && motorSpeed > 0) {
        analogWrite(MOTOR_PWM, motorSpeed);
        digitalWrite(MOTOR_DIR, motorDirection ? HIGH : LOW);
    } else {
        analogWrite(MOTOR_PWM, 0);
    }
}

void changeDirection() {
    // 안전한 방향 변경을 위해 모터 정지
    bool wasEnabled = motorEnabled;
    int oldSpeed = motorSpeed;
    
    motorEnabled = false;
    updateMotor();
    delay(500);  // 완전 정지 대기
    
    motorDirection = !motorDirection;
    
    motorEnabled = wasEnabled;
    motorSpeed = oldSpeed;
    
    Serial.print("Direction changed to: ");
    Serial.println(motorDirection ? "Forward" : "Reverse");
}

void showStatus() {
    Serial.println("=== Motor Status ===");
    Serial.print("Enabled: ");
    Serial.println(motorEnabled ? "Yes" : "No");
    Serial.print("Speed: ");
    Serial.print(map(motorSpeed, 0, 255, 0, 100));
    Serial.println("%");
    Serial.print("Direction: ");
    Serial.println(motorDirection ? "Forward" : "Reverse");
    Serial.print("PWM Value: ");
    Serial.println(motorSpeed);
}
```

### 🛡️ 4-2. 안전 시스템 구현 (90분)

**과전류/과열 보호 기능**:
```cpp
// Motor_Safety_System.ino - 안전 시스템이 포함된 모터 제어
#include <DHT.h>

// 핀 정의
#define MOTOR_PWM  3
#define MOTOR_DIR  12
#define DHT_PIN    7
#define LED_POWER  4   // 초록 LED
#define LED_WARNING 5  // 빨강 LED
#define LED_STATUS 8   // 노랑 LED
#define BUZZER_PIN 6

// 센서 초기화
DHT dht(DHT_PIN, DHT22);

// 안전 임계값
#define MAX_CURRENT    1.5   // 최대 전류 (A)
#define MAX_TEMPERATURE 55   // 최대 온도 (°C)
#define WARNING_TEMPERATURE 50 // 경고 온도 (°C)

// 전역 변수
struct MotorState {
    int speed = 0;
    bool direction = true;
    bool enabled = false;
    bool safetyMode = false;
    float currentDraw = 0;
    float temperature = 0;
    unsigned long lastSafetyCheck = 0;
} motor;

void setup() {
    Serial.begin(115200);
    
    // 핀 모드 설정
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_WARNING, OUTPUT);
    pinMode(LED_STATUS, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    // DHT22 초기화
    dht.begin();
    
    // 초기 LED 상태
    digitalWrite(LED_POWER, HIGH);   // 전원 LED 켜기
    digitalWrite(LED_WARNING, LOW);
    digitalWrite(LED_STATUS, LOW);
    
    Serial.println("Motor Safety System Started");
    delay(2000);
}

void loop() {
    // 안전 검사 (1초마다)
    if (millis() - motor.lastSafetyCheck > 1000) {
        performSafetyCheck();
        motor.lastSafetyCheck = millis();
    }
    
    // 시리얼 명령 처리
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        processCommand(command);
    }
    
    // 모터 및 LED 상태 업데이트
    updateMotor();
    updateLEDs();
    
    delay(50);
}

void performSafetyCheck() {
    // 온도 읽기
    motor.temperature = dht.readTemperature();
    if (isnan(motor.temperature)) {
        motor.temperature = 25;  // 기본값
    }
    
    // 전류 읽기
    motor.currentDraw = readCurrent();
    
    // 안전 검사
    bool newSafetyMode = false;
    
    // 과전류 검사
    if (motor.currentDraw > MAX_CURRENT) {
        Serial.println("SAFETY: Over current detected!");
        newSafetyMode = true;
    }
    
    // 과열 검사
    if (motor.temperature > MAX_TEMPERATURE) {
        Serial.println("SAFETY: Over temperature detected!");
        newSafetyMode = true;
    }
    
    // 안전 모드 전환
    if (newSafetyMode && !motor.safetyMode) {
        activateSafetyMode();
    } else if (!newSafetyMode && motor.safetyMode) {
        // 자동 복구 (온도/전류가 정상으로 돌아왔을 때)
        if (motor.temperature < WARNING_TEMPERATURE && 
            motor.currentDraw < (MAX_CURRENT * 0.8)) {
            deactivateSafetyMode();
        }
    }
    
    // 경고 조건 (안전 모드는 아니지만 주의 필요)
    if (motor.temperature > WARNING_TEMPERATURE || 
        motor.currentDraw > (MAX_CURRENT * 0.8)) {
        // 속도 제한
        if (motor.speed > 150) {
            motor.speed = 150;  // 속도를 60%로 제한
            Serial.println("WARNING: Performance limited due to high load");
        }
    }
    
    // 상태 출력
    Serial.print("Temp: ");
    Serial.print(motor.temperature, 1);
    Serial.print("°C, Current: ");
    Serial.print(motor.currentDraw, 2);
    Serial.print("A, Safety: ");
    Serial.println(motor.safetyMode ? "ACTIVE" : "OK");
}

void activateSafetyMode() {
    motor.safetyMode = true;
    motor.enabled = false;
    motor.speed = 0;
    
    // 알람 소리
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 1000, 200);
        delay(300);
    }
    
    Serial.println("*** SAFETY MODE ACTIVATED ***");
    Serial.println("Motor stopped for safety. Check system!");
}

void deactivateSafetyMode() {
    motor.safetyMode = false;
    
    // 확인 소리
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 100);
    
    Serial.println("Safety mode deactivated - System OK");
}

float readCurrent() {
    // ACS712 전류 센서 읽기 (이전 코드 재사용)
    int rawValue = analogRead(A1);
    float voltage = (rawValue * 5.0) / 1024.0;
    float current = abs((voltage - 2.5) / 0.185);
    return current;
}

void updateMotor() {
    if (motor.enabled && !motor.safetyMode && motor.speed > 0) {
        analogWrite(MOTOR_PWM, motor.speed);
        digitalWrite(MOTOR_DIR, motor.direction ? HIGH : LOW);
    } else {
        analogWrite(MOTOR_PWM, 0);
    }
}

void updateLEDs() {
    // 전원 LED (항상 켜짐)
    digitalWrite(LED_POWER, HIGH);
    
    // 경고 LED (안전 모드시 빠른 깜빡임)
    if (motor.safetyMode) {
        digitalWrite(LED_WARNING, (millis() / 100) % 2);
    } else if (motor.temperature > WARNING_TEMPERATURE || 
               motor.currentDraw > (MAX_CURRENT * 0.8)) {
        digitalWrite(LED_WARNING, (millis() / 500) % 2);  // 느린 깜빡임
    } else {
        digitalWrite(LED_WARNING, LOW);
    }
    
    // 상태 LED (모터 동작시 속도에 따라 깜빡임)
    if (motor.enabled && motor.speed > 0) {
        int blinkRate = map(motor.speed, 0, 255, 1000, 100);
        digitalWrite(LED_STATUS, (millis() / blinkRate) % 2);
    } else {
        digitalWrite(LED_STATUS, LOW);
    }
}

void processCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    
    if (motor.safetyMode && cmd != "RESET" && cmd != "STATUS") {
        Serial.println("Error: System in safety mode. Use RESET to clear.");
        return;
    }
    
    if (cmd == "START") {
        motor.enabled = true;
        if (motor.speed == 0) motor.speed = 100;
        Serial.println("Motor started");
        
    } else if (cmd == "STOP") {
        motor.enabled = false;
        Serial.println("Motor stopped");
        
    } else if (cmd.startsWith("SPEED:")) {
        int speed = cmd.substring(6).toInt();
        if (speed >= 0 && speed <= 100) {
            motor.speed = map(speed, 0, 100, 0, 255);
            Serial.print("Speed set to: ");
            Serial.print(speed);
            Serial.println("%");
        }
        
    } else if (cmd == "RESET") {
        if (motor.safetyMode) {
            // 수동 안전 모드 해제 (주의: 문제가 해결되었는지 확인 필요)
            motor.safetyMode = false;
            Serial.println("Safety mode manually reset - Check system carefully!");
        } else {
            Serial.println("System reset");
        }
        
    } else if (cmd == "STATUS") {
        showDetailedStatus();
    }
}

void showDetailedStatus() {
    Serial.println("=== Detailed System Status ===");
    Serial.print("Motor Enabled: ");
    Serial.println(motor.enabled ? "Yes" : "No");
    Serial.print("Motor Speed: ");
    Serial.print(map(motor.speed, 0, 255, 0, 100));
    Serial.println("%");
    Serial.print("Direction: ");
    Serial.println(motor.direction ? "Forward" : "Reverse");
    Serial.print("Safety Mode: ");
    Serial.println(motor.safetyMode ? "ACTIVE" : "OK");
    Serial.print("Temperature: ");
    Serial.print(motor.temperature, 1);
    Serial.println("°C");
    Serial.print("Current Draw: ");
    Serial.print(motor.currentDraw, 2);
    Serial.println("A");
    Serial.println("=============================");
}
```

---

## 📅 5일차: ESP32 WiFi 연동 시스템

### 📡 5-1. ESP32 기본 WiFi 설정 (90분)

**ESP32 WiFi AP/Client 모드 코드**:
```cpp
// ESP32_WiFi_Setup.ino - WiFi 연결 및 웹서버 기본 설정
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// WiFi 설정
WebServer server(80);
String ssid = "";
String password = "";
bool apMode = true;

// AP 모드 설정
const char* ap_ssid = "SmartFan_Setup";
const char* ap_password = "12345678";

void setup() {
    Serial.begin(115200);
    
    // EEPROM 초기화
    EEPROM.begin(512);
    
    // 저장된 WiFi 설정 로드
    loadWiFiSettings();
    
    // WiFi 연결 시도
    if (ssid.length() > 0) {
        Serial.println("Attempting to connect to saved WiFi...");
        connectToWiFi();
    } else {
        Serial.println("No saved WiFi settings found.");
        startAPMode();
    }
    
    // 웹서버 라우팅 설정
    setupWebServer();
    
    // 웹서버 시작
    server.begin();
    Serial.println("Web server started");
}

void loop() {
    // 웹서버 요청 처리
    server.handleClient();
    
    // WiFi 연결 상태 모니터링
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 10000) {  // 10초마다 확인
        checkWiFiConnection();
        lastCheck = millis();
    }
    
    delay(10);
}

void connectToWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    Serial.print("Connecting to ");
    Serial.print(ssid);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi connected successfully!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        
        apMode = false;
    } else {
        Serial.println();
        Serial.println("Failed to connect to WiFi");
        startAPMode();
    }
}

void startAPMode() {
    Serial.println("Starting AP mode...");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.println("Connect to WiFi network: " + String(ap_ssid));
    Serial.println("Password: " + String(ap_password));
    Serial.println("Open browser to: http://192.168.4.1");
    
    apMode = true;
}

void setupWebServer() {
    // 루트 페이지 - WiFi 설정 폼
    server.on("/", []() {
        String html = generateSetupPage();
        server.send(200, "text/html", html);
    });
    
    // WiFi 설정 처리
    server.on("/connect", HTTP_POST, []() {
        ssid = server.arg("ssid");
        password = server.arg("password");
        
        if (ssid.length() > 0) {
            Serial.println("New WiFi settings received");
            Serial.println("SSID: " + ssid);
            
            // EEPROM에 저장
            saveWiFiSettings();
            
            server.send(200, "text/html", 
                "<h1>Settings Saved</h1>"
                "<p>Attempting to connect to: " + ssid + "</p>"
                "<p>Device will restart in 3 seconds...</p>"
                "<script>setTimeout(function(){window.location.href='/';}, 3000);</script>");
            
            delay(1000);
            ESP.restart();
        } else {
            server.send(400, "text/html", 
                "<h1>Error</h1><p>SSID cannot be empty</p>"
                "<a href='/'>Go back</a>");
        }
    });
    
    // WiFi 스캔
    server.on("/scan", []() {
        String json = scanWiFiNetworks();
        server.send(200, "application/json", json);
    });
    
    // 상태 확인
    server.on("/status", []() {
        String json = "{";
        json += "\"mode\":\"" + String(apMode ? "AP" : "STA") + "\",";
        json += "\"ssid\":\"" + String(WiFi.SSID()) + "\",";
        json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
        json += "\"connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
        json += "}";
        
        server.send(200, "application/json", json);
    });
}

String generateSetupPage() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Fan WiFi Setup</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 500px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        h1 { color: #333; text-align: center; }
        .form-group { margin: 15px 0; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input[type="text"], input[type="password"], select { 
            width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 5px; 
        }
        button { 
            background: #4CAF50; color: white; padding: 12px 20px; 
            border: none; border-radius: 5px; cursor: pointer; width: 100%; 
        }
        button:hover { background: #45a049; }
        .scan-btn { background: #2196F3; margin-bottom: 10px; }
        .network-list { margin: 10px 0; }
        .network-item { 
            padding: 8px; border: 1px solid #ddd; margin: 2px 0; 
            cursor: pointer; border-radius: 3px; 
        }
        .network-item:hover { background: #f5f5f5; }
        .status { padding: 10px; background: #e7f3ff; border-radius: 5px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🌀 Smart Fan WiFi Setup</h1>
        
        <div class="status" id="status">
            <strong>Current Status:</strong><br>
            <span id="current-status">Loading...</span>
        </div>
        
        <button class="scan-btn" onclick="scanNetworks()">🔍 Scan WiFi Networks</button>
        
        <div class="network-list" id="networks"></div>
        
        <form action="/connect" method="POST">
            <div class="form-group">
                <label for="ssid">WiFi Network (SSID):</label>
                <input type="text" id="ssid" name="ssid" required>
            </div>
            
            <div class="form-group">
                <label for="password">Password:</label>
                <input type="password" id="password" name="password">
            </div>
            
            <button type="submit">📶 Connect to WiFi</button>
        </form>
    </div>

    <script>
        // 페이지 로드시 상태 확인
        loadStatus();
        
        function loadStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    let statusText = `Mode: ${data.mode}<br>`;
                    if (data.connected) {
                        statusText += `Connected to: ${data.ssid}<br>`;
                        statusText += `IP Address: ${data.ip}<br>`;
                        statusText += `Signal: ${data.rssi} dBm`;
                    } else {
                        statusText += `Not connected to WiFi`;
                    }
                    document.getElementById('current-status').innerHTML = statusText;
                });
        }
        
        function scanNetworks() {
            document.getElementById('networks').innerHTML = 'Scanning...';
            
            fetch('/scan')
                .then(response => response.json())
                .then(data => {
                    let html = '';
                    data.networks.forEach(network => {
                        html += `<div class="network-item" onclick="selectNetwork('${network.ssid}')">
                            📶 ${network.ssid} (${network.rssi} dBm) ${network.encryption}
                        </div>`;
                    });
                    document.getElementById('networks').innerHTML = html;
                });
        }
        
        function selectNetwork(ssid) {
            document.getElementById('ssid').value = ssid;
        }
    </script>
</body>
</html>
)";
    
    return html;
}

String scanWiFiNetworks() {
    WiFi.scanDelete();
    int n = WiFi.scanNetworks();
    
    String json = "{\"networks\":[";
    
    for (int i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
        json += "\"encryption\":\"" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "Open" : "Secured") + "\"";
        json += "}";
    }
    
    json += "]}";
    return json;
}

void saveWiFiSettings() {
    // SSID 저장 (주소 0-31)
    for (int i = 0; i < 32; i++) {
        if (i < ssid.length()) {
            EEPROM.write(i, ssid[i]);
        } else {
            EEPROM.write(i, 0);
        }
    }
    
    // 비밀번호 저장 (주소 32-95)
    for (int i = 0; i < 64; i++) {
        if (i < password.length()) {
            EEPROM.write(32 + i, password[i]);
        } else {
            EEPROM.write(32 + i, 0);
        }
    }
    
    EEPROM.commit();
    Serial.println("WiFi settings saved to EEPROM");
}

void loadWiFiSettings() {
    // SSID 로드
    ssid = "";
    for (int i = 0; i < 32; i++) {
        char c = EEPROM.read(i);
        if (c == 0) break;
        ssid += c;
    }
    
    // 비밀번호 로드
    password = "";
    for (int i = 0; i < 64; i++) {
        char c = EEPROM.read(32 + i);
        if (c == 0) break;
        password += c;
    }
    
    if (ssid.length() > 0) {
        Serial.println("Loaded WiFi settings from EEPROM");
        Serial.println("SSID: " + ssid);
    }
}

void checkWiFiConnection() {
    if (!apMode && WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connection lost. Attempting to reconnect...");
        connectToWiFi();
    }
}
```

### 🌐 5-2. 웹소켓 실시간 통신 (90분)

**WebSocket 서버 구현**:
```cpp
// ESP32_WebSocket_Server.ino - WebSocket을 이용한 실시간 통신
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// 전역 변수
struct SystemData {
    float temperature = 0;
    float humidity = 0;
    float current = 0;
    int fanSpeed = 0;
    bool fanEnabled = false;
    bool autoMode = false;
    int lightLevel = 0;
    unsigned long runtime = 0;
} systemData;

void setup() {
    Serial.begin(115200);
    
    // WiFi 연결 (이전 코드 재사용)
    connectToWiFi();
    
    // 웹서버 설정
    setupWebServer();
    
    // WebSocket 설정
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    Serial.println("WebSocket server started on port 81");
}

void loop() {
    server.handleClient();
    webSocket.loop();
    
    // Arduino에서 데이터 수신 (시리얼 통신)
    if (Serial.available()) {
        String data = Serial.readString();
        data.trim();
        processArduinoData(data);
    }
    
    // 주기적으로 클라이언트에 데이터 전송 (2초마다)
    static unsigned long lastSend = 0;
    if (millis() - lastSend > 2000) {
        broadcastSystemData();
        lastSend = millis();
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("Client %u disconnected\n", num);
            break;
            
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("Client %u connected from %d.%d.%d.%d\n", 
                num, ip[0], ip[1], ip[2], ip[3]);
            
            // 연결된 클라이언트에게 현재 상태 전송
            sendSystemData(num);
            break;
        }
        
        case WStype_TEXT: {
            String message = String((char*)payload);
            Serial.printf("Received from client %u: %s\n", num, message.c_str());
            
            // JSON 명령 처리
            processWebSocketCommand(num, message);
            break;
        }
        
        default:
            break;
    }
}

void processWebSocketCommand(uint8_t clientNum, String message) {
    // JSON 파싱
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.println("JSON parsing failed");
        return;
    }
    
    String action = doc["action"];
    
    if (action == "start") {
        // 팬 시작
        Serial.println("CMD:START");
        systemData.fanEnabled = true;
        
    } else if (action == "stop") {
        // 팬 정지
        Serial.println("CMD:STOP");
        systemData.fanEnabled = false;
        
    } else if (action == "speed") {
        // 속도 조절
        int speed = doc["value"];
        Serial.println("CMD:SPEED:" + String(speed));
        systemData.fanSpeed = speed;
        
    } else if (action == "auto") {
        // 자동 모드 토글
        systemData.autoMode = doc["value"];
        Serial.println("CMD:AUTO:" + String(systemData.autoMode ? "ON" : "OFF"));
        
    } else if (action == "direction") {
        // 방향 변경
        Serial.println("CMD:DIR");
        
    } else if (action == "emergency") {
        // 비상 정지
        Serial.println("CMD:EMERGENCY");
        systemData.fanEnabled = false;
        systemData.fanSpeed = 0;
    }
    
    // 모든 클라이언트에게 업데이트된 상태 전송
    broadcastSystemData();
}

void processArduinoData(String data) {
    // Arduino에서 JSON 형태로 받은 데이터 처리
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, data);
    
    if (error) {
        Serial.println("Arduino data parsing failed: " + data);
        return;
    }
    
    // 센서 데이터 업데이트
    if (doc.containsKey("temperature")) {
        systemData.temperature = doc["temperature"];
    }
    
    if (doc.containsKey("humidity")) {
        systemData.humidity = doc["humidity"];
    }
    
    if (doc.containsKey("current")) {
        systemData.current = doc["current"];
    }
    
    if (doc.containsKey("light")) {
        systemData.lightLevel = doc["light"];
    }
    
    if (doc.containsKey("speed")) {
        systemData.fanSpeed = doc["speed"];
    }
    
    if (doc.containsKey("enabled")) {
        systemData.fanEnabled = doc["enabled"];
    }
    
    if (doc.containsKey("runtime")) {
        systemData.runtime = doc["runtime"];
    }
}

void sendSystemData(uint8_t clientNum) {
    DynamicJsonDocument doc(1024);
    
    doc["type"] = "data";
    doc["temperature"] = systemData.temperature;
    doc["humidity"] = systemData.humidity;
    doc["current"] = systemData.current;
    doc["power"] = systemData.current * 12.0;
    doc["fanSpeed"] = systemData.fanSpeed;
    doc["fanEnabled"] = systemData.fanEnabled;
    doc["autoMode"] = systemData.autoMode;
    doc["lightLevel"] = systemData.lightLevel;
    doc["runtime"] = systemData.runtime;
    doc["timestamp"] = millis();
    
    String json;
    serializeJson(doc, json);
    
    webSocket.sendTXT(clientNum, json);
}

void broadcastSystemData() {
    DynamicJsonDocument doc(1024);
    
    doc["type"] = "data";
    doc["temperature"] = systemData.temperature;
    doc["humidity"] = systemData.humidity;
    doc["current"] = systemData.current;
    doc["power"] = systemData.current * 12.0;
    doc["fanSpeed"] = systemData.fanSpeed;
    doc["fanEnabled"] = systemData.fanEnabled;
    doc["autoMode"] = systemData.autoMode;
    doc["lightLevel"] = systemData.lightLevel;
    doc["runtime"] = systemData.runtime;
    doc["timestamp"] = millis();
    
    String json;
    serializeJson(doc, json);
    
    webSocket.broadcastTXT(json);
}

void setupWebServer() {
    // 메인 페이지
    server.on("/", []() {
        String html = generateMainPage();
        server.send(200, "text/html", html);
    });
    
    // API 엔드포인트
    server.on("/api/data", []() {
        DynamicJsonDocument doc(1024);
        doc["temperature"] = systemData.temperature;
        doc["humidity"] = systemData.humidity;
        doc["current"] = systemData.current;
        doc["fanSpeed"] = systemData.fanSpeed;
        doc["fanEnabled"] = systemData.fanEnabled;
        
        String json;
        serializeJson(doc, json);
        
        server.send(200, "application/json", json);
    });
    
    server.begin();
}

String generateMainPage() {
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Smart Fan Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Arial, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh; padding: 20px;
        }
        .container { 
            max-width: 400px; margin: 0 auto; 
            background: rgba(255,255,255,0.95); 
            border-radius: 15px; padding: 20px; 
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
        }
        h1 { 
            text-align: center; color: #333; margin-bottom: 20px; 
            font-size: 24px;
        }
        .status-grid { 
            display: grid; grid-template-columns: 1fr 1fr; 
            gap: 10px; margin-bottom: 20px; 
        }
        .status-card { 
            background: #f8f9fa; padding: 15px; border-radius: 10px; 
            text-align: center; border: 2px solid #e9ecef;
        }
        .status-value { 
            font-size: 24px; font-weight: bold; color: #495057; 
            margin: 5px 0;
        }
        .status-label { 
            font-size: 12px; color: #6c757d; text-transform: uppercase; 
        }
        .controls { margin: 20px 0; }
        .button { 
            width: 100%; padding: 15px; margin: 10px 0; 
            border: none; border-radius: 10px; font-size: 16px; 
            cursor: pointer; transition: all 0.3s;
        }
        .button-start { background: #28a745; color: white; }
        .button-stop { background: #dc3545; color: white; }
        .button-auto { background: #17a2b8; color: white; }
        .button:hover { transform: translateY(-2px); box-shadow: 0 5px 15px rgba(0,0,0,0.2); }
        .slider-container { margin: 15px 0; }
        .slider { 
            width: 100%; height: 8px; border-radius: 5px; 
            background: #ddd; outline: none; appearance: none;
        }
        .slider::-webkit-slider-thumb { 
            appearance: none; width: 20px; height: 20px; 
            border-radius: 50%; background: #4CAF50; cursor: pointer;
        }
        .connection-status { 
            padding: 10px; border-radius: 5px; margin-bottom: 15px;
            text-align: center; font-weight: bold;
        }
        .connected { background: #d4edda; color: #155724; }
        .disconnected { background: #f8d7da; color: #721c24; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🌀 Smart Fan Control</h1>
        
        <div class="connection-status" id="connectionStatus">
            <span id="statusText">Connecting...</span>
        </div>
        
        <div class="status-grid">
            <div class="status-card">
                <div class="status-value" id="temperature">--</div>
                <div class="status-label">Temperature (°C)</div>
            </div>
            <div class="status-card">
                <div class="status-value" id="humidity">--</div>
                <div class="status-label">Humidity (%)</div>
            </div>
            <div class="status-card">
                <div class="status-value" id="current">--</div>
                <div class="status-label">Current (A)</div>
            </div>
            <div class="status-card">
                <div class="status-value" id="power">--</div>
                <div class="status-label">Power (W)</div>
            </div>
        </div>
        
        <div class="controls">
            <button class="button button-start" onclick="sendCommand('start')">
                🟢 Start Fan
            </button>
            
            <button class="button button-stop" onclick="sendCommand('stop')">
                🔴 Stop Fan
            </button>
            
            <button class="button button-auto" id="autoButton" onclick="toggleAuto()">
                🤖 Auto Mode: OFF
            </button>
            
            <div class="slider-container">
                <label>Fan Speed: <span id="speedValue">0</span>%</label>
                <input type="range" class="slider" id="speedSlider" 
                       min="0" max="100" value="0" 
                       oninput="updateSpeed(this.value)">
            </div>
            
            <button class="button" onclick="sendCommand('emergency')" 
                    style="background: #ff6b6b; color: white;">
                🚨 Emergency Stop
            </button>
        </div>
    </div>

    <script>
        let ws;
        let autoMode = false;
        
        function initWebSocket() {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = protocol + '//' + window.location.hostname + ':81';
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                updateConnectionStatus(true);
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateDisplay(data);
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                updateConnectionStatus(false);
                // 5초 후 재연결 시도
                setTimeout(initWebSocket, 5000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
                updateConnectionStatus(false);
            };
        }
        
        function updateConnectionStatus(connected) {
            const statusElement = document.getElementById('connectionStatus');
            const statusText = document.getElementById('statusText');
            
            if (connected) {
                statusElement.className = 'connection-status connected';
                statusText.textContent = '🟢 Connected';
            } else {
                statusElement.className = 'connection-status disconnected';
                statusText.textContent = '🔴 Disconnected';
            }
        }
        
        function updateDisplay(data) {
            if (data.type === 'data') {
                document.getElementById('temperature').textContent = data.temperature.toFixed(1);
                document.getElementById('humidity').textContent = data.humidity.toFixed(1);
                document.getElementById('current').textContent = data.current.toFixed(2);
                document.getElementById('power').textContent = data.power.toFixed(1);
                
                // 속도 슬라이더 업데이트
                const speedSlider = document.getElementById('speedSlider');
                const speedValue = document.getElementById('speedValue');
                speedSlider.value = data.fanSpeed;
                speedValue.textContent = data.fanSpeed;
                
                // 자동 모드 버튼 업데이트
                autoMode = data.autoMode;
                updateAutoButton();
            }
        }
        
        function sendCommand(action, value = null) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const command = {
                    action: action,
                    value: value,
                    timestamp: Date.now()
                };
                ws.send(JSON.stringify(command));
            } else {
                alert('WebSocket not connected');
            }
        }
        
        function updateSpeed(value) {
            document.getElementById('speedValue').textContent = value;
            sendCommand('speed', parseInt(value));
        }
        
        function toggleAuto() {
            autoMode = !autoMode;
            sendCommand('auto', autoMode);
            updateAutoButton();
        }
        
        function updateAutoButton() {
            const button = document.getElementById('autoButton');
            button.textContent = autoMode ? '🤖 Auto Mode: ON' : '🤖 Auto Mode: OFF';
            button.style.background = autoMode ? '#28a745' : '#17a2b8';
        }
        
        // 페이지 로드시 WebSocket 연결
        window.addEventListener('load', initWebSocket);
    </script>
</body>
</html>
)";
    
    return html;
}
```

---

## 📅 6일차: 시스템 통합 및 테스트

### 🔗 6-1. Arduino-ESP32 통합 통신 (120분)

**Arduino 메인 코드 (통합 버전)**:
```cpp
// Arduino_Final_Integration.ino - 최종 통합 코드
#include <DHT.h>
#include <Servo.h>
#include <ArduinoJson.h>

// 핀 정의
#define DHT_PIN      7
#define MOTOR_PWM    3
#define MOTOR_DIR    12
#define SERVO_PIN    11
#define LED_POWER    4
#define LED_WARNING  5
#define LED_STATUS   8
#define BUZZER_PIN   6
#define BUTTON_PIN   2

// 센서 초기화
DHT dht(DHT_PIN, DHT22);
Servo windServo;

// 시스템 상태 구조체
struct SystemState {
    // 센서 데이터
    float temperature = 0;
    float humidity = 0;
    float current = 0;
    int lightLevel = 0;
    
    // 모터 상태
    int motorSpeed = 0;
    bool motorEnabled = false;
    bool motorDirection = true;
    int servoAngle = 90;
    
    // 제어 모드
    bool autoMode = false;
    bool ecoMode = false;
    bool nightMode = false;
    bool safetyMode = false;
    
    // 시간 관련
    unsigned long runtime = 0;
    unsigned long lastSensorRead = 0;
    unsigned long lastDataSend = 0;
    unsigned long systemStartTime = 0;
} sys;

// 자동 모드 설정
struct AutoSettings {
    float targetTemp = 25.0;
    float tempThreshold = 2.0;
    int maxSpeed = 80;
    int minSpeed = 20;
    bool adaptiveControl = true;
} autoSettings;

void setup() {
    Serial.begin(115200);
    
    // 핀 모드 설정
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_WARNING, OUTPUT);
    pinMode(LED_STATUS, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // 센서 초기화
    dht.begin();
    windServo.attach(SERVO_PIN);
    windServo.write(90);
    
    // 인터럽트 설정
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), emergencyStop, FALLING);
    
    // 초기 상태 설정
    sys.systemStartTime = millis();
    digitalWrite(LED_POWER, HIGH);
    
    // 시작 신호음
    playStartupSound();
    
    Serial.println("Smart Fan System v3.0 Started");
    delay(2000);
}

void loop() {
    // 센서 데이터 읽기 (2초마다)
    if (millis() - sys.lastSensorRead > 2000) {
        readSensors();
        sys.lastSensorRead = millis();
    }
    
    // 시리얼 명령 처리
    if (Serial.available()) {
        String command = Serial.readString();
        command.trim();
        processCommand(command);
    }
    
    // 자동 모드 처리
    if (sys.autoMode) {
        processAutoMode();
    }
    
    // 안전 검사
    performSafetyCheck();
    
    // 모터 및 서보 제어
    updateMotor();
    
    // LED 상태 업데이트
    updateLEDs();
    
    // ESP32로 데이터 전송 (2초마다)
    if (millis() - sys.lastDataSend > 2000) {
        sendDataToESP32();
        sys.lastDataSend = millis();
    }
    
    // 런타임 업데이트
    sys.runtime = millis() - sys.systemStartTime;
    
    delay(50);
}

void readSensors() {
    // DHT22 온습도
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    
    if (!isnan(temp) && !isnan(hum)) {
        sys.temperature = temp;
        sys.humidity = hum;
    }
    
    // 조도 센서
    int lightRaw = analogRead(A2);
    sys.lightLevel = map(lightRaw, 0, 1023, 0, 100);
    
    // 전류 센서
    sys.current = readCurrentSensor();
    
    // 야간 모드 자동 감지
    if (sys.lightLevel < 20 && !sys.nightMode) {
        enableNightMode();
    } else if (sys.lightLevel > 40 && sys.nightMode) {
        disableNightMode();
    }
}

float readCurrentSensor() {
    float total = 0;
    int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        int raw = analogRead(A1);
        float voltage = (raw * 5.0) / 1024.0;
        total += abs((voltage - 2.5) / 0.185);  // ACS712-05B
        delay(1);
    }
    
    return total / samples;
}

void processCommand(String cmd) {
    if (cmd.startsWith("CMD:")) {
        cmd = cmd.substring(4);
        
        if (cmd == "START") {
            sys.motorEnabled = true;
            if (sys.motorSpeed == 0) sys.motorSpeed = 50;
            Serial.println("Motor started");
            
        } else if (cmd == "STOP") {
            sys.motorEnabled = false;
            Serial.println("Motor stopped");
            
        } else if (cmd.startsWith("SPEED:")) {
            int speed = cmd.substring(6).toInt();
            setMotorSpeed(speed);
            
        } else if (cmd == "DIR") {
            changeDirection();
            
        } else if (cmd.startsWith("AUTO:")) {
            sys.autoMode = (cmd.substring(5) == "ON");
            Serial.println("Auto mode: " + String(sys.autoMode ? "ON" : "OFF"));
            
        } else if (cmd == "EMERGENCY") {
            emergencyStop();
            
        } else if (cmd.startsWith("SERVO:")) {
            int angle = cmd.substring(6).toInt();
            setServoAngle(angle);
        }
    }
}

void processAutoMode() {
    if (!sys.autoMode) return;
    
    static unsigned long lastAutoUpdate = 0;
    if (millis() - lastAutoUpdate < 5000) return;  // 5초마다 업데이트
    
    // 온도 기반 속도 조절
    float tempDiff = sys.temperature - autoSettings.targetTemp;
    
    if (tempDiff > autoSettings.tempThreshold) {
        // 너무 더우면 속도 증가
        int newSpeed = map(tempDiff * 10, 0, 100, autoSettings.minSpeed, autoSettings.maxSpeed);
        newSpeed = constrain(newSpeed, autoSettings.minSpeed, autoSettings.maxSpeed);
        
        if (!sys.motorEnabled) {
            sys.motorEnabled = true;
        }
        setMotorSpeed(newSpeed);
        
    } else if (tempDiff < -autoSettings.tempThreshold) {
        // 충분히 시원하면 속도 감소 또는 정지
        if (sys.motorSpeed > autoSettings.minSpeed) {
            setMotorSpeed(autoSettings.minSpeed);
        } else {
            sys.motorEnabled = false;
        }
    }
    
    // 습도 기반 풍향 조절
    if (sys.humidity > 70) {
        // 습도가 높으면 좌우로 흔들기
        static bool swingDirection = true;
        int angle = swingDirection ? 45 : 135;
        setServoAngle(angle);
        swingDirection = !swingDirection;
    } else {
        // 정면 방향
        setServoAngle(90);
    }
    
    lastAutoUpdate = millis();
}

void performSafetyCheck() {
    bool newSafetyMode = false;
    
    // 과전류 검사
    if (sys.current > 1.5) {
        Serial.println("SAFETY: Over current!");
        newSafetyMode = true;
    }
    
    // 과열 검사
    if (sys.temperature > 55) {
        Serial.println("SAFETY: Over temperature!");
        newSafetyMode = true;
    }
    
    // 안전 모드 전환
    if (newSafetyMode && !sys.safetyMode) {
        sys.safetyMode = true;
        sys.motorEnabled = false;
        playAlarmSound();
    } else if (!newSafetyMode && sys.safetyMode) {
        // 자동 복구
        if (sys.temperature < 50 && sys.current < 1.2) {
            sys.safetyMode = false;
            playRecoverySound();
        }
    }
}

void setMotorSpeed(int speed) {
    speed = constrain(speed, 0, 100);
    sys.motorSpeed = map(speed, 0, 100, 0, 255);
    
    // 야간 모드에서는 최대 50%로 제한
    if (sys.nightMode && sys.motorSpeed > 127) {
        sys.motorSpeed = 127;
    }
    
    Serial.println("Speed set to: " + String(speed) + "%");
}

void setServoAngle(int angle) {
    angle = constrain(angle, 0, 180);
    sys.servoAngle = angle;
    windServo.write(angle);
    Serial.println("Wind direction set to: " + String(angle) + "°");
}

void changeDirection() {
    // 안전한 방향 변경
    bool wasEnabled = sys.motorEnabled;
    int oldSpeed = sys.motorSpeed;
    
    sys.motorEnabled = false;
    updateMotor();
    delay(500);
    
    sys.motorDirection = !sys.motorDirection;
    
    sys.motorEnabled = wasEnabled;
    sys.motorSpeed = oldSpeed;
    
    Serial.println("Direction changed");
}

void updateMotor() {
    if (sys.motorEnabled && !sys.safetyMode && sys.motorSpeed > 0) {
        analogWrite(MOTOR_PWM, sys.motorSpeed);
        digitalWrite(MOTOR_DIR, sys.motorDirection ? HIGH : LOW);
    } else {
        analogWrite(MOTOR_PWM, 0);
    }
}

void updateLEDs() {
    // 전원 LED (항상 켜짐)
    digitalWrite(LED_POWER, HIGH);
    
    // 경고 LED (안전 모드)
    if (sys.safetyMode) {
        digitalWrite(LED_WARNING, (millis() / 100) % 2);  // 빠른 깜빡임
    } else {
        digitalWrite(LED_WARNING, LOW);
    }
    
    // 상태 LED (모터 동작 상태)
    if (sys.motorEnabled && sys.motorSpeed > 0) {
        int blinkRate = map(sys.motorSpeed, 0, 255, 1000, 100);
        digitalWrite(LED_STATUS, (millis() / blinkRate) % 2);
    } else {
        digitalWrite(LED_STATUS, LOW);
    }
}

void sendDataToESP32() {
    DynamicJsonDocument doc(1024);
    
    doc["temperature"] = sys.temperature;
    doc["humidity"] = sys.humidity;
    doc["current"] = sys.current;
    doc["light"] = sys.lightLevel;
    doc["speed"] = map(sys.motorSpeed, 0, 255, 0, 100);
    doc["enabled"] = sys.motorEnabled;
    doc["direction"] = sys.motorDirection;
    doc["servo"] = sys.servoAngle;
    doc["auto"] = sys.autoMode;
    doc["eco"] = sys.ecoMode;
    doc["night"] = sys.nightMode;
    doc["safety"] = sys.safetyMode;
    doc["runtime"] = sys.runtime;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    Serial.println("DATA:" + jsonString);
}

void enableNightMode() {
    sys.nightMode = true;
    Serial.println("Night mode enabled");
    playNightModeSound();
}

void disableNightMode() {
    sys.nightMode = false;
    Serial.println("Night mode disabled");
}

void emergencyStop() {
    sys.motorEnabled = false;
    sys.motorSpeed = 0;
    analogWrite(MOTOR_PWM, 0);
    
    playEmergencySound();
    Serial.println("EMERGENCY STOP ACTIVATED");
}

// 사운드 함수들
void playStartupSound() {
    tone(BUZZER_PIN, 1000, 100);
    delay(150);
    tone(BUZZER_PIN, 1500, 100);
    delay(150);
    tone(BUZZER_PIN, 2000, 200);
}

void playAlarmSound() {
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 800, 200);
        delay(250);
        tone(BUZZER_PIN, 1200, 200);
        delay(250);
    }
}

void playRecoverySound() {
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
    tone(BUZZER_PIN, 2500, 200);
}

void playNightModeSound() {
    tone(BUZZER_PIN, 1500, 50);
    delay(100);
    tone(BUZZER_PIN, 1000, 50);
}

void playEmergencySound() {
    for (int i = 0; i < 5; i++) {
        tone(BUZZER_PIN, 2500, 100);
        delay(100);
    }
}
```

---

## 📅 7일차: 케이스 제작 및 마무리

### 📦 7-1. 3D 프린팅 케이스 설계 (60분)

**케이스 설계 사양**:
```
메인 케이스 (Arduino + HW-020):
• 크기: 120mm × 80mm × 40mm
• 재질: PLA 또는 PETG
• 특징: 방열 홀, LED 창, 버튼 홀

센서 모듈 케이스:
• 크기: 60mm × 40mm × 20mm  
• 재질: PLA
• 특징: DHT22 환기구, 조도센서 창

ESP32 케이스:
• 크기: 70mm × 50mm × 25mm
• 재질: PLA
• 특징: WiFi 안테나 공간, USB 포트

3D 프린팅 설정:
• 레이어 높이: 0.2mm
• 인필: 20%
• 서포트: 필요시
• 프린팅 시간: 총 4-6시간
```

### 🔧 7-2. 최종 조립 및 테스트 (120분)

**최종 조립 체크리스트**:
```
하드웨어 조립:
□ 모든 연결부 재확인
□ 케이블 정리 및 고정
□ 케이스 내부 배치 최적화
□ 방열 및 환기 확인

소프트웨어 검증:
□ Arduino 코드 업로드 완료
□ ESP32 코드 업로드 완료
□ WiFi 연결 테스트
□ 웹 인터페이스 접속 확인

기능 테스트:
□ 모터 제어 (시작/정지/속도/방향)
□ 센서 데이터 읽기 (온도/습도/전류/조도)
□ 서보 모터 풍향 조절
□ LED 상태 표시
□ 부저 알림 소리
□ 자동 모드 동작
□ 안전 시스템 동작
□ WebSocket 실시간 통신

성능 테스트:
□ 연속 동작 1시간 테스트
□ 부하 테스트 (최대 속도)
□ 온도 상승 측정
□ 전력 소비 측정
□ WiFi 연결 안정성
□ 응답 시간 측정
```

**성능 평가 기준**:
```
기본 성능:
• 모터 시작 시간: < 2초
• 속도 변경 응답: < 1초
• 센서 데이터 갱신: 2초 주기
• WiFi 응답 시간: < 500ms

안정성:
• 연속 동작: 8시간 이상
• 온도 상승: < 15°C (주변 온도 대비)
• 전력 소비: < 25W (최대 부하)
• 메모리 사용: < 80% (Arduino/ESP32)

안전성:
• 과전류 보호: 1.5A에서 즉시 정지
• 과열 보호: 55°C에서 즉시 정지
• 비상 정지: < 100ms 응답
• 복구 기능: 자동 복구 동작
```

---

## 🎓 학습 성과 및 다음 단계

### 📊 습득한 기술 스킬

**하드웨어 스킬**:
- ✅ Arduino 보드 활용 및 프로그래밍
- ✅ ESP32 WiFi/Bluetooth 통신
- ✅ 센서 인터페이싱 (DHT22, ACS712, LDR)
- ✅ 모터 제어 (PWM, 방향 제어)
- ✅ 서보 모터 제어
- ✅ 전자회로 설계 및 조립
- ✅ 브레드보드 프로토타이핑

**소프트웨어 스킬**:
- ✅ Arduino IDE 및 C++ 프로그래밍
- ✅ JSON 데이터 처리
- ✅ 시리얼 통신 프로토콜
- ✅ WebSocket 실시간 통신
- ✅ 반응형 웹 인터페이스 개발
- ✅ 인터럽트 및 타이머 프로그래밍
- ✅ 안전 시스템 설계

**시스템 스킬**:
- ✅ IoT 시스템 아키텍처 설계
- ✅ 데이터 수집 및 처리
- ✅ 실시간 모니터링 시스템
- ✅ 자동 제어 알고리즘
- ✅ 안전 시스템 구현
- ✅ 문제 해결 및 디버깅

### 🚀 추천 다음 프로젝트

**초급 프로젝트**:
1. **온도 기반 LED 컨트롤러** - 온도에 따라 색상이 변하는 RGB LED
2. **자동 화분 급수 시스템** - 토양 습도 센서로 자동 급수
3. **초음파 거리계** - HC-SR04로 거리 측정 및 표시

**중급 프로젝트**:
1. **스마트 도어락** - RFID/지문 인식 보안 시스템
2. **환경 모니터링 스테이션** - 다중 센서 데이터 클라우드 전송
3. **로봇 청소기** - 장애물 회피 및 자동 청소

**고급 프로젝트**:
1. **AI 기반 홈 보안 시스템** - 얼굴 인식 및 침입 감지
2. **드론 자동 비행 시스템** - GPS 기반 자율 비행
3. **스마트 농장 시스템** - 완전 자동화 식물 재배

### 📚 추가 학습 자료

**온라인 자료**:
- [Arduino 공식 튜토리얼](https://www.arduino.cc/en/Tutorial)
- [ESP32 개발 가이드](https://docs.espressif.com/projects/esp-idf/)
- [Adafruit Learning System](https://learn.adafruit.com/)

**추천 도서**:
- "Arduino Cookbook" by Michael Margolis
- "Programming Arduino" by Simon Monk
- "ESP32 for IoT" by Venkatesh Varadachari

**커뮤니티**:
- [Arduino 포럼](https://forum.arduino.cc/)
- [ESP32 커뮤니티](https://www.esp32.com/)
- [Reddit r/arduino](https://www.reddit.com/r/arduino/)

---

**🎉 축하합니다! 스마트 선풍기 v3.0 제작 완료! 🎉**

7일간의 여정을 통해 단순한 부품들로 완전한 IoT 시스템을 만들어냈습니다. 이제 여러분은 더 복잡한 프로젝트에 도전할 준비가 되었습니다! 🚀

**계속해서 만들고, 배우고, 혁신하세요!** 💪