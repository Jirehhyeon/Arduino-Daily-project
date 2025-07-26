# 🎓 초보자를 위한 스마트 선풍기 학습 가이드

**Arduino를 처음 접하는 분들을 위한 완전 단계별 학습 과정**

> 전자공학 지식이 전혀 없어도 따라할 수 있도록 구성된 체계적인 학습 가이드입니다.

## 🎯 학습 목표

이 가이드를 완료하면 다음을 할 수 있게 됩니다:
- ✅ Arduino 기본 개념 이해
- ✅ 기본 전자 부품 사용법
- ✅ 센서 데이터 읽기
- ✅ 모터 제어하기
- ✅ WiFi로 스마트폰과 연결하기
- ✅ 완전한 IoT 시스템 구축하기

## 📚 학습 과정 개요

```
📖 1단계: 기초 지식 (1-2일)
├── Arduino란 무엇인가?
├── 전기의 기본 개념
├── 필수 도구와 부품
└── 안전 수칙

⚡ 2단계: 첫 번째 프로젝트 (1일)
├── LED 켜기/끄기
├── 버튼으로 LED 제어
└── 시리얼 통신 기초

🔧 3단계: 센서와 액추에이터 (2-3일)
├── 온도 센서 사용하기
├── 모터 기초 제어
├── 서보 모터 제어
└── LCD 디스플레이

🌐 4단계: 무선 통신 (2-3일)
├── ESP32 기초
├── WiFi 연결
├── 웹 서버 만들기
└── 스마트폰 연동

🎯 5단계: 최종 프로젝트 (3-4일)
├── 시스템 통합
├── 안전 기능 추가
├── 최적화
└── 테스트 및 완성
```

---

# 📖 1단계: 기초 지식 (1-2일)

## 🤔 Arduino란 무엇인가?

### Arduino의 정의
Arduino는 **마이크로컨트롤러**라는 작은 컴퓨터를 쉽게 사용할 수 있게 만든 **보드**입니다.

```
일반 컴퓨터 vs Arduino:

💻 일반 컴퓨터:
- 화면, 키보드, 마우스 필요
- 인터넷, 게임, 문서 작업
- 복잡하고 크고 전력 많이 사용

🔧 Arduino:
- 센서, LED, 모터 연결
- 자동화, 로봇, IoT 제어
- 간단하고 작고 전력 적게 사용
```

### Arduino로 할 수 있는 것들
- 🏠 **스마트 홈**: 조명, 온도, 보안 시스템
- 🚗 **로봇**: 자율주행 로봇, 드론
- 🌱 **농업**: 자동 물주기, 온실 관리
- 🎵 **예술**: 인터랙티브 설치물, 음악 장치

## ⚡ 전기의 기본 개념

### 물과 전기의 비유로 이해하기

```
물의 흐름 = 전기의 흐름

🚰 수압 (압력) = 전압 (Voltage, V)
- 물이 흐르게 하는 힘
- 전기가 흐르게 하는 힘
- 단위: 볼트 (V)

💧 물의 양 = 전류 (Current, A)  
- 실제로 흐르는 물의 양
- 실제로 흐르는 전기의 양
- 단위: 암페어 (A)

🚪 파이프 굵기 = 저항 (Resistance, Ω)
- 물의 흐름을 방해하는 정도
- 전기의 흐름을 방해하는 정도  
- 단위: 옴 (Ω)
```

### 옴의 법칙 (간단히!)
```
전압 = 전류 × 저항
V = I × R

🔋 배터리 5V에 저항 1000Ω 연결하면
→ 전류 = 5V ÷ 1000Ω = 0.005A = 5mA
```

### 기본 단위들
```
전압 (V):
- 1.5V: AA 건전지
- 5V: Arduino 동작 전압
- 12V: 자동차 배터리
- 220V: 집 콘센트 (위험!)

전류 (A):
- 1mA = 0.001A (LED 정도)
- 100mA = 0.1A (작은 모터)
- 1A = 큰 모터나 여러 부품

저항 (Ω):
- 220Ω: LED 전류 제한용
- 10kΩ: 버튼 풀업/풀다운
- 1MΩ = 1,000,000Ω (매우 큰 저항)
```

## 🛠️ 필수 도구와 부품

### 기본 도구
```
🔧 필수 도구:
□ 납땜인두 (30-40W)
□ 납땜선 (0.6-0.8mm)
□ 니퍼 (전선 자르기)
□ 스트리퍼 (전선 피복 벗기기)
□ 멀티미터 (전압/전류 측정)
□ 브레드보드 (납땜 없이 연결)
□ 점퍼선 세트
□ 나사드라이버 세트

💰 예산: 약 5-10만원
```

### 학습용 부품 키트
```
📦 Arduino 스타터 키트 (추천):

🔧 메인 보드:
□ Arduino Uno R3 × 1
□ USB 케이블 × 1
□ 브레드보드 × 1
□ 점퍼선 세트

💡 기본 부품:
□ LED (빨강, 노랑, 초록) × 각 5개
□ 저항 (220Ω, 1kΩ, 10kΩ) × 각 10개  
□ 택트 스위치 × 5개
□ 포텐셔미터 10kΩ × 1
□ 부저 × 1

📊 센서들:
□ DHT22 온습도 센서 × 1
□ 초음파 센서 HC-SR04 × 1
□ 조도 센서 (LDR) × 1
□ 가변저항 × 2

🔧 액추에이터:
□ SG90 서보모터 × 1
□ DC 모터 소형 × 1
□ 모터 드라이버 L298N × 1

💰 예산: 약 3-5만원 (인터넷 쇼핑몰)
```

### 부품 구매 가이드
```
🛒 추천 쇼핑몰:
- 디바이스마트 (devicemart.co.kr)
- 엘레파츠 (eleparts.co.kr)  
- 아두이노코리아 (arduino.co.kr)
- 알리익스프레스 (저렴하지만 배송 오래)

💡 구매 팁:
- 스타터 키트로 시작 (개별 구매보다 저렴)
- 점퍼선은 여분으로 많이 구매
- 저항은 키트로 구매 (여러 값이 필요)
- LED는 소모품이므로 넉넉히 구매
```

## ⚠️ 안전 수칙

### 전기 안전
```
🚨 절대 하지 말 것:
❌ 젖은 손으로 전기 부품 만지기
❌ 220V 콘센트에 Arduino 연결
❌ 배터리 합선시키기 (폭발 위험)
❌ 극성 바꿔서 연결하기 (부품 손상)

✅ 반드시 지킬 것:
✓ 작업 전 전원 차단
✓ 연결 전 극성 확인 (빨강=+, 검정=-)
✓ 멀티미터로 전압 확인
✓ 단락(합선) 없는지 확인
```

### 부품 보호
```
🔧 Arduino 보호:
- USB로만 전원 공급 (처음에는)
- 5V 이상 전압 직접 연결 금지
- 핀에 과전류 흘리지 않기 (최대 40mA)

📦 부품 보관:
- 정전기 방지 봉투에 보관
- 습기 없는 곳에 보관
- 핀 구부러지지 않게 주의
```

---

# ⚡ 2단계: 첫 번째 프로젝트 (1일)

## 🚀 Arduino IDE 설치 및 설정

### Arduino IDE 다운로드
```
1. arduino.cc 접속
2. "Software" → "Downloads" 클릭
3. 운영체제에 맞는 버전 다운로드
4. 설치 후 실행
```

### 첫 설정
```
🔧 보드 설정:
도구 → 보드 → Arduino Uno 선택

🔌 포트 설정:
도구 → 포트 → COM3 (번호는 다를 수 있음)

📝 언어 설정:
File → Preferences → Language → 한국어 (선택사항)
```

## 💡 프로젝트 1: LED 켜기

### 1.1 회로 구성 (브레드보드)

```
LED 연결 방법:

Arduino Uno
    ┌─────┐
    │ 13 ●─────┬─── 220Ω 저항 ─── LED 긴 다리 (+)
    │     │     │
    │ GND●─────┴─────────────── LED 짧은 다리 (-)
    └─────┘

📝 중요 포인트:
- LED 긴 다리(+)가 저항을 통해 13번 핀으로
- LED 짧은 다리(-)가 GND로
- 저항 없으면 LED 타버림!
```

### 1.2 브레드보드 사용법

```
브레드보드 구조:

    + - - - - - - - - - - - - - - +
    a b c d e   f g h i j
    ○ ○ ○ ○ ○   ○ ○ ○ ○ ○  ← 1행
    ○ ○ ○ ○ ○   ○ ○ ○ ○ ○  ← 2행
    ○ ○ ○ ○ ○   ○ ○ ○ ○ ○  ← 3행
    ...

연결 규칙:
- 같은 행의 a-e는 내부적으로 연결됨
- 같은 행의 f-j는 내부적으로 연결됨  
- 가운데 홈으로 분리됨
- 위/아래 전원 레일(+,-)은 가로로 연결
```

### 1.3 첫 번째 코드

```cpp
// LED 깜빡이 프로그램 (Blink)

void setup() {
  // 13번 핀을 출력으로 설정
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);  // LED 켜기
  delay(1000);             // 1초 대기
  digitalWrite(13, LOW);   // LED 끄기  
  delay(1000);             // 1초 대기
}
```

### 1.4 코드 업로드 및 테스트

```
📝 업로드 순서:
1. Arduino를 USB로 컴퓨터 연결
2. Arduino IDE에서 코드 입력
3. 확인 버튼 클릭 (컴파일)
4. 업로드 버튼 클릭 (→)
5. LED가 1초 간격으로 깜빡이는지 확인

❗ 오류 시 체크사항:
- 보드 선택 확인 (Arduino Uno)
- 포트 선택 확인 (COM번호)
- USB 케이블 연결 확인
- 코드 오타 확인
```

## 🔘 프로젝트 2: 버튼으로 LED 제어

### 2.1 회로 구성

```
버튼 + LED 회로:

Arduino Uno
    ┌─────┐
    │  2 ●─────┬─── 버튼 ─── GND
    │     │     │
    │     │     └─── 10kΩ 저항 ─── 5V (풀업 저항)
    │     │
    │ 13 ●─────── 220Ω 저항 ─── LED ─── GND
    └─────┘

💡 풀업 저항이 필요한 이유:
버튼을 누르지 않았을 때 핀이 불안정한 상태가 되는 것을 방지
```

### 2.2 풀업 저항 이해하기

```
풀업 저항 없을 때:
버튼 안 누름 → 핀이 허공에 떠있음 → 0인지 1인지 모름 (플로팅)

풀업 저항 있을 때:  
버튼 안 누름 → 저항을 통해 5V 연결 → 핀이 HIGH (1)
버튼 누름 → GND로 직접 연결 → 핀이 LOW (0)
```

### 2.3 코드 작성

```cpp
// 버튼으로 LED 제어

const int buttonPin = 2;    // 버튼이 연결된 핀
const int ledPin = 13;      // LED가 연결된 핀

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);  // 내장 풀업 저항 사용
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);  // 시리얼 통신 시작 (디버깅용)
}

void loop() {
  int buttonState = digitalRead(buttonPin);  // 버튼 상태 읽기
  
  if (buttonState == LOW) {  // 버튼이 눌렸을 때 (풀업이므로 LOW)
    digitalWrite(ledPin, HIGH);  // LED 켜기
    Serial.println("버튼이 눌렸습니다 - LED ON");
  } else {
    digitalWrite(ledPin, LOW);   // LED 끄기
    Serial.println("버튼이 안 눌렸습니다 - LED OFF");
  }
  
  delay(100);  // 100ms 대기 (디바운싱)
}
```

### 2.4 시리얼 모니터 사용하기

```
🖥️ 시리얼 모니터 열기:
도구 → 시리얼 모니터 (또는 Ctrl+Shift+M)

⚙️ 설정:
- 보드레이트: 9600 선택
- 줄 끝: "개행 문자" 선택

👀 확인할 것:
- 버튼 누를 때 "버튼이 눌렸습니다" 메시지
- 버튼 뗄 때 "버튼이 안 눌렸습니다" 메시지
- LED가 버튼에 따라 켜지고 꺼지는지
```

---

# 🔧 3단계: 센서와 액추에이터 (2-3일)

## 🌡️ 프로젝트 3: 온도 센서 사용하기

### 3.1 DHT22 센서 이해하기

```
DHT22 센서란?
- 온도와 습도를 동시에 측정
- 디지털 신호로 데이터 전송
- 정확도: ±0.5°C, ±2%RH
- 3개 핀: VCC(전원), GND(접지), DATA(데이터)

핀 구성:
DHT22
┌─────┐
│  1  │ ── VCC (3.3V 또는 5V)
│  2  │ ── DATA (데이터 핀)  
│  3  │ ── 사용 안함
│  4  │ ── GND (접지)
└─────┘
```

### 3.2 라이브러리 설치

```
📚 라이브러리 설치 방법:
1. Arduino IDE에서 "도구" → "라이브러리 관리"
2. "DHT sensor library" 검색
3. "Adafruit" 제작자의 라이브러리 설치
4. 종속성 라이브러리도 함께 설치 (Adafruit Unified Sensor)
```

### 3.3 회로 연결

```
DHT22 연결:

Arduino Uno         DHT22
    ┌─────┐         ┌─────┐
    │  5V ●─────────● VCC  │
    │ GND ●─────────● GND  │  
    │  7  ●─────────● DATA │
    └─────┘         └─────┘

추가 연결 (권장):
DATA ── 10kΩ 저항 ── VCC (풀업 저항)
```

### 3.4 온도 센서 코드

```cpp
// DHT22 온습도 센서 사용하기

#include "DHT.h"

#define DHTPIN 7        // DHT22가 연결된 핀
#define DHTTYPE DHT22   // 센서 타입

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHT22 온습도 센서 테스트");
  dht.begin();
}

void loop() {
  // 센서 읽기 (2초 정도 걸림)
  delay(2000);
  
  float humidity = dht.readHumidity();        // 습도 읽기
  float temperature = dht.readTemperature();  // 온도 읽기 (섭씨)
  
  // 읽기 실패 체크
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("센서 읽기 실패!");
    return;
  }
  
  // 결과 출력
  Serial.print("습도: ");
  Serial.print(humidity);
  Serial.print("%  온도: ");
  Serial.print(temperature);
  Serial.println("°C");
  
  // 온도에 따른 LED 제어
  if (temperature > 25) {
    digitalWrite(13, HIGH);  // 25도 이상이면 LED 켜기
    Serial.println("🔥 더워요! LED ON");
  } else {
    digitalWrite(13, LOW);   // 25도 미만이면 LED 끄기
    Serial.println("❄️ 시원해요! LED OFF");
  }
}
```

### 3.5 테스트 및 디버깅

```
🧪 테스트 방법:
1. 센서에 입김 불어보기 (온도/습도 변화 확인)
2. 손으로 센서 감싸기 (온도 변화 확인)
3. 시리얼 모니터로 실시간 값 확인

❗ 문제 해결:
- "센서 읽기 실패!" 메시지 → 연결 재확인
- 값이 이상함 → 2초 이상 간격으로 읽기
- 라이브러리 오류 → 재설치
```

## 🔄 프로젝트 4: 모터 기초 제어

### 4.1 DC 모터와 PWM 이해하기

```
DC 모터란?
- Direct Current Motor (직류 모터)
- 전압을 가하면 회전
- 전압 크기 = 속도, 극성 = 방향

PWM (Pulse Width Modulation):
- 디지털 신호로 아날로그 효과 내기
- 켜짐/꺼짐을 빠르게 반복
- 듀티 사이클 = 켜진 시간의 비율

PWM 0%:   ______________________ (항상 꺼짐)
PWM 25%:  ▅▅▅▅__________________ (25% 켜짐)  
PWM 50%:  ▅▅▅▅▅▅▅▅▅▅____________ (50% 켜짐)
PWM 75%:  ▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅______ (75% 켜짐)
PWM 100%: ▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅▅ (항상 켜짐)
```

### 4.2 트랜지스터를 이용한 모터 제어

```
왜 트랜지스터가 필요한가?
- Arduino 핀 최대 전류: 40mA
- 모터 필요 전류: 100mA 이상
- 트랜지스터 = 전류 증폭기 역할

기본 회로:

      5V ──┬── 모터 (+)
           │
           └── 트랜지스터 컬렉터
               │
Arduino 9 ──── 1kΩ ──── 트랜지스터 베이스
               │
              GND ──── 트랜지스터 이미터
                   │
                   └── 모터 (-)
```

### 4.3 간단한 모터 제어 코드

```cpp
// DC 모터 PWM 제어

const int motorPin = 9;    // PWM 핀에 연결
const int potPin = A0;     // 포텐셔미터로 속도 조절

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);           // 0-1023 값 읽기
  int motorSpeed = map(potValue, 0, 1023, 0, 255);  // 0-255로 변환
  
  analogWrite(motorPin, motorSpeed);           // PWM 출력
  
  Serial.print("포텐셔미터: ");
  Serial.print(potValue);
  Serial.print(" → 모터 속도: ");
  Serial.println(motorSpeed);
  
  delay(100);
}
```

## 🎛️ 프로젝트 5: 서보 모터 제어

### 5.1 서보 모터 이해하기

```
서보 모터 vs DC 모터:

DC 모터:
- 계속 회전
- 속도 제어 가능
- 정확한 위치 제어 어려움

서보 모터:
- 180도 범위 내에서 정확한 각도 제어
- PWM 신호로 각도 지정
- 내부에 위치 센서와 제어 회로 내장

서보 핀:
갈색 선: GND
빨간 선: 5V
노란 선: 신호 (PWM)
```

### 5.2 서보 제어 회로

```
서보 모터 연결:

Arduino Uno         SG90 서보
    ┌─────┐         ┌─────┐
    │  5V ●─────────● 빨간 선 (VCC)
    │ GND ●─────────● 갈색 선 (GND)
    │  9  ●─────────● 노란 선 (신호)
    └─────┘         └─────┘

💡 전원 주의사항:
- 큰 서보는 별도 전원 필요 (Arduino 5V로 부족)
- 여러 서보 사용시 외부 5V 전원 권장
```

### 5.3 서보 제어 코드

```cpp
// 서보 모터 제어

#include <Servo.h>

Servo myServo;              // 서보 객체 생성
const int potPin = A0;      // 포텐셔미터 핀

void setup() {
  myServo.attach(9);        // 9번 핀에 서보 연결
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);                    // 0-1023
  int angle = map(potValue, 0, 1023, 0, 180);         // 0-180도로 변환
  
  myServo.write(angle);                                // 서보를 해당 각도로 이동
  
  Serial.print("포텐셔미터: ");
  Serial.print(potValue);
  Serial.print(" → 서보 각도: ");
  Serial.println(angle);
  
  delay(15);  // 서보가 움직일 시간
}
```

### 5.4 서보 자동 스윕 코드

```cpp
// 서보 자동 스윕 (왔다갔다)

#include <Servo.h>

Servo myServo;
int currentAngle = 0;    // 현재 각도
int direction = 1;       // 방향 (1: 증가, -1: 감소)

void setup() {
  myServo.attach(9);
  Serial.begin(9600);
}

void loop() {
  myServo.write(currentAngle);
  
  Serial.print("각도: ");
  Serial.println(currentAngle);
  
  // 각도 업데이트
  currentAngle += direction * 2;  // 2도씩 이동
  
  // 방향 전환
  if (currentAngle >= 180) {
    direction = -1;
  } else if (currentAngle <= 0) {
    direction = 1;
  }
  
  delay(50);  // 부드러운 움직임을 위한 딜레이
}
```

## 📺 프로젝트 6: LCD 디스플레이

### 6.1 I2C LCD 이해하기

```
일반 LCD vs I2C LCD:

일반 LCD:
- 16개 핀 연결 필요
- Arduino 핀 많이 사용
- 배선 복잡

I2C LCD:
- 4개 핀만 연결 (VCC, GND, SDA, SCL)
- I2C 통신 사용
- 배선 간단
- 여러 I2C 장치 연결 가능

I2C 주소:
- 각 I2C 장치는 고유 주소 가짐
- 일반적으로 0x27 또는 0x3F
```

### 6.2 I2C LCD 연결

```
I2C LCD 연결:

Arduino Uno         I2C LCD
    ┌─────┐         ┌─────┐
    │  5V ●─────────● VCC  │
    │ GND ●─────────● GND  │
    │ A4  ●─────────● SDA  │ (데이터)
    │ A5  ●─────────● SCL  │ (클럭)
    └─────┘         └─────┘

💡 I2C 핀 위치 (Arduino Uno):
- SDA: A4 핀
- SCL: A5 핀
- 다른 Arduino는 핀 위치 다를 수 있음
```

### 6.3 라이브러리 설치 및 주소 찾기

```cpp
// I2C 스캐너 - LCD 주소 찾기

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C 주소 스캔 중...");
}

void loop() {
  byte error, address;
  int deviceCount = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C 장치 발견! 주소: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("I2C 장치 없음");
  }
  
  delay(5000);
}
```

### 6.4 LCD 기본 사용법

```cpp
// I2C LCD 기본 사용

#include <LiquidCrystal_I2C.h>

// LCD 주소는 스캐너로 찾은 값 사용 (보통 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // 주소, 가로, 세로

void setup() {
  lcd.init();                  // LCD 초기화
  lcd.backlight();             // 백라이트 켜기
  
  lcd.setCursor(0, 0);         // 첫 번째 줄, 첫 번째 칸
  lcd.print("Hello, World!");
  
  lcd.setCursor(0, 1);         // 두 번째 줄, 첫 번째 칸
  lcd.print("Arduino LCD");
}

void loop() {
  // 메인 루프는 비워둠
}
```

### 6.5 LCD + 센서 통합

```cpp
// LCD에 온도 표시

#include <DHT.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN 7
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  dht.begin();
  lcd.init();
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("Smart Thermometer");
  delay(2000);
  lcd.clear();
}

void loop() {
  float temp = dht.readTemperature();
  float humid = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(humid)) {
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp, 1);  // 소수점 1자리
    lcd.print("C");
    
    lcd.setCursor(0, 1);
    lcd.print("Humid: ");
    lcd.print(humid, 1);
    lcd.print("%");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
  }
  
  delay(2000);
}
```

---

# 🌐 4단계: 무선 통신 (2-3일)

## 📡 ESP32 소개 및 설정

### 4.1 ESP32란?

```
ESP32 특징:
- WiFi + 블루투스 내장
- Arduino보다 빠른 프로세서 (듀얼 코어)
- 더 많은 메모리
- Arduino IDE에서 사용 가능
- 저렴한 가격

ESP32 vs Arduino Uno:
항목        Arduino Uno    ESP32
프로세서    8bit 16MHz     32bit 240MHz
메모리      2KB           520KB
WiFi        없음          있음
블루투스    없음          있음
핀 개수     14개          30개 이상
```

### 4.2 ESP32 보드 패키지 설치

```
📦 ESP32 설정 순서:
1. Arduino IDE 열기
2. 파일 → 환경설정
3. "추가적인 보드 매니저 URLs"에 추가:
   https://dl.espressif.com/dl/package_esp32_index.json
4. 도구 → 보드 매니저
5. "ESP32" 검색 후 설치
6. 도구 → 보드 → ESP32 Arduino → ESP32 Dev Module 선택
```

### 4.3 ESP32 핀아웃 이해하기

```
ESP32 DevKit v1 주요 핀:

전원:
- 3V3: 3.3V 출력
- 5V: 5V 입력/출력
- GND: 접지

GPIO (입출력):
- GPIO2: 내장 LED
- GPIO0: 부팅 모드 선택 (일반적으로 건드리지 않음)
- GPIO4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23: 자유롭게 사용

ADC (아날로그 입력):
- A0 (GPIO36), A3 (GPIO39), A6 (GPIO34), A7 (GPIO35)

PWM:
- 거의 모든 GPIO 핀에서 사용 가능

I2C:
- SDA: GPIO21
- SCL: GPIO22
```

## 🔗 프로젝트 7: WiFi 연결하기

### 7.1 기본 WiFi 연결

```cpp
// ESP32 WiFi 기본 연결

#include <WiFi.h>

const char* ssid = "your_wifi_name";      // WiFi 이름
const char* password = "your_password";   // WiFi 비밀번호

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("WiFi 연결 시작...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
  Serial.print("신호 강도: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void loop() {
  // WiFi 연결 상태 확인
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi 연결됨");
  } else {
    Serial.println("WiFi 연결 끊어짐");
  }
  delay(5000);
}
```

### 7.2 WiFi 상태 해석

```
WiFi 상태 코드:
WL_CONNECTED (3): 연결됨
WL_NO_SSID_AVAIL (1): SSID를 찾을 수 없음
WL_CONNECT_FAILED (4): 연결 실패 (비밀번호 오류)
WL_CONNECTION_LOST (5): 연결 끊어짐
WL_DISCONNECTED (6): 연결 해제

신호 강도 해석:
-30 to -50 dBm: 매우 좋음
-50 to -60 dBm: 좋음  
-60 to -70 dBm: 보통
-70 to -80 dBm: 약함
-80 dBm 이하: 매우 약함
```

## 🌐 프로젝트 8: 웹 서버 만들기

### 8.1 간단한 웹 서버

```cpp
// ESP32 웹 서버 기초

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "your_wifi_name";
const char* password = "your_password";

WebServer server(80);  // 포트 80에서 서버 시작

void setup() {
  Serial.begin(115200);
  
  // WiFi 연결
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("WiFi 연결됨!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  // 웹 서버 라우트 설정
  server.on("/", handleRoot);           // 메인 페이지
  server.on("/led/on", handleLedOn);    // LED 켜기
  server.on("/led/off", handleLedOff);  // LED 끄기
  
  server.begin();
  Serial.println("웹 서버 시작됨!");
}

void loop() {
  server.handleClient();  // 클라이언트 요청 처리
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 웹 서버</h1>";
  html += "<p><a href='/led/on'>LED 켜기</a></p>";
  html += "<p><a href='/led/off'>LED 끄기</a></p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleLedOn() {
  digitalWrite(2, HIGH);  // 내장 LED 켜기
  server.send(200, "text/plain", "LED가 켜졌습니다!");
}

void handleLedOff() {
  digitalWrite(2, LOW);   // 내장 LED 끄기
  server.send(200, "text/plain", "LED가 꺼졌습니다!");
}
```

### 8.2 웹 서버 테스트

```
🧪 테스트 방법:
1. 코드 업로드 후 시리얼 모니터 확인
2. WiFi 연결 성공 후 IP 주소 메모
3. 같은 WiFi에 연결된 컴퓨터나 폰에서 브라우저 열기
4. http://192.168.x.x (표시된 IP 주소) 접속
5. "LED 켜기", "LED 끄기" 링크 클릭 테스트

❗ 문제 해결:
- IP 접속 안됨 → 같은 WiFi 연결 확인
- 페이지 로딩 안됨 → ESP32 재시작
- LED 안 켜짐 → GPIO2 핀 확인
```

## 📱 프로젝트 9: 스마트폰 제어 인터페이스

### 9.1 모바일 친화적 웹 페이지

```cpp
// 모바일 최적화 웹 서버

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "your_wifi_name";
const char* password = "your_password";

WebServer server(80);
int ledState = 0;

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("WiFi 연결됨!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  
  server.begin();
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ESP32 컨트롤러</title>
    <style>
        body { 
            font-family: Arial; 
            text-align: center; 
            margin: 50px;
            background-color: #f0f0f0;
        }
        .button {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 20px 40px;
            text-align: center;
            font-size: 16px;
            margin: 10px;
            border-radius: 10px;
            cursor: pointer;
            width: 200px;
        }
        .button:hover {
            background-color: #45a049;
        }
        .status {
            font-size: 20px;
            margin: 20px;
            padding: 10px;
            background-color: white;
            border-radius: 10px;
        }
    </style>
</head>
<body>
    <h1>🔗 ESP32 컨트롤러</h1>
    <div class="status" id="status">LED 상태: 확인 중...</div>
    <button class="button" onclick="toggleLED()">LED 켜기/끄기</button>
    
    <script>
        function toggleLED() {
            fetch('/toggle')
                .then(response => response.text())
                .then(data => {
                    updateStatus();
                });
        }
        
        function updateStatus() {
            fetch('/status')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = 'LED 상태: ' + data;
                });
        }
        
        // 페이지 로드시 상태 업데이트
        updateStatus();
        
        // 3초마다 상태 업데이트
        setInterval(updateStatus, 3000);
    </script>
</body>
</html>
  )";
  
  server.send(200, "text/html", html);
}

void handleToggle() {
  ledState = !ledState;
  digitalWrite(2, ledState);
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  server.send(200, "text/plain", ledState ? "켜짐 🟢" : "꺼짐 🔴");
}
```

### 9.2 센서 데이터 웹 모니터링

```cpp
// 센서 데이터 웹 모니터링

#include <WiFi.h>
#include <WebServer.h>
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT22

const char* ssid = "your_wifi_name";
const char* password = "your_password";

WebServer server(80);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  dht.begin();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  
  Serial.println("WiFi 연결됨!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  
  server.begin();
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>센서 모니터링</title>
    <style>
        body { 
            font-family: Arial; 
            text-align: center; 
            margin: 20px;
            background-color: #f0f0f0;
        }
        .sensor-box {
            background-color: white;
            margin: 10px;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .temp { color: #ff6b6b; }
        .humid { color: #4ecdc4; }
    </style>
</head>
<body>
    <h1>🌡️ 센서 모니터링</h1>
    <div class="sensor-box">
        <h2 class="temp">온도</h2>
        <div id="temperature" style="font-size: 24px;">--°C</div>
    </div>
    <div class="sensor-box">
        <h2 class="humid">습도</h2>
        <div id="humidity" style="font-size: 24px;">--%</div>
    </div>
    
    <script>
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('temperature').innerHTML = data.temperature + '°C';
                    document.getElementById('humidity').innerHTML = data.humidity + '%';
                });
        }
        
        updateData();
        setInterval(updateData, 2000);  // 2초마다 업데이트
    </script>
</body>
</html>
  )";
  
  server.send(200, "text/html", html);
}

void handleData() {
  float temp = dht.readTemperature();
  float humid = dht.readHumidity();
  
  String json = "{";
  json += "\"temperature\":" + String(temp, 1) + ",";
  json += "\"humidity\":" + String(humid, 1);
  json += "}";
  
  server.send(200, "application/json", json);
}
```

---

# 🎯 5단계: 최종 프로젝트 (3-4일)

## 🏗️ 시스템 통합 계획

### 5.1 전체 시스템 아키텍처

```
스마트 선풍기 시스템 구조:

📱 스마트폰 (웹 브라우저)
    ↕ WiFi
🌐 ESP32 (통신 허브)
    ↕ 시리얼 통신
🔧 Arduino Uno (메인 제어)
    ↕ 센서 및 액추에이터
🌡️ DHT22, 🔄 모터, 🎛️ 서보, 📺 LCD

데이터 흐름:
1. Arduino가 센서 데이터 읽기
2. ESP32로 데이터 전송
3. ESP32가 웹 서버로 데이터 제공
4. 스마트폰에서 실시간 모니터링
5. 스마트폰에서 제어 명령 전송
6. ESP32가 Arduino로 명령 전달
7. Arduino가 모터/서보 제어
```

### 5.2 필요한 부품 최종 점검

```
📦 최종 부품 리스트:

메인 보드:
□ Arduino Uno R3 × 1
□ ESP32 DevKit v1 × 1
□ 브레드보드 대형 × 1 (또는 2개)

센서:
□ DHT22 온습도 센서 × 1
□ ACS712 전류 센서 × 1 (선택사항)
□ LDR 조도 센서 × 1
□ 포텐셔미터 10kΩ × 1

액추에이터:
□ 소형 DC 모터 × 1
□ SG90 서보 모터 × 1
□ I2C LCD 16x2 × 1

제어 및 인터페이스:
□ 택트 스위치 × 2
□ LED (빨강, 노랑, 초록) × 각 2개
□ 부저 × 1

전원 및 제어:
□ 트랜지스터 2N2222 × 1 (모터용)
□ 다이오드 1N4007 × 1 (모터 보호용)
□ 저항 (220Ω, 1kΩ, 10kΩ) × 각 5개
□ 점퍼선 세트
□ 12V 어댑터 × 1 (모터용)

추정 비용: 7-10만원
```

## 🔌 단계별 회로 구성

### 5.3 1단계: 기본 통신 회로

```
Arduino ↔ ESP32 통신 연결:

Arduino Uno    ESP32 DevKit
    ┌─────┐         ┌─────┐
    │ TX 1●─────────● RX2  │ (GPIO16)
    │ RX 0●─────────● TX2  │ (GPIO17)
    │ GND ●─────────● GND  │
    │ 5V  ●─────────● VIN  │ (ESP32 전원)
    └─────┘         └─────┘

💡 주의사항:
- ESP32는 3.3V 로직이지만 5V 톨러런트
- 시리얼 통신 시 Arduino는 프로그래밍 불가
- 프로그래밍 시 ESP32 연결 해제 필요
```

### 5.4 2단계: 센서 회로 추가

```
센서 연결도:

Arduino Uno
    ┌─────┐
    │  5V ●─────┬─── DHT22 VCC
    │ GND ●─────┼─── DHT22 GND
    │     │     ├─── LCD VCC
    │  7  ●─────┼─── DHT22 DATA
    │     │     ├─── LCD GND
    │ A4  ●─────┼─── LCD SDA
    │ A5  ●─────┼─── LCD SCL
    │     │     │
    │ A0  ●─────┼─── 포텐셔미터 와이퍼
    │ A2  ●─────┼─── LDR (조도센서)
    │     │     │
    │  2  ●─────┼─── 비상정지 버튼
    │  4  ●─────┼─── 전원 LED (220Ω 저항 통해)
    │  5  ●─────┼─── 경고 LED (220Ω 저항 통해)
    │  6  ●─────┼─── 상태 LED (220Ω 저항 통해)
    │  8  ●─────┼─── 부저 (+)
    └─────┘     │
                └─── 공통 GND
```

### 5.5 3단계: 모터 제어 회로 추가

```
모터 제어 회로:

              12V 전원
                │
                ├── 모터 (+)
                │    │
            다이오드 ←┘ (보호용, 캐소드가 12V 쪽)
                │
      트랜지스터 컬렉터
          │
Arduino 3 ──1kΩ── 트랜지스터 베이스 (2N2222)
          │
         GND ── 트랜지스터 이미터
          │
          └── 모터 (-)

서보 모터:
Arduino 9 ──── 서보 신호 (노란선)
Arduino 5V ─── 서보 VCC (빨간선)  
Arduino GND ── 서보 GND (갈색선)
```

### 5.6 브레드보드 레이아웃 예시

```
브레드보드 배치 계획:

상단 전원 레일: +5V (빨간선)
하단 전원 레일: GND (검은선)

┌─────────────────────────────────────┐
│ +  +  +  +  +  +  +  +  +  +  +  + │ ← +5V 레일
│ -  -  -  -  -  -  -  -  -  -  -  - │ ← GND 레일
├─────────────────────────────────────┤
│ [DHT22] [LCD] [LED×3] [저항들]      │ ← 상단 영역
│                                     │
│ [포텐셔미터] [버튼] [부저] [LDR]     │ ← 중단 영역  
│                                     │
│ [트랜지스터] [다이오드] [모터 터미널] │ ← 하단 영역
├─────────────────────────────────────┤
│ +  +  +  +  +  +  +  +  +  +  +  + │ ← +12V 레일 (모터용)
│ -  -  -  -  -  -  -  -  -  -  -  - │ ← GND 레일
└─────────────────────────────────────┘

배선 색상 규칙:
🔴 빨강: +5V, +12V (전원)
⚫ 검정: GND (접지)
🔵 파랑: 디지털 신호
🟢 초록: 아날로그 신호
🟡 노랑: PWM 신호
```

## 💻 소프트웨어 통합

### 5.7 Arduino 메인 코드 (간소화 버전)

```cpp
// 스마트 선풍기 Arduino 메인 제어
// 초보자용 간소화 버전

#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// 핀 정의
#define DHT_PIN 7
#define MOTOR_PIN 3
#define SERVO_PIN 9
#define BUTTON_PIN 2
#define LED_POWER 4
#define LED_WARN 5
#define LED_STATUS 6
#define BUZZER_PIN 8
#define POT_PIN A0
#define LIGHT_PIN A2

// 객체 생성
DHT dht(DHT_PIN, DHT22);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo windServo;

// 변수
float temperature = 0;
float humidity = 0;
int fanSpeed = 0;
int windDirection = 90;
bool fanEnabled = false;
bool nightMode = false;
bool emergencyStop = false;

void setup() {
  Serial.begin(115200);
  
  // 핀 설정
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_WARN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 센서 초기화
  dht.begin();
  lcd.init();
  lcd.backlight();
  windServo.attach(SERVO_PIN);
  
  // 시작 메시지
  lcd.setCursor(0, 0);
  lcd.print("Smart Fan v3.0");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  
  // 비상정지 인터럽트
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), emergencyStopISR, FALLING);
  
  delay(2000);
  lcd.clear();
  
  Serial.println("Arduino Ready");
}

void loop() {
  // 센서 데이터 읽기
  readSensors();
  
  // ESP32 통신 처리
  handleCommunication();
  
  // 자동 제어 (야간 모드 등)
  autoControl();
  
  // 모터 제어
  controlMotor();
  
  // LED 상태 업데이트
  updateLEDs();
  
  // LCD 업데이트
  updateLCD();
  
  delay(100);
}

void readSensors() {
  static unsigned long lastRead = 0;
  
  if (millis() - lastRead > 2000) {  // 2초마다 읽기
    lastRead = millis();
    
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
    
    // 조도 센서로 야간 모드 판단
    int lightLevel = analogRead(LIGHT_PIN);
    nightMode = (lightLevel < 300);  // 어두우면 야간 모드
    
    // 포텐셔미터로 속도 조절
    if (!emergencyStop) {
      int potValue = analogRead(POT_PIN);
      fanSpeed = map(potValue, 0, 1023, 0, 255);
      
      if (nightMode && fanSpeed > 128) {
        fanSpeed = 128;  // 야간에는 최대 50%
      }
    }
  }
}

void handleCommunication() {
  // ESP32로 데이터 전송
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 1000) {  // 1초마다 전송
    lastSend = millis();
    
    Serial.print("DATA:");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print(fanSpeed);
    Serial.print(",");
    Serial.print(windDirection);
    Serial.print(",");
    Serial.print(fanEnabled ? "1" : "0");
    Serial.print(",");
    Serial.print(nightMode ? "1" : "0");
    Serial.print(",");
    Serial.println(emergencyStop ? "1" : "0");
  }
  
  // ESP32로부터 명령 수신
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("SPEED:")) {
      int speed = command.substring(6).toInt();
      if (!emergencyStop) {
        fanSpeed = constrain(speed, 0, 255);
      }
    }
    else if (command.startsWith("WIND:")) {
      int direction = command.substring(5).toInt();
      windDirection = constrain(direction, 0, 180);
      windServo.write(windDirection);
    }
    else if (command == "START") {
      if (!emergencyStop) {
        fanEnabled = true;
      }
    }
    else if (command == "STOP") {
      fanEnabled = false;
    }
    else if (command == "EMERGENCY_RESET") {
      emergencyStop = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void autoControl() {
  // 온도 기반 자동 제어 (간단 버전)
  if (fanEnabled && !emergencyStop && temperature > 0) {
    if (temperature > 28) {
      // 뜨거우면 자동으로 빠르게
      fanSpeed = max(fanSpeed, 200);
    } else if (temperature < 20) {
      // 춥면 자동으로 정지
      fanSpeed = 0;
    }
  }
}

void controlMotor() {
  if (fanEnabled && !emergencyStop) {
    analogWrite(MOTOR_PIN, fanSpeed);
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void updateLEDs() {
  // 전원 LED
  digitalWrite(LED_POWER, fanEnabled && fanSpeed > 0);
  
  // 경고 LED (비상정지 또는 고온)
  bool warning = emergencyStop || temperature > 35;
  if (warning) {
    static bool blinkState = false;
    static unsigned long blinkTime = 0;
    if (millis() - blinkTime > 300) {
      blinkTime = millis();
      blinkState = !blinkState;
      digitalWrite(LED_WARN, blinkState);
    }
  } else {
    digitalWrite(LED_WARN, LOW);
  }
  
  // 상태 LED (야간 모드)
  digitalWrite(LED_STATUS, nightMode);
}

void updateLCD() {
  static unsigned long lastUpdate = 0;
  
  if (millis() - lastUpdate > 1000) {  // 1초마다 업데이트
    lastUpdate = millis();
    
    lcd.clear();
    
    if (emergencyStop) {
      lcd.setCursor(0, 0);
      lcd.print("EMERGENCY STOP");
      lcd.setCursor(0, 1);
      lcd.print("Press Reset");
    } else {
      // 첫 번째 줄: 온도, 습도
      lcd.setCursor(0, 0);
      lcd.print(temperature, 1);
      lcd.print("C ");
      lcd.print(humidity, 0);
      lcd.print("% ");
      
      // 두 번째 줄: 팬 속도, 모드
      lcd.setCursor(0, 1);
      lcd.print("Fan:");
      lcd.print(map(fanSpeed, 0, 255, 0, 100));
      lcd.print("%");
      
      if (nightMode) lcd.print(" NIGHT");
      else if (fanEnabled) lcd.print(" ON");
      else lcd.print(" OFF");
    }
  }
}

void emergencyStopISR() {
  emergencyStop = true;
  fanEnabled = false;
  analogWrite(MOTOR_PIN, 0);
  digitalWrite(BUZZER_PIN, HIGH);  // 경고음
}
```

### 5.8 ESP32 WiFi 코드 (간소화 버전)

```cpp
// 스마트 선풍기 ESP32 WiFi 제어
// 초보자용 간소화 버전

#include <WiFi.h>
#include <WebServer.h>

// WiFi 설정 (여기에 본인의 WiFi 정보 입력)
const char* ssid = "your_wifi_name";
const char* password = "your_password";

WebServer server(80);

// 팬 상태 변수
struct FanStatus {
  float temperature = 0;
  float humidity = 0;
  int speed = 0;
  int windDirection = 90;
  bool enabled = false;
  bool nightMode = false;
  bool emergency = false;
} fanStatus;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 Smart Fan Controller");
  
  // WiFi 연결
  WiFi.begin(ssid, password);
  Serial.print("WiFi 연결 중");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
  
  // 웹 서버 라우트 설정
  server.on("/", handleRoot);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/status", handleStatus);
  
  server.begin();
  Serial.println("웹 서버 시작됨!");
  Serial.println("브라우저에서 다음 주소로 접속하세요:");
  Serial.println("http://" + WiFi.localIP().toString());
}

void loop() {
  // 웹 서버 처리
  server.handleClient();
  
  // Arduino로부터 데이터 수신
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.startsWith("DATA:")) {
      parseArduinoData(data.substring(5));
    }
  }
  
  delay(10);
}

void parseArduinoData(String data) {
  // DATA: temp,humid,speed,wind,enabled,night,emergency
  int commaIndex[6];
  int lastIndex = 0;
  
  // 콤마 위치 찾기
  for (int i = 0; i < 6; i++) {
    commaIndex[i] = data.indexOf(',', lastIndex);
    lastIndex = commaIndex[i] + 1;
  }
  
  // 데이터 파싱
  fanStatus.temperature = data.substring(0, commaIndex[0]).toFloat();
  fanStatus.humidity = data.substring(commaIndex[0] + 1, commaIndex[1]).toFloat();
  fanStatus.speed = data.substring(commaIndex[1] + 1, commaIndex[2]).toInt();
  fanStatus.windDirection = data.substring(commaIndex[2] + 1, commaIndex[3]).toInt();
  fanStatus.enabled = data.substring(commaIndex[3] + 1, commaIndex[4]).toInt();
  fanStatus.nightMode = data.substring(commaIndex[4] + 1, commaIndex[5]).toInt();
  fanStatus.emergency = data.substring(commaIndex[5] + 1).toInt();
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>🌀 스마트 선풍기</title>
    <style>
        body { 
            font-family: Arial; 
            text-align: center; 
            margin: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }
        .container {
            max-width: 400px;
            margin: 0 auto;
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 15px;
            backdrop-filter: blur(10px);
        }
        .status-box {
            background: rgba(255,255,255,0.2);
            margin: 10px 0;
            padding: 15px;
            border-radius: 10px;
        }
        .button {
            background: #4CAF50;
            border: none;
            color: white;
            padding: 15px 30px;
            margin: 5px;
            border-radius: 25px;
            cursor: pointer;
            font-size: 16px;
            min-width: 120px;
        }
        .button:hover { background: #45a049; }
        .button.stop { background: #f44336; }
        .button.stop:hover { background: #da190b; }
        .slider {
            width: 100%;
            margin: 10px 0;
            height: 6px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
        }
        .emergency {
            background: #ff4444 !important;
            animation: blink 1s infinite;
        }
        @keyframes blink {
            0%, 50% { opacity: 1; }
            51%, 100% { opacity: 0.5; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🌀 스마트 선풍기</h1>
        
        <div class="status-box" id="statusBox">
            <h3>📊 상태</h3>
            <div>🌡️ 온도: <span id="temp">--</span>°C</div>
            <div>💧 습도: <span id="humid">--</span>%</div>
            <div>💨 속도: <span id="speed">--</span>%</div>
            <div>🌪️ 방향: <span id="wind">--</span>°</div>
            <div id="modeInfo"></div>
        </div>
        
        <div class="status-box">
            <h3>🎛️ 제어</h3>
            <button class="button" onclick="controlFan('start')">▶️ 시작</button>
            <button class="button stop" onclick="controlFan('stop')">⏹️ 정지</button>
            
            <div style="margin: 15px 0;">
                <label>속도 조절:</label><br>
                <input type="range" class="slider" min="0" max="255" value="128" 
                       id="speedSlider" onchange="setSpeed(this.value)">
                <span id="speedValue">50%</span>
            </div>
            
            <div style="margin: 15px 0;">
                <label>바람 방향:</label><br>
                <input type="range" class="slider" min="0" max="180" value="90" 
                       id="windSlider" onchange="setWind(this.value)">
                <span id="windValue">90°</span>
            </div>
            
            <button class="button" onclick="controlFan('emergency_reset')" 
                    id="emergencyBtn" style="display:none; background:#ff9800;">
                🔄 비상정지 해제
            </button>
        </div>
    </div>
    
    <script>
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('temp').innerText = data.temperature.toFixed(1);
                    document.getElementById('humid').innerText = data.humidity.toFixed(0);
                    document.getElementById('speed').innerText = Math.round(data.speed / 255 * 100);
                    document.getElementById('wind').innerText = data.windDirection;
                    
                    let modeInfo = '';
                    if (data.emergency) {
                        modeInfo = '🚨 비상정지 상태';
                        document.getElementById('statusBox').className = 'status-box emergency';
                        document.getElementById('emergencyBtn').style.display = 'inline-block';
                    } else {
                        document.getElementById('statusBox').className = 'status-box';
                        document.getElementById('emergencyBtn').style.display = 'none';
                        
                        if (data.enabled) {
                            modeInfo = '✅ 동작 중';
                            if (data.nightMode) modeInfo += ' (야간 모드)';
                        } else {
                            modeInfo = '⏸️ 정지 상태';
                        }
                    }
                    document.getElementById('modeInfo').innerHTML = modeInfo;
                });
        }
        
        function controlFan(action) {
            fetch('/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action})
            }).then(() => updateStatus());
        }
        
        function setSpeed(value) {
            document.getElementById('speedValue').innerText = Math.round(value / 255 * 100) + '%';
            controlFan('speed:' + value);
        }
        
        function setWind(value) {
            document.getElementById('windValue').innerText = value + '°';
            controlFan('wind:' + value);
        }
        
        // 초기 업데이트 및 주기적 업데이트
        updateStatus();
        setInterval(updateStatus, 2000);
    </script>
</body>
</html>
  )";
  
  server.send(200, "text/html", html);
}

void handleControl() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    
    // JSON 파싱 (간단한 방법)
    int actionStart = body.indexOf("action\":\"") + 9;
    int actionEnd = body.indexOf("\"", actionStart);
    String action = body.substring(actionStart, actionEnd);
    
    Serial.println(action);  // Arduino로 명령 전송
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"temperature\":" + String(fanStatus.temperature, 1) + ",";
  json += "\"humidity\":" + String(fanStatus.humidity, 1) + ",";
  json += "\"speed\":" + String(fanStatus.speed) + ",";
  json += "\"windDirection\":" + String(fanStatus.windDirection) + ",";
  json += "\"enabled\":" + String(fanStatus.enabled ? "true" : "false") + ",";
  json += "\"nightMode\":" + String(fanStatus.nightMode ? "true" : "false") + ",";
  json += "\"emergency\":" + String(fanStatus.emergency ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}
```

## 🧪 테스트 및 디버깅

### 5.9 단계별 테스트 과정

```
🧪 테스트 체크리스트:

1단계: 기본 연결 테스트
□ Arduino 전원 LED 켜짐
□ ESP32 전원 LED 켜짐  
□ LCD 백라이트 켜짐
□ 시리얼 모니터에서 "Arduino Ready" 메시지

2단계: 센서 테스트
□ DHT22 온도/습도 값 정상 출력
□ 포텐셔미터 돌릴 때 값 변화
□ LDR 가릴 때 야간 모드 전환
□ LCD에 센서값 표시

3단계: 모터 테스트  
□ 포텐셔미터로 모터 속도 제어
□ 서보 모터 각도 제어
□ 비상정지 버튼 동작
□ LED 상태 표시

4단계: 통신 테스트
□ ESP32 WiFi 연결 성공
□ 웹 페이지 접속 가능
□ Arduino ↔ ESP32 데이터 전송
□ 웹에서 실시간 데이터 확인

5단계: 통합 테스트
□ 웹에서 팬 제어 가능
□ 온도 변화에 따른 자동 제어
□ 야간 모드 자동 전환
□ 비상정지 기능 정상 동작
```

### 5.10 일반적인 문제와 해결책

```
❗ 자주 발생하는 문제들:

1. "센서 읽기 실패"
   → DHT22 연결 재확인
   → 2초 이상 간격으로 읽기
   → 다른 핀으로 테스트

2. "WiFi 연결 안됨"  
   → SSID/비밀번호 재확인
   → 2.4GHz 네트워크인지 확인
   → 라우터 근처에서 테스트

3. "웹 페이지 접속 안됨"
   → 같은 WiFi 네트워크 연결 확인
   → IP 주소 직접 입력
   → 방화벽 설정 확인

4. "Arduino ↔ ESP32 통신 안됨"
   → 시리얼 연결 재확인 (TX-RX, RX-TX)
   → 보드레이트 일치 확인 (115200)
   → 공통 GND 연결 확인

5. "모터 안 돌아감"
   → 12V 전원 공급 확인
   → 트랜지스터 연결 확인
   → PWM 신호 확인

6. "LCD 안 보임"
   → I2C 주소 스캔으로 확인
   → SDA/SCL 연결 재확인
   → 백라이트 설정 확인
```

## 🎓 학습 완료 및 다음 단계

### 5.11 성취도 확인

```
✅ 학습 완료 체크리스트:

기초 지식:
□ Arduino와 전자부품 기본 이해
□ 회로도 읽기 및 연결 가능
□ 안전 수칙 숙지

프로그래밍:
□ Arduino IDE 사용법 숙달  
□ 센서 데이터 읽기
□ 모터/서보 제어
□ 시리얼 통신

IoT 기술:
□ ESP32 WiFi 연결
□ 웹 서버 구축
□ 스마트폰 연동
□ 실시간 모니터링

프로젝트 완성:
□ 하드웨어 통합
□ 소프트웨어 통합  
□ 테스트 및 디버깅
□ 문제 해결 능력
```

### 5.12 추가 학습 방향

```
🚀 다음 단계 학습 주제:

중급 과정:
1. PCB 설계 (KiCad, Eagle)
2. 3D 프린팅으로 케이스 제작
3. 더 고급 센서 활용
4. 데이터베이스 연동
5. 클라우드 서비스 (AWS, Google Cloud)

고급 과정:
1. 머신러닝 통합 (TensorFlow Lite)
2. 음성 인식 기능
3. 스마트홈 플랫폼 연동
4. 산업용 제어 시스템
5. 상용화 제품 개발

관련 분야:
1. 로봇공학 (ROS)
2. 드론 개발
3. 자율주행 자동차
4. 산업 자동화
5. 의료기기 개발
```

---

## 🎉 축하합니다!

**스마트 선풍기 프로젝트를 완성하셨습니다!** 🎉

이제 여러분은:
- ✅ Arduino 기초부터 고급 IoT 시스템까지 구축 가능
- ✅ 센서와 액추에이터를 활용한 자동화 시스템 개발 가능  
- ✅ 스마트폰과 연동되는 IoT 제품 제작 가능
- ✅ 문제 해결과 디버깅 능력 보유

**다음 프로젝트로 도전해보세요:**
- 🏠 스마트 홈 시스템
- 🌱 자동 화분 관리 시스템  
- 🚗 RC카 또는 자율주행 로봇
- 📊 환경 모니터링 시스템
- 🎵 IoT 음악 플레이어

**커뮤니티에 참여하여 경험을 나누세요:**
- GitHub에서 프로젝트 공유
- Arduino 커뮤니티 참여
- 메이커 페어 참가
- 오픈소스 프로젝트 기여

---

**🌟 여러분의 창의력으로 더 멋진 프로젝트를 만들어보세요! 🌟**