# 🔧 문제 해결 가이드

HW-020 모터 쉴드 프로젝트의 일반적인 문제들과 해결 방법을 정리했습니다.

## 🚨 급한 문제들 (즉시 해결 필요)

### ⚡ 모터가 전혀 돌아가지 않을 때

**증상**: 모터에서 아무 소리도 나지 않고, 전혀 움직이지 않음

**원인 및 해결**:
1. **전원 문제**
   ```bash
   확인사항:
   - 12V 어댑터가 제대로 연결되었는지 확인
   - 어댑터 LED가 켜져 있는지 확인
   - 극성이 올바른지 확인 (+ → VCC, - → GND)
   
   해결방법:
   - 멀티미터로 12V 출력 확인
   - 다른 12V 어댑터로 테스트
   - 터미널 연결 다시 확인
   ```

2. **신호 연결 문제**
   ```bash
   확인사항:
   - Arduino D3 → HW-020 ENA 연결
   - Arduino D12 → HW-020 IN1 연결
   - 점퍼선 연결 상태
   
   해결방법:
   - 점퍼선 교체
   - 연결부 다시 확인
   - 브레드보드 접촉 확인
   ```

3. **코드 문제**
   ```cpp
   // 기본 테스트 코드
   void setup() {
     pinMode(3, OUTPUT);   // PWM
     pinMode(12, OUTPUT);  // Direction
     digitalWrite(12, HIGH);
     analogWrite(3, 100);  // 저속 테스트
   }
   ```

### 🔥 과열/과전류 경고

**증상**: 경고 LED가 깜빡이거나, 모터가 갑자기 정지

**즉시 조치**:
1. **전원 즉시 차단**
2. **모터와 쉴드 온도 확인**
3. **연결부 단락 검사**

**원인 및 해결**:
```bash
과전류 원인:
- 모터 사양 초과 (12V 모터가 아닌 경우)
- 단락 발생
- 메카니컬 과부하 (프로펠러 막힘)

해결방법:
- 모터 사양서 확인
- 연결부 재점검
- 프로펠러 자유 회전 확인
- 전류 센서로 실제 전류 측정
```

## 🔧 일반적인 문제들

### 1. 속도 제어가 안 될 때

**증상**: 포텐셔미터를 돌려도 속도가 변하지 않음

**해결 순서**:
```bash
1단계: 하드웨어 확인
- 포텐셔미터 A0 연결 확인
- 5V, GND 연결 확인
- 포텐셔미터 저항값 측정 (10kΩ)

2단계: 소프트웨어 확인
- Serial.println(analogRead(A0)) 으로 값 확인
- map() 함수 범위 확인
- PWM 출력 확인

3단계: 테스트 코드
```
```cpp
void loop() {
  int pot = analogRead(A0);
  Serial.print("Pot: ");
  Serial.print(pot);
  
  int speed = map(pot, 0, 1023, 0, 255);
  Serial.print(" | Speed: ");
  Serial.println(speed);
  
  analogWrite(3, speed);
  delay(100);
}
```

### 2. 방향이 바뀌지 않을 때

**증상**: 방향 버튼을 눌러도 모터가 같은 방향으로만 돔

**해결 순서**:
```bash
1단계: 하드웨어 확인
- D2 핀 연결 확인
- 버튼 GND 연결 확인
- 버튼 동작 확인 (멀티미터)

2단계: 방향 신호 확인
- D12 핀 연결 확인
- HIGH/LOW 신호 토글 확인

3단계: 모터 터미널 확인
- OUT1, OUT2 연결 바꿔보기
- 터미널 나사 조임 상태 확인
```

**테스트 코드**:
```cpp
void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(12, OUTPUT);
  pinMode(3, OUTPUT);
}

void loop() {
  bool button = !digitalRead(2);
  digitalWrite(12, button ? HIGH : LOW);
  analogWrite(3, 150);
  
  Serial.print("Button: ");
  Serial.print(button);
  Serial.print(" | Direction: ");
  Serial.println(button ? "CW" : "CCW");
  
  delay(100);
}
```

### 3. LED가 제대로 작동하지 않을 때

**증상**: 상태 LED가 켜지지 않거나 잘못된 상태를 표시

**해결 순서**:
```bash
1단계: LED 자체 테스트
- LED를 5V와 GND에 직접 연결 (220Ω 저항 포함)
- LED 극성 확인 (긴 다리가 +)

2단계: 저항값 확인
- 220Ω 저항 사용 확인
- 멀티미터로 저항값 측정

3단계: 핀 연결 확인
- D4, D5, D6 핀 연결 확인
- 점퍼선 상태 확인
```

**LED 테스트 코드**:
```cpp
void setup() {
  pinMode(4, OUTPUT);  // Direction LED
  pinMode(5, OUTPUT);  // Power LED
  pinMode(6, OUTPUT);  // Warning LED
}

void loop() {
  // LED 순차 테스트
  digitalWrite(4, HIGH); delay(500); digitalWrite(4, LOW);
  digitalWrite(5, HIGH); delay(500); digitalWrite(5, LOW);
  digitalWrite(6, HIGH); delay(500); digitalWrite(6, LOW);
  delay(1000);
}
```

### 4. 시리얼 통신 문제

**증상**: 시리얼 모니터에 아무것도 출력되지 않거나 이상한 문자가 나옴

**해결 순서**:
```bash
1단계: 기본 설정 확인
- 보드레이트 115200 확인
- Arduino IDE에서 올바른 포트 선택
- USB 케이블 상태 확인

2단계: 드라이버 확인
- Arduino 드라이버 설치 상태
- 장치 관리자에서 COM 포트 확인

3단계: 테스트
```
```cpp
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Test OK");
}

void loop() {
  Serial.println("Loop running...");
  delay(1000);
}
```

## 🧪 고급 문제 해결

### 전류 측정 문제

**ACS712 센서 사용 시**:
```cpp
void checkCurrentSensor() {
  int reading = analogRead(A1);
  float voltage = (reading * 5.0) / 1024.0;
  float current = abs((voltage - 2.5) / 0.185);
  
  Serial.print("Raw: ");
  Serial.print(reading);
  Serial.print(" | Voltage: ");
  Serial.print(voltage);
  Serial.print("V | Current: ");
  Serial.print(current);
  Serial.println("A");
}
```

**예상 값들**:
- 센서 없음: 0-50 (노이즈)
- 센서 있음, 전류 없음: ~512 (2.5V)
- 센서 있음, 전류 있음: 512±값

### PWM 주파수 문제

**증상**: 모터에서 고음이나 진동이 심함

**해결**:
```cpp
// PWM 주파수 변경 (핀 3, 11용)
void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // 31.25kHz
  // 기본값으로 되돌리려면: 0x04 (490Hz)
}
```

### 메모리 문제

**증상**: Arduino가 재시작되거나 이상하게 동작

**확인**:
```cpp
void checkMemory() {
  extern int __heap_start, *__brkval;
  int v;
  int free_memory = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  Serial.print("Free memory: ");
  Serial.println(free_memory);
}
```

**해결**:
- 불필요한 변수 제거
- String 사용 최소화
- PROGMEM 사용

## 📋 체크리스트

### 초기 설치 시
- [ ] 12V 어댑터 극성 확인
- [ ] 모터 전압 사양 확인 (12V)
- [ ] 모든 연결부 단락 검사
- [ ] LED 테스트 실행
- [ ] 기본 동작 테스트

### 문제 발생 시
- [ ] 전원 상태 확인
- [ ] 시리얼 모니터 오류 메시지 확인
- [ ] LED 상태 확인
- [ ] 연결부 재점검
- [ ] 테스트 코드로 각 부분 개별 테스트

### 정기 점검
- [ ] 터미널 나사 조임 상태
- [ ] 점퍼선 접촉 상태
- [ ] 모터 온도 확인
- [ ] 성능 로그 확인

## 🆘 응급상황 대처

### 연기나 이상한 냄새가 날 때
1. **즉시 전원 차단** (12V 어댑터 뽑기)
2. **USB 케이블 분리**
3. **화재 대비** (소화기 준비)
4. **환기** 실시
5. **모든 연결 재검토** 후 재시작

### 쇼트(단락) 발생 시
1. **전원 즉시 차단**
2. **멀티미터로 저항 측정**
3. **모든 연결부 확인**
4. **손상된 부품 교체**
5. **재조립 후 저전압 테스트**

## 📞 추가 도움

문제가 해결되지 않으면:
1. **시리얼 모니터 출력** 저장
2. **연결 사진** 촬영
3. **GitHub Issues**에 문제 리포트
4. **Arduino 커뮤니티**에 질문

**문제 리포트 시 포함할 정보**:
- Arduino 모델 및 IDE 버전
- 사용 중인 모터 사양
- 시리얼 모니터 출력
- 발생한 증상과 재현 방법
- 시도해본 해결 방법들

이 가이드로 대부분의 문제를 해결할 수 있습니다! 🔧