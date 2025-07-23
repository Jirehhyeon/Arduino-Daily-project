# 📚 라이브러리 설치 및 설정 가이드

## 🔧 필수 라이브러리 목록

### 📋 **기본 시스템용 라이브러리**
| 라이브러리 | 버전 | 용도 | 필수도 |
|-----------|------|------|--------|
| DHT sensor library | 1.4.4+ | DHT22 센서 제어 | 필수 |
| Servo | 1.1.8+ | 서보모터 제어 | 필수 |

### 📋 **고급 시스템용 추가 라이브러리**  
| 라이브러리 | 버전 | 용도 | 필수도 |
|-----------|------|------|--------|
| LiquidCrystal I2C | 1.1.2+ | I2C LCD 제어 | 고급 기능 |
| EEPROM | 내장 | 설정값 저장 | 고급 기능 |

## 📥 아두이노 IDE 라이브러리 설치

### 🎯 **방법 1: 라이브러리 매니저 사용 (권장)**

**1단계: 아두이노 IDE 실행**
- 아두이노 IDE 1.8.0 이상 버전 사용
- 최신 버전 다운로드: https://www.arduino.cc/en/software

**2단계: 라이브러리 매니저 열기**
```
메뉴: 스케치 → 라이브러리 포함하기 → 라이브러리 관리...
또는: Ctrl + Shift + I
```

**3단계: DHT sensor library 설치**
```
1. 검색창에 "DHT sensor" 입력
2. "DHT sensor library by Adafruit" 선택
3. 최신 버전 선택 후 "설치" 클릭
4. 의존성 라이브러리도 함께 설치 승인
```

**4단계: LiquidCrystal I2C 라이브러리 설치**
```
1. 검색창에 "LiquidCrystal I2C" 입력  
2. "LiquidCrystal I2C by Frank de Brabander" 선택
3. 최신 버전 설치
```

### 🎯 **방법 2: 수동 설치**

**GitHub에서 직접 다운로드**
1. DHT sensor library: https://github.com/adafruit/DHT-sensor-library
2. LiquidCrystal I2C: https://github.com/johnrickman/LiquidCrystal_I2C

**설치 경로**
```
Windows: Documents\Arduino\libraries\
Mac: ~/Documents/Arduino/libraries/
Linux: ~/Arduino/libraries/
```

## 🔍 라이브러리 설치 확인

### ✅ **설치 확인 방법**

**1. 라이브러리 목록 확인**
```
메뉴: 스케치 → 라이브러리 포함하기 → 라이브러리 관리...
설치된 라이브러리 탭에서 확인
```

**2. 예제 코드 확인**
```
메뉴: 파일 → 예제 → DHT sensor library
예제가 보이면 정상 설치 완료
```

**3. 컴파일 테스트**
```cpp
#include <DHT.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

void setup() {
  // 라이브러리 정상 로드 확인
}

void loop() {
  // 테스트 코드
}
```

## 🛠️ 라이브러리별 상세 설정

### 🌡️ **DHT sensor library 설정**

**지원 센서 타입**
- DHT11: 기본형 (정확도 낮음)
- DHT22: 고성능형 (권장)
- DHT21: 중급형

**기본 사용법**
```cpp
#include <DHT.h>

#define DHT_PIN 2
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  dht.begin();
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
}
```

### 🔄 **Servo 라이브러리 설정**

**내장 라이브러리**
- 아두이노 IDE에 기본 포함
- 별도 설치 불필요

**기본 사용법**
```cpp
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);  // 9번 핀 연결
}

void loop() {
  myServo.write(90);  // 90도 회전
  delay(1000);
}
```

### 📺 **LiquidCrystal I2C 설정**

**I2C 주소 확인**
- 일반적 주소: 0x27, 0x3F
- I2C 스캐너로 실제 주소 확인

**I2C 주소 스캐너 코드**
```cpp
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;
  
  Serial.println("Scanning...");
  
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      
      nDevices++;
    }
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
  delay(5000);
}
```

**LCD 기본 사용법**
```cpp
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // 주소, 너비, 높이

void setup() {
  lcd.init();      // LCD 초기화
  lcd.backlight(); // 백라이트 켜기
  
  lcd.setCursor(0, 0);
  lcd.print("Hello World!");
}
```

### 💾 **EEPROM 라이브러리 설정**

**내장 라이브러리**
- 아두이노 IDE에 기본 포함
- 별도 설치 불필요

**기본 사용법**
```cpp
#include <EEPROM.h>

void setup() {
  // float 값 쓰기
  float value = 25.5;
  EEPROM.put(0, value);
  
  // float 값 읽기
  float readValue;
  EEPROM.get(0, readValue);
}
```

## 🚨 문제 해결 가이드

### ❗ **일반적인 설치 오류**

**"라이브러리를 찾을 수 없습니다"**
```
증상: #include <DHT.h> 오류
원인: 라이브러리 설치 실패
해결: 라이브러리 매니저에서 재설치
```

**"컴파일 오류: 'DHT' was not declared"**
```
증상: DHT 객체 생성 시 오류
원인: 헤더 파일 누락
해결: #include <DHT.h> 추가 확인
```

**"Wire.h: No such file or directory"**
```
증상: I2C 관련 컴파일 오류
원인: Wire 라이브러리 누락
해결: #include <Wire.h> 추가
```

### 🔧 **고급 문제 해결**

**라이브러리 버전 충돌**
```
증상: 컴파일은 되지만 오작동
원인: 호환되지 않는 버전
해결: 라이브러리 매니저에서 특정 버전 선택
```

**메모리 부족 오류**
```
증상: "low memory available" 경고
원인: 너무 많은 라이브러리 포함
해결: 불필요한 라이브러리 제거, 코드 최적화
```

**I2C LCD 화면 깨짐**
```
증상: 이상한 문자 출력
원인: 잘못된 I2C 주소
해결: I2C 스캐너로 정확한 주소 확인
```

## 📊 라이브러리 메모리 사용량

### 💾 **메모리 사용량 분석**

| 라이브러리 | Flash 메모리 | SRAM | 비고 |
|-----------|-------------|------|------|
| DHT sensor | ~2KB | ~50B | 기본 |
| Servo | ~1.5KB | ~30B | PWM 타이머 사용 |
| LiquidCrystal I2C | ~3KB | ~100B | 문자 버퍼 |
| EEPROM | ~500B | ~10B | 내장 |
| **전체** | **~7KB** | **~200B** | 안전 여유 있음 |

**아두이노 우노 사양**
- Flash 메모리: 32KB (부트로더 0.5KB 제외)
- SRAM: 2KB
- EEPROM: 1KB

### ⚡ **메모리 최적화 팁**

**Flash 메모리 절약**
```cpp
// 문자열을 Flash에 저장
Serial.println(F("Hello World"));

// 불필요한 부동소수점 연산 줄이기
int temp_int = (int)(temperature * 10); // 소수점 1자리만
```

**SRAM 절약**
```cpp
// 전역 변수 대신 지역 변수 사용
// 큰 배열 사용 시 Flash 저장
const PROGMEM char messages[] = "Very long string...";
```

## 🔄 라이브러리 업데이트

### 📈 **정기 업데이트 체크**

**업데이트 확인 방법**
1. 라이브러리 매니저 열기
2. "업데이트 가능" 필터 적용
3. 업데이트 가능한 라이브러리 확인

**업데이트 주의사항**
- 메이저 버전 업데이트 시 호환성 확인
- 백업 후 업데이트 진행
- 테스트 코드로 동작 검증

**권장 업데이트 주기**
- DHT sensor library: 6개월마다
- LiquidCrystal I2C: 1년마다
- 보안 업데이트는 즉시 적용

## 📚 추가 학습 자료

### 🎓 **공식 문서**
- Arduino Reference: https://www.arduino.cc/reference/
- DHT sensor library: https://github.com/adafruit/DHT-sensor-library
- LiquidCrystal I2C: https://github.com/johnrickman/LiquidCrystal_I2C

### 🔍 **유용한 도구**
- Arduino Library Manager
- PlatformIO (VS Code 확장)
- Arduino CLI

### 💡 **팁 & 트릭**
- 라이브러리 소스코드 읽기 (학습 효과 높음)
- 예제 코드 활용 (빠른 시작)
- 커뮤니티 포럼 활용 (문제 해결)

---

**🎯 이제 모든 라이브러리가 준비되었습니다!**

각 라이브러리의 예제 코드를 먼저 실행해보신 후, 본 프로젝트 코드를 업로드하시면 됩니다. 🚀