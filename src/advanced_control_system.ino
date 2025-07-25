/*
  고급 온습도 자동 제어 시스템 v2.0
  - 개선된 센서 데이터 필터링
  - LCD 디스플레이 지원
  - EEPROM 설정 저장
  - 다중 동작 모드
  - 데이터 로깅 기능
*/

#include <DHT.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ==================== 핀 정의 ====================
#define DHT_PIN 2        // DHT22 센서 데이터 핀
#define DHT_TYPE DHT22   // DHT22 센서 타입
#define SERVO_PIN 9      // 서보모터 제어 핀
#define LED_PIN 13       // LED 핀
#define BUTTON_MODE 3    // 모드 변경 버튼
#define BUTTON_SET 4     // 설정 버튼
#define BUZZER_PIN 8     // 부저 핀

// ==================== 객체 생성 ====================
DHT dht(DHT_PIN, DHT_TYPE);
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C LCD (주소: 0x27, 16x2)

// ==================== 시스템 설정 ====================
// 동작 모드
enum SystemMode {
  AUTO_MODE = 0,    // 자동 모드
  MANUAL_MODE = 1,  // 수동 모드  
  ECO_MODE = 2      // 절약 모드
};

// EEPROM 주소 정의
#define EEPROM_TEMP_MIN 0      // 최소 온도 설정
#define EEPROM_TEMP_MAX 4      // 최대 온도 설정
#define EEPROM_HUMID_MIN 8     // 최소 습도 설정
#define EEPROM_HUMID_MAX 12    // 최대 습도 설정
#define EEPROM_MODE 16         // 시스템 모드
#define EEPROM_CALIBRATION 20  // 센서 보정값

// ==================== 전역 변수 ====================
// 쾌적 환경 기준값 (EEPROM에서 로드)
float comfortTempMin = 20.0;
float comfortTempMax = 26.0;
float comfortHumidMin = 40.0;
float comfortHumidMax = 60.0;

// 센서 보정값
float tempCalibration = 0.0;
float humidCalibration = 0.0;

// 시스템 상태
SystemMode currentMode = AUTO_MODE;
bool systemActive = false;
bool settingMode = false;
int settingStep = 0;

// 시간 관리
unsigned long lastReadTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastLogTime = 0;
unsigned long buttonPressTime = 0;

const unsigned long READ_INTERVAL = 2000;     // 센서 읽기 간격
const unsigned long DISPLAY_INTERVAL = 1000;  // 디스플레이 업데이트 간격
const unsigned long LOG_INTERVAL = 60000;     // 로그 저장 간격 (1분)
const unsigned long DEBOUNCE_DELAY = 200;     // 버튼 디바운스 지연

// 데이터 필터링
#define FILTER_SIZE 5
float tempBuffer[FILTER_SIZE];
float humidBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// 통계 데이터
float tempSum = 0.0, humidSum = 0.0;
int readingCount = 0;
float tempMin = 999.0, tempMax = -999.0;
float humidMin = 999.0, humidMax = -999.0;

void setup() {
  Serial.begin(9600);
  Serial.println(F("고급 온습도 제어 시스템 v2.0 시작"));
  
  // 하드웨어 초기화
  initializeHardware();
  
  // EEPROM에서 설정값 로드
  loadSettingsFromEEPROM();
  
  // 초기 화면 표시
  displayWelcomeScreen();
  
  Serial.println(F("시스템 초기화 완료"));
  printSystemInfo();
}

void loop() {
  // 버튼 처리
  handleButtons();
  
  // 센서 데이터 읽기 및 처리
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();
    processSensorData();
  }
  
  // LCD 디스플레이 업데이트
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // 데이터 로깅
  if (millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    logData();
  }
}

// ==================== 하드웨어 초기화 ====================
void initializeHardware() {
  // 센서 및 액추에이터 초기화
  dht.begin();
  myServo.attach(SERVO_PIN);
  
  // 핀 모드 설정
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  
  // LCD 초기화
  lcd.init();
  lcd.backlight();
  
  // 초기 상태 설정
  myServo.write(0);
  digitalWrite(LED_PIN, LOW);
  
  // 필터 버퍼 초기화
  for (int i = 0; i < FILTER_SIZE; i++) {
    tempBuffer[i] = 0.0;
    humidBuffer[i] = 0.0;
  }
}

// ==================== EEPROM 설정 관리 ====================
void loadSettingsFromEEPROM() {
  // 설정값이 저장되어 있는지 확인 (첫 번째 바이트가 0xFF가 아니면 저장된 값 존재)
  if (EEPROM.read(EEPROM_TEMP_MIN) != 0xFF) {
    EEPROM.get(EEPROM_TEMP_MIN, comfortTempMin);
    EEPROM.get(EEPROM_TEMP_MAX, comfortTempMax);
    EEPROM.get(EEPROM_HUMID_MIN, comfortHumidMin);
    EEPROM.get(EEPROM_HUMID_MAX, comfortHumidMax);
    EEPROM.get(EEPROM_MODE, currentMode);
    EEPROM.get(EEPROM_CALIBRATION, tempCalibration);
    EEPROM.get(EEPROM_CALIBRATION + 4, humidCalibration);
    
    Serial.println(F("EEPROM에서 설정값 로드 완료"));
  } else {
    Serial.println(F("기본 설정값 사용"));
    saveSettingsToEEPROM(); // 기본값을 EEPROM에 저장
  }
}

void saveSettingsToEEPROM() {
  EEPROM.put(EEPROM_TEMP_MIN, comfortTempMin);
  EEPROM.put(EEPROM_TEMP_MAX, comfortTempMax);
  EEPROM.put(EEPROM_HUMID_MIN, comfortHumidMin);
  EEPROM.put(EEPROM_HUMID_MAX, comfortHumidMax);
  EEPROM.put(EEPROM_MODE, currentMode);
  EEPROM.put(EEPROM_CALIBRATION, tempCalibration);
  EEPROM.put(EEPROM_CALIBRATION + 4, humidCalibration);
  
  Serial.println(F("설정값 EEPROM에 저장 완료"));
  playBeep(2); // 저장 완료 신호음
}

// ==================== 데이터 필터링 ====================
float getFilteredValue(float newValue, float buffer[], bool isTemperature) {
  // 이동 평균 필터 적용
  buffer[bufferIndex] = newValue;
  
  float sum = 0.0;
  int count = bufferFull ? FILTER_SIZE : (bufferIndex + 1);
  
  for (int i = 0; i < count; i++) {
    sum += buffer[i];
  }
  
  // 이상값 검출 및 제거 (표준편차 기반)
  float average = sum / count;
  float variance = 0.0;
  
  for (int i = 0; i < count; i++) {
    variance += pow(buffer[i] - average, 2);
  }
  
  float stdDev = sqrt(variance / count);
  float threshold = isTemperature ? 3.0 : 5.0; // 온도: 3도, 습도: 5% 임계값
  
  // 이상값 제거된 평균 계산
  float filteredSum = 0.0;
  int validCount = 0;
  
  for (int i = 0; i < count; i++) {
    if (abs(buffer[i] - average) <= threshold) {
      filteredSum += buffer[i];
      validCount++;
    }
  }
  
  return validCount > 0 ? filteredSum / validCount : average;
}

// ==================== 센서 데이터 처리 ====================
void processSensorData() {
  float rawTemp = dht.readTemperature();
  float rawHumid = dht.readHumidity();
  
  // 센서 읽기 오류 확인
  if (isnan(rawTemp) || isnan(rawHumid)) {
    Serial.println(F("센서 읽기 오류!"));
    displayError("센서 오류");
    return;
  }
  
  // 보정값 적용
  rawTemp += tempCalibration;
  rawHumid += humidCalibration;
  
  // 데이터 필터링
  float filteredTemp = getFilteredValue(rawTemp, tempBuffer, true);
  float filteredHumid = getFilteredValue(rawHumid, humidBuffer, false);
  
  // 버퍼 인덱스 업데이트
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  if (bufferIndex == 0) bufferFull = true;
  
  // 통계 업데이트
  updateStatistics(filteredTemp, filteredHumid);
  
  // 불쾌지수 계산
  float discomfortIndex = calculateDiscomfortIndex(filteredTemp, filteredHumid);
  
  // 쾌적도 판단
  bool isComfortable = checkComfortLevel(filteredTemp, filteredHumid);
  
  // 시스템 제어
  controlSystem(isComfortable, filteredTemp, filteredHumid);
  
  // 시리얼 출력
  printDetailedData(rawTemp, rawHumid, filteredTemp, filteredHumid, discomfortIndex, isComfortable);
}

// ==================== 통계 업데이트 ====================
void updateStatistics(float temp, float humid) {
  tempSum += temp;
  humidSum += humid;
  readingCount++;
  
  if (temp < tempMin) tempMin = temp;
  if (temp > tempMax) tempMax = temp;
  if (humid < humidMin) humidMin = humid;
  if (humid > humidMax) humidMax = humid;
}

// ==================== 불쾌지수 계산 ====================
float calculateDiscomfortIndex(float temp, float humid) {
  return 0.81 * temp + 0.01 * humid * (0.99 * temp - 14.3) + 46.3;
}

// ==================== 쾌적도 판단 ====================
bool checkComfortLevel(float temp, float humid) {
  bool tempOK = (temp >= comfortTempMin && temp <= comfortTempMax);
  bool humidOK = (humid >= comfortHumidMin && humid <= comfortHumidMax);
  return tempOK && humidOK;
}

// ==================== 시스템 제어 ====================
void controlSystem(bool isComfortable, float temp, float humid) {
  bool shouldActivate = false;
  
  switch (currentMode) {
    case AUTO_MODE:
      shouldActivate = isComfortable;
      break;
      
    case MANUAL_MODE:
      // 수동 모드에서는 사용자가 직접 제어
      return;
      
    case ECO_MODE:
      // 절약 모드: 더 엄격한 조건 적용
      shouldActivate = isComfortable && (temp > (comfortTempMin + 1.0)) && (temp < (comfortTempMax - 1.0));
      break;
  }
  
  if (shouldActivate && !systemActive) {
    systemActive = true;
    activateComfortMode();
    Serial.println(F("🌟 쾌적 모드 활성화"));
    
  } else if (!shouldActivate && systemActive) {
    systemActive = false;
    deactivateComfortMode();
    Serial.println(F("❌ 쾌적 모드 비활성화"));
  }
}

// ==================== 액추에이터 제어 ====================
void activateComfortMode() {
  digitalWrite(LED_PIN, HIGH);
  playBeep(1);
  
  // 서보모터 부드러운 회전
  for (int pos = 0; pos <= 180; pos += 2) {
    myServo.write(pos);
    delay(20);
  }
  for (int pos = 180; pos >= 0; pos -= 2) {
    myServo.write(pos);
    delay(20);
  }
}

void deactivateComfortMode() {
  digitalWrite(LED_PIN, LOW);
  myServo.write(0);
}

// ==================== 사운드 효과 ====================
void playBeep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// ==================== 버튼 처리 ====================
void handleButtons() {
  static bool modeButtonPressed = false;
  static bool setButtonPressed = false;
  
  // 모드 변경 버튼
  if (digitalRead(BUTTON_MODE) == LOW && !modeButtonPressed) {
    if (millis() - buttonPressTime > DEBOUNCE_DELAY) {
      buttonPressTime = millis();
      modeButtonPressed = true;
      
      if (!settingMode) {
        // 모드 변경
        currentMode = (SystemMode)((currentMode + 1) % 3);
        saveSettingsToEEPROM();
        Serial.print(F("모드 변경: "));
        printCurrentMode();
      }
    }
  } else if (digitalRead(BUTTON_MODE) == HIGH) {
    modeButtonPressed = false;
  }
  
  // 설정 버튼
  if (digitalRead(BUTTON_SET) == LOW && !setButtonPressed) {
    if (millis() - buttonPressTime > DEBOUNCE_DELAY) {
      buttonPressTime = millis();
      setButtonPressed = true;
      handleSettingButton();
    }
  } else if (digitalRead(BUTTON_SET) == HIGH) {
    setButtonPressed = false;
  }
}

void handleSettingButton() {
  if (!settingMode) {
    settingMode = true;
    settingStep = 0;
    Serial.println(F("설정 모드 진입"));
  } else {
    settingStep++;
    if (settingStep > 4) { // 온도 최소/최대, 습도 최소/최대, 보정
      settingMode = false;
      settingStep = 0;
      saveSettingsToEEPROM();
      Serial.println(F("설정 모드 종료"));
    }
  }
  playBeep(1);
}

// ==================== LCD 디스플레이 ====================
void displayWelcomeScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Climate");
  lcd.setCursor(0, 1); 
  lcd.print("Control v2.0");
  delay(2000);
}

void updateDisplay() {
  if (settingMode) {
    displaySettingScreen();
  } else {
    displayMainScreen();
  }
}

void displayMainScreen() {
  // 현재 온습도 센서값 가져오기
  float temp = dht.readTemperature() + tempCalibration;
  float humid = dht.readHumidity() + humidCalibration;
  
  if (isnan(temp) || isnan(humid)) return;
  
  lcd.clear();
  
  // 첫 번째 줄: 온도, 습도, 모드
  lcd.setCursor(0, 0);
  lcd.print(temp, 1);
  lcd.print("C ");
  lcd.print(humid, 0);
  lcd.print("% ");
  
  // 모드 표시
  switch (currentMode) {
    case AUTO_MODE: lcd.print("AUTO"); break;
    case MANUAL_MODE: lcd.print("MAN"); break;
    case ECO_MODE: lcd.print("ECO"); break;
  }
  
  // 두 번째 줄: 상태 및 불쾌지수
  lcd.setCursor(0, 1);
  float di = calculateDiscomfortIndex(temp, humid);
  lcd.print("DI:");
  lcd.print(di, 0);
  
  lcd.setCursor(8, 1);
  if (systemActive) {
    lcd.print("COMFORT");
  } else {
    lcd.print("STANDBY");
  }
}

void displaySettingScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  switch (settingStep) {
    case 0:
      lcd.print("Set TempMin:");
      lcd.setCursor(0, 1);
      lcd.print(comfortTempMin, 1);
      lcd.print("C");
      break;
    case 1:
      lcd.print("Set TempMax:");
      lcd.setCursor(0, 1);
      lcd.print(comfortTempMax, 1);
      lcd.print("C");
      break;
    case 2:
      lcd.print("Set HumidMin:");
      lcd.setCursor(0, 1);
      lcd.print(comfortHumidMin, 0);
      lcd.print("%");
      break;
    case 3:
      lcd.print("Set HumidMax:");
      lcd.setCursor(0, 1);
      lcd.print(comfortHumidMax, 0);
      lcd.print("%");
      break;
    case 4:
      lcd.print("Calibration:");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(tempCalibration, 1);
      lcd.print(" H:");
      lcd.print(humidCalibration, 0);
      break;
  }
}

void displayError(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.setCursor(0, 1);
  lcd.print(message);
  playBeep(3);
}

// ==================== 데이터 출력 ====================
void printDetailedData(float rawTemp, float rawHumid, float filteredTemp, float filteredHumid, float di, bool comfortable) {
  Serial.println(F("========== 센서 데이터 =========="));
  Serial.print(F("Raw: T="));
  Serial.print(rawTemp, 2);
  Serial.print(F("°C, H="));
  Serial.print(rawHumid, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Filtered: T="));
  Serial.print(filteredTemp, 2);
  Serial.print(F("°C, H="));
  Serial.print(filteredHumid, 1);
  Serial.println(F("%"));
  
  Serial.print(F("불쾌지수: "));
  Serial.println(di, 1);
  
  Serial.print(F("모드: "));
  printCurrentMode();
  
  Serial.print(F("상태: "));
  Serial.println(comfortable ? F("쾌적 ✓") : F("불쾌적 ✗"));
  
  Serial.println(F("=========================="));
}

void printCurrentMode() {
  switch (currentMode) {
    case AUTO_MODE: Serial.println(F("자동")); break;
    case MANUAL_MODE: Serial.println(F("수동")); break;
    case ECO_MODE: Serial.println(F("절약")); break;
  }
}

void printSystemInfo() {
  Serial.println(F("========== 시스템 정보 =========="));
  Serial.print(F("온도 범위: "));
  Serial.print(comfortTempMin, 1);
  Serial.print(F("°C ~ "));
  Serial.print(comfortTempMax, 1);
  Serial.println(F("°C"));
  
  Serial.print(F("습도 범위: "));
  Serial.print(comfortHumidMin, 0);
  Serial.print(F("% ~ "));
  Serial.print(comfortHumidMax, 0);
  Serial.println(F("%"));
  
  Serial.print(F("센서 보정: T"));
  Serial.print(tempCalibration > 0 ? "+" : "");
  Serial.print(tempCalibration, 1);
  Serial.print(F("°C, H"));
  Serial.print(humidCalibration > 0 ? "+" : "");
  Serial.print(humidCalibration, 0);
  Serial.println(F("%"));
  
  Serial.print(F("현재 모드: "));
  printCurrentMode();
  Serial.println(F("============================="));
}

// ==================== 데이터 로깅 ====================
void logData() {
  if (readingCount > 0) {
    float avgTemp = tempSum / readingCount;
    float avgHumid = humidSum / readingCount;
    
    Serial.println(F("========== 통계 데이터 =========="));
    Serial.print(F("측정 횟수: "));
    Serial.println(readingCount);
    
    Serial.print(F("평균 온도: "));
    Serial.print(avgTemp, 1);
    Serial.print(F("°C ("));
    Serial.print(tempMin, 1);
    Serial.print(F("~"));
    Serial.print(tempMax, 1);
    Serial.println(F("°C)"));
    
    Serial.print(F("평균 습도: "));
    Serial.print(avgHumid, 0);
    Serial.print(F("% ("));
    Serial.print(humidMin, 0);
    Serial.print(F("~"));
    Serial.print(humidMax, 0);
    Serial.println(F("%)"));
    
    Serial.print(F("시스템 가동률: "));
    Serial.print((float)readingCount / (millis() / READ_INTERVAL) * 100, 1);
    Serial.println(F("%"));
    Serial.println(F("============================="));
    
    // 통계 초기화
    tempSum = humidSum = 0.0;
    readingCount = 0;
    tempMin = humidMin = 999.0;
    tempMax = humidMax = -999.0;
  }
}