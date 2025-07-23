/*
  온습도 자동 제어 시스템
  - DHT22 센서로 온도/습도 측정
  - 쾌적한 환경일 때 서보모터 작동 및 LED 점등
  - 시리얼 모니터로 실시간 데이터 확인
*/

#include <DHT.h>
#include <Servo.h>

// 핀 정의
#define DHT_PIN 2        // DHT22 센서 데이터 핀
#define DHT_TYPE DHT22   // DHT22 센서 타입
#define SERVO_PIN 9      // 서보모터 제어 핀
#define LED_PIN 13       // LED 핀

// 센서 및 모터 객체 생성
DHT dht(DHT_PIN, DHT_TYPE);
Servo myServo;

// 쾌적 환경 기준값 (불쾌지수 기반)
const float COMFORT_TEMP_MIN = 20.0;  // 최적 온도 하한 (°C)
const float COMFORT_TEMP_MAX = 26.0;  // 최적 온도 상한 (°C)
const float COMFORT_HUMIDITY_MIN = 40.0;  // 최적 습도 하한 (%)
const float COMFORT_HUMIDITY_MAX = 60.0;  // 최적 습도 상한 (%)

// 시스템 상태 변수
bool systemActive = false;
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 2000;  // 2초마다 센서 읽기

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(9600);
  Serial.println("온습도 자동 제어 시스템 시작");
  Serial.println("================================");
  
  // 센서 및 액추에이터 초기화
  dht.begin();
  myServo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);
  
  // 초기 상태 설정
  myServo.write(0);        // 서보모터 초기 위치
  digitalWrite(LED_PIN, LOW);  // LED 끄기
  
  Serial.println("시스템 초기화 완료");
  Serial.println("최적 온도: " + String(COMFORT_TEMP_MIN) + "-" + String(COMFORT_TEMP_MAX) + "°C");
  Serial.println("최적 습도: " + String(COMFORT_HUMIDITY_MIN) + "-" + String(COMFORT_HUMIDITY_MAX) + "%");
  Serial.println("================================");
}

void loop() {
  // 지정된 간격마다 센서 읽기
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();
    
    // 온습도 측정
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    // 측정값 유효성 검사
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("센서 읽기 오류!");
      return;
    }
    
    // 불쾌지수 계산 (간단한 공식 사용)
    float discomfortIndex = calculateDiscomfortIndex(temperature, humidity);
    
    // 쾌적 환경 여부 판단
    bool isComfortable = checkComfortLevel(temperature, humidity);
    
    // 시리얼 모니터 출력
    printSensorData(temperature, humidity, discomfortIndex, isComfortable);
    
    // 시스템 제어
    controlSystem(isComfortable);
  }
}

// 불쾌지수 계산 함수
float calculateDiscomfortIndex(float temp, float humid) {
  // 불쾌지수 = 0.81 × 온도 + 0.01 × 습도 × (0.99 × 온도 - 14.3) + 46.3
  return 0.81 * temp + 0.01 * humid * (0.99 * temp - 14.3) + 46.3;
}

// 쾌적도 판단 함수
bool checkComfortLevel(float temp, float humid) {
  bool tempOK = (temp >= COMFORT_TEMP_MIN && temp <= COMFORT_TEMP_MAX);
  bool humidOK = (humid >= COMFORT_HUMIDITY_MIN && humid <= COMFORT_HUMIDITY_MAX);
  
  return tempOK && humidOK;
}

// 센서 데이터 출력 함수
void printSensorData(float temp, float humid, float discomfort, bool comfortable) {
  Serial.println("--- 센서 측정값 ---");
  Serial.println("온도: " + String(temp, 1) + "°C");
  Serial.println("습도: " + String(humid, 1) + "%");
  Serial.println("불쾌지수: " + String(discomfort, 1));
  Serial.print("상태: ");
  
  if (comfortable) {
    Serial.println("쾌적 ✓");
  } else {
    Serial.println("불쾌적 ✗");
    
    // 개선 권장사항 출력
    if (temp < COMFORT_TEMP_MIN) Serial.println("→ 온도가 낮습니다 (난방 권장)");
    if (temp > COMFORT_TEMP_MAX) Serial.println("→ 온도가 높습니다 (냉방 권장)");
    if (humid < COMFORT_HUMIDITY_MIN) Serial.println("→ 습도가 낮습니다 (가습 권장)");
    if (humid > COMFORT_HUMIDITY_MAX) Serial.println("→ 습도가 높습니다 (제습 권장)");
  }
  
  Serial.println("==================");
}

// 시스템 제어 함수
void controlSystem(bool shouldActivate) {
  if (shouldActivate && !systemActive) {
    // 쾌적한 환경 → 시스템 활성화
    systemActive = true;
    activateComfortMode();
    Serial.println("🌟 쾌적 모드 활성화!");
    
  } else if (!shouldActivate && systemActive) {
    // 불쾌적한 환경 → 시스템 비활성화
    systemActive = false;
    deactivateComfortMode();
    Serial.println("❌ 쾌적 모드 비활성화");
  }
}

// 쾌적 모드 활성화
void activateComfortMode() {
  // LED 켜기
  digitalWrite(LED_PIN, HIGH);
  
  // 서보모터 회전 (선풍기 시뮬레이션)
  for (int pos = 0; pos <= 180; pos += 5) {
    myServo.write(pos);
    delay(50);
  }
  for (int pos = 180; pos >= 0; pos -= 5) {
    myServo.write(pos);
    delay(50);
  }
}

// 쾌적 모드 비활성화
void deactivateComfortMode() {
  // LED 끄기
  digitalWrite(LED_PIN, LOW);
  
  // 서보모터 정지 위치로 이동
  myServo.write(0);
}