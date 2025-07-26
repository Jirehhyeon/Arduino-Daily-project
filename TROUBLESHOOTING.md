# 🔧 문제 해결 가이드 - 스마트 선풍기 v3.0

**포괄적인 트러블슈팅 및 진단 가이드**

## 🎯 문제 진단 체크리스트

### 📋 시스템 상태 확인 (먼저 해보세요!)

```bash
1. ⚡ 전원 상태 확인
   □ 12V 어댑터 LED 켜짐
   □ Arduino 전원 LED 켜짐  
   □ ESP32 전원 LED 켜짐

2. 🔗 연결 상태 확인
   □ Arduino ↔ ESP32 시리얼 연결
   □ WiFi 연결 상태
   □ 모터 쉴드 장착 상태

3. 💻 소프트웨어 상태 확인  
   □ Arduino IDE 시리얼 모니터 (115200)
   □ 웹 인터페이스 접속 가능
   □ 센서 데이터 업데이트
```

---

## 🚨 긴급 상황 대응

### ⚠️ 비상 정지 상황

**증상**: 시스템이 응답하지 않거나 이상 동작
```bash
즉시 조치:
1. 🔴 비상정지 버튼 누르기 (Arduino D2)
2. 🔌 12V 어댑터 뽑기
3. 💻 시리얼 모니터에서 "STOP" 명령 전송
4. 📱 웹 앱에서 긴급 정지 클릭
```

**복구 절차**:
```bash
1. 모든 연결 재확인
2. Arduino 재시작 (리셋 버튼)
3. ESP32 재시작 (EN 버튼)
4. 시스템 초기화: 시리얼로 "RESET" 명령
```

### 🔥 과열/과전류 경고

**자동 보호 기능**:
```bash
과전류 (>1.2A):
→ 즉시 모터 정지
→ 빨간 LED 빠른 깜빡임
→ 부저 알림 3회

과열 (>55°C):  
→ 속도 자동 감소
→ 노란 LED 경고
→ 웹 앱 알림 전송
```

**수동 복구**:
```bash
1. 전원 차단 후 5분 대기 (냉각)
2. 모터 및 연결부 점검
3. 시리얼로 "STATUS" 명령으로 상태 확인
4. 안전 확인 후 재시작
```

---

## 🔌 하드웨어 문제 해결

### ⚡ 전원 관련 문제

**문제**: Arduino 전원이 켜지지 않음
```bash
진단 단계:
1. 12V 어댑터 출력 전압 측정 (멀티미터)
2. Arduino VIN 핀 전압 확인 (11-12V)
3. USB 케이블로 Arduino만 테스트
4. 퓨즈 또는 보호 회로 확인

해결책:
✓ 어댑터 교체 (12V 2A 권장)
✓ 전원 케이블 교체
✓ Arduino 보드 교체
✓ 극성 확인 (빨강=+, 검정=-)
```

**문제**: ESP32가 부팅되지 않음
```bash
진단 단계:
1. ESP32 전원 LED 확인
2. Arduino 3.3V 출력 측정
3. USB로 ESP32 직접 연결 테스트
4. 부트 모드 확인 (GPIO0 상태)

해결책:
✓ 3.3V 전원 공급 확인
✓ EN 핀을 HIGH로 연결
✓ 부트 스트래핑 핀 확인
✓ ESP32 보드 교체
```

### 🔧 모터 관련 문제

**문제**: 모터가 전혀 돌아가지 않음
```bash
체크리스트:
□ HW-020 쉴드 올바른 장착
□ 12V 모터 전원 공급
□ PWM 신호선 (D3) 연결
□ 방향 신호선 (D12) 연결
□ 모터 터미널 나사 조임

진단 방법:
1. 시리얼 모니터: "STATUS" 명령으로 PWM 값 확인
2. 멀티미터: 모터 터미널 전압 측정
3. 오실로스코프: PWM 신호 확인 (가능시)
4. 다른 모터로 교체 테스트

일반적 해결책:
✓ 터미널 나사 재조임
✓ 점퍼선 교체
✓ 모터 극성 바꿔서 연결
✓ HW-020 쉴드 재장착
```

**문제**: 모터가 한 방향으로만 돌아감
```bash
원인 분석:
• IN1 신호선 (D12) 문제
• HW-020 쉴드 방향 제어 회로 문제
• 코드에서 방향 제어 로직 오류

해결 단계:
1. 시리얼로 "DIR" 명령 테스트
2. D12 핀 전압 측정 (HIGH/LOW 전환 확인)
3. 코드에서 digitalWrite(MOTOR_DIR, ...) 확인
4. 물리적으로 모터 선 바꿔 연결 테스트
```

**문제**: 모터 속도 제어가 안 됨
```bash
진단 포인트:
• PWM 신호 (D3) 문제
• 포텐셔미터 (A0) 연결 문제  
• ENA 핀 연결 문제
• analogWrite() 함수 동작 확인

테스트 방법:
1. 시리얼로 "SPEED:50" 명령 테스트
2. 포텐셔미터 돌려가며 A0 값 확인
3. D3 핀에서 PWM 신호 측정
4. 고정 속도로 코드 테스트 (analogWrite(3, 128))
```

### 📡 센서 관련 문제

**문제**: DHT22 온습도 센서 읽기 실패
```bash
오류 메시지: "Failed to read from DHT sensor!"

진단 절차:
1. 연결 확인: VCC(5V), GND, Data(D7)
2. 전원 공급 확인: 멀티미터로 5V 측정
3. 데이터 핀 전압 확인: 유휴시 HIGH 상태
4. 다른 센서로 교체 테스트

해결 방법:
✓ 점퍼선 교체 (특히 데이터 선)
✓ 풀업 저항 추가 (10kΩ, Data-VCC 간)
✓ 전원 공급 개선 (커패시터 추가)
✓ DHT22 센서 교체

코드 확인:
• dht.begin() 호출 확인
• 읽기 간격 (2초 이상) 확인
• isnan() 함수로 유효성 검사 확인
```

**문제**: 전류 센서 (ACS712) 값이 이상함
```bash
증상: 전류 값이 음수이거나 과도하게 높음

보정 방법:
1. 무부하 상태에서 기준값(2.5V) 측정
2. 알려진 부하로 실제 전류와 비교
3. 코드에서 보정 상수 조정

보정 공식:
float voltage = (analogRead(A1) * 5.0) / 1024.0;
float current = (voltage - 2.5) / 0.185; // ACS712-05B
float calibrated = current + OFFSET; // 오프셋 보정
```

**문제**: 조도 센서 (LDR) 반응 없음
```bash
테스트 방법:
1. 손으로 센서 가려서 값 변화 확인
2. 강한 조명 비춰서 값 변화 확인
3. 멀티미터로 저항값 측정

회로 확인:
• LDR - 10kΩ 저항 분배 회로
• A2 핀 연결 확인
• 5V, GND 연결 확인

코드 보정:
int raw = analogRead(A2);
int light = map(raw, 0, 1023, 0, 100);
// 어두움: 낮은 값, 밝음: 높은 값
```

---

## 📶 통신 문제 해결

### 🌐 WiFi 연결 문제

**문제**: ESP32가 WiFi에 연결되지 않음
```bash
진단 체크리스트:
□ 2.4GHz 네트워크인지 확인 (5GHz 지원 안함)
□ WiFi 비밀번호 정확성
□ SSID에 특수문자 확인
□ 라우터와의 거리 (30m 이내)
□ 동시 연결 기기 수 제한

해결 방법:
1. 시리얼 모니터에서 연결 과정 확인
2. WiFi.printDiag() 함수로 상세 정보 확인
3. 고정 IP 설정 시도
4. 라우터 재시작

코드 디버깅:
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  Serial.println(WiFi.status()); // 상태 코드 확인
}
```

**WiFi 상태 코드 해석**:
```bash
0 (WL_IDLE_STATUS): 유휴 상태
1 (WL_NO_SSID_AVAIL): SSID를 찾을 수 없음
2 (WL_SCAN_COMPLETED): 스캔 완료
3 (WL_CONNECTED): 연결됨
4 (WL_CONNECT_FAILED): 연결 실패  
5 (WL_CONNECTION_LOST): 연결 끊어짐
6 (WL_DISCONNECTED): 연결 해제
```

**문제**: WiFi 연결이 자주 끊어짐
```bash
원인 분석:
• 신호 강도 약함 (RSSI < -70dBm)
• 전력 관리 설정 문제
• 라우터 설정 문제

해결책:
1. WiFi.setAutoReconnect(true) 설정
2. WiFi 신호 강도 개선 (안테나, 위치)
3. 전력 절약 모드 비활성화
4. 하트비트 주기 단축 (30초 → 10초)
```

### 🔗 Arduino ↔ ESP32 통신 문제

**문제**: 시리얼 통신이 되지 않음
```bash
연결 확인:
• Arduino TX (D1) → ESP32 RX2 (GPIO16)
• Arduino RX (D0) → ESP32 TX2 (GPIO17)  
• 공통 GND 연결

통신 테스트:
1. Arduino에서 "PING" 전송
2. ESP32에서 "PONG" 응답 확인
3. 보드레이트 일치 확인 (115200)
4. 시리얼 모니터에서 양방향 통신 테스트

디버깅 팁:
• 한 번에 하나씩 연결해서 테스트
• 시리얼 스와프 기능 사용 (ESP32)
• 로직 레벨 확인 (3.3V vs 5V)
```

**문제**: 데이터 전송이 불완전함
```bash
증상: JSON 데이터가 깨지거나 누락됨

해결 방법:
1. 버퍼 크기 증가
2. 전송 간격 조정 (2초 권장)
3. 체크섬 또는 CRC 추가
4. 재전송 메커니즘 구현

코드 개선:
String data = "DATA:" + jsonString + "\n";
Serial.print(data);
Serial.flush(); // 전송 완료 대기
```

### 📱 웹 인터페이스 문제

**문제**: 웹 페이지에 접속할 수 없음
```bash
진단 단계:
1. ESP32 IP 주소 확인 (시리얼 모니터)
2. 같은 네트워크 연결 확인 (PC와 ESP32)
3. 방화벽 설정 확인
4. 브라우저 캐시 삭제

접속 방법:
• http://smartfan.local (mDNS)
• http://192.168.x.x (직접 IP)
• AP 모드: http://192.168.4.1

네트워크 테스트:
ping smartfan.local
ping [ESP32_IP_ADDRESS]
```

**문제**: WebSocket 연결이 끊어짐
```bash
증상: 실시간 데이터 업데이트 중단

원인:
• 네트워크 불안정
• 브라우저 탭 비활성화
• ESP32 메모리 부족

해결책:
1. 자동 재연결 로직 강화
2. 하트비트 주기 조정
3. 메모리 사용량 최적화
4. 연결 상태 표시 개선

JavaScript 개선:
ws.onclose = function() {
  console.log('WebSocket closed, reconnecting...');
  setTimeout(initWebSocket, 5000); // 5초 후 재연결
};
```

---

## 💻 소프트웨어 문제 해결

### 🔧 Arduino IDE 관련

**문제**: 코드 컴파일 오류
```bash
일반적 오류들:

1. 라이브러리 없음:
   → 라이브러리 매니저에서 필수 라이브러리 설치
   → #include 경로 확인

2. 보드 선택 오류:
   → Arduino: "Arduino Uno" 선택
   → ESP32: "ESP32 Dev Module" 선택

3. 포트 인식 안됨:
   → USB 드라이버 설치 (CH340, CP2102 등)
   → 다른 USB 포트 시도
   → 케이블 교체

4. 메모리 부족:
   → 불필요한 변수 제거
   → PROGMEM 사용
   → 문자열 최적화
```

**문제**: 업로드 실패
```bash
ESP32 업로드 실패 시:
1. GPIO0을 GND에 연결하고 리셋 (부트 모드)
2. 업로드 중 EN 버튼 누름
3. 보드레이트 115200 → 921600 변경 시도
4. Flash 모드를 QIO → DIO로 변경

Arduino 업로드 실패 시:
1. 다른 프로그램에서 시리얼 포트 사용 중인지 확인
2. Arduino 리셋 버튼 누르고 업로드
3. 부트로더 손상 시 ISP로 복구
```

### 🐛 런타임 오류

**문제**: 시스템 재시작 반복
```bash
원인 분석:
• 무한 루프
• 스택 오버플로우  
• WatchDog 타임아웃
• 전원 공급 불안정

디버깅 방법:
1. 시리얼 모니터로 마지막 메시지 확인
2. 단계별 주석 처리로 문제 구간 찾기
3. 메모리 사용량 모니터링
4. 전원 공급 안정화

예방 조치:
• delay() 함수 적절히 사용
• while 루프에 탈출 조건 추가
• WatchDog 타이머 활용
• 전원 커패시터 추가
```

**문제**: 메모리 누수
```bash
증상: 시간이 지날수록 시스템 느려짐

진단:
Serial.print("Free heap: ");
Serial.println(ESP.getFreeHeap()); // ESP32
Serial.println(freeMemory()); // Arduino

해결 방법:
• 동적 할당 최소화
• String 대신 char 배열 사용
• 큰 배열을 PROGMEM에 저장
• 정기적 메모리 상태 모니터링
```

---

## 🔍 고급 진단 도구

### 📊 시리얼 명령어

**시스템 진단 명령어**:
```bash
STATUS      → 전체 시스템 상태 확인
RESET       → 시스템 초기화
SAVE        → 설정 EEPROM 저장
DEBUG:ON    → 디버그 모드 활성화
DEBUG:OFF   → 디버그 모드 비활성화
SENSOR      → 센서 테스트 모드
MOTOR:TEST  → 모터 테스트 모드
WIFI:SCAN   → WiFi 네트워크 스캔
```

**실시간 모니터링**:
```bash
MONITOR:START  → 실시간 데이터 스트림 시작
MONITOR:STOP   → 실시간 데이터 스트림 중지
LOG:CLEAR      → 로그 버퍼 클리어
LOG:DUMP       → 전체 로그 출력
```

### 🌐 웹 기반 진단

**진단 URL**:
```bash
http://smartfan.local/api/system/info     → 시스템 정보
http://smartfan.local/api/fan/status      → 팬 상태
http://smartfan.local/api/data/realtime   → 실시간 데이터
http://smartfan.local/api/system/reset    → 시스템 리셋
```

**JavaScript 콘솔 명령어**:
```javascript
// WebSocket 연결 상태 확인
console.log(ws.readyState);

// 수동으로 명령 전송
ws.send('{"action":"speed","value":50}');

// 실시간 데이터 로깅
ws.onmessage = function(event) {
  console.log('Data:', JSON.parse(event.data));
};
```

---

## 📞 추가 지원

### 🆘 도움 요청하기

**GitHub Issues 템플릿**:
```markdown
## 문제 설명
[간단한 문제 설명]

## 환경 정보
- Arduino IDE 버전: 
- ESP32 보드 패키지 버전:
- 사용 중인 보드: 
- 운영체제: 

## 재현 단계
1. 
2. 
3. 

## 예상 결과
[기대했던 동작]

## 실제 결과  
[실제 발생한 현상]

## 시리얼 로그
```
[시리얼 모니터 출력 내용]
```

## 추가 정보
[스크린샷, 회로 사진 등]
```

### 📚 참고 자료

**공식 문서**:
- [Arduino 레퍼런스](https://www.arduino.cc/reference/)
- [ESP32 가이드](https://docs.espressif.com/projects/esp-idf/)
- [DHT22 데이터시트](https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf)

**커뮤니티**:
- [프로젝트 Discussions](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)
- [Arduino 포럼](https://forum.arduino.cc/)
- [ESP32 커뮤니티](https://www.esp32.com/)

---

**🔧 문제 해결은 학습의 과정입니다. 포기하지 마세요! 🔧**

이 가이드로 해결되지 않는 문제가 있다면, 언제든 커뮤니티에 도움을 요청하세요. 함께 해결해나가겠습니다! 💪