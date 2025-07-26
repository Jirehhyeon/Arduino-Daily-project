# ⚡ 빠른 시작 가이드 - 스마트 선풍기 v3.0

**10분 안에 스마트 선풍기 시작하기!**

## 🎯 시작하기 전에

### ✅ 준비물 체크리스트
```
하드웨어 (필수):
☐ Arduino Uno R3
☐ ESP32 DevKit v1  
☐ HW-020 Motor Shield
☐ 12V DC 선풍기 모터
☐ 12V 2A 어댑터
☐ DHT22 온습도 센서
☐ 점퍼선 세트

소프트웨어:
☐ Arduino IDE 1.8.x+
☐ ESP32 보드 패키지
☐ 필수 라이브러리들
```

### 🔧 라이브러리 설치 (5분)
Arduino IDE에서 다음 라이브러리를 설치하세요:

```
라이브러리 매니저에서 검색:
1. "DHT sensor library" by Adafruit
2. "LiquidCrystal I2C" 
3. "ArduinoJson"
4. "Servo" (내장)
5. "WebSockets" by Markus Sattler
```

**ESP32 보드 패키지 설치:**
1. 파일 → 환경설정
2. 보드 매니저 URL에 추가: `https://dl.espressif.com/dl/package_esp32_index.json`
3. 도구 → 보드 매니저에서 "ESP32" 검색 후 설치

---

## 🚀 3단계 빠른 설정

### 1️⃣ 하드웨어 연결 (3분)

**기본 연결만으로 시작:**
```
Arduino ←→ ESP32:
• Arduino TX (D1) → ESP32 RX2
• Arduino RX (D0) → ESP32 TX2
• Arduino GND → ESP32 GND

Arduino ←→ 센서:
• DHT22 Data → Arduino D7
• DHT22 VCC → Arduino 5V
• DHT22 GND → Arduino GND

Arduino ←→ 모터:
• HW-020 쉴드를 Arduino에 장착
• 12V 모터를 쉴드 OUT1, OUT2에 연결
• 12V 어댑터를 쉴드에 연결 (극성 주의!)
```

### 2️⃣ 코드 업로드 (2분)

**Arduino 코드 업로드:**
```bash
1. Smart_Fan_Controller.ino 파일 열기
2. 보드: "Arduino Uno" 선택
3. 포트 선택 후 업로드 ▶️
```

**ESP32 코드 업로드:**
```bash
1. ESP32_WiFi_Controller.ino 파일 열기  
2. 보드: "ESP32 Dev Module" 선택
3. 포트 선택 후 업로드 ▶️
```

### 3️⃣ WiFi 설정 및 테스트 (2분)

**WiFi 연결:**
1. 스마트폰에서 WiFi 설정
2. "SmartFan_Setup" 네트워크 연결 (비밀번호: `12345678`)
3. 브라우저에서 `192.168.4.1` 접속
4. 홈 WiFi 정보 입력 후 "Connect" 클릭

**시스템 테스트:**
1. WiFi 연결 후 `http://smartfan.local` 접속
2. "🟢 Start" 버튼 클릭
3. 속도 슬라이더로 팬 속도 조절 테스트
4. 실시간 센서 데이터 확인

---

## 📱 모바일 앱 사용법

### 기본 제어
```
🟢 Start/🔴 Stop: 즉시 팬 on/off
🤖 Auto Mode: AI 자동 제어 
🌱 Eco Mode: 에너지 절약 모드
속도 슬라이더: 0-100% 조절
```

### 실시간 모니터링
```
🌡️ 온도/습도: 실시간 환경 데이터
⚡ 전류: 전력 사용량 모니터링  
🔄 런타임: 누적 동작 시간
🚨 안전 상태: 시스템 안전 모드
```

### 고급 기능
```
⏰ 스케줄: 시간대별 자동 동작
🌙 야간 모드: 조용한 밤 운전
🎛️ 풍향 조절: 0-180° 바람 방향
```

---

## 🛠️ 빠른 문제 해결

### ❓ 자주 묻는 질문

**Q: 모터가 돌아가지 않아요**
```
A: 체크 포인트:
✓ 12V 어댑터 연결 확인
✓ 모터 연결 (OUT1, OUT2) 확인
✓ Arduino D3 PWM 연결 확인
✓ 웹 앱에서 "Start" 버튼 클릭했는지 확인
```

**Q: WiFi 연결이 안 돼요**
```
A: 해결 방법:
✓ 2.4GHz WiFi인지 확인 (5GHz 지원 안함)
✓ 비밀번호 정확히 입력
✓ 라우터와 거리 확인 (30m 이내)
✓ ESP32 전원 재시작
```

**Q: 센서 데이터가 이상해요**
```
A: 확인 사항:
✓ DHT22 연결 (VCC=5V, GND, Data=D7)
✓ 센서 전원 공급 확인
✓ 2초 정도 기다린 후 데이터 확인
✓ 다른 센서로 교체 테스트
```

**Q: 웹 앱 접속이 안 돼요**
```
A: 시도해보세요:
✓ 같은 WiFi 네트워크 연결 확인
✓ http://smartfan.local 대신 IP 주소 직접 입력
✓ 브라우저 캐시 삭제
✓ 다른 브라우저로 테스트
```

### 🔧 빠른 수리 팁

**LED 상태 해석:**
```
🟢 전원 LED: 모터 동작 중
🟡 경고 LED: 주의 필요 (깜빡임)
🔴 상태 LED: 현재 모드 표시 (깜빡임 패턴으로 구분)
```

**비상 상황 대처:**
```
🚨 비상정지: 하드웨어 버튼 또는 앱의 긴급정지
🔥 과열 경고: 자동으로 속도 감소 또는 정지
⚡ 과전류: 즉시 안전 모드 전환
```

---

## 🎓 다음 단계

### 📚 더 자세히 알아보기
- **완전한 매뉴얼**: [Smart_Fan_README.md](Smart_Fan_README.md)
- **회로도**: [Smart_Fan_Circuit_Diagram.md](Smart_Fan_Circuit_Diagram.md)
- **개발 로드맵**: [PROJECT_ROADMAP.md](PROJECT_ROADMAP.md)

### 🛠️ 고급 설정
```
센서 추가:
• ACS712 전류 센서 (정밀한 전력 모니터링)
• LDR 조도 센서 (자동 야간 모드)
• 마이크 센서 (소음 감지)
• I2C LCD (상태 디스플레이)

고급 기능:
• 서보 모터 (풍향 자동 조절)
• 스케줄 설정 (시간대별 자동 동작)
• 음성 제어 (확장 모듈)
• 클라우드 연동 (v3.1 예정)
```

### 🤝 커뮤니티 참여
- **GitHub Issues**: 문제 신고 및 기능 제안
- **Discussions**: 사용 경험 공유
- **Wiki**: 팁과 트릭 공유

---

## 📞 도움이 필요하면

**즉시 도움:**
1. 🔍 [FAQ 섹션](Smart_Fan_README.md#문제-해결-가이드) 먼저 확인
2. 💬 [GitHub Discussions](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)에서 질문
3. 🐛 [GitHub Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)로 버그 리포트

**유용한 팁:**
- 시리얼 모니터(115200 보드레이트)로 디버그 정보 확인
- LED 상태로 시스템 상태 파악
- 단계별로 하나씩 테스트하며 문제 범위 좁히기

---

**🌀 10분 만에 스마트 선풍기 완성! 이제 시원한 바람을 즐기세요! 🌀**

문제가 있으면 언제든 도움을 요청하세요. 우리 커뮤니티가 함께 해결해드립니다! 💪