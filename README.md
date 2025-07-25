# 🔧 Arduino Daily Project

**일일 Arduino 프로젝트 모음 - 실용적이고 교육적인 하드웨어 프로젝트들**

## 🎯 최신 프로젝트: HW-020 모터 쉴드 제어 시스템

**HW-020 모터 쉴드를 활용한 12V 선풍기 모터 제어 시스템**

> 점퍼선을 통한 안전한 모터 제어와 실시간 모니터링 기능을 제공하는 완전한 Arduino 프로젝트

[![Arduino](https://img.shields.io/badge/Arduino-UNO%20R3-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Motor Shield](https://img.shields.io/badge/HW--020-L298P-red?style=for-the-badge)](https://github.com/Jirehhyeon/Arduino-Daily-project)
[![Safety First](https://img.shields.io/badge/Safety-Verified-green?style=for-the-badge&logo=shield&logoColor=white)](https://github.com/Jirehhyeon/Arduino-Daily-project)

## ✨ 주요 기능

### 🎛️ **완전한 모터 제어**
- **PWM 속도 제어**: 0-100% 부드러운 속도 조절
- **방향 제어**: 시계방향/반시계방향 전환
- **부드러운 가속/감속**: 급작스러운 시작/정지 방지
- **실시간 상태 모니터링**: 속도, 방향, 온도, 전류

### 🛡️ **안전 시스템**
- **과전류 보호**: 1.5A 임계값으로 자동 정지
- **온도 모니터링**: 60°C 초과시 경고 및 감속
- **안전 모드**: 위험 감지시 즉시 모터 정지
- **전기적 안전성**: 점퍼선 안전성 2.5배 마진 확보

### 💡 **지능형 기능**
- **EEPROM 설정 저장**: 재시작 후에도 설정 유지
- **LED 상태 표시**: 전원, 방향, 경고 LED
- **시리얼 명령**: PC에서 실시간 제어 및 모니터링
- **성능 리포트**: 주기적 상태 보고

## 🔌 하드웨어 구성

### 필수 부품
| 부품명 | 수량 | 용도 |
|--------|------|------|
| Arduino Uno R3 | 1개 | 메인 컨트롤러 |
| HW-020 Motor Shield | 1개 | L298P 기반 모터 드라이버 |
| 12V DC 선풍기 모터 | 1개 | 소형, 프로펠러 장착 가능 |
| 12V 2A 어댑터 | 1개 | 외부 전원 공급 |
| 점퍼선 20AWG | 10개 | 신호 연결 (Male-Female) |
| 10kΩ 포텐셔미터 | 1개 | 속도 제어용 |
| 택트 스위치 | 1개 | 방향 제어용 |
| LED (빨강/초록/노랑) | 3개 | 상태 표시용 |
| 220Ω 저항 | 3개 | LED 전류 제한용 |

### 선택 부품
- **ACS712 전류 센서** - 정확한 전류 모니터링
- **LM35 온도 센서** - 실제 온도 측정
- **브레드보드** - 추가 회로 구성

## ⚡ 전기적 안전성

### 전류 계산 결과
```
12V 소형 선풍기 모터: 0.3-0.8A (정격)
20AWG 점퍼선 허용: 2A (안전), 11A (최대)
HW-020 쉴드 허용: 2A (채널당)

안전 마진: 2.5배 ✅ 충분히 안전
```

## 📋 핀 배치도

| Arduino 핀 | 연결 대상 | 기능 | 타입 |
|------------|-----------|------|------|
| **D3** | HW-020 ENA | 모터 PWM | PWM 출력 |
| **D12** | HW-020 IN1 | 모터 방향 | 디지털 출력 |
| **A0** | 포텐셔미터 | 속도 제어 | 아날로그 입력 |
| **A1** | 전류 센서 | 전류 감지 | 아날로그 입력 |
| **D2** | 방향 버튼 | 방향 토글 | 디지털 입력 (인터럽트) |
| **D4** | 방향 LED | 상태 표시 | 디지털 출력 |
| **D5** | 전원 LED | 상태 표시 | 디지털 출력 |
| **D6** | 경고 LED | 경고 표시 | 디지털 출력 |

## 🚀 빠른 시작

### 1. 하드웨어 조립
```bash
1. Arduino에 HW-020 쉴드 장착
2. 12V 어댑터 극성 확인 후 연결
3. 모터를 쉴드 OUT1, OUT2에 연결
4. 제어 회로 (포텐셔미터, 버튼, LED) 연결
5. 점퍼선으로 신호선 연결
```

### 2. 소프트웨어 업로드
```bash
1. Arduino IDE에서 Arduino_Motor_Fan_Project.ino 열기
2. Arduino Uno 보드 선택
3. 포트 선택 후 업로드
4. 시리얼 모니터 열기 (115200 보드레이트)
```

### 3. 초기 테스트
```bash
1. LED 테스트 시퀀스 확인
2. 저속 모터 테스트 실행
3. 안전 검사 통과 확인
4. 포텐셔미터로 속도 조절 테스트
```

## 💻 제어 명령어

### 시리얼 명령어
| 명령어 | 기능 | 예시 |
|--------|------|------|
| `START` | 모터 시작 | `START` |
| `STOP` | 모터 정지 | `STOP` |
| `SPEED:<0-100>` | 속도 설정 | `SPEED:75` |
| `DIR` | 방향 전환 | `DIR` |
| `STATUS` | 상태 조회 | `STATUS` |
| `SAVE` | 설정 저장 | `SAVE` |
| `RESET` | 초기화 | `RESET` |
| `HELP` | 도움말 | `HELP` |

### 하드웨어 제어
- **포텐셔미터**: 실시간 속도 조절 (0-100%)
- **방향 버튼**: 방향 전환 (디바운싱 적용)
- **LED 표시**: 
  - 초록: 전원 상태
  - 노랑: 방향 (깜빡임 속도로 구분)
  - 빨강: 경고/안전 모드

## 🔧 주요 기능 상세

### 1. 부드러운 속도 제어
```cpp
// 점진적 속도 변화 (램핑)
if (current_speed < target_speed) {
    current_speed = min(current_speed + 2, target_speed);
} else if (current_speed > target_speed) {
    current_speed = max(current_speed - 2, target_speed);
}
```

### 2. 안전 모니터링
```cpp
// 과전류/과열 검사
if (motor.current_ma > SAFETY_CURRENT) {
    motor.safety_mode = true;
    Serial.println("SAFETY MODE ACTIVATED");
}
```

### 3. 방향 변경 로직
```cpp
// 안전한 방향 변경
motor.enabled = false;    // 모터 정지
delay(200);               // 200ms 대기
motor.direction = !motor.direction;  // 방향 전환
motor.enabled = true;     // 재시작
```

### 4. EEPROM 설정 저장
```cpp
// 설정 자동 저장 (30초마다)
EEPROM.write(EEPROM_SPEED, motor.speed);
EEPROM.write(EEPROM_DIRECTION, motor.direction);
```

## 📊 성능 사양

### 시스템 성능
- **제어 주기**: 50Hz (20ms 간격)
- **속도 분해능**: 256단계 (0-255 PWM)
- **안전 검사 주기**: 1초
- **상태 리포트**: 5초마다
- **설정 저장**: 30초마다

### 모터 성능
- **속도 범위**: 0-100% (부드러운 제어)
- **방향**: 양방향 (CW/CCW)
- **가속/감속**: 점진적 (충격 방지)
- **정밀도**: ±2% (PWM 기반)

## 🛡️ 안전 기능

### 다중 보호 시스템
1. **과전류 보호**: 1.5A 초과시 즉시 정지
2. **과열 보호**: 60°C 초과시 감속 및 경고
3. **전압 모니터링**: 저전압 감지 및 경고
4. **안전 모드**: 위험 상황 자동 대응
5. **비상 정지**: 시리얼 명령으로 즉시 정지

### 전기적 안전
- **절연**: 모든 연결부 절연 확인
- **극성 보호**: 전원 연결 극성 표시
- **과부하 방지**: 모터 사양 내 동작
- **단락 보호**: 연결 전 단락 검사

## 🔍 문제 해결

### 일반적인 문제들

**모터가 돌아가지 않을 때:**
1. 12V 어댑터 연결 및 극성 확인
2. PWM 신호 (D3) 연결 확인
3. `motor.enabled = true` 설정 확인

**속도 제어가 안 될 때:**
1. 포텐셔미터 A0 연결 확인
2. 5V 전원 공급 확인
3. PWM 매핑 로직 검증

**방향이 바뀌지 않을 때:**
1. D12 신호선 연결 확인
2. OUT1, OUT2 터미널 확인
3. 방향 로직 디버깅

## 📈 고급 기능

### 1. PWM 주파수 최적화
```cpp
// 고주파 PWM으로 더 부드러운 제어
setPWMFrequency(3, 1);  // 31.25kHz
```

### 2. 성능 모니터링
```cpp
// 실시간 성능 데이터
Serial.print("Performance: Speed=");
Serial.print(map(motor.speed, 0, MAX_SPEED, 0, 100));
Serial.print("%, Current=");
Serial.println(motor.current_ma);
```

### 3. 예측적 안전 제어
```cpp
// 온도 기반 예측 제어
if (motor.temperature > TEMP_THRESHOLD - 10) {
    // 온도가 임계값에 근접하면 미리 감속
    target_speed = motor.speed * 0.8;
}
```

## 🔄 확장 계획

### 하드웨어 확장
- [ ] **엔코더 추가**: 정확한 RPM 피드백
- [ ] **온도 센서**: 실제 모터 온도 측정
- [ ] **진동 센서**: 불균형 감지
- [ ] **원격 제어**: ESP32로 WiFi 제어

### 소프트웨어 확장  
- [ ] **PID 제어**: 정밀한 속도 제어
- [ ] **데이터 로깅**: SD카드에 운전 기록
- [ ] **웹 인터페이스**: 브라우저에서 모니터링
- [ ] **모바일 앱**: 스마트폰 제어

## 🤝 기여하기

이 프로젝트는 오픈소스입니다. 기여를 환영합니다!

1. 이 저장소를 Fork 하세요
2. 새로운 기능 브랜치를 만드세요 (`git checkout -b feature/새기능`)
3. 변경사항을 커밋하세요 (`git commit -am '새 기능 추가'`)
4. 브랜치에 Push 하세요 (`git push origin feature/새기능`)
5. Pull Request를 생성하세요

### 개발 가이드라인
- **안전 우선**: 모든 변경사항은 안전성 검토 필수
- **테스트**: 하드웨어 테스트 후 코드 제출
- **문서화**: 새 기능은 문서 업데이트 포함
- **호환성**: Arduino IDE 호환성 유지

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 📞 지원 및 문의

- **GitHub Issues**: [프로젝트 이슈](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)
- **이메일**: jirehhyeon@example.com
- **문서**: 이 README 파일과 코드 주석 참조

## 🙏 감사의 말

- **Arduino 커뮤니티**: 오픈소스 하드웨어 플랫폼
- **L298P 칩**: 안정적인 모터 드라이버
- **모든 기여자들**: 프로젝트 개선에 참여해주신 분들

---

<div align="center">

**🔧 Arduino Daily Project - 안전하고 스마트한 모터 제어의 시작! 🔧**

![Made with Arduino](https://img.shields.io/badge/Made%20with-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![Safety First](https://img.shields.io/badge/Safety-First-green?style=for-the-badge&logo=shield&logoColor=white)
![Open Source](https://img.shields.io/badge/Open-Source-blue?style=for-the-badge&logo=opensource&logoColor=white)

⭐ **이 프로젝트가 도움이 되었다면 스타를 눌러주세요!** ⭐

</div>