# 🤝 기여 가이드라인 (Contributing Guidelines)

**Arduino Daily Project에 기여해주셔서 감사합니다!**

[🇰🇷 한국어](#한국어) | [🇺🇸 English](#english)

---

## 한국어

### 🌟 기여 방법

#### 1. 💡 아이디어 제안
- **[Discussions](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)** 섹션에서 새로운 기능이나 개선사항을 제안해주세요
- 제목 형식: `[IDEA] 간단한 설명`
- 상세한 설명과 예상 효과를 포함해주세요

#### 2. 🐛 버그 리포트
- **[Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)** 섹션에서 버그를 신고해주세요
- 제목 형식: `[BUG] 간단한 문제 설명`
- 아래 템플릿을 사용해주세요:

```markdown
## 🐛 버그 설명
[버그에 대한 명확하고 간결한 설명]

## 🔄 재현 단계
1. ...
2. ...
3. ...

## 💭 예상 결과
[기대했던 동작]

## 📱 실제 결과
[실제로 발생한 현상]

## 🖥️ 환경 정보
- Arduino IDE 버전: 
- ESP32 보드 패키지 버전:
- 운영체제: 
- 사용 중인 보드: 

## 📸 추가 정보
[스크린샷, 로그, 회로 사진 등]
```

#### 3. ✨ 기능 요청
- **[Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)** 섹션에서 새 기능을 요청해주세요
- 제목 형식: `[FEATURE] 기능 이름`
- 아래 템플릿을 사용해주세요:

```markdown
## ✨ 기능 설명
[제안하는 기능에 대한 명확한 설명]

## 🎯 해결하려는 문제
[이 기능이 해결할 문제나 개선할 점]

## 💡 제안하는 해결책
[기능이 어떻게 작동해야 하는지 설명]

## 🔄 대안 고려사항
[고려해본 다른 방법들]

## 📊 추가 컨텍스트
[관련 스크린샷, 링크, 참고자료 등]
```

#### 4. 🔧 코드 기여

##### Fork & Clone
```bash
# 1. GitHub에서 저장소 Fork
# 2. 로컬에 클론
git clone https://github.com/YOUR_USERNAME/Arduino-Daily-project.git
cd Arduino-Daily-project

# 3. 원본 저장소를 upstream으로 추가
git remote add upstream https://github.com/Jirehhyeon/Arduino-Daily-project.git
```

##### 브랜치 전략
```bash
# 기능 개발용 브랜치 생성
git checkout -b feature/새기능이름

# 버그 수정용 브랜치 생성  
git checkout -b bugfix/버그설명

# 문서 수정용 브랜치 생성
git checkout -b docs/문서수정내용
```

##### 커밋 메시지 규칙
```bash
# 형식: [타입] 간단한 설명
#
# 타입:
# - feat: 새로운 기능
# - fix: 버그 수정
# - docs: 문서 수정
# - style: 코드 스타일 변경
# - refactor: 리팩토링
# - test: 테스트 추가/수정
# - chore: 기타 작업

# 예시:
git commit -m "feat: DHT22 센서 자동 보정 기능 추가

- 부팅 시 자동으로 센서 제로점 보정
- 보정값을 EEPROM에 저장
- 시리얼 모니터에 보정 결과 출력"
```

##### Pull Request 과정
```bash
# 1. 최신 코드로 업데이트
git fetch upstream
git rebase upstream/main

# 2. 변경사항 푸시
git push origin feature/새기능이름

# 3. GitHub에서 Pull Request 생성
# 4. 아래 템플릿 사용
```

**Pull Request 템플릿:**
```markdown
## 📋 변경사항 요약
[이 PR에서 변경한 내용을 간단히 설명]

## 🎯 변경 이유
[왜 이런 변경이 필요한지 설명]

## 🧪 테스트 방법
- [ ] 하드웨어 테스트 완료
- [ ] 코드 컴파일 확인
- [ ] 기능 테스트 완료
- [ ] 문서 업데이트 완료

## 📸 스크린샷/동영상
[결과물 스크린샷이나 동영상]

## ✅ 체크리스트
- [ ] 코드가 프로젝트 스타일 가이드를 따름
- [ ] 자체 리뷰 완료
- [ ] 관련 문서 업데이트 완료
- [ ] 안전성 검토 완료
```

### 🎨 코딩 스타일 가이드

#### Arduino/C++ 코드
```cpp
// 1. 인덴테이션: 2칸 공백 사용
void setup() {
  // 코드
}

// 2. 변수명: camelCase 사용
int motorSpeed = 0;
bool isEnabled = false;

// 3. 상수: UPPER_CASE 사용
#define MAX_SPEED 255
#define DEFAULT_TIMEOUT 5000

// 4. 함수명: camelCase 사용
void setMotorSpeed(int speed) {
  // 구현
}

// 5. 주석: 한국어 주석 권장
// 모터 속도를 설정하는 함수
void setMotorSpeed(int speed) {
  // 안전 범위 검사
  if (speed < 0 || speed > MAX_SPEED) {
    return;
  }
  
  // PWM 출력
  analogWrite(MOTOR_PWM_PIN, speed);
}

// 6. 에러 처리: 필수
bool readSensor() {
  float temperature = dht.readTemperature();
  
  // 센서 읽기 실패 처리
  if (isnan(temperature)) {
    Serial.println("ERROR: 센서 읽기 실패");
    return false;
  }
  
  return true;
}
```

#### 문서 작성 가이드
```markdown
# 1. 제목: 명확하고 간결하게
## 2. 구조: 논리적 순서로 구성
### 3. 코드 블록: 언어 명시
```cpp
// 코드 예시
```

### 4. 이모지 사용: 일관성 유지
- 🔧 하드웨어 관련
- 📱 소프트웨어 관련  
- ⚠️ 주의사항
- ✅ 완료/성공
- ❌ 실패/오류

### 5. 링크: 상대 경로 사용
[관련 문서](RELATED_DOC.md)
```

### 🔒 보안 및 안전 가이드라인

#### 코드 보안
- **API 키나 비밀번호를 코드에 하드코딩하지 마세요**
- **사용자 입력값을 반드시 검증하세요**
- **버퍼 오버플로우를 방지하세요**

```cpp
// 나쁜 예
char ssid[] = "MyWiFi";
char password[] = "mypassword123"; // 하드코딩된 비밀번호

// 좋은 예  
char ssid[32];
char password[64];
loadCredentialsFromEEPROM(ssid, password);
```

#### 하드웨어 안전
- **전압/전류 사양을 반드시 확인하세요**
- **연결 전 회로도를 검토하세요** 
- **안전 장치를 우회하지 마세요**

```cpp
// 안전 검사 예시
bool safetyCheck() {
  if (motorCurrent > MAX_CURRENT) {
    emergencyStop();
    return false;
  }
  
  if (temperature > MAX_TEMPERATURE) {
    reduceSpeed();
    return false;
  }
  
  return true;
}
```

### 📋 리뷰 프로세스

#### 코드 리뷰 기준
1. **기능성**: 요구사항을 충족하는가?
2. **안전성**: 하드웨어 손상 위험은 없는가?
3. **성능**: 메모리/처리 효율성은 적절한가?
4. **가독성**: 코드를 이해하기 쉬운가?
5. **문서화**: 충분한 설명이 있는가?

#### 승인 절차
- **자동 검사**: GitHub Actions 통과 필수
- **하드웨어 테스트**: 실제 하드웨어에서 테스트 완료
- **피어 리뷰**: 최소 1명의 승인 필요
- **메인테이너 승인**: 최종 병합 승인

### 🏆 기여자 인정

#### 기여 유형별 인정
- **코드 기여**: Contributors 섹션에 이름 추가
- **문서 개선**: Documentation 섹션에 이름 추가  
- **버그 발견**: Bug Hunters 섹션에 이름 추가
- **아이디어 제안**: Ideas 섹션에 이름 추가

#### 특별 인정
- **월간 최고 기여자**: README에 특별 언급
- **주요 기능 개발**: 릴리즈 노트에 특별 감사
- **장기 기여자**: Hall of Fame 등록

---

## English

### 🌟 How to Contribute

#### 1. 💡 Suggest Ideas
- Propose new features or improvements in the **[Discussions](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)** section
- Title format: `[IDEA] Brief description`
- Include detailed explanation and expected benefits

#### 2. 🐛 Report Bugs
- Report bugs in the **[Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)** section
- Title format: `[BUG] Brief problem description`
- Please use this template:

```markdown
## 🐛 Bug Description
[Clear and concise description of the bug]

## 🔄 Steps to Reproduce
1. ...
2. ...
3. ...

## 💭 Expected Behavior
[What you expected to happen]

## 📱 Actual Behavior
[What actually happened]

## 🖥️ Environment Information
- Arduino IDE Version: 
- ESP32 Board Package Version:
- Operating System: 
- Board in Use: 

## 📸 Additional Information
[Screenshots, logs, circuit photos, etc.]
```

#### 3. ✨ Feature Requests
- Request new features in the **[Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)** section
- Title format: `[FEATURE] Feature name`
- Please use this template:

```markdown
## ✨ Feature Description
[Clear description of the proposed feature]

## 🎯 Problem to Solve
[What problem this feature would solve or improve]

## 💡 Proposed Solution
[How the feature should work]

## 🔄 Alternative Considerations
[Other approaches you've considered]

## 📊 Additional Context
[Related screenshots, links, references, etc.]
```

#### 4. 🔧 Code Contributions

##### Fork & Clone
```bash
# 1. Fork the repository on GitHub
# 2. Clone locally
git clone https://github.com/YOUR_USERNAME/Arduino-Daily-project.git
cd Arduino-Daily-project

# 3. Add original repository as upstream
git remote add upstream https://github.com/Jirehhyeon/Arduino-Daily-project.git
```

##### Branch Strategy
```bash
# Create feature development branch
git checkout -b feature/new-feature-name

# Create bug fix branch  
git checkout -b bugfix/bug-description

# Create documentation branch
git checkout -b docs/documentation-update
```

##### Commit Message Rules
```bash
# Format: [type] brief description
#
# Types:
# - feat: new feature
# - fix: bug fix
# - docs: documentation update
# - style: code style changes
# - refactor: refactoring
# - test: add/modify tests
# - chore: other tasks

# Example:
git commit -m "feat: Add DHT22 sensor auto-calibration

- Automatically calibrate sensor zero point on boot
- Save calibration values to EEPROM
- Display calibration results in serial monitor"
```

##### Pull Request Process
```bash
# 1. Update to latest code
git fetch upstream
git rebase upstream/main

# 2. Push changes
git push origin feature/new-feature-name

# 3. Create Pull Request on GitHub
# 4. Use template below
```

**Pull Request Template:**
```markdown
## 📋 Changes Summary
[Brief description of changes in this PR]

## 🎯 Reason for Changes
[Why these changes are necessary]

## 🧪 Testing Method
- [ ] Hardware testing completed
- [ ] Code compilation verified
- [ ] Functionality testing completed
- [ ] Documentation updated

## 📸 Screenshots/Videos
[Screenshots or videos of results]

## ✅ Checklist
- [ ] Code follows project style guidelines
- [ ] Self-review completed
- [ ] Related documentation updated
- [ ] Safety review completed
```

### 🎨 Coding Style Guide

#### Arduino/C++ Code
```cpp
// 1. Indentation: Use 2 spaces
void setup() {
  // code
}

// 2. Variable names: Use camelCase
int motorSpeed = 0;
bool isEnabled = false;

// 3. Constants: Use UPPER_CASE
#define MAX_SPEED 255
#define DEFAULT_TIMEOUT 5000

// 4. Function names: Use camelCase
void setMotorSpeed(int speed) {
  // implementation
}

// 5. Comments: English comments preferred for international collaboration
// Function to set motor speed
void setMotorSpeed(int speed) {
  // Safety range check
  if (speed < 0 || speed > MAX_SPEED) {
    return;
  }
  
  // PWM output
  analogWrite(MOTOR_PWM_PIN, speed);
}

// 6. Error handling: Mandatory
bool readSensor() {
  float temperature = dht.readTemperature();
  
  // Handle sensor read failure
  if (isnan(temperature)) {
    Serial.println("ERROR: Failed to read sensor");
    return false;
  }
  
  return true;
}
```

#### Documentation Guidelines
```markdown
# 1. Titles: Clear and concise
## 2. Structure: Logical order
### 3. Code blocks: Specify language
```cpp
// Code example
```

### 4. Emoji usage: Maintain consistency
- 🔧 Hardware related
- 📱 Software related  
- ⚠️ Warnings
- ✅ Complete/Success
- ❌ Failure/Error

### 5. Links: Use relative paths
[Related Document](RELATED_DOC.md)
```

### 🔒 Security and Safety Guidelines

#### Code Security
- **Never hardcode API keys or passwords**
- **Always validate user inputs**
- **Prevent buffer overflows**

```cpp
// Bad example
char ssid[] = "MyWiFi";
char password[] = "mypassword123"; // Hardcoded password

// Good example  
char ssid[32];
char password[64];
loadCredentialsFromEEPROM(ssid, password);
```

#### Hardware Safety
- **Always verify voltage/current specifications**
- **Review circuit diagrams before connecting** 
- **Never bypass safety devices**

```cpp
// Safety check example
bool safetyCheck() {
  if (motorCurrent > MAX_CURRENT) {
    emergencyStop();
    return false;
  }
  
  if (temperature > MAX_TEMPERATURE) {
    reduceSpeed();
    return false;
  }
  
  return true;
}
```

### 📋 Review Process

#### Code Review Criteria
1. **Functionality**: Does it meet requirements?
2. **Safety**: Any risk of hardware damage?
3. **Performance**: Is memory/processing efficiency appropriate?
4. **Readability**: Is the code easy to understand?
5. **Documentation**: Is there sufficient explanation?

#### Approval Process
- **Automated Checks**: Must pass GitHub Actions
- **Hardware Testing**: Must be tested on actual hardware
- **Peer Review**: Minimum 1 approval required
- **Maintainer Approval**: Final merge approval

### 🏆 Contributor Recognition

#### Recognition by Contribution Type
- **Code Contributions**: Name added to Contributors section
- **Documentation Improvements**: Name added to Documentation section  
- **Bug Discovery**: Name added to Bug Hunters section
- **Idea Proposals**: Name added to Ideas section

#### Special Recognition
- **Monthly Top Contributor**: Special mention in README
- **Major Feature Development**: Special thanks in release notes
- **Long-term Contributors**: Hall of Fame registration

---

## 📞 Contact

질문이나 도움이 필요하시면 언제든 연락주세요!

**For questions or help, please feel free to contact us!**

- **GitHub Discussions**: [Community Forum](https://github.com/Jirehhyeon/Arduino-Daily-project/discussions)
- **Issues**: [Report Issues](https://github.com/Jirehhyeon/Arduino-Daily-project/issues)
- **Email**: jirehhyeon@example.com

---

**🤝 함께 만들어가는 오픈소스 프로젝트! | Building Open Source Together! 🤝**

여러분의 기여가 이 프로젝트를 더욱 발전시킵니다. 작은 기여라도 큰 도움이 됩니다!

*Your contributions make this project better. Even small contributions are greatly appreciated!*