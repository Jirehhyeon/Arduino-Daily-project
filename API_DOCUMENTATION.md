# 📡 API 문서 - 스마트 선풍기 v3.0

**Arduino-ESP32 통신 프로토콜 및 웹 API 완전 가이드**

## 📋 목차

1. [🔗 시리얼 통신 프로토콜](#-시리얼-통신-프로토콜)
2. [🌐 HTTP REST API](#-http-rest-api)
3. [⚡ WebSocket 실시간 API](#-websocket-실시간-api)
4. [📊 데이터 스키마](#-데이터-스키마)
5. [🔧 SDK 및 라이브러리](#-sdk-및-라이브러리)
6. [🧪 테스트 및 디버깅](#-테스트-및-디버깅)

---

## 🔗 시리얼 통신 프로토콜

### 연결 설정

**Arduino ↔ ESP32 시리얼 연결**:
```
Baud Rate: 115200
Data Bits: 8
Stop Bits: 1
Parity: None
Flow Control: None
```

**연결 확인**:
```cpp
// Arduino → ESP32
Serial.println("PING");

// ESP32 → Arduino  
Serial.println("PONG");
```

### 명령 프로토콜

#### Arduino → ESP32 (센서 데이터)

**데이터 전송 형식**:
```json
DATA:{"temperature":25.5,"humidity":60.2,"current":0.85,"light":75,"speed":50,"enabled":true,"direction":true,"servo":90,"auto":false,"eco":false,"night":false,"safety":false,"runtime":123456}
```

**데이터 필드 설명**:
| 필드 | 타입 | 범위 | 설명 |
|------|------|------|------|
| `temperature` | float | -40.0 ~ 80.0 | 온도 (°C) |
| `humidity` | float | 0.0 ~ 100.0 | 습도 (%) |
| `current` | float | 0.0 ~ 5.0 | 전류 (A) |
| `light` | int | 0 ~ 100 | 조도 (%) |
| `speed` | int | 0 ~ 100 | 팬 속도 (%) |
| `enabled` | boolean | true/false | 팬 동작 상태 |
| `direction` | boolean | true/false | 팬 방향 (true=정방향) |
| `servo` | int | 0 ~ 180 | 서보 각도 (도) |
| `auto` | boolean | true/false | 자동 모드 |
| `eco` | boolean | true/false | 에코 모드 |
| `night` | boolean | true/false | 야간 모드 |
| `safety` | boolean | true/false | 안전 모드 |
| `runtime` | long | 0 ~ 4294967295 | 실행 시간 (ms) |

#### ESP32 → Arduino (제어 명령)

**명령 형식**: `CMD:<COMMAND>`

**기본 제어 명령**:
```bash
CMD:START          # 팬 시작
CMD:STOP           # 팬 정지  
CMD:SPEED:75       # 속도 설정 (0-100%)
CMD:DIR            # 방향 전환
CMD:AUTO:ON        # 자동 모드 켜기
CMD:AUTO:OFF       # 자동 모드 끄기
CMD:SERVO:45       # 서보 각도 설정 (0-180°)
CMD:EMERGENCY      # 비상 정지
CMD:RESET          # 시스템 리셋
CMD:STATUS         # 상태 조회
```

**고급 제어 명령**:
```bash
CMD:ECO:ON         # 에코 모드 활성화
CMD:ECO:OFF        # 에코 모드 비활성화
CMD:NIGHT:ON       # 야간 모드 활성화  
CMD:NIGHT:OFF      # 야간 모드 비활성화
CMD:SCHEDULE:ADD   # 스케줄 추가
CMD:SCHEDULE:DEL   # 스케줄 삭제
CMD:CALIBRATE      # 센서 보정
```

### 응답 코드

**Arduino 응답 메시지**:
```bash
OK                 # 명령 성공
ERROR:INVALID_CMD  # 잘못된 명령
ERROR:OUT_OF_RANGE # 범위 초과
ERROR:SAFETY_MODE  # 안전 모드 활성
WARN:OVER_TEMP     # 온도 경고
WARN:OVER_CURRENT  # 전류 경고
```

---

## 🌐 HTTP REST API

### 기본 정보

**Base URL**: `http://smartfan.local` 또는 `http://<ESP32_IP>`
**Port**: 80 (HTTP)
**Content-Type**: `application/json`

### 인증

현재 버전에서는 인증이 없으나, 향후 버전에서 API 키 또는 JWT 토큰 방식 도입 예정.

### 엔드포인트

#### 📊 시스템 정보

**GET /api/system/info**
```json
{
  "device_name": "SmartFan_v3.0",
  "firmware_version": "3.0.1",
  "hardware_version": "1.0",
  "mac_address": "24:0A:C4:XX:XX:XX",
  "ip_address": "192.168.1.100",
  "wifi_ssid": "MyHome_WiFi",
  "wifi_rssi": -45,
  "uptime": 123456789,
  "free_heap": 180000,
  "chip_id": "ESP32-D0WDQ6"
}
```

**GET /api/system/status**
```json
{
  "status": "online",
  "wifi_connected": true,
  "arduino_connected": true,
  "last_heartbeat": "2024-01-15T10:30:00Z",
  "errors": [],
  "warnings": ["TEMP_HIGH"]
}
```

#### 🌀 팬 제어

**GET /api/fan/status**
```json
{
  "enabled": true,
  "speed": 75,
  "direction": "forward",
  "servo_angle": 90,
  "mode": {
    "auto": false,
    "eco": true,
    "night": false,
    "safety": false
  },
  "runtime": 3600000
}
```

**POST /api/fan/control**
```json
// Request
{
  "action": "start|stop|speed|direction|servo",
  "value": 75,  // speed (0-100) or servo angle (0-180)
}

// Response
{
  "success": true,
  "message": "Fan speed set to 75%",
  "timestamp": "2024-01-15T10:30:00Z"
}
```

**POST /api/fan/mode**
```json
// Request
{
  "auto": true,
  "eco": false,
  "night": false
}

// Response  
{
  "success": true,
  "applied_modes": {
    "auto": true,
    "eco": false,
    "night": false
  }
}
```

#### 🌡️ 센서 데이터

**GET /api/sensors/current**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "temperature": {
    "value": 25.5,
    "unit": "celsius",
    "status": "normal"
  },
  "humidity": {
    "value": 60.2,
    "unit": "percent",
    "status": "normal"
  },
  "current": {
    "value": 0.85,
    "unit": "ampere",
    "status": "normal"
  },
  "power": {
    "value": 10.2,
    "unit": "watt"
  },
  "light": {
    "value": 75,
    "unit": "percent",
    "level": "bright"
  }
}
```

**GET /api/sensors/history?hours=24**
```json
{
  "period": {
    "start": "2024-01-14T10:30:00Z",
    "end": "2024-01-15T10:30:00Z",
    "interval": "5min"
  },
  "data": [
    {
      "timestamp": "2024-01-14T10:30:00Z",
      "temperature": 24.8,
      "humidity": 58.5,
      "current": 0.92,
      "light": 80
    }
    // ... more data points
  ]
}
```

#### ⚙️ 설정 관리

**GET /api/settings**
```json
{
  "auto_mode": {
    "target_temperature": 25.0,
    "temperature_threshold": 2.0,
    "max_speed": 80,
    "min_speed": 20,
    "adaptive_control": true
  },
  "safety": {
    "max_current": 1.5,
    "max_temperature": 55.0,
    "auto_recovery": true
  },
  "display": {
    "brightness": 80,
    "timeout": 30,
    "units": "metric"
  }
}
```

**PUT /api/settings**
```json
// Request
{
  "auto_mode": {
    "target_temperature": 24.0,
    "max_speed": 85
  }
}

// Response
{
  "success": true,
  "updated_settings": {
    "auto_mode.target_temperature": 24.0,
    "auto_mode.max_speed": 85
  }
}
```

#### 📅 스케줄 관리

**GET /api/schedule**
```json
{
  "enabled": true,
  "schedules": [
    {
      "id": 1,
      "name": "Morning Start",
      "time": "07:00",
      "days": ["mon", "tue", "wed", "thu", "fri"],
      "action": {
        "enabled": true,
        "speed": 30,
        "mode": "eco"
      },
      "active": true
    }
  ]
}
```

**POST /api/schedule**
```json
// Request
{
  "name": "Evening Cool",
  "time": "19:00", 
  "days": ["daily"],
  "action": {
    "enabled": true,
    "speed": 60,
    "auto": true
  }
}

// Response
{
  "success": true,
  "schedule_id": 2,
  "message": "Schedule created successfully"
}
```

#### 🔧 시스템 제어

**POST /api/system/restart**
```json
{
  "success": true,
  "message": "System will restart in 3 seconds"
}
```

**POST /api/system/reset**
```json
// Request
{
  "type": "settings|wifi|factory"
}

// Response
{
  "success": true,
  "message": "Settings reset completed"
}
```

### HTTP 상태 코드

| 코드 | 설명 | 예시 |
|------|------|------|
| 200 | 성공 | 데이터 조회 성공 |
| 201 | 생성 완료 | 스케줄 생성 완료 |
| 400 | 잘못된 요청 | 유효하지 않은 JSON |
| 404 | 찾을 수 없음 | 존재하지 않는 엔드포인트 |
| 500 | 서버 오류 | Arduino 통신 실패 |
| 503 | 서비스 불가 | 안전 모드 활성화 |

---

## ⚡ WebSocket 실시간 API

### 연결 정보

**WebSocket URL**: `ws://smartfan.local:81` 또는 `ws://<ESP32_IP>:81`

### 연결 프로세스

```javascript
const ws = new WebSocket('ws://smartfan.local:81');

ws.onopen = function(event) {
    console.log('WebSocket connected');
    // 연결 성공시 현재 상태 자동 수신
};

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    handleMessage(data);
};

ws.onclose = function(event) {
    console.log('WebSocket disconnected');
    // 자동 재연결 로직
    setTimeout(() => initWebSocket(), 5000);
};

ws.onerror = function(error) {
    console.error('WebSocket error:', error);
};
```

### 메시지 형식

#### 클라이언트 → 서버 (제어 명령)

**팬 제어**:
```json
{
  "type": "command",
  "action": "start|stop|speed|direction|servo",
  "value": 75,
  "timestamp": 1642235400000
}
```

**모드 제어**:
```json
{
  "type": "mode",
  "auto": true,
  "eco": false,
  "night": false,
  "timestamp": 1642235400000
}
```

**설정 변경**:
```json
{
  "type": "settings",
  "category": "auto_mode",
  "settings": {
    "target_temperature": 24.0,
    "max_speed": 85
  },
  "timestamp": 1642235400000
}
```

#### 서버 → 클라이언트 (상태 업데이트)

**실시간 센서 데이터**:
```json
{
  "type": "sensor_data",
  "timestamp": 1642235400000,
  "data": {
    "temperature": 25.5,
    "humidity": 60.2,
    "current": 0.85,
    "power": 10.2,
    "light": 75
  }
}
```

**팬 상태 업데이트**:
```json
{
  "type": "fan_status",
  "timestamp": 1642235400000,
  "status": {
    "enabled": true,
    "speed": 75,
    "direction": "forward",
    "servo_angle": 90,
    "mode": {
      "auto": false,
      "eco": true,
      "night": false,
      "safety": false
    }
  }
}
```

**시스템 이벤트**:
```json
{
  "type": "system_event",
  "timestamp": 1642235400000,
  "event": {
    "level": "warning|error|info",
    "code": "TEMP_HIGH|CURRENT_HIGH|WIFI_DISCONNECTED",
    "message": "Temperature above threshold: 58°C",
    "data": {
      "current_temp": 58.0,
      "threshold": 55.0
    }
  }
}
```

### 하트비트

**클라이언트 → 서버** (30초마다):
```json
{
  "type": "ping",
  "timestamp": 1642235400000
}
```

**서버 → 클라이언트**:
```json
{
  "type": "pong", 
  "timestamp": 1642235400000,
  "server_time": 1642235400123
}
```

---

## 📊 데이터 스키마

### 센서 데이터 타입

```typescript
interface SensorData {
  temperature: number;    // -40.0 to 80.0 (°C)
  humidity: number;       // 0.0 to 100.0 (%)
  current: number;        // 0.0 to 5.0 (A)
  light: number;          // 0 to 100 (%)
  timestamp: number;      // Unix timestamp (ms)
}
```

### 팬 상태 타입

```typescript
interface FanStatus {
  enabled: boolean;
  speed: number;          // 0 to 100 (%)
  direction: 'forward' | 'reverse';
  servo_angle: number;    // 0 to 180 (degrees)
  mode: {
    auto: boolean;
    eco: boolean;
    night: boolean;
    safety: boolean;
  };
  runtime: number;        // Total runtime (ms)
}
```

### 시스템 정보 타입

```typescript
interface SystemInfo {
  device_name: string;
  firmware_version: string;
  hardware_version: string;
  mac_address: string;
  ip_address: string;
  wifi_ssid: string;
  wifi_rssi: number;      // Signal strength (dBm)
  uptime: number;         // System uptime (ms)
  free_heap: number;      // Available memory (bytes)
  chip_id: string;
}
```

### 오류 및 경고 타입

```typescript
interface SystemEvent {
  level: 'info' | 'warning' | 'error' | 'critical';
  code: string;
  message: string;
  timestamp: number;
  data?: any;
}

// 오류 코드 정의
enum ErrorCode {
  TEMP_HIGH = 'TEMP_HIGH',
  CURRENT_HIGH = 'CURRENT_HIGH', 
  WIFI_DISCONNECTED = 'WIFI_DISCONNECTED',
  ARDUINO_DISCONNECTED = 'ARDUINO_DISCONNECTED',
  SENSOR_FAILURE = 'SENSOR_FAILURE',
  MEMORY_LOW = 'MEMORY_LOW'
}
```

---

## 🔧 SDK 및 라이브러리

### JavaScript/TypeScript SDK

```javascript
class SmartFanAPI {
  constructor(baseUrl, options = {}) {
    this.baseUrl = baseUrl;
    this.websocket = null;
    this.options = {
      timeout: 5000,
      retryAttempts: 3,
      ...options
    };
  }

  // HTTP API Methods
  async getSystemInfo() {
    return this.get('/api/system/info');
  }

  async getFanStatus() {
    return this.get('/api/fan/status');
  }

  async setFanSpeed(speed) {
    return this.post('/api/fan/control', {
      action: 'speed',
      value: speed
    });
  }

  async getSensorData() {
    return this.get('/api/sensors/current');
  }

  // WebSocket Methods
  connectWebSocket() {
    return new Promise((resolve, reject) => {
      this.websocket = new WebSocket(`ws://${this.baseUrl}:81`);
      
      this.websocket.onopen = () => {
        console.log('WebSocket connected');
        resolve(this.websocket);
      };
      
      this.websocket.onerror = (error) => {
        reject(error);
      };
      
      this.websocket.onmessage = (event) => {
        const data = JSON.parse(event.data);
        this.handleWebSocketMessage(data);
      };
    });
  }

  sendCommand(action, value = null) {
    if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
      const command = {
        type: 'command',
        action: action,
        value: value,
        timestamp: Date.now()
      };
      this.websocket.send(JSON.stringify(command));
    }
  }

  // Event Emitter
  on(event, callback) {
    if (!this.listeners) this.listeners = {};
    if (!this.listeners[event]) this.listeners[event] = [];
    this.listeners[event].push(callback);
  }

  emit(event, data) {
    if (this.listeners && this.listeners[event]) {
      this.listeners[event].forEach(callback => callback(data));
    }
  }

  // Private methods
  async get(endpoint) {
    const response = await fetch(`http://${this.baseUrl}${endpoint}`, {
      timeout: this.options.timeout
    });
    return response.json();
  }

  async post(endpoint, data) {
    const response = await fetch(`http://${this.baseUrl}${endpoint}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(data),
      timeout: this.options.timeout
    });
    return response.json();
  }

  handleWebSocketMessage(data) {
    switch(data.type) {
      case 'sensor_data':
        this.emit('sensorData', data.data);
        break;
      case 'fan_status':
        this.emit('fanStatus', data.status);
        break;
      case 'system_event':
        this.emit('systemEvent', data.event);
        break;
    }
  }
}

// 사용 예시
const fan = new SmartFanAPI('smartfan.local');

// HTTP API 사용
fan.getSystemInfo().then(info => {
  console.log('System Info:', info);
});

// WebSocket 사용
fan.connectWebSocket().then(() => {
  fan.on('sensorData', (data) => {
    console.log('Sensor update:', data);
  });
  
  fan.on('fanStatus', (status) => {
    console.log('Fan status:', status);
  });
  
  // 팬 속도 변경
  fan.sendCommand('speed', 75);
});
```

### Python SDK

```python
import asyncio
import aiohttp
import websockets
import json
from typing import Dict, Any, Callable, Optional

class SmartFanAPI:
    def __init__(self, base_url: str, timeout: int = 5):
        self.base_url = base_url
        self.timeout = timeout
        self.websocket = None
        self.listeners = {}

    # HTTP API Methods
    async def get_system_info(self) -> Dict[str, Any]:
        async with aiohttp.ClientSession() as session:
            async with session.get(
                f"http://{self.base_url}/api/system/info",
                timeout=self.timeout
            ) as response:
                return await response.json()

    async def get_fan_status(self) -> Dict[str, Any]:
        async with aiohttp.ClientSession() as session:
            async with session.get(
                f"http://{self.base_url}/api/fan/status",
                timeout=self.timeout
            ) as response:
                return await response.json()

    async def set_fan_speed(self, speed: int) -> Dict[str, Any]:
        data = {"action": "speed", "value": speed}
        async with aiohttp.ClientSession() as session:
            async with session.post(
                f"http://{self.base_url}/api/fan/control",
                json=data,
                timeout=self.timeout
            ) as response:
                return await response.json()

    # WebSocket Methods
    async def connect_websocket(self):
        uri = f"ws://{self.base_url}:81"
        self.websocket = await websockets.connect(uri)
        
        # Start listening for messages
        asyncio.create_task(self._listen_websocket())

    async def _listen_websocket(self):
        try:
            async for message in self.websocket:
                data = json.loads(message)
                await self._handle_websocket_message(data)
        except websockets.exceptions.ConnectionClosed:
            print("WebSocket connection closed")

    async def send_command(self, action: str, value: Optional[int] = None):
        if self.websocket:
            command = {
                "type": "command",
                "action": action,
                "value": value,
                "timestamp": int(asyncio.get_event_loop().time() * 1000)
            }
            await self.websocket.send(json.dumps(command))

    # Event System
    def on(self, event: str, callback: Callable):
        if event not in self.listeners:
            self.listeners[event] = []
        self.listeners[event].append(callback)

    async def _emit(self, event: str, data: Any):
        if event in self.listeners:
            for callback in self.listeners[event]:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)

    async def _handle_websocket_message(self, data: Dict[str, Any]):
        message_type = data.get('type')
        
        if message_type == 'sensor_data':
            await self._emit('sensor_data', data['data'])
        elif message_type == 'fan_status':
            await self._emit('fan_status', data['status'])
        elif message_type == 'system_event':
            await self._emit('system_event', data['event'])

# 사용 예시
async def main():
    fan = SmartFanAPI('smartfan.local')
    
    # HTTP API 사용
    system_info = await fan.get_system_info()
    print(f"System Info: {system_info}")
    
    # WebSocket 사용
    await fan.connect_websocket()
    
    def on_sensor_data(data):
        print(f"Sensor update: {data}")
    
    def on_fan_status(status):
        print(f"Fan status: {status}")
    
    fan.on('sensor_data', on_sensor_data)
    fan.on('fan_status', on_fan_status)
    
    # 팬 속도 변경
    await fan.send_command('speed', 75)
    
    # 계속 실행
    await asyncio.sleep(3600)

if __name__ == "__main__":
    asyncio.run(main())
```

---

## 🧪 테스트 및 디버깅

### 단위 테스트

**HTTP API 테스트 (Jest)**:
```javascript
const SmartFanAPI = require('./smart-fan-api');

describe('SmartFan API', () => {
  let api;
  
  beforeEach(() => {
    api = new SmartFanAPI('192.168.1.100');
  });

  test('should get system info', async () => {
    const info = await api.getSystemInfo();
    
    expect(info).toHaveProperty('device_name');
    expect(info).toHaveProperty('firmware_version');
    expect(info.device_name).toBe('SmartFan_v3.0');
  });

  test('should set fan speed', async () => {
    const result = await api.setFanSpeed(75);
    
    expect(result.success).toBe(true);
    expect(result.message).toContain('75%');
  });

  test('should handle invalid speed', async () => {
    await expect(api.setFanSpeed(150)).rejects.toThrow();
  });
});
```

**WebSocket 테스트**:
```javascript
describe('WebSocket API', () => {
  test('should connect and receive data', (done) => {
    const api = new SmartFanAPI('192.168.1.100');
    
    api.connectWebSocket().then(() => {
      api.on('sensorData', (data) => {
        expect(data).toHaveProperty('temperature');
        expect(data.temperature).toBeGreaterThan(-40);
        expect(data.temperature).toBeLessThan(80);
        done();
      });
    });
  });
});
```

### 통합 테스트

**End-to-End 테스트**:
```python
import pytest
import asyncio
from smart_fan_api import SmartFanAPI

@pytest.mark.asyncio
async def test_full_workflow():
    api = SmartFanAPI('smartfan.local')
    
    # 1. 시스템 정보 확인
    info = await api.get_system_info()
    assert info['device_name'] == 'SmartFan_v3.0'
    
    # 2. 현재 상태 확인
    status = await api.get_fan_status()
    initial_speed = status['speed']
    
    # 3. 속도 변경
    await api.set_fan_speed(50)
    await asyncio.sleep(2)  # 변경 대기
    
    # 4. 변경 확인
    new_status = await api.get_fan_status()
    assert new_status['speed'] == 50
    
    # 5. 원래 속도로 복원
    await api.set_fan_speed(initial_speed)
```

### 성능 테스트

**부하 테스트**:
```python
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor

async def load_test():
    api = SmartFanAPI('smartfan.local')
    start_time = time.time()
    
    # 100개의 동시 요청
    tasks = []
    for i in range(100):
        task = api.get_sensor_data()
        tasks.append(task)
    
    results = await asyncio.gather(*tasks, return_exceptions=True)
    end_time = time.time()
    
    success_count = len([r for r in results if not isinstance(r, Exception)])
    
    print(f"Success rate: {success_count}/100")
    print(f"Total time: {end_time - start_time:.2f}s")
    print(f"Avg response time: {(end_time - start_time) / success_count:.3f}s")

asyncio.run(load_test())
```

### 디버깅 도구

**시리얼 모니터링**:
```python
import serial
import json

def monitor_serial_communication():
    ser = serial.Serial('COM3', 115200, timeout=1)
    
    print("Monitoring Arduino-ESP32 communication...")
    
    while True:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            
            if line.startswith('DATA:'):
                try:
                    data = json.loads(line[5:])
                    print(f"Sensor Data: {data}")
                except json.JSONDecodeError:
                    print(f"Invalid JSON: {line}")
            
            elif line.startswith('CMD:'):
                print(f"Command: {line}")
            
            else:
                print(f"Other: {line}")

if __name__ == "__main__":
    monitor_serial_communication()
```

**네트워크 디버깅**:
```bash
# WiFi 신호 강도 모니터링
ping smartfan.local

# WebSocket 연결 테스트
wscat -c ws://smartfan.local:81

# HTTP API 테스트
curl -X GET http://smartfan.local/api/system/info | jq

# 실시간 로그 모니터링
curl -N http://smartfan.local/api/system/logs
```

### 오류 처리

**일반적인 오류와 해결방법**:

| 오류 코드 | 설명 | 해결방법 |
|-----------|------|----------|
| `TIMEOUT` | 요청 시간 초과 | 네트워크 연결 확인, 타임아웃 값 증가 |
| `CONNECTION_REFUSED` | 연결 거부 | ESP32 전원 및 WiFi 연결 확인 |
| `INVALID_JSON` | JSON 파싱 오류 | 요청 데이터 형식 확인 |
| `ARDUINO_DISCONNECTED` | Arduino 통신 끊김 | 시리얼 연결 및 케이블 확인 |
| `SAFETY_MODE_ACTIVE` | 안전 모드 활성 | 시스템 상태 확인 후 리셋 |

---

**📡 완벽한 API 문서로 개발 효율성 극대화! 📡**

이 API 문서를 활용하여 스마트 선풍기와 연동하는 다양한 애플리케이션을 개발할 수 있습니다. 궁금한 점이나 추가 기능이 필요하면 언제든 문의하세요! 🚀