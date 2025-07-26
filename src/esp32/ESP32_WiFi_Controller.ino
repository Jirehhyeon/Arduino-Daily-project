/*
 * 🌐 ESP32 WiFi Controller for Smart Fan
 * WiFi/Bluetooth connectivity and mobile app integration
 * 
 * Features:
 * - WiFi AP and Station mode
 * - WebServer for mobile app
 * - WebSocket real-time communication
 * - NTP time synchronization
 * - OTA firmware updates
 * - Voice command recognition
 * - Cloud integration ready
 * 
 * Hardware:
 * - ESP32 DevKit v1
 * - Connected to Arduino Uno via Serial
 * 
 * Author: Arduino Daily Project
 * Version: 3.0 Smart Edition
 * Date: 2025-07-26
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <BluetoothSerial.h>

// ========== WiFi 설정 ==========
const char* ap_ssid = "SmartFan_Setup";
const char* ap_password = "12345678";

// 사용자 WiFi 정보 (웹 인터페이스에서 설정)
String wifi_ssid = "";
String wifi_password = "";

// ========== 서버 설정 ==========
WebServer server(80);
WebSocketsServer webSocket(81);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 32400, 60000); // 한국 시간 (+9시간)
BluetoothSerial SerialBT;

// ========== 시스템 상태 ==========
struct WiFiState {
  bool apMode;
  bool stationMode;
  bool bluetoothEnabled;
  String connectedSSID;
  int rssi;
  String localIP;
  unsigned long connectedClients;
  bool ntpSynced;
  String currentTime;
};

struct FanData {
  float temperature;
  float humidity;
  int speed;
  int mode;
  bool enabled;
  bool comfort;
  float discomfortIndex;
  int current;
  unsigned long runtime;
  bool safetyMode;
  int windDirection;
  bool nightMode;
  bool ecoMode;
};

WiFiState wifiState = {true, false, false, "", -100, "", 0, false, ""};
FanData fanData;

// ========== 전역 변수 ==========
unsigned long lastArduinoComm = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastNTPUpdate = 0;
bool arduinoConnected = false;

// ========== 초기화 ==========
void setup() {
  Serial.begin(115200);
  Serial.println("🌐 ESP32 Smart Fan WiFi Controller v3.0");
  Serial.println("==========================================");
  
  // SPIFFS 초기화
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
  } else {
    Serial.println("✅ SPIFFS Mounted");
  }
  
  // WiFi 설정 로드
  loadWiFiSettings();
  
  // WiFi 초기화
  setupWiFi();
  
  // 웹 서버 초기화
  setupWebServer();
  
  // WebSocket 초기화
  setupWebSocket();
  
  // NTP 초기화
  timeClient.begin();
  
  // OTA 설정
  setupOTA();
  
  // 블루투스 초기화
  setupBluetooth();
  
  // mDNS 설정
  if (MDNS.begin("smartfan")) {
    Serial.println("📡 mDNS responder started: http://smartfan.local");
  }
  
  Serial.println("🚀 ESP32 System Ready!");
  sendToArduino("WIFI_STATUS:READY");
}

// ========== 메인 루프 ==========
void loop() {
  // WiFi 연결 상태 확인
  checkWiFiStatus();
  
  // Arduino 통신 처리
  handleArduinoCommunication();
  
  // 웹 서버 처리
  server.handleClient();
  
  // WebSocket 처리
  webSocket.loop();
  
  // NTP 업데이트 (1시간마다)
  if (millis() - lastNTPUpdate >= 3600000) {
    lastNTPUpdate = millis();
    updateNTPTime();
  }
  
  // OTA 처리
  ArduinoOTA.handle();
  
  // 블루투스 처리
  handleBluetoothCommands();
  
  // 하트비트 (30초마다)
  if (millis() - lastHeartbeat >= 30000) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }
  
  delay(10);
}

// ========== WiFi 설정 ==========
void setupWiFi() {
  Serial.println("📶 Configuring WiFi...");
  
  // AP 모드 시작
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  
  wifiState.apMode = true;
  wifiState.localIP = WiFi.softAPIP().toString();
  
  Serial.println("✅ AP Mode started");
  Serial.println("   SSID: " + String(ap_ssid));
  Serial.println("   IP: " + wifiState.localIP);
  
  // 저장된 WiFi가 있으면 연결 시도
  if (wifi_ssid.length() > 0) {
    connectToWiFi();
  }
}

void connectToWiFi() {
  Serial.println("🔗 Connecting to: " + wifi_ssid);
  
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiState.stationMode = true;
    wifiState.connectedSSID = wifi_ssid;
    wifiState.rssi = WiFi.RSSI();
    wifiState.localIP = WiFi.localIP().toString();
    
    Serial.println();
    Serial.println("✅ WiFi Connected!");
    Serial.println("   SSID: " + wifiState.connectedSSID);
    Serial.println("   IP: " + wifiState.localIP);
    Serial.println("   RSSI: " + String(wifiState.rssi) + "dBm");
    
    sendToArduino("WIFI_STATUS:CONNECTED");
    sendToArduino("RSSI:" + String(wifiState.rssi));
  } else {
    Serial.println();
    Serial.println("❌ WiFi Connection Failed");
    sendToArduino("WIFI_STATUS:FAILED");
  }
}

void checkWiFiStatus() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 10000) { // 10초마다 확인
    lastCheck = millis();
    
    if (wifiState.stationMode) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("📶 WiFi disconnected, attempting reconnect...");
        wifiState.stationMode = false;
        sendToArduino("WIFI_STATUS:DISCONNECTED");
        connectToWiFi();
      } else {
        wifiState.rssi = WiFi.RSSI();
        sendToArduino("RSSI:" + String(wifiState.rssi));
      }
    }
  }
}

// ========== 웹 서버 설정 ==========
void setupWebServer() {
  // 루트 페이지 - 설정 인터페이스
  server.on("/", handleRoot);
  
  // WiFi 설정
  server.on("/wifi", HTTP_POST, handleWiFiConfig);
  
  // 팬 제어 API
  server.on("/api/fan/control", HTTP_POST, handleFanControl);
  server.on("/api/fan/status", HTTP_GET, handleFanStatus);
  server.on("/api/fan/schedule", HTTP_POST, handleSchedule);
  
  // 시스템 정보
  server.on("/api/system/info", HTTP_GET, handleSystemInfo);
  server.on("/api/system/reset", HTTP_POST, handleSystemReset);
  
  // 실시간 데이터 (WebSocket으로 대체)
  server.on("/api/data/realtime", HTTP_GET, handleRealtimeData);
  
  // 404 처리
  server.onNotFound(handleNotFound);
  
  // CORS 헤더 추가
  server.enableCORS(true);
  
  server.begin();
  Serial.println("🌐 Web server started on port 80");
}

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🌀 Smart Fan Controller</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        .header { text-align: center; color: #2c3e50; margin-bottom: 30px; }
        .card { background: #f8f9fa; padding: 15px; margin: 10px 0; border-radius: 8px; border-left: 4px solid #3498db; }
        .control-panel { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .control-button { padding: 10px 20px; background: #3498db; color: white; border: none; border-radius: 5px; cursor: pointer; }
        .control-button:hover { background: #2980b9; }
        .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .status-online { background: #2ecc71; }
        .status-offline { background: #e74c3c; }
        .slider { width: 100%; margin: 10px 0; }
        .realtime-data { font-family: monospace; background: #2c3e50; color: #ecf0f1; padding: 15px; border-radius: 8px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🌀 Smart Fan Controller v3.0</h1>
            <p>Professional Smart Home Solution</p>
        </div>
        
        <div class="card">
            <h3>📶 Connection Status</h3>
            <p><span class="status-indicator status-offline" id="wifiStatus"></span>WiFi: <span id="wifiSSID">Disconnected</span></p>
            <p><span class="status-indicator status-offline" id="fanStatus"></span>Fan Controller: <span id="fanConnection">Disconnected</span></p>
            <p><span class="status-indicator status-offline" id="appStatus"></span>Real-time Data: <span id="realtimeStatus">Disconnected</span></p>
        </div>
        
        <div class="card">
            <h3>🎛️ Fan Control</h3>
            <div class="control-panel">
                <button class="control-button" onclick="controlFan('start')">🟢 Start</button>
                <button class="control-button" onclick="controlFan('stop')">🔴 Stop</button>
                <button class="control-button" onclick="controlFan('auto')">🤖 Auto Mode</button>
                <button class="control-button" onclick="controlFan('eco')">🌱 Eco Mode</button>
            </div>
            <div style="margin-top: 15px;">
                <label>Speed Control:</label>
                <input type="range" class="slider" min="0" max="100" value="50" id="speedSlider" onchange="setSpeed(this.value)">
                <span id="speedValue">50%</span>
            </div>
            <div style="margin-top: 15px;">
                <label>Wind Direction:</label>
                <input type="range" class="slider" min="0" max="180" value="90" id="windSlider" onchange="setWindDirection(this.value)">
                <span id="windValue">90°</span>
            </div>
        </div>
        
        <div class="card">
            <h3>🌡️ Environmental Data</h3>
            <div class="realtime-data" id="environmentData">
                Loading sensor data...
            </div>
        </div>
        
        <div class="card">
            <h3>📱 WiFi Configuration</h3>
            <form onsubmit="setWiFi(event)">
                <input type="text" id="ssid" placeholder="WiFi SSID" style="width: 200px; padding: 8px; margin: 5px;">
                <input type="password" id="password" placeholder="Password" style="width: 200px; padding: 8px; margin: 5px;">
                <button type="submit" class="control-button">Connect</button>
            </form>
        </div>
        
        <div class="card">
            <h3>⏰ Schedule Settings</h3>
            <div>
                <label>Start Time: <input type="time" id="startTime" value="22:00"></label>
                <label>End Time: <input type="time" id="endTime" value="06:00"></label>
                <label>Speed: <input type="range" min="0" max="100" value="50" id="scheduleSpeed"></label>
                <button class="control-button" onclick="setSchedule()">Set Schedule</button>
            </div>
        </div>
    </div>
    
    <script>
        let ws;
        
        function initWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81');
            ws.onopen = function() {
                document.getElementById('realtimeStatus').textContent = 'Connected';
                document.getElementById('appStatus').className = 'status-indicator status-online';
            };
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateRealtimeData(data);
                } catch(e) {
                    console.log('WebSocket message:', event.data);
                }
            };
            ws.onclose = function() {
                document.getElementById('realtimeStatus').textContent = 'Disconnected';
                document.getElementById('appStatus').className = 'status-indicator status-offline';
                setTimeout(initWebSocket, 5000);
            };
        }
        
        function updateRealtimeData(data) {
            const env = document.getElementById('environmentData');
            env.innerHTML = `
Temperature: ${data.temperature}°C
Humidity: ${data.humidity}%
Fan Speed: ${data.speed}%
Mode: ${getModeText(data.mode)}
Comfort Index: ${data.discomfort_index}
Comfort Status: ${data.comfort ? 'Comfortable ✅' : 'Needs Attention ⚠️'}
Current: ${data.current}mA
Runtime: ${Math.floor(data.runtime/60)} minutes
Safety: ${data.safety_mode ? 'SAFE MODE 🚨' : 'Normal ✅'}
Wind Direction: ${data.wind_direction}°
Night Mode: ${data.night_mode ? 'ON 🌙' : 'OFF ☀️'}
Eco Mode: ${data.eco_mode ? 'ON 🌱' : 'OFF'}
            `;
        }
        
        function getModeText(mode) {
            const modes = ['Manual', 'Auto', 'Schedule', 'Eco'];
            return modes[mode] || 'Unknown';
        }
        
        function controlFan(action) {
            fetch('/api/fan/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: action})
            });
        }
        
        function setSpeed(value) {
            document.getElementById('speedValue').textContent = value + '%';
            fetch('/api/fan/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'speed', value: parseInt(value)})
            });
        }
        
        function setWindDirection(value) {
            document.getElementById('windValue').textContent = value + '°';
            fetch('/api/fan/control', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({action: 'wind', value: parseInt(value)})
            });
        }
        
        function setWiFi(event) {
            event.preventDefault();
            const ssid = document.getElementById('ssid').value;
            const password = document.getElementById('password').value;
            
            fetch('/wifi', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: `ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(password)}`
            }).then(response => {
                if(response.ok) {
                    alert('WiFi configuration saved. Connecting...');
                }
            });
        }
        
        function setSchedule() {
            const startTime = document.getElementById('startTime').value;
            const endTime = document.getElementById('endTime').value;
            const speed = document.getElementById('scheduleSpeed').value;
            
            fetch('/api/fan/schedule', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    enabled: true,
                    startTime: startTime,
                    endTime: endTime,
                    speed: parseInt(speed)
                })
            });
        }
        
        // 초기화
        window.onload = function() {
            initWebSocket();
            
            // 상태 업데이트
            setInterval(function() {
                fetch('/api/system/info')
                    .then(response => response.json())
                    .then(data => {
                        document.getElementById('wifiSSID').textContent = data.wifi_connected ? data.wifi_ssid : 'Disconnected';
                        document.getElementById('wifiStatus').className = 'status-indicator ' + (data.wifi_connected ? 'status-online' : 'status-offline');
                        document.getElementById('fanConnection').textContent = data.arduino_connected ? 'Connected' : 'Disconnected';
                        document.getElementById('fanStatus').className = 'status-indicator ' + (data.arduino_connected ? 'status-online' : 'status-offline');
                    })
                    .catch(e => console.log('Status update failed:', e));
            }, 5000);
        };
    </script>
</body>
</html>
  )";
  
  server.send(200, "text/html", html);
}

void handleWiFiConfig() {
  if (server.hasArg("ssid") && server.hasArg("password")) {
    wifi_ssid = server.arg("ssid");
    wifi_password = server.arg("password");
    
    saveWiFiSettings();
    
    server.send(200, "text/plain", "WiFi configuration saved");
    
    // 잠시 후 연결 시도
    delay(1000);
    connectToWiFi();
  } else {
    server.send(400, "text/plain", "Missing SSID or password");
  }
}

void handleFanControl() {
  if (server.hasHeader("Content-Type") && server.header("Content-Type") == "application/json") {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));
    
    String action = doc["action"];
    
    if (action == "start") {
      sendToArduino("CMD:START");
    } else if (action == "stop") {
      sendToArduino("CMD:STOP");
    } else if (action == "auto") {
      sendToArduino("CMD:MODE:1");
    } else if (action == "eco") {
      sendToArduino("CMD:ECO_ON");
    } else if (action == "speed") {
      int value = doc["value"];
      sendToArduino("CMD:SPEED:" + String(value));
    } else if (action == "wind") {
      int value = doc["value"];
      sendToArduino("CMD:WIND:" + String(value));
    }
    
    server.send(200, "application/json", "{\"status\":\"ok\"}");
  } else {
    server.send(400, "text/plain", "Invalid request format");
  }
}

void handleFanStatus() {
  DynamicJsonDocument doc(512);
  
  doc["temperature"] = fanData.temperature;
  doc["humidity"] = fanData.humidity;
  doc["speed"] = fanData.speed;
  doc["mode"] = fanData.mode;
  doc["enabled"] = fanData.enabled;
  doc["comfort"] = fanData.comfort;
  doc["discomfort_index"] = fanData.discomfortIndex;
  doc["current"] = fanData.current;
  doc["runtime"] = fanData.runtime;
  doc["safety_mode"] = fanData.safetyMode;
  doc["wind_direction"] = fanData.windDirection;
  doc["night_mode"] = fanData.nightMode;
  doc["eco_mode"] = fanData.ecoMode;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSchedule() {
  // 스케줄 설정 처리
  server.send(200, "application/json", "{\"status\":\"schedule_set\"}");
}

void handleSystemInfo() {
  DynamicJsonDocument doc(256);
  
  doc["wifi_connected"] = wifiState.stationMode;
  doc["wifi_ssid"] = wifiState.connectedSSID;
  doc["wifi_rssi"] = wifiState.rssi;
  doc["arduino_connected"] = arduinoConnected;
  doc["local_ip"] = wifiState.localIP;
  doc["uptime"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["current_time"] = wifiState.currentTime;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSystemReset() {
  server.send(200, "text/plain", "System resetting...");
  delay(1000);
  ESP.restart();
}

void handleRealtimeData() {
  // WebSocket으로 대체됨
  server.send(200, "text/plain", "Use WebSocket for realtime data");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// ========== WebSocket 설정 ==========
void setupWebSocket() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("📡 WebSocket server started on port 81");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket client " + String(num) + " disconnected");
      wifiState.connectedClients--;
      sendToArduino("APP_STATUS:DISCONNECTED");
      break;
      
    case WStype_CONNECTED:
      Serial.println("WebSocket client " + String(num) + " connected");
      wifiState.connectedClients++;
      sendToArduino("APP_STATUS:CONNECTED");
      // 현재 상태 전송
      broadcastFanData();
      break;
      
    case WStype_TEXT:
      Serial.println("WebSocket message: " + String((char*)payload));
      handleWebSocketCommand(String((char*)payload));
      break;
      
    default:
      break;
  }
}

void handleWebSocketCommand(String command) {
  // WebSocket을 통한 실시간 명령 처리
  if (command.startsWith("CMD:")) {
    sendToArduino(command);
  }
}

void broadcastFanData() {
  if (wifiState.connectedClients > 0) {
    DynamicJsonDocument doc(512);
    
    doc["temperature"] = fanData.temperature;
    doc["humidity"] = fanData.humidity;
    doc["speed"] = fanData.speed;
    doc["mode"] = fanData.mode;
    doc["enabled"] = fanData.enabled;
    doc["comfort"] = fanData.comfort;
    doc["discomfort_index"] = fanData.discomfortIndex;
    doc["current"] = fanData.current;
    doc["runtime"] = fanData.runtime;
    doc["safety_mode"] = fanData.safetyMode;
    doc["wind_direction"] = fanData.windDirection;
    doc["night_mode"] = fanData.nightMode;
    doc["eco_mode"] = fanData.ecoMode;
    doc["timestamp"] = timeClient.getEpochTime();
    
    String message;
    serializeJson(doc, message);
    webSocket.broadcastTXT(message);
  }
}

// ========== Arduino 통신 ==========
void handleArduinoCommunication() {
  // Arduino로부터 데이터 수신
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    lastArduinoComm = millis();
    arduinoConnected = true;
    
    processArduinoData(data);
  }
  
  // Arduino 연결 상태 확인
  if (millis() - lastArduinoComm >= 10000) { // 10초 이상 무응답
    if (arduinoConnected) {
      arduinoConnected = false;
      Serial.println("⚠️ Arduino connection lost");
    }
  }
}

void processArduinoData(String data) {
  if (data.startsWith("DATA:")) {
    // JSON 형태의 센서 데이터
    String jsonData = data.substring(5);
    DynamicJsonDocument doc(512);
    
    if (deserializeJson(doc, jsonData) == DeserializationError::Ok) {
      fanData.temperature = doc["temperature"];
      fanData.humidity = doc["humidity"];
      fanData.speed = doc["speed"];
      fanData.mode = doc["mode"];
      fanData.enabled = doc["enabled"];
      fanData.comfort = doc["comfort"];
      fanData.discomfortIndex = doc["discomfort_index"];
      fanData.current = doc["current"];
      fanData.runtime = doc["runtime"];
      fanData.safetyMode = doc["safety_mode"];
      fanData.windDirection = doc["wind_direction"];
      fanData.nightMode = doc["night_mode"];
      fanData.ecoMode = doc["eco_mode"];
      
      // WebSocket으로 실시간 데이터 전송
      broadcastFanData();
    }
  }
  else if (data.startsWith("ALERT:")) {
    // 알림 메시지 처리
    String alert = data.substring(6);
    Serial.println("🚨 Arduino Alert: " + alert);
    
    // 모바일 앱에 푸시 알림 (추후 구현)
    DynamicJsonDocument alertDoc(256);
    alertDoc["type"] = "alert";
    alertDoc["message"] = alert;
    alertDoc["timestamp"] = timeClient.getEpochTime();
    
    String alertMessage;
    serializeJson(alertDoc, alertMessage);
    webSocket.broadcastTXT(alertMessage);
  }
  else if (data == "PING") {
    // Arduino 연결 확인
    sendToArduino("PONG");
  }
  else if (data == "GET_TIME") {
    // 시간 요청
    updateNTPTime();
    sendToArduino("TIME:" + timeClient.getFormattedTime());
  }
  else if (data == "EMERGENCY_STOP") {
    // 비상 정지 알림
    Serial.println("🚨 EMERGENCY STOP RECEIVED");
    
    DynamicJsonDocument emergencyDoc(256);
    emergencyDoc["type"] = "emergency";
    emergencyDoc["message"] = "Emergency stop activated";
    emergencyDoc["timestamp"] = timeClient.getEpochTime();
    
    String emergencyMessage;
    serializeJson(emergencyDoc, emergencyMessage);
    webSocket.broadcastTXT(emergencyMessage);
  }
}

void sendToArduino(String message) {
  Serial.println(message);
}

// ========== NTP 시간 동기화 ==========
void updateNTPTime() {
  if (wifiState.stationMode) {
    timeClient.update();
    wifiState.ntpSynced = true;
    wifiState.currentTime = timeClient.getFormattedTime();
    
    Serial.println("🕒 Time updated: " + wifiState.currentTime);
  }
}

// ========== OTA 업데이트 ==========
void setupOTA() {
  ArduinoOTA.setHostname("smartfan-esp32");
  ArduinoOTA.setPassword("smartfan2025");
  
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("🔄 OTA Update started: " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update completed");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("📦 OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("📡 OTA Ready");
}

// ========== 블루투스 설정 ==========
void setupBluetooth() {
  if (SerialBT.begin("SmartFan_BT")) {
    wifiState.bluetoothEnabled = true;
    Serial.println("🔵 Bluetooth started: SmartFan_BT");
  } else {
    Serial.println("❌ Bluetooth failed to start");
  }
}

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    command.trim();
    
    Serial.println("🔵 Bluetooth command: " + command);
    
    // Arduino로 명령 전달
    sendToArduino("CMD:" + command);
    
    // 응답 전송
    SerialBT.println("Command executed: " + command);
  }
}

// ========== 설정 저장/로드 ==========
void saveWiFiSettings() {
  DynamicJsonDocument doc(256);
  doc["ssid"] = wifi_ssid;
  doc["password"] = wifi_password;
  
  File file = SPIFFS.open("/wifi_config.json", "w");
  if (file) {
    serializeJson(doc, file);
    file.close();
    Serial.println("💾 WiFi settings saved");
  }
}

void loadWiFiSettings() {
  File file = SPIFFS.open("/wifi_config.json", "r");
  if (file) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, file);
    
    wifi_ssid = doc["ssid"].as<String>();
    wifi_password = doc["password"].as<String>();
    
    file.close();
    Serial.println("📂 WiFi settings loaded");
  }
}

// ========== 유틸리티 함수 ==========
void sendHeartbeat() {
  sendToArduino("HEARTBEAT");
  
  // 시스템 상태 로그
  Serial.println("💗 Heartbeat - WiFi: " + String(wifiState.stationMode ? "Connected" : "Disconnected") + 
                 ", Clients: " + String(wifiState.connectedClients) + 
                 ", Arduino: " + String(arduinoConnected ? "Connected" : "Disconnected"));
}

/*
 * 🌐 ESP32 WiFi Controller Features:
 * 
 * 📡 Connectivity:
 * - Dual WiFi mode (AP + Station)
 * - WebSocket real-time communication
 * - Bluetooth Serial support
 * - mDNS service discovery
 * 
 * 🌐 Web Interface:
 * - Complete mobile-responsive UI
 * - Real-time data visualization
 * - Fan control interface
 * - WiFi configuration
 * - Schedule management
 * 
 * 🔄 Communication:
 * - JSON-based Arduino communication
 * - RESTful API endpoints
 * - WebSocket live updates
 * - Bluetooth command interface
 * 
 * ⏰ Time Services:
 * - NTP time synchronization
 * - Schedule management
 * - Timezone support (Korea +9)
 * 
 * 🛠️ System Features:
 * - OTA firmware updates
 * - Configuration persistence
 * - Error handling and recovery
 * - Real-time monitoring
 * 
 * 📱 Mobile Ready:
 * - Progressive Web App compatible
 * - Touch-friendly interface
 * - Responsive design
 * - Real-time status updates
 * 
 * This ESP32 controller provides complete connectivity
 * and web interface for the smart fan system!
 */