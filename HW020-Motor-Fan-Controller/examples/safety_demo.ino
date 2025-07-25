/*
 * Safety Features Demonstration
 * Demonstrates safety monitoring and protection systems
 * 
 * This example shows:
 * - Current monitoring and protection
 * - Temperature estimation and protection
 * - Emergency stop procedures
 * - Safety mode operation
 * 
 * Hardware:
 * - Arduino Uno R3
 * - HW-020 Motor Driver Shield (L298P)
 * - 12V DC Motor
 * - 12V Power Supply
 * - ACS712 Current Sensor (optional, on A1)
 * - LED indicators on pins 4, 5, 6
 */

// Pin definitions
#define MOTOR_PWM       3     // Motor PWM (ENA)
#define MOTOR_DIR       12    // Motor Direction (IN1)
#define CURRENT_SENSE   A1    // Current sensor input
#define EMERGENCY_BTN   2     // Emergency stop button

#define LED_POWER       4     // Power status LED (Green)
#define LED_WARNING     5     // Warning LED (Yellow)  
#define LED_EMERGENCY   6     // Emergency LED (Red)

// Safety limits
#define MAX_CURRENT     1200  // Maximum current (mA)
#define WARNING_CURRENT 800   // Warning threshold (mA)
#define MAX_TEMPERATURE 55    // Maximum temperature (°C)
#define WARNING_TEMP    45    // Warning temperature (°C)

// Safety system state
struct SafetyState {
  bool safety_mode;           // Safety mode active
  bool emergency_stop;        // Emergency stop active
  int current_ma;            // Measured current (mA)
  int temperature;           // Estimated temperature (°C)
  unsigned long runtime;     // Runtime in seconds
  bool over_current;         // Over-current condition
  bool over_temperature;     // Over-temperature condition
};

// Global variables
SafetyState safety = {false, false, 0, 25, 0, false, false};
int motor_speed = 0;
bool motor_enabled = false;
unsigned long safety_check_time = 0;
unsigned long runtime_start = 0;
volatile bool emergency_pressed = false;

void setup() {
  Serial.begin(115200);
  Serial.println("=== SAFETY FEATURES DEMONSTRATION ===");
  Serial.println("Motor safety monitoring and protection");
  Serial.println("====================================");
  
  // Initialize pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(LED_EMERGENCY, OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  
  // Setup emergency interrupt
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), emergencyStop, FALLING);
  
  // Initial LED test
  ledTest();
  
  // Initial safety check
  performSafetyCheck();
  
  Serial.println("Commands:");
  Serial.println("'START' - Start motor (if safe)");
  Serial.println("'STOP' - Stop motor");
  Serial.println("'SPEED:<0-100>' - Set speed percentage");
  Serial.println("'RESET' - Reset safety system");
  Serial.println("'STATUS' - Show safety status");
  Serial.println("'TEST' - Run safety tests");
  Serial.println();
}

void loop() {
  // Safety monitoring (every 500ms)
  if (millis() - safety_check_time >= 500) {
    safety_check_time = millis();
    monitorSafety();
  }
  
  // Handle serial commands
  handleCommands();
  
  // Update motor control
  updateMotorControl();
  
  // Update LED indicators
  updateLEDs();
  
  // Handle emergency stop
  if (emergency_pressed) {
    emergency_pressed = false;
    handleEmergencyStop();
  }
  
  delay(10);
}

void monitorSafety() {
  // Measure current (simulated if no sensor)
  int current_reading = analogRead(CURRENT_SENSE);
  if (current_reading > 10) {  // If sensor connected
    // ACS712-05B: 185mV/A, 2.5V center
    float voltage = (current_reading * 5.0) / 1024.0;
    safety.current_ma = abs((voltage - 2.5) / 0.185) * 1000;
  } else {
    // Simulate current based on speed
    safety.current_ma = map(motor_speed, 0, 255, 0, 800);
  }
  
  // Estimate temperature based on runtime and load
  if (motor_enabled && motor_speed > 0) {
    if (runtime_start == 0) runtime_start = millis();
    safety.runtime = (millis() - runtime_start) / 1000;
    
    float load_factor = motor_speed / 255.0;
    safety.temperature = 25 + (load_factor * 15) + (safety.runtime * 0.05);
  } else {
    if (runtime_start > 0) {
      runtime_start = 0;
    }
    safety.temperature = max(25, safety.temperature - 1);  // Cooling
  }
  
  // Check safety conditions
  safety.over_current = (safety.current_ma > MAX_CURRENT);
  safety.over_temperature = (safety.temperature > MAX_TEMPERATURE);
  
  // Activate safety mode if needed
  bool should_be_safe = safety.over_current || safety.over_temperature || safety.emergency_stop;
  
  if (should_be_safe && !safety.safety_mode) {
    activateSafetyMode();
  } else if (!should_be_safe && safety.safety_mode) {
    // Auto-recovery only if manually reset
    // safety.safety_mode = false;
    // Serial.println("Safety conditions cleared - manual reset required");
  }
}

void activateSafetyMode() {
  safety.safety_mode = true;
  motor_enabled = false;
  motor_speed = 0;
  
  Serial.println("*** SAFETY MODE ACTIVATED ***");
  
  if (safety.over_current) {
    Serial.print("OVER-CURRENT: ");
    Serial.print(safety.current_ma);
    Serial.println("mA");
  }
  
  if (safety.over_temperature) {
    Serial.print("OVER-TEMPERATURE: ");
    Serial.print(safety.temperature);
    Serial.println("°C");
  }
  
  if (safety.emergency_stop) {
    Serial.println("EMERGENCY STOP ACTIVATED");
  }
  
  Serial.println("Motor stopped for safety. Use 'RESET' to clear.");
}

void handleCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command == "START") {
      if (!safety.safety_mode) {
        motor_enabled = true;
        Serial.println("Motor started");
      } else {
        Serial.println("Cannot start - Safety mode active");
      }
      
    } else if (command == "STOP") {
      motor_enabled = false;
      motor_speed = 0;
      Serial.println("Motor stopped");
      
    } else if (command.startsWith("SPEED:")) {
      if (!safety.safety_mode) {
        int speed_percent = command.substring(6).toInt();
        speed_percent = constrain(speed_percent, 0, 100);
        motor_speed = map(speed_percent, 0, 100, 0, 255);
        Serial.print("Speed set to: ");
        Serial.print(speed_percent);
        Serial.println("%");
      } else {
        Serial.println("Cannot change speed - Safety mode active");
      }
      
    } else if (command == "RESET") {
      resetSafetySystem();
      
    } else if (command == "STATUS") {
      showSafetyStatus();
      
    } else if (command == "TEST") {
      runSafetyTests();
      
    } else {
      Serial.println("Unknown command");
    }
  }
}

void updateMotorControl() {
  if (motor_enabled && !safety.safety_mode) {
    analogWrite(MOTOR_PWM, motor_speed);
    digitalWrite(MOTOR_DIR, HIGH);
  } else {
    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_DIR, LOW);
  }
}

void updateLEDs() {
  // Power LED (green) - motor running
  digitalWrite(LED_POWER, motor_enabled && motor_speed > 0 && !safety.safety_mode);
  
  // Warning LED (yellow) - warning conditions
  bool warning = (safety.current_ma > WARNING_CURRENT) || 
                 (safety.temperature > WARNING_TEMP) ||
                 safety.safety_mode;
  
  if (warning) {
    // Slow blink for warnings
    static unsigned long warning_blink = 0;
    static bool warning_state = false;
    if (millis() - warning_blink >= 500) {
      warning_blink = millis();
      warning_state = !warning_state;
      digitalWrite(LED_WARNING, warning_state);
    }
  } else {
    digitalWrite(LED_WARNING, LOW);
  }
  
  // Emergency LED (red) - emergency/safety conditions
  if (safety.safety_mode || safety.emergency_stop) {
    // Fast blink for emergencies
    static unsigned long emergency_blink = 0;
    static bool emergency_state = false;
    if (millis() - emergency_blink >= 100) {
      emergency_blink = millis();
      emergency_state = !emergency_state;
      digitalWrite(LED_EMERGENCY, emergency_state);
    }
  } else {
    digitalWrite(LED_EMERGENCY, LOW);
  }
}

void emergencyStop() {
  emergency_pressed = true;
}

void handleEmergencyStop() {
  safety.emergency_stop = true;
  activateSafetyMode();
  Serial.println("*** EMERGENCY STOP BUTTON PRESSED ***");
}

void resetSafetySystem() {
  safety.safety_mode = false;
  safety.emergency_stop = false;
  safety.over_current = false;
  safety.over_temperature = false;
  safety.runtime = 0;
  runtime_start = 0;
  
  Serial.println("Safety system reset");
  Serial.println("Motor ready (use START to begin)");
}

void showSafetyStatus() {
  Serial.println("\n=== SAFETY STATUS ===");
  Serial.print("Safety Mode: ");
  Serial.println(safety.safety_mode ? "ACTIVE" : "Normal");
  Serial.print("Emergency Stop: ");
  Serial.println(safety.emergency_stop ? "ACTIVE" : "Normal");
  Serial.print("Current: ");
  Serial.print(safety.current_ma);
  Serial.print("mA (Limit: ");
  Serial.print(MAX_CURRENT);
  Serial.println("mA)");
  Serial.print("Temperature: ");
  Serial.print(safety.temperature);
  Serial.print("°C (Limit: ");
  Serial.print(MAX_TEMPERATURE);
  Serial.println("°C)");
  Serial.print("Runtime: ");
  Serial.print(safety.runtime);
  Serial.println(" seconds");
  Serial.print("Motor Speed: ");
  Serial.print(map(motor_speed, 0, 255, 0, 100));
  Serial.println("%");
  Serial.println("====================\n");
}

void performSafetyCheck() {
  Serial.println("Performing initial safety check...");
  
  // Check initial conditions
  int initial_current = analogRead(CURRENT_SENSE);
  if (initial_current > 100) {
    Serial.println("WARNING: High initial current reading");
  }
  
  Serial.print("Initial current sensor reading: ");
  Serial.println(initial_current);
  Serial.print("Initial temperature: ");
  Serial.print(safety.temperature);
  Serial.println("°C");
  
  Serial.println("Safety check complete - System ready");
}

void ledTest() {
  Serial.println("LED Test sequence...");
  
  // Test each LED
  Serial.println("  Power LED (Green)");
  digitalWrite(LED_POWER, HIGH);
  delay(500);
  digitalWrite(LED_POWER, LOW);
  
  Serial.println("  Warning LED (Yellow)");
  digitalWrite(LED_WARNING, HIGH);
  delay(500);
  digitalWrite(LED_WARNING, LOW);
  
  Serial.println("  Emergency LED (Red)");
  digitalWrite(LED_EMERGENCY, HIGH);
  delay(500);
  digitalWrite(LED_EMERGENCY, LOW);
  
  Serial.println("LED test complete");
}

void runSafetyTests() {
  Serial.println("Running safety system tests...");
  
  // Test 1: Over-current simulation
  Serial.println("Test 1: Over-current protection");
  safety.current_ma = MAX_CURRENT + 100;  // Simulate over-current
  monitorSafety();
  delay(1000);
  
  // Test 2: Over-temperature simulation  
  Serial.println("Test 2: Over-temperature protection");
  safety.temperature = MAX_TEMPERATURE + 5;  // Simulate over-temp
  monitorSafety();
  delay(1000);
  
  // Test 3: Emergency stop
  Serial.println("Test 3: Emergency stop test");
  handleEmergencyStop();
  delay(1000);
  
  Serial.println("Safety tests complete - System in safety mode");
  Serial.println("Use 'RESET' to return to normal operation");
}

/*
 * Safety Features Demonstrated:
 * 
 * 1. Current Monitoring: Continuous current measurement and protection
 * 2. Temperature Protection: Thermal monitoring and over-temp shutdown
 * 3. Emergency Stop: Immediate motor shutdown via interrupt
 * 4. Safety Mode: Comprehensive protection state with manual reset
 * 5. Visual Indicators: LED status for quick safety assessment
 * 6. Automatic Recovery: System ready after manual safety reset
 * 
 * LED Indicators:
 * - Green (Power): Motor running normally
 * - Yellow (Warning): Warning conditions (slow blink)
 * - Red (Emergency): Safety mode active (fast blink)
 * 
 * This demonstration shows how to implement comprehensive safety
 * systems for motor control applications.
 */