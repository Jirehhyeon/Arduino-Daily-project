/*
 * Motor Speed Test Example
 * Tests different speed levels and ramping
 * 
 * This example demonstrates:
 * - Automated speed testing
 * - Smooth speed transitions
 * - Speed measurement and validation
 * 
 * Hardware:
 * - Arduino Uno R3  
 * - HW-020 Motor Driver Shield (L298P)
 * - 12V DC Motor
 * - 12V Power Supply
 */

// Pin definitions
#define MOTOR_PWM       3     // Motor PWM (ENA)
#define MOTOR_DIR       12    // Motor Direction (IN1) 
#define LED_STATUS      13    // Built-in LED for status

// Test parameters
#define MIN_SPEED       50    // Minimum test speed
#define MAX_SPEED       255   // Maximum test speed
#define SPEED_STEP      25    // Speed increment
#define TEST_DURATION   3000  // Duration at each speed (ms)
#define RAMP_DELAY      20    // Delay between ramp steps (ms)

// Variables
int current_speed = 0;
int target_speed = 0;
bool test_running = false;
bool direction = true;

void setup() {
  Serial.begin(115200);
  Serial.println("=== MOTOR SPEED TEST ===");
  Serial.println("Automated speed testing and validation");
  Serial.println("=====================================");
  
  // Initialize pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  // Initial state
  analogWrite(MOTOR_PWM, 0);
  digitalWrite(MOTOR_DIR, HIGH);
  digitalWrite(LED_STATUS, LOW);
  
  Serial.println("Commands:");
  Serial.println("'START' - Begin speed test");
  Serial.println("'STOP'  - Stop motor");  
  Serial.println("'RAMP'  - Test smooth ramping");
  Serial.println("'DIRECTION' - Test direction change");
  Serial.println();
  
  delay(2000);
}

void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // Update motor control
  updateMotorControl();
  
  // Status LED
  digitalWrite(LED_STATUS, current_speed > 0);
  
  delay(10);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toUpperCase();
    
    if (command == "START") {
      startSpeedTest();
    } else if (command == "STOP") {
      stopMotor();
    } else if (command == "RAMP") {
      testRamping();
    } else if (command == "DIRECTION") {
      testDirectionChange();
    } else {
      Serial.println("Unknown command");
    }
  }
}

void startSpeedTest() {
  Serial.println("Starting automated speed test...");
  test_running = true;
  
  for (int test_speed = MIN_SPEED; test_speed <= MAX_SPEED; test_speed += SPEED_STEP) {
    if (!test_running) break;
    
    Serial.print("Testing speed: ");
    Serial.print(map(test_speed, 0, 255, 0, 100));
    Serial.print("% (PWM: ");
    Serial.print(test_speed);
    Serial.println(")");
    
    // Smooth transition to test speed
    smoothTransition(test_speed);
    
    // Hold speed for test duration
    unsigned long test_start = millis();
    while (millis() - test_start < TEST_DURATION && test_running) {
      updateMotorControl();
      
      // Report every second
      static unsigned long last_report = 0;
      if (millis() - last_report >= 1000) {
        last_report = millis();
        Serial.print("  Current: ");
        Serial.print(map(current_speed, 0, 255, 0, 100));
        Serial.println("%");
      }
      
      delay(50);
    }
  }
  
  // Test complete
  Serial.println("Speed test complete!");
  stopMotor();
}

void testRamping() {
  Serial.println("Testing smooth speed ramping...");
  
  // Test increasing speed
  Serial.println("Ramping UP: 0% -> 100%");
  smoothTransition(MAX_SPEED);
  delay(2000);
  
  // Test decreasing speed  
  Serial.println("Ramping DOWN: 100% -> 0%");
  smoothTransition(0);
  delay(1000);
  
  Serial.println("Ramp test complete!");
}

void testDirectionChange() {
  Serial.println("Testing direction changes...");
  
  // Start at medium speed
  smoothTransition(150);
  delay(2000);
  
  for (int i = 0; i < 3; i++) {
    Serial.print("Direction change ");
    Serial.println(i + 1);
    
    // Stop motor
    smoothTransition(0);
    delay(500);
    
    // Change direction
    direction = !direction;
    digitalWrite(MOTOR_DIR, direction ? HIGH : LOW);
    Serial.println(direction ? "  -> Clockwise" : "  -> Counter-clockwise");
    
    // Resume speed
    smoothTransition(150);
    delay(3000);
  }
  
  stopMotor();
  Serial.println("Direction test complete!");
}

void smoothTransition(int new_target) {
  target_speed = new_target;
  
  Serial.print("Smooth transition: ");
  Serial.print(map(current_speed, 0, 255, 0, 100));
  Serial.print("% -> ");
  Serial.print(map(target_speed, 0, 255, 0, 100));
  Serial.println("%");
  
  while (current_speed != target_speed) {
    if (current_speed < target_speed) {
      current_speed = min(current_speed + 2, target_speed);
    } else {
      current_speed = max(current_speed - 2, target_speed);
    }
    
    analogWrite(MOTOR_PWM, current_speed);
    delay(RAMP_DELAY);
  }
  
  Serial.println("  Transition complete");
}

void updateMotorControl() {
  // Apply current speed to motor
  analogWrite(MOTOR_PWM, current_speed);
}

void stopMotor() {
  Serial.println("Stopping motor...");
  test_running = false;
  target_speed = 0;
  
  // Smooth stop
  while (current_speed > 0) {
    current_speed = max(current_speed - 5, 0);
    analogWrite(MOTOR_PWM, current_speed);
    delay(50);
  }
  
  Serial.println("Motor stopped");
}

/*
 * Speed Test Results Interpretation:
 * 
 * 1. Minimum Speed: Motor should start reliably at MIN_SPEED
 * 2. Speed Linearity: Speed should increase proportionally with PWM
 * 3. Smooth Ramping: No sudden jumps or vibrations during transitions
 * 4. Direction Changes: Motor should stop cleanly before direction change
 * 
 * Expected Output:
 * - Clean motor startup at minimum speed
 * - Proportional speed increases
 * - Smooth acceleration/deceleration curves
 * - Stable operation at all test speeds
 * 
 * If you notice issues:
 * - Jerky motion: Check power supply capacity
 * - Won't start: Increase MIN_SPEED value
 * - Speed not linear: Check motor specifications
 * - Direction issues: Verify wiring and delays
 */