/*
 * Basic Motor Control Example
 * Simplified version of Arduino Motor Fan Project
 * 
 * This example demonstrates basic motor control using HW-020 shield
 * without advanced features like safety monitoring or EEPROM storage
 * 
 * Hardware:
 * - Arduino Uno R3
 * - HW-020 Motor Driver Shield (L298P)
 * - 12V DC Motor
 * - 12V Power Supply
 * - Potentiometer (10kΩ) on A0
 * - Direction button on D2
 */

// Pin definitions - minimal setup  
#define MOTOR_PWM       3     // Motor PWM (ENA)
#define MOTOR_DIR       12    // Motor Direction (IN1)
#define SPEED_POT       A0    // Speed control potentiometer
#define DIRECTION_BTN   2     // Direction control button

// Basic variables
int motor_speed = 0;
bool motor_direction = true;  // true = CW, false = CCW
volatile bool direction_changed = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Motor Control Example");
  Serial.println("=============================");
  
  // Initialize pins
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(DIRECTION_BTN, INPUT_PULLUP);
  
  // Setup interrupt for direction button
  attachInterrupt(digitalPinToInterrupt(DIRECTION_BTN), onDirectionButton, FALLING);
  
  Serial.println("Motor ready! Use potentiometer to control speed");
  Serial.println("Press button to change direction");
}

void loop() {
  // Read speed from potentiometer
  int pot_value = analogRead(SPEED_POT);
  motor_speed = map(pot_value, 0, 1023, 0, 255);
  
  // Apply motor control
  analogWrite(MOTOR_PWM, motor_speed);
  digitalWrite(MOTOR_DIR, motor_direction ? HIGH : LOW);
  
  // Handle direction change
  if (direction_changed) {
    direction_changed = false;
    
    // Brief stop before changing direction
    analogWrite(MOTOR_PWM, 0);
    delay(200);
    
    // Toggle direction
    motor_direction = !motor_direction;
    
    Serial.print("Direction changed to: ");
    Serial.println(motor_direction ? "Clockwise" : "Counter-clockwise");
  }
  
  // Simple status report every 2 seconds
  static unsigned long last_report = 0;
  if (millis() - last_report >= 2000) {
    last_report = millis();
    
    Serial.print("Speed: ");
    Serial.print(map(motor_speed, 0, 255, 0, 100));
    Serial.print("% | Direction: ");
    Serial.println(motor_direction ? "CW" : "CCW");
  }
  
  delay(50);  // Small delay for stability
}

// Interrupt handler for direction button
void onDirectionButton() {
  static unsigned long last_interrupt = 0;
  
  // Simple debouncing
  if (millis() - last_interrupt > 500) {
    last_interrupt = millis();
    direction_changed = true;
  }
}

/*
 * Usage Instructions:
 * 1. Connect hardware as described in main project documentation
 * 2. Upload this code to Arduino
 * 3. Open Serial Monitor (115200 baud)
 * 4. Turn potentiometer to control speed
 * 5. Press button to change direction
 * 
 * This is a simplified version for learning purposes.
 * For production use, please use the full version with safety features.
 */