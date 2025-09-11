// Differential Drive Robot Arduino Code
// Compatible with ROS2 motor_controller.py

// Motor pins for differential drive (2 wheels)
#define LEFT_MOTOR_EN 6   // PWM pin for left motor
#define LEFT_MOTOR_IN1 4  // Direction pin 1 for left motor
#define LEFT_MOTOR_IN2 7  // Direction pin 2 for left motor

#define RIGHT_MOTOR_EN 10 // PWM pin for right motor
#define RIGHT_MOTOR_IN1 13 // Direction pin 1 for right motor
#define RIGHT_MOTOR_IN2 12 // Direction pin 2 for right motor

// Encoder pins
#define LEFT_ENCODER_A 2   // Interrupt pin for left encoder
#define LEFT_ENCODER_B 24
#define RIGHT_ENCODER_A 3  // Interrupt pin for right encoder
#define RIGHT_ENCODER_B 26

// Encoder counters
volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;

// Previous encoder counts for velocity calculation
long prev_left_count = 0;
long prev_right_count = 0;

// Timing variables
unsigned long last_cmd_time = 0;
unsigned long last_odom_time = 0;
const unsigned long CMD_TIMEOUT = 1000;  // Stop motors if no command received for 1 second
const unsigned long ODOM_INTERVAL = 50;  // Send odometry every 50ms

// Robot parameters
#define ENCODER_RESOLUTION 751.8  // ticks per revolution
#define WHEEL_RADIUS 0.070        // meters
#define WHEEL_SEPARATION 0.360    // meters (left-right distance)

void setup() {
  // Initialize motor pins
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Initialize encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);

  Serial.begin(115200);
  Serial.setTimeout(10);
  stopMotors();
  
  last_odom_time = millis();
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("V,")) {
      // Parse velocity commands: V,left_pwm,right_pwm
      data = data.substring(2); // Remove "V,"
      int commaIndex = data.indexOf(',');
      
      if (commaIndex != -1) {
        int left_pwm = constrain(data.substring(0, commaIndex).toInt(), -255, 255);
        int right_pwm = constrain(data.substring(commaIndex + 1).toInt(), -255, 255);
        
        setMotorSpeed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, left_pwm);
        setMotorSpeed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, right_pwm);
        
        last_cmd_time = millis();
      }
    }
  }

  // Safety check - stop motors if no command received for a while
  if (millis() - last_cmd_time > CMD_TIMEOUT) {
    stopMotors();
  }

  // Send odometry data every ODOM_INTERVAL ms
  if (millis() - last_odom_time >= ODOM_INTERVAL) {
    sendOdometryData();
    last_odom_time = millis();
  }
}

void setMotorSpeed(int in1_pin, int in2_pin, int en_pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }
  analogWrite(en_pin, abs(speed));
}

void stopMotors() {
  setMotorSpeed(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 0);
  setMotorSpeed(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 0);
}

void leftEncoderISR() {
  // Read the other encoder pin to determine direction
  if (digitalRead(LEFT_ENCODER_B) == HIGH) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void rightEncoderISR() {
  // Read the other encoder pin to determine direction
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

void sendOdometryData() {
  // Get current encoder counts
  noInterrupts();
  long current_left = left_encoder_count;
  long current_right = right_encoder_count;
  interrupts();
  
  // Calculate wheel velocities (delta counts per time interval)
  float dt = ODOM_INTERVAL / 1000.0; // Convert to seconds
  float v_left = (current_left - prev_left_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  float v_right = (current_right - prev_right_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  
  // Update previous counts
  prev_left_count = current_left;
  prev_right_count = current_right;
  
  // Send in format expected by ROS2 motor_controller.py: ODOM_LR:v_left,v_right
  Serial.print("ODOM_LR:");
  Serial.print(v_left, 4);
  Serial.print(",");
  Serial.println(v_right, 4);
} 