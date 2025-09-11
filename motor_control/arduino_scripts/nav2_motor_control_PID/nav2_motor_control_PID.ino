
#include <PID_v1.h>

// Motor pins definitions
// Motor 1 - back left (forward: high -> low)
#define L298N_enBL 5  // PWM
#define L298N_in3 A1  // Dir Motor BL
#define L298N_in4 A0  // Dir Motor BL

// Motor 2 - front left (forward: high -> low)
#define L298N_enFL 6  // PWM
#define L298N_in1 4   // Dir Motor FL
#define L298N_in2 7   // Dir Motor FL

// Motor 3 - back right (forward: low -> high)
#define L298N_enBR 9  // PWM
#define L298N_in7 11   // Dir Motor BR
#define L298N_in8 8  // Dir Motor BR

// Motor 4 - front right (forward: low -> high)
#define L298N_enFR 10  // PWM
#define L298N_in5 13  // Dir Motor FR
#define L298N_in6 12  // Dir Motor FR

// Encoder pins
#define FL_encoder_phaseA 2   // Interrupt
#define FL_encoder_phaseB 24
#define BL_encoder_phaseA 3   // Interrupt
#define BL_encoder_phaseB 26
#define FR_encoder_phaseA 21  // Interrupt
#define FR_encoder_phaseB 30
#define BR_encoder_phaseA 20  // Interrupt
#define BR_encoder_phaseB 28

// Encoder counters
volatile long FL_encoder_count = 0;
volatile long BL_encoder_count = 0;
volatile long FR_encoder_count = 0;
volatile long BR_encoder_count = 0;

// Previous encoder counts for velocity calculation
long prev_FL_count = 0;
long prev_BL_count = 0;
long prev_FR_count = 0;
long prev_BR_count = 0;

// PID
// Setpoint - Desired
// was -> right_wheel_cmd_vel  to -> cmd_vel_..   // rad/s
double cmd_vel_FL = 0.0, cmd_vel_BL = 0.0, cmd_vel_FR = 0.0, cmd_vel_BR = 0.0;
// Input - Measurement
// was -> right_wheel_meas_vel  to -> meas_vel_..   // rad/s
double meas_vel_FL = 0.0, meas_vel_BL = 0.0, meas_vel_FR = 0.0, meas_vel_BR = 0.0;
// Output - Command
// was -> right_wheel_cmd  to -> cmd_..   // 0-255
double cmd_FL = 0.0, cmd_BL = 0.0, cmd_FR = 0.0, cmd_BR = 0.0;

// Tuning
double Kp_fl = 11.5;
double Ki_fl = 7.5;
double Kd_fl = 0.1;
double Kp_bl = 12.8;
double Ki_bl = 8.3;
double Kd_bl = 0.1;
/// --------------------------------------------------Change the values----------------------------------------------------------- ///
double Kp_fr = 11.5;
double Ki_fr = 7.5;
double Kd_fr = 0.1;
double Kp_br = 12.8;
double Ki_br = 8.3;
double Kd_br = 0.1;

// Controller
PID FLMotor(&meas_vel_FL, &cmd_FL, &cmd_vel_FL, Kp_fl, Ki_fl, Kd_fl, DIRECT);
PID BLMotor(&meas_vel_BL, &cmd_BL, &cmd_vel_BL, Kp_bl, Ki_bl, Kd_bl, DIRECT);
PID FRMotor(&meas_vel_FR, &cmd_FR, &cmd_vel_FR, Kp_fr, Ki_fr, Kd_fr, DIRECT);
PID BRMotor(&meas_vel_BR, &cmd_BR, &cmd_vel_BR, Kp_br, Ki_br, Kd_br, DIRECT);

// Timing variables
unsigned long last_cmd_time = 0;
unsigned long last_odom_time = 0;
const unsigned long CMD_TIMEOUT = 1000;  // Stop motors if no command received for 1 second
const unsigned long ODOM_INTERVAL = 50;  // Send odometry every 50ms

#define ENCODER_RESOLUTION 187.95  // ticks per revolution
#define WHEEL_RADIUS 0.070        // meters
#define WHEEL_DISTANCE_LR 0.360   // meters (left-right)
#define WHEEL_DISTANCE_FB 0.312   // meters (front-back)

// Calculate wheel distance for rotation (diagonal distance from center)
#define WHEEL_BASE ((WHEEL_DISTANCE_LR + WHEEL_DISTANCE_FB) / 2.0)

void setup() {
  // Initialize motor pins
  pinMode(L298N_enFL, OUTPUT); pinMode(L298N_in1, OUTPUT); pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_enBL, OUTPUT); pinMode(L298N_in3, OUTPUT); pinMode(L298N_in4, OUTPUT);
  pinMode(L298N_enFR, OUTPUT); pinMode(L298N_in5, OUTPUT); pinMode(L298N_in6, OUTPUT);
  pinMode(L298N_enBR, OUTPUT); pinMode(L298N_in7, OUTPUT); pinMode(L298N_in8, OUTPUT);

  // Initialize encoder pins
  pinMode(FL_encoder_phaseA, INPUT_PULLUP); pinMode(FL_encoder_phaseB, INPUT_PULLUP);
  pinMode(BL_encoder_phaseA, INPUT_PULLUP); pinMode(BL_encoder_phaseB, INPUT_PULLUP);
  pinMode(FR_encoder_phaseA, INPUT_PULLUP); pinMode(FR_encoder_phaseB, INPUT_PULLUP);
  pinMode(BR_encoder_phaseA, INPUT_PULLUP); pinMode(BR_encoder_phaseB, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(FL_encoder_phaseA), FL_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_encoder_phaseA), BL_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_encoder_phaseA), FR_encoder_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_encoder_phaseA), BR_encoder_isr, RISING);

  FLMotor.SetMode(AUTOMATIC);
  BLMotor.SetMode(AUTOMATIC);
  FRMotor.SetMode(AUTOMATIC);
  BRMotor.SetMode(AUTOMATIC);

  FLMotor.SetOutputLimits(-255, 255);
  BLMotor.SetOutputLimits(-255, 255);
  FRMotor.SetOutputLimits(-255, 255);
  BRMotor.SetOutputLimits(-255, 255);

  Serial.begin(115200);
  Serial.setTimeout(10);
  stopMotors();
  
  last_odom_time = millis();
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data.startsWith("V,")) {
      // Parse velocity commands: V,fl_speed,fr_speed,bl_speed,br_speed
      data = data.substring(2); // Remove "V,"
      int commaIndex1 = data.indexOf(',');
      int commaIndex2 = data.indexOf(',', commaIndex1 + 1);
      int commaIndex3 = data.indexOf(',', commaIndex2 + 1);
      
      if (commaIndex1 != -1 && commaIndex2 != -1 && commaIndex3 != -1) {
        double fl_speed = data.substring(0, commaIndex1).toFloat();
        double fr_speed = data.substring(commaIndex1 + 1, commaIndex2).toFloat();
        double bl_speed = data.substring(commaIndex2 + 1, commaIndex3).toFloat();
        double br_speed = data.substring(commaIndex3 + 1).toFloat();

        cmd_vel_FL = fl_speed;
        cmd_vel_FR = fr_speed;
        cmd_vel_BL = bl_speed;
        cmd_vel_BR = br_speed;
        
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

void stopMotors() {
  applyMotorCommand(L298N_in1, L298N_in2, L298N_enFL, 0);
  applyMotorCommand(L298N_in3, L298N_in4, L298N_enBL, 0);
  applyMotorCommand(L298N_in5, L298N_in6, L298N_enFR, 0);
  applyMotorCommand(L298N_in7, L298N_in8, L298N_enBR, 0);
}

void applyMotorCommand(int in1, int in2, int en, double cmd) {
  if (cmd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (cmd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, min(abs(cmd), 255));  // Clamp to 255
}


void sendOdometryData() {
  // Get current encoder counts
  noInterrupts();
  long current_FL = FL_encoder_count;
  long current_BL = BL_encoder_count;
  long current_FR = FR_encoder_count;
  long current_BR = BR_encoder_count;
  interrupts();
  
  // Calculate wheel velocities (delta counts per time interval)
  float dt = ODOM_INTERVAL / 1000.0; // Convert to seconds
  float vFL = (current_FL - prev_FL_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  float vBL = (current_BL - prev_BL_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  float vFR = (current_FR - prev_FR_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  float vBR = (current_BR - prev_BR_count) * (2.0 * PI * WHEEL_RADIUS) / (ENCODER_RESOLUTION * dt);
  // Update measured velocities for PID
  meas_vel_FL = vFL;
  meas_vel_FR = vFR;
  meas_vel_BL = vBL;
  meas_vel_BR = vBR;

  // Compute new PWM outputs from PID
  FLMotor.Compute();
  FRMotor.Compute();
  BLMotor.Compute();
  BRMotor.Compute();

  // Send computed PWM to motors
  applyMotorCommand(L298N_in1, L298N_in2, L298N_enFL, cmd_FL);
  applyMotorCommand(L298N_in3, L298N_in4, L298N_enBL, cmd_BL);
  applyMotorCommand(L298N_in5, L298N_in6, L298N_enFR, cmd_FR);
  applyMotorCommand(L298N_in7, L298N_in8, L298N_enBR, cmd_BR);


  // FIXED: Correct mecanum wheel kinematics
  // For a standard mecanum wheel configuration:
  // Front Left (FL): + for forward, - for left strafe, - for CCW rotation
  // Front Right (FR): + for forward, + for left strafe, + for CCW rotation  
  // Back Left (BL): + for forward, + for left strafe, - for CCW rotation
  // Back Right (BR): + for forward, - for left strafe, + for CCW rotation
  
  // Robot velocities in body frame
  float vx = (vFL + vFR + vBL + vBR) / 4.0;  // Forward velocity
  float vy = (-vFL + vFR + vBL - vBR) / 4.0; // Left strafe velocity  
  float omega = (-vFL + vFR - vBL + vBR) / (4.0 * WHEEL_BASE); // Angular velocity
  
  // Send in format expected by ROS: ODOM:vx,vy,omega,FL,BL,FR,BR
  Serial.print("ODOM:");
  Serial.print(vx, 4); Serial.print(",");
  Serial.print(vy, 4); Serial.print(",");
  Serial.print(omega, 4); Serial.print(",");
  Serial.print(current_FL); Serial.print(",");
  Serial.print(current_BL); Serial.print(",");
  Serial.print(current_FR); Serial.print(",");
  Serial.println(current_BR);
  
  // Update previous counts
  prev_FL_count = current_FL;
  prev_BL_count = current_BL;
  prev_FR_count = current_FR;
  prev_BR_count = current_BR;
}

// Encoder interrupt service routines
void FL_encoder_isr() {
  FL_encoder_count += (digitalRead(FL_encoder_phaseB) == HIGH) ? -1 : 1;
}

void BL_encoder_isr() {
  BL_encoder_count += (digitalRead(BL_encoder_phaseB) == HIGH) ? -1 : 1;
}

void FR_encoder_isr() {
  FR_encoder_count += (digitalRead(FR_encoder_phaseB) == HIGH) ? 1 : -1;
}

void BR_encoder_isr() {
  BR_encoder_count += (digitalRead(BR_encoder_phaseB) == HIGH) ? 1 : -1;
}
