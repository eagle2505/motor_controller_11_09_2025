#include <PID_v1.h>

// a = FL    pin  -enFL -in1 -in2 <front left>
// b = BL    pin  -enBL -in3 -in4 <back left >
// c = FR    pin  -enFR -in5 -in6 <front right>
// d = BR    pin  -enBR -in7 -in8 <back right>

// L298N H-Bridge Connection PINs
#define L298N_enFL 6  // PWM
#define L298N_in1 4  // Dir Motor FL
#define L298N_in2 7  // Dir Motor FL


#define L298N_enBL 5  // PWM
#define L298N_in4 A0 // Dir Motor BL
#define L298N_in3 A1  // Dir Motor BL

#define L298N_enFR 10 //PWM
#define L298N_in5 13//Dir Motor FR
#define L298N_in6 12 // Dir Motor FR

#define L298N_enBR 9 // PWM
#define L298N_in7 8// Dir Motor BR
#define L298N_in8 11// Dir Motor BR

// Wheel Encoders Connection PINs
#define FL_encoder_phaseA 2  // Interrupt 
#define FL_encoder_phaseB 24

#define BL_encoder_phaseA 3   // Interrupt
#define BL_encoder_phaseB 26


#define FR_encoder_phaseA 21   // Interrupt
#define FR_encoder_phaseB 30


#define BR_encoder_phaseA 20   // Interrupt
#define BR_encoder_phaseB 28

// Add these constants at the top of the file with other defines
#define WHEEL_DIAMETER 0.140  // meters
#define WHEEL_RADIUS (WHEEL_DIAMETER/2.0)  // meters
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)  // meters
#define ENCODER_RESOLUTION 751.8  // ticks per revolution
#define WHEEL_DISTANCE_LR 0.360  // meters (left-right distance)
#define WHEEL_DISTANCE_FB 0.312  // meters (front-back distance)

void FLEncoderCallback();
void BLEncoderCallback();
void FREncoderCallback();
void BREncoderCallback();
void stopMotors();
void moveRight();
void moveLeft();
void moveBackward() ;
void moveForward();
void updateEncoderCounts();


// Encoders "count ticks"
unsigned int FL_encoder_counter = 0;
unsigned int BL_encoder_counter = 0;
unsigned int FR_encoder_counter = 0;
unsigned int BR_encoder_counter = 0;

 // 'p' = positive, 'n' = negative
String FL_wheel_sign = "p";
String BL_wheel_sign = "p";
String FR_wheel_sign = "p";
String BR_wheel_sign = "p";

unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
// was -> is_right_wheel_cmd  to -> is_cmd_..
// was -> is_right_wheel_forward  to -> is_forward_..

bool is_cmd_FL = false, is_cmd_BL = false, is_cmd_FR = false, is_cmd_BR = false;
bool is_forward_FL = true, is_forward_BL = true, is_forward_FR = true, is_forward_BR = true;

char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

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

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enFL, OUTPUT);
  pinMode(L298N_enBL, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  pinMode(L298N_enFR, OUTPUT);
  pinMode(L298N_enBR, OUTPUT);
  pinMode(L298N_in5, OUTPUT);
  pinMode(L298N_in6, OUTPUT);
  pinMode(L298N_in7, OUTPUT);
  pinMode(L298N_in8, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  digitalWrite(L298N_in5, HIGH);
  digitalWrite(L298N_in6, LOW);
  digitalWrite(L298N_in7, HIGH);
  digitalWrite(L298N_in8, LOW);

  FLMotor.SetMode(AUTOMATIC);
  BLMotor.SetMode(AUTOMATIC);
  FRMotor.SetMode(AUTOMATIC);
  BRMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(FL_encoder_phaseB, INPUT);
  pinMode(BL_encoder_phaseB, INPUT);
  pinMode(FR_encoder_phaseB, INPUT);
  pinMode(BR_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(FL_encoder_phaseA), FLEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_encoder_phaseA), BLEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_encoder_phaseA), FREncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_encoder_phaseA), BREncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  // ------------------ this part still same --------------------
    if (Serial.available())
    {
      char chr = Serial.read();
      Serial.println(chr);
  
      switch (chr) {
        case 'U': moveForward(); break;
        case 'B': moveBackward(); break;
        
        case 'R': rotateLeft(); break;
        case 'L': rotateRight();break;
        
        case 'F': moveRight(); break;
        case 'D': moveLeft(); break;

      case 'Q': moveDiagonalUpLeft(); break;
      case 'W': moveDiagonalUpRight(); break;
      case 'A': moveDiagonalDownLeft(); break;
      case 'S': moveDiagonalDownRight(); break;
        case 'Z': stopMotors(); break;
      }
    }
    updateEncoderCounts();
}

void moveDiagonalUpLeft() {
  // FL stop
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);

  // BL forward
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // FR forward
  digitalWrite(L298N_in5, HIGH);
  digitalWrite(L298N_in6, LOW);

  // BR stop
  digitalWrite(L298N_in7, LOW);
  digitalWrite(L298N_in8, LOW);

  cmd_vel_FL = 0.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 0.0;
}

void moveDiagonalUpRight() {
  // FL forward
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  // BL stop
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, LOW);

  // FR stop
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, LOW);

  // BR forward
  digitalWrite(L298N_in8, HIGH);
  digitalWrite(L298N_in7, LOW);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 0.0;
  cmd_vel_FR = 0.0;
  cmd_vel_BR = 5.0;
}

void moveDiagonalDownLeft() {
  // FL backward
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);

  // BL stop
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, LOW);

  // FR stop
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, LOW);

  // BR backward
  digitalWrite(L298N_in8, LOW);
  digitalWrite(L298N_in7, HIGH);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 0.0;
  cmd_vel_FR = 0.0;
  cmd_vel_BR = 5.0;
}


void moveDiagonalDownRight() {
  // FL stop
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);

  // BL backward
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);

  // FR backward
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, HIGH);

  // BR stop
  digitalWrite(L298N_in7, LOW);
  digitalWrite(L298N_in8, LOW);

  cmd_vel_FL = 0.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 0.0;
}


void rotateRight() {
  // FL forward
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  // BL backward
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);

  // FR backward
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, HIGH);

  // BR forward
  digitalWrite(L298N_in7, HIGH);
  digitalWrite(L298N_in8, LOW);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void rotateLeft() {
  // FL backward
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);

  // BL forward
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  // FR forward
  digitalWrite(L298N_in5, HIGH);
  digitalWrite(L298N_in6, LOW);

  // BR backward
  digitalWrite(L298N_in7, LOW);
  digitalWrite(L298N_in8, HIGH);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void moveForward() {
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);
  digitalWrite(L298N_in5, HIGH);
  digitalWrite(L298N_in6, LOW);
  digitalWrite(L298N_in7, LOW);
  digitalWrite(L298N_in8, HIGH);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void moveBackward() {
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, HIGH);
  digitalWrite(L298N_in7, HIGH);
  digitalWrite(L298N_in8, LOW);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void moveLeft() {
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, HIGH);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);
  digitalWrite(L298N_in5, HIGH);
  digitalWrite(L298N_in6, LOW);
  digitalWrite(L298N_in7, HIGH);
  digitalWrite(L298N_in8, LOW);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void moveRight() {
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, LOW);
  digitalWrite(L298N_in4, HIGH);
  digitalWrite(L298N_in5, LOW);
  digitalWrite(L298N_in6, HIGH);
  digitalWrite(L298N_in7, LOW);
  digitalWrite(L298N_in8, HIGH);

  cmd_vel_FL = 5.0;
  cmd_vel_BL = 5.0;
  cmd_vel_FR = 5.0;
  cmd_vel_BR = 5.0;
}

void stopMotors() {
  // Stop all motors
  cmd_vel_FL = 0.0;
  cmd_vel_BL = 0.0;
  cmd_vel_FR = 0.0;
  cmd_vel_BR = 0.0;
}

  // ----------------------- until here ------------------

  // Encoder
 void updateEncoderCounts() {
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    // Convert encoder counts to angular velocities (rad/s)
    meas_vel_FL = (10 * FL_encoder_counter * (60.0/ENCODER_RESOLUTION)) * 0.10472;
    meas_vel_BL = (10 * BL_encoder_counter * (60.0/ENCODER_RESOLUTION)) * 0.10472;
    meas_vel_FR = (10 * FR_encoder_counter * (60.0/ENCODER_RESOLUTION)) * 0.10472;
    meas_vel_BR = (10 * BR_encoder_counter * (60.0/ENCODER_RESOLUTION)) * 0.10472;
    
    FLMotor.Compute();
    BLMotor.Compute();
    FRMotor.Compute();
    BRMotor.Compute();

    // Ignore commands smaller than inertia
    if(cmd_vel_FL == 0.0) cmd_FL = 0.0;
    if(cmd_vel_BL == 0.0) cmd_BL = 0.0;
    if(cmd_vel_FR == 0.0) cmd_FR = 0.0;
    if(cmd_vel_BR == 0.0) cmd_BR = 0.0;

    // Calculate linear and angular velocities for mecanum wheels
    // Convert angular velocities to linear velocities (m/s)
    float v_FL = meas_vel_FL * WHEEL_RADIUS;
    float v_BL = meas_vel_BL * WHEEL_RADIUS;
    float v_FR = meas_vel_FR * WHEEL_RADIUS;
    float v_BR = meas_vel_BR * WHEEL_RADIUS;

    // Calculate robot velocities
    float vx = (v_FL + v_BL + v_FR + v_BR) / 4.0;  // Forward/backward velocity
    float vy = (-v_FL + v_BL + v_FR - v_BR) / 4.0; // Sideways velocity
    float omega = (-v_FL + v_BL - v_FR + v_BR) / (4.0 * (WHEEL_DISTANCE_LR/2.0)); // Angular velocity
    
    // Send odometry data
    Serial.print("ODOM:");
    Serial.print(vx);
    Serial.print(",");
    Serial.print(vy);
    Serial.print(",");
    Serial.print(omega);
    Serial.print(",");
    Serial.print(FL_encoder_counter);
    Serial.print(",");
    Serial.print(BL_encoder_counter);
    Serial.print(",");
    Serial.print(FR_encoder_counter);
    Serial.print(",");
    Serial.println(BR_encoder_counter);

    // Debug output
    String encoder_read = "FL" + FL_wheel_sign + String(meas_vel_FL) + ", BL" + BL_wheel_sign + String(meas_vel_BL) + ", FR" + FR_wheel_sign + String(meas_vel_FR) + ", BR" + BR_wheel_sign + String(meas_vel_BR);
    Serial.println(encoder_read);
    
    last_millis = current_millis;
    FL_encoder_counter = 0;
    BL_encoder_counter = 0;
    FR_encoder_counter = 0;
    BR_encoder_counter = 0;

    analogWrite(L298N_enFL, cmd_FL);
    analogWrite(L298N_enBL, cmd_BL);
    analogWrite(L298N_enFR, cmd_FR);
    analogWrite(L298N_enBR, cmd_BR);
  }
}

// New pulse from Right Wheel Encoder
void FLEncoderCallback()
{
  if(digitalRead(FL_encoder_phaseB) == HIGH)
  {
    FL_wheel_sign = "p";
  }
  else
  {
    FL_wheel_sign = "n";
  }
  FL_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void BLEncoderCallback()
{
  if(digitalRead(BL_encoder_phaseB) == HIGH)
  {
    BL_wheel_sign = "n";
  }
  else
  {
    BL_wheel_sign = "p";
  }
  BL_encoder_counter++;
}

// New pulse from Right Wheel Encoder
void FREncoderCallback()
{
  if(digitalRead(FR_encoder_phaseB) == HIGH)
  {
    FR_wheel_sign = "p";
  }
  else
  {
    FR_wheel_sign = "n";
  }
  FR_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void BREncoderCallback()
{
  if(digitalRead(BR_encoder_phaseB) == HIGH)
  {
    BR_wheel_sign = "n";
  }
  else
  {
    BR_wheel_sign = "p";
  }
  BR_encoder_counter++;
}
