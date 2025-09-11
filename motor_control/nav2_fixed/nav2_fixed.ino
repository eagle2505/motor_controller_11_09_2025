#define LEFT_MOTOR_PIN1 7
#define LEFT_MOTOR_PIN2 6
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 3

#define LEFT_encoder_phaseA 19
#define LEFT_encoder_phaseB 18
#define RIGHT_encoder_phaseA 20
#define RIGHT_encoder_phaseB 21

volatile long LEFT_encoder_count = 0;
volatile long RIGHT_encoder_count = 0;
long prev_LEFT_count = 0;
long prev_RIGHT_count = 0;

unsigned long last_cmd_time = 0;
unsigned long last_odom_time = 0;
const unsigned long CMD_TIMEOUT = 1000;
const unsigned long ODOM_INTERVAL = 20; // ms

// Tune to your encoders
#define ENCODER_RESOLUTION 1320.0 // counts per revolution (1x on A rising edges)
#define WHEEL_RADIUS 0.025 // m
#define WHEEL_DISTANCE 0.280 // m

void setup() {
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

    pinMode(LEFT_encoder_phaseA, INPUT_PULLUP);
    pinMode(LEFT_encoder_phaseB, INPUT_PULLUP);
    pinMode(RIGHT_encoder_phaseA, INPUT_PULLUP);
    pinMode(RIGHT_encoder_phaseB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_encoder_phaseA), LEFT_encoder_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_encoder_phaseA), RIGHT_encoder_isr, RISING);

    Serial.begin(115200);
    Serial.setTimeout(10);

    stopMotors();
    last_cmd_time = millis();
    last_odom_time = millis();
}

void loop() {
    // Read velocity commands
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        if (data.startsWith("V,")) {
            data = data.substring(2);
            int comma = data.indexOf(',');
            if (comma > 0) {
                int left_pwm = constrain(data.substring(0, comma).toInt(), -255, 255);
                int right_pwm = constrain(data.substring(comma + 1).toInt(), -255, 255);
                
                // Fixed: Use proper motor control functions
                setLeftMotorSpeed(left_pwm);
                setRightMotorSpeed(right_pwm);
                
                last_cmd_time = millis();
            }
        }
    }

    // Safety: stop if stale
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        stopMotors();
    }

    // Odometry publish
    if (millis() - last_odom_time >= ODOM_INTERVAL) {
        sendOdometryLR();
        last_odom_time = millis();
    }
}

// Fixed: Separate functions for left and right motors
void setLeftMotorSpeed(int speed) {
    int pwm = constrain(abs(speed), 0, 255);
    if (speed == 0) {
        digitalWrite(LEFT_MOTOR_PIN1, LOW);
        digitalWrite(LEFT_MOTOR_PIN2, LOW);
        analogWrite(LEFT_MOTOR_PIN2, 0);
        return;
    }
    bool forward = speed > 0;
    digitalWrite(LEFT_MOTOR_PIN1, forward ? LOW : HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, forward ? HIGH : LOW);
    analogWrite(LEFT_MOTOR_PIN2, pwm);
}

void setRightMotorSpeed(int speed) {
    int pwm = constrain(abs(speed), 0, 255);
    if (speed == 0) {
        digitalWrite(RIGHT_MOTOR_PIN1, LOW);
        digitalWrite(RIGHT_MOTOR_PIN2, LOW);
        analogWrite(RIGHT_MOTOR_PIN2, 0);
        return;
    }
    bool forward = speed > 0;
    digitalWrite(RIGHT_MOTOR_PIN1, forward ? LOW : HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, forward ? HIGH : LOW);
    analogWrite(RIGHT_MOTOR_PIN2, pwm);
}

// Fixed: Proper stop function
void stopMotors() {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
    analogWrite(LEFT_MOTOR_PIN2, 0);
    analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void sendOdometryLR() {
    noInterrupts();
    long curL = LEFT_encoder_count;
    long curR = RIGHT_encoder_count;
    interrupts();

    long dL = curL - prev_LEFT_count;
    long dR = curR - prev_RIGHT_count;

    prev_LEFT_count = curL;
    prev_RIGHT_count = curR;

    float dt = ODOM_INTERVAL / 1000.0f;

    // Convert counts->m: (counts / CPR) * (2πR)
    float v_left = (dL / ENCODER_RESOLUTION) * (2.0f * PI * WHEEL_RADIUS) / dt;
    float v_right = (dR / ENCODER_RESOLUTION) * (2.0f * PI * WHEEL_RADIUS) / dt;

    Serial.print("ODOM_LR:");
    Serial.print(v_left, 4);
    Serial.print(",");
    Serial.println(v_right, 4);
}

// Quadrature ISR (A rising, read B for direction)
void LEFT_encoder_isr() {
    LEFT_encoder_count += (digitalRead(LEFT_encoder_phaseB) == HIGH) ? -1 : 1;
}
void RIGHT_encoder_isr() {
    RIGHT_encoder_count += (digitalRead(RIGHT_encoder_phaseB) == HIGH) ? 1 : -1;
} 
