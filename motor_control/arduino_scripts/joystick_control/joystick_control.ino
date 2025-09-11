// Motor pins for omni-wheel setup
#define L298N_enFL 6  // PWM Front Left
#define L298N_in1 4   // Dir Motor FL
#define L298N_in2 7   // Dir Motor FL

#define L298N_enBL 5  // PWM Back Left
#define L298N_in4 A0  // Dir Motor BL
#define L298N_in3 A1  // Dir Motor BL

#define L298N_enFR 10 // PWM Front Right
#define L298N_in5 13  // Dir Motor FR
#define L298N_in6 12  // Dir Motor FR

#define L298N_enBR 9  // PWM Back Right
#define L298N_in8 8   // Dir Motor BR//
#define L298N_in7 11  // Dir Motor BR//

const int maxPWM = 255;  // Maximum PWM value
const float angle_thresh = 15.0;  // Threshold for angle matching (degrees)

void setup() {
  // Initialize motor control pins
  pinMode(L298N_enFL, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  pinMode(L298N_enBL, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  pinMode(L298N_enFR, OUTPUT);
  pinMode(L298N_in5, OUTPUT);
  pinMode(L298N_in6, OUTPUT);

  pinMode(L298N_enBR, OUTPUT);
  pinMode(L298N_in7, OUTPUT);
  pinMode(L298N_in8, OUTPUT);

  Serial.begin(115200);
  delay(2000);  // wait for serial port to settle
  Serial.println("Omni-wheel robot initialized and ready!");
}

void loop() {
  static String input = "";

  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      // Parse all four values
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        // Parse all velocities
        float linear = input.substring(0, firstComma).toFloat();
        float angular = input.substring(firstComma + 1, secondComma).toFloat();
        float linear2 = input.substring(secondComma + 1, thirdComma).toFloat();
        float angular2 = input.substring(thirdComma + 1).toFloat();

        // Debug output
        Serial.print("Received - Linear: ");
        Serial.print(linear);
        Serial.print(", Angular: ");
        Serial.print(angular);
        Serial.print(", Linear2: ");
        Serial.print(linear2);
        Serial.print(", Angular2: ");
        Serial.println(angular2);

        // Calculate wheel speeds for forward/backward and rotation
        float speedFL = linear - angular;
        float speedFR = linear + angular;
        float speedBL = linear - angular;
        float speedBR = linear + angular;

        // Calculate angle of right stick (Joy 2)
        float joy2_angle = atan2(angular2, linear2) * 180.0 / PI;
        if (joy2_angle < 0) joy2_angle += 360.0;
        
        // Calculate magnitude of right stick movement
        float joy2_magnitude = sqrt(linear2 * linear2 + angular2 * angular2);

        // Check for pure slide right (90°) or left (270°)
        if (joy2_magnitude > 0.1) {  // Only if stick is moved enough
          if (abs(joy2_angle - 90.0) < angle_thresh) {
            // Slide right (90°)
            speedFL = -joy2_magnitude; // backward
            speedBR = -joy2_magnitude; // backward
            speedFR = joy2_magnitude;  // forward
            speedBL = joy2_magnitude;  // forward
          } else if (abs(joy2_angle - 270.0) < angle_thresh) {
            // Slide left (270°)
            speedFL = joy2_magnitude;  // forward
            speedBR = joy2_magnitude;  // forward
            speedFR = -joy2_magnitude; // backward
            speedBL = -joy2_magnitude; // backward
          } else {
            // Normal diagonal movement
            // Add side movement
            // speedFL += linear2;
            // speedFR -= linear2;
            // speedBL -= linear2;
            // speedBR += linear2;

            // Add diagonal movement
            if (abs(linear2) > 0.01 && abs(angular2) > 0.01) {
              float diag = (abs(linear2) < abs(angular2)) ? abs(linear2) : abs(angular2);
              if (linear2 > 0 && angular2 > 0) { // Up-Left
                speedFR += diag;
                speedBL += diag;
                // speedFL = 0;
                // speedBR = 0;
                // speedFR = 0;
                // speedBL = 0;
              } else if (linear2 < 0 && angular2 > 0) { // Down left
                speedFL -= diag;
                speedBR -= diag;
                // speedFL = 0;
                // speedBR = 0;
                // speedFR = 0;
                // speedBL = 0;
              } else if (linear2 > 0 && angular2 < 0) { // Up-Right
                speedFL += diag;
                speedBR += diag;
                // speedFL = 0;
                // speedBR = 0;
                // speedFR = 0;
                // speedBL = 0;
              } else if (linear2 < 0 && angular2 < 0) { // Down-Right
                speedFR -= diag;
                speedBL -= diag;
                // speedFL = 0;
                // speedBR = 0;
                // speedFR = 0;
                // speedBL = 0;
              }
            }
          }
        }

        // Convert to PWM values (-255 to 255)
        int pwmFL = constrain((int)(speedFL * 255), -maxPWM, maxPWM);
        int pwmFR = constrain((int)(speedFR * 255), -maxPWM, maxPWM);
        int pwmBL = constrain((int)(speedBL * 255), -maxPWM, maxPWM);
        int pwmBR = constrain((int)(speedBR * 255), -maxPWM, maxPWM);

        // Debug PWM values
        Serial.print("PWM Values - FL: ");
        Serial.print(pwmFL);
        Serial.print(", FR: ");
        Serial.print(pwmFR);
        Serial.print(", BL: ");
        Serial.print(pwmBL);
        Serial.print(", BR: ");
        Serial.println(pwmBR);

        // Set motor directions and speeds
        // Front Left Motor
        if (pwmFL >= 0) {
          digitalWrite(L298N_in1, HIGH);
          digitalWrite(L298N_in2, LOW);
          analogWrite(L298N_enFL, pwmFL);
        } else {
          digitalWrite(L298N_in1, LOW);
          digitalWrite(L298N_in2, HIGH);
          analogWrite(L298N_enFL, -pwmFL);
        }

        // Back Left Motor
        if (pwmBL >= 0) {
          digitalWrite(L298N_in3, HIGH);
          digitalWrite(L298N_in4, LOW);
          analogWrite(L298N_enBL, pwmBL);
        } else {
          digitalWrite(L298N_in3, LOW);
          digitalWrite(L298N_in4, HIGH);
          analogWrite(L298N_enBL, -pwmBL);
        }

        // Front Right Motor
        if (pwmFR >= 0) {
          digitalWrite(L298N_in5, HIGH);
          digitalWrite(L298N_in6, LOW);
          analogWrite(L298N_enFR, pwmFR);
        } else {
          digitalWrite(L298N_in5, LOW);
          digitalWrite(L298N_in6, HIGH);
          analogWrite(L298N_enFR, -pwmFR);
        }

        // Back Right Motor
        if (pwmBR >= 0) {
          digitalWrite(L298N_in7, HIGH);
          digitalWrite(L298N_in8, LOW);
          analogWrite(L298N_enBR, pwmBR);
        } else {
          digitalWrite(L298N_in7, LOW);
          digitalWrite(L298N_in8, HIGH);
          analogWrite(L298N_enBR, -pwmBR);
        }
      }
      input = ""; // Clear input for next command
    } else {
      input += c; // Build input string
    }
  }
} 