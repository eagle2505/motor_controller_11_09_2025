
// Motor 1 - back left (forward: high -> low)
int in1 = A1; // A1
int in2 = A0; // A0
int ena = 5;                                                                                                                                                                                                
// Motor 2 - front left (forward: high -> low)
int in3 = 4;
int in4 = 7;
int enb = 6;

// Motor 3 - back right (forward: low -> high)
int in5 = 8;
int in6 = 11;
int ena2 = 9;

// Motor sa4 - front right (forward: low -> high)
int in7 = 12;
int in8 = 13;
int enb2 = 10;


void setup() {

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(ena, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT); pinMode(enb, OUTPUT);
  pinMode(in5, OUTPUT); pinMode(in6, OUTPUT); pinMode(ena2, OUTPUT);
  pinMode(in7, OUTPUT); pinMode(in8, OUTPUT); pinMode(enb2, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(1);

}

void loop() {
  if (Serial.available())
  {
    char c = Serial.read();
    switch (c) {
      case 'W': moveForward(); break;
      case 'S': moveBackward(); break;
      case 'A': moveRight(); break;
      case 'D': moveLeft(); break;
      case 'Z': stopMotors(); break;
    }

  }
}

void moveForward()
{
       digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
       digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
       digitalWrite(in5, LOW); digitalWrite(in6, HIGH);
       digitalWrite(in7, LOW); digitalWrite(in8, HIGH);
       analogWrite(ena, 100); analogWrite(enb, 100); analogWrite(ena2, 100); analogWrite(enb2, 100); 
}

void moveBackward()
{
       digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
       digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
       digitalWrite(in5, HIGH); digitalWrite(in6, LOW);
       digitalWrite(in7, HIGH); digitalWrite(in8, LOW);
       analogWrite(ena, 100); analogWrite(enb, 100); analogWrite(ena2, 100); analogWrite(enb2, 100);
}

void moveRight()
{
      digitalWrite(in1, LOW); digitalWrite(in2, HIGH); // <-
      digitalWrite(in3, HIGH); digitalWrite(in4, LOW); // ->
      digitalWrite(in5, LOW); digitalWrite(in6, HIGH); // <-
      digitalWrite(in7, HIGH); digitalWrite(in8, LOW); // ->
      analogWrite(ena, 100); analogWrite(enb, 100); analogWrite(ena2, 100); analogWrite(enb2, 100);
     
      
}

void moveLeft()
{
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW); // ->
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH); // <-
      digitalWrite(in5, HIGH); digitalWrite(in6, LOW); // ->
      digitalWrite(in7, LOW); digitalWrite(in8, HIGH); // <-
      analogWrite(ena, 100); analogWrite(enb, 100); analogWrite(ena2, 100); analogWrite(enb2, 100);
      
}

void stopMotors()
{
       digitalWrite(in1, LOW); digitalWrite(in2, LOW);
       digitalWrite(in3, LOW); digitalWrite(in4, LOW);
       digitalWrite(in5, LOW); digitalWrite(in6, LOW);
       digitalWrite(in7, LOW); digitalWrite(in8, LOW);
       analogWrite(ena, 0); analogWrite(ena2, 0); analogWrite(enb, 0); analogWrite(enb2, 0);
}
