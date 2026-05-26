// Robot Team Project
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h> // *** NEW — needed for servo motor ***
#define DECODE_NEC
#include <IRremote.hpp>



// ---------------- LCD / IR ----------------
#define IR_Pin 8
LiquidCrystal_I2C lcd(0x27, 16, 2);


// ----------- Ultrasonic sensor ---------------
const int trig_Front = 13;
const int echo_Front = 12;
const int trig_Right = A0;
const int echo_Right = A1;


// -------- Right Ultrasonic sensor thresholds
// const float WALL_TARGET_IN = 24.0;   // 2 feet
const float right_dist_too_close = 2.5;
const float right_dist_too_far = 8.0;


// ----------- Servo ----------------
const int servoPin = 11;
Servo scanServo;


// ---------Ultrasonic sensor -----------
const int CENTER_ANGLE = 90;   // servo pointing straight ahead
const int LEFT_ANGLE   = 179;  // servo pointing left
const int RIGHT_ANGLE  = 1;    // servo pointing right


// --- Motor driver pins ---
const int enAPin = 6; // left motor control speed
const int in1Pin = 4;
const int in2Pin = 7;
const int in3Pin = 9;
const int in4Pin = 10;
const int enBPin = 5; // right motor speed control used to be 9, now 3



// --- Encoder interrupt pins ---
const int LEncPin = 3; // used to be 3, now 9
const int REncPin = 2;




// --- Base speeds ---
const int LEFT_SPEED  = 80;
const int RIGHT_SPEED = 80;
const int MIN_SPEED   = 0;   // never go below this — deadband floor


// --- Obstacle distance threshold (inches) ---
const float OBSTACLE_THRESHOLD_IN = 8.0;




// --- Pledge Algorithm heading tracker ---
int heading = 0;


unsigned long lastTurnTime = 0;


const int WALL_BASE_SPEED = 75;
const int WALL_CORRECTION = 15;


// ============================================================
// PID CONTROLLER PARAMETERS
// ============================================================
int   gain      = 15;     // Kp — proportional: reacts to current error
float Ki        = 0.01;  // integral: reacts to accumulated error over time
float Kd        = 1.0;   // derivative: reacts to rate of change of error
float integral  = 0.0;   // running sum of error over time
float prevError = 0.0;   // stores last loop's error for derivative calculation


// --- Encoder counters ---
volatile long cntrL    = 0;
volatile long cntrR    = 0;
volatile long LIntTime = 0;
volatile long RIntTime = 0;


// --- Turn constant: encoder counts per degree of turn (tune if needed) ---
const float cntrPerDegree = 0.400;


// --- Grace period after pressing forward (ms) ---
unsigned long forwardStartTime = 0;




// --- FSM states ---
enum State { STOP, FORWARD, WALL_FOLLOWING, TURN_LEFT, TURN_RIGHT};
State currentState = STOP;


// ============================================================
// MOTOR FUNCTIONS
// ============================================================

// Immediately cut power to both motors
void stopMotors() {
  analogWrite(enAPin, 0);
  analogWrite(enBPin, 0);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
}

// Drive forward and reset PID state so controller starts fresh
void moveForward() {
  cntrL = 0;
  cntrR = 0;
  integral = 0.0;
  prevError = 0.0;


  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);


  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);


  analogWrite(enAPin, LEFT_SPEED);
  analogWrite(enBPin, RIGHT_SPEED);
}


void moveForwardWithPID() {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);


  long tmpLcntr = cntrL;
  long tmpRcntr = cntrR;


  float error = (float)(tmpLcntr - tmpRcntr);
  integral = constrain(integral + error, -200.0, 200.0);
  float derivative = error - prevError;


  float correction = (gain * error)
                   + (Ki * integral)
                   + (Kd * derivative);




  prevError = error;


  if (tmpLcntr > tmpRcntr) {
    int adjSpeedL = LEFT_SPEED - abs((int)correction);
    analogWrite(enAPin, constrain(adjSpeedL, MIN_SPEED, 255));
    analogWrite(enBPin, RIGHT_SPEED);
  }
  else if (tmpLcntr < tmpRcntr) {
    int adjSpeedR = RIGHT_SPEED - abs((int)correction);
    analogWrite(enBPin, constrain(adjSpeedR, MIN_SPEED, 255));
    analogWrite(enAPin, LEFT_SPEED);
  }
  else {
    analogWrite(enAPin, LEFT_SPEED);
    analogWrite(enBPin, RIGHT_SPEED);
  }
}




// Drive backward and reset PID state so controller starts fresh
void moveBackward() {
  cntrL = 0;
  cntrR = 0;
  integral = 0.0;
  prevError = 0.0;




  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(enAPin, LEFT_SPEED);




  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, HIGH);
  analogWrite(enBPin, RIGHT_SPEED);
}




// ============================================================
// TURN FUNCTIONS
// ============================================================




// Left wheel drives forward, right wheel stopped → robot pivots right
void turnRight90() {
  long turnCounts = (long)(90.0 * cntrPerDegree + 0.5);




  cntrL = 0;
  cntrR = 0;




  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  analogWrite(enAPin, LEFT_SPEED);
  analogWrite(enBPin, 0); // right wheel stopped




  unsigned long startTime = millis();
  while (cntrL < turnCounts && millis() - startTime < 1000) {
    // wait for encoder counts or timeout
  }




  stopMotors();
  delay(200);
  cntrL = 0;
  cntrR = 0;
}




// Right wheel drives forward, left wheel stopped → robot pivots left
void turnLeft90() {
  long turnCounts = (long)(90.0 * cntrPerDegree + 0.5);




  cntrL = 0;
  cntrR = 0;




  analogWrite(enAPin, 0); // left wheel stopped
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enBPin, RIGHT_SPEED);




  unsigned long startTime = millis();
  while (cntrR < turnCounts && millis() - startTime < 1000) {
    // wait for encoder counts or timeout
  }




  stopMotors();
  delay(200);
  cntrL = 0;
  cntrR = 0;
}




// ============================================================
// LCD DISPLAY
// ============================================================
void updateLCD(float frontDist, float rightDist) {
  lcd.setCursor(0, 0);
  lcd.print("F:");
  lcd.setCursor(2, 0);
  lcd.print("      ");
  lcd.setCursor(2, 0);
  lcd.print(frontDist, 1);




  lcd.setCursor(0, 1);
  lcd.print("R:");
  lcd.setCursor(2, 1);
  lcd.print("      ");
  lcd.setCursor(2, 1);
  lcd.print(rightDist, 1);
}




// ============================================================
// ENCODERS
// ============================================================
void leftWhlCnt() {
  long intTime = micros();
  if (intTime > LIntTime + 1000L) {
    LIntTime = intTime;
    cntrL++;
  }
}




void rightWhlCnt() {
  long intTime = micros();
  if (intTime > RIntTime + 1000L) {
    RIntTime = intTime;
    cntrR++;
  }
}




// ============================================================
// ULTRASONIC
// ============================================================
// ---------------- Front -------------------------
float readFrontDistanceInches() {
  digitalWrite(trig_Front, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Front, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Front, LOW);




  unsigned long duration = pulseIn(echo_Front, HIGH, 30000);




  if (duration == 0) {
    return 999.0;
  }
  return duration / 74.0 / 2.0;
}




float readFrontAverageDistance() {
  float d1 = readFrontDistanceInches();
  delay(30);
  float d2 = readFrontDistanceInches();
  delay(30);
  float d3 = readFrontDistanceInches();
  return (d1 + d2 + d3) / 3.0;
}




// -------------------- Right --------------------------
float readRightDistanceInches() {
  digitalWrite(trig_Right, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Right, LOW);




  unsigned long duration = pulseIn(echo_Right, HIGH, 30000);




  if (duration == 0) {
    return 999.0;
  }
  return duration / 74.0 / 2.0;
}




float readRightAverageDistance() {
  float d1 = readRightDistanceInches();
  delay(30);
  float d2 = readRightDistanceInches();
  delay(30);
  float d3 = readRightDistanceInches();
  return (d1 + d2 + d3) / 3.0;
}




// ============================================================
// SERVO HELPERS
// ============================================================
void centerServo() {
  scanServo.write(CENTER_ANGLE);
  delay(350);
}




float lookLeft() {
  scanServo.write(LEFT_ANGLE);
  delay(450);
  return readFrontAverageDistance();
}




float lookRight() {
  scanServo.write(RIGHT_ANGLE);
  delay(450);
  return readFrontAverageDistance();
}


//============================================================
//FSM Helper Functions
//============================================================
void rightWallFollowing(){
  float rightDist = readRightAverageDistance();


  if (rightDist >= right_dist_too_close && rightDist <= right_dist_too_far) {
    // within good range, keep moving forward
    moveForwardWithPID();
  }
  else if (rightDist < right_dist_too_close) {
    // too close to right wall, stop and turn left a little
    stopMotors();       //
    delay(200);         // stop completely


    analogWrite(enAPin, 0);
    digitalWrite(in3Pin, HIGH);
    digitalWrite(in4Pin, LOW);
    analogWrite(enBPin, 90);


    delay(200);
    stopMotors();
    delay(100);


    // move forward a little after correcting
    moveForward();
    delay(250);
    stopMotors();
  }
  else if (rightDist > right_dist_too_far) {
    // too far from right wall, stop and turn right a little
    stopMotors();
    delay(200);


    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enAPin, 90);    // change this to change
    analogWrite(enBPin, 0);


    delay(200);
    stopMotors();
    delay(100);


    // move forward a little after correcting
    moveForward();
    delay(250);
    stopMotors();
  }
}








bool isRightCornerDetected(){
  float rightDist = readRightAverageDistance();
  return (rightDist > 24.0);
}




bool isFrontClear(){
  float frontDist = readFrontAverageDistance();
  return (frontDist > OBSTACLE_THRESHOLD_IN);
}




// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);




  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);




  pinMode(LEncPin, INPUT);
  pinMode(REncPin, INPUT);




  pinMode(trig_Front, OUTPUT);
  pinMode(echo_Front, INPUT);




  pinMode(trig_Right, OUTPUT);
  pinMode(echo_Right, INPUT);




  stopMotors();




  attachInterrupt(digitalPinToInterrupt(LEncPin), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(REncPin), rightWhlCnt, CHANGE);




  pinMode(IR_Pin, INPUT);
  IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK);




  lcd.init();
  lcd.backlight();




  scanServo.attach(servoPin);
  centerServo();




  updateLCD(0.0, 0.0);
}


// ============================================================
// MAIN LOOP - PLEDGE FSM
// ============================================================
void loop() {
  float frontDist = readFrontDistanceInches();
  float rightDist = readRightDistanceInches();




  // ---------------- IR REMOTE CONTROL ----------------
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;




    if (command == 12) { // button 1
      heading = 0;
      currentState = FORWARD;
      moveForward();
      forwardStartTime = millis();
      centerServo();
      updateLCD(frontDist, rightDist);
    }




    else if (command == 8) { // button 4
      currentState = STOP;
      stopMotors();
      updateLCD(frontDist, rightDist);
    }




    IrReceiver.resume();
  }




  // ---------------- FINITE STATE MACHINE ----------------
  switch (currentState) {
    case STOP:
      stopMotors();
      updateLCD(frontDist, rightDist);
      break;




    case FORWARD:
      updateLCD(frontDist, rightDist);




      if (!isFrontClear()) {
        stopMotors();
        currentState = TURN_LEFT;
      }
      else {
        moveForwardWithPID();
      }
      break;




    case TURN_LEFT:
      updateLCD(frontDist, rightDist);
      stopMotors();
      delay(150);




      turnLeft90();
      heading += 90;




      stopMotors();
      delay(20);




      cntrL = 0;
      cntrR = 0;
      integral = 0.0;
      prevError = 0.0;




      moveForward();


      delay(300);
      stopMotors();




      lastTurnTime = millis();
      currentState = WALL_FOLLOWING;
      break;




    case WALL_FOLLOWING:
      updateLCD(frontDist, rightDist);




      if (!isFrontClear()) {
        stopMotors();
        currentState = TURN_LEFT;
      }
      // else if (millis() - lastTurnTime > 500 && isRightCornerDetected()) {
      //   stopMotors();
      //   currentState = TURN_RIGHT;
      // }
      else {
        rightWallFollowing();
      }
      break;




    case TURN_RIGHT:
      updateLCD(frontDist, rightDist);
      stopMotors();
      delay(200);




      turnRight90();
      heading -= 90;




      // Pledge idea:
      // leave wall-following only when the robot's net heading is back to 0.
      if (heading == 0) {
        currentState = FORWARD;
        moveForward();
        forwardStartTime = millis();
      }
      else {
        currentState = WALL_FOLLOWING;
        moveForward();
      }
      break;
  }




