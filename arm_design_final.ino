/*
 ************************************************************************************
 * Mechanismic Inc.
 *
 * Created 2023.08.24
 * By Sam Cao
 * Last Modified 08/27/2023
 *
 *
 * This project is a fun entertainment project similar to a claw arm, 
 * which is made up of one motor and two servos.
 *
 * https://snappyxo.io/

 ************************************************************************************
 */


#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <Servo.h>

// two servos 
Servo head;
Servo body;

// bluetooth connection
const int BLUETOOTH_TX = 7;
const int BLUETOOTH_RX = 8;

SoftwareSerial softSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(softSerial);

// parameters definition
const int HEAD_MIN_ANGLE = 0;
const int HEAD_MAX_ANGLE = 180;
const int BODY_MIN_ANGLE = 0;
const int BODY_MAX_ANGLE = 180;

// -------------------------------------- motor start --------------------------------
const int MOTOR_CONTROL_PIN1 = 4; // pin for motor control (IN1)
const int MOTOR_CONTROL_PIN2 = 5; //  pin for motor control (IN2)
const int MOTOR_ENABLE_PIN = 6;   //  pin for motor enable (ENA)

void setPins() {
  // Set pins as input or output
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_CONTROL_PIN1, OUTPUT);
  pinMode(MOTOR_CONTROL_PIN2, OUTPUT);
}

void motorSetForward() {
  digitalWrite(MOTOR_CONTROL_PIN1, LOW);
  digitalWrite(MOTOR_CONTROL_PIN2, HIGH);
}
void motorBrake() {
	digitalWrite(MOTOR_ENABLE_PIN, LOW);
	digitalWrite(MOTOR_CONTROL_PIN1, LOW);
	digitalWrite(MOTOR_CONTROL_PIN2, LOW);
	digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}
// ----- overall horizontal movements
void fund_move() {
  int move = phone.getThrottle() - 49;
  if (move > 0) {
    // Forward the motor works
    motorSetForward();
  }
  if (move == 0) {
		// If move is zero, don't move.
		motorBrake();
		return;
	}
}
// -------------------------------------- motor end --------------------------------
void setup() {
  delay(500);
  softSerial.begin(9600);
  Serial.begin(9600);
  setPins();
  head.attach(9); // the 1st servo connection
  body.attach(A0); // the 2nd servo connection
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_CONTROL_PIN1, OUTPUT);
  pinMode(MOTOR_CONTROL_PIN2, OUTPUT);
}

void loop() {
  fund_move(); // Call the driveControl function here

  int sliderId = phone.getSliderId();
  int sliderVal = phone.getSliderVal() * 1.8;
  // assign the slider to 0
  if (sliderId == 0) {
    // Limit the head servo angle
    int headAngle = constrain(sliderVal, HEAD_MIN_ANGLE, HEAD_MAX_ANGLE);
    head.write(headAngle);
    Serial.println(headAngle);
  }
  // assign the slider to 1 
  if (sliderId == 1) {
    int bodyAngle = constrain(sliderVal, BODY_MIN_ANGLE, BODY_MAX_ANGLE);
    body.write(bodyAngle);
    Serial.println(bodyAngle);
  }

  // Add a small delay to prevent rapid servo movements and reduce interference
  delay(50);
}
