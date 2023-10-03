#include "RobotParams.h"
#include <Wire.h>
#define MY_ADDR 8

/*
  SEED Lab: Mini Project 
  Authors: Landon Berg, Rose Ganshert
  Description: This code takes I2C input from the raspberry pi and turns the wheels to different positions.
  Positional control for the motors is implemented using a PI controller. Pins, structs, and constants are declared in the associated
  RobotParams.h header file. 
*/

// Function Prototypes
void encoderOneISR(); // Interrupt for Motor One
void encoderTwoISR(); // Interrupt for Motor Two
void printReceived();
void recieve();
void request();

// Global Variables
MotorShield myShield;
unsigned long desired_Ts = 1;
unsigned long lastSampleTime = 0;

// I2C Variables
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;


void setup() {
  // Runs the initShield method in the struct. Among other things this sets the pinMode for each of the motors
  myShield.initShield();
  attachInterrupt(digitalPinToInterrupt(MOTOR_ONE_ENCA), encoderOneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_TWO_ENCA), encoderTwoISR, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
  Wire.onRequest(request);

  Serial.begin(9600);

  myShield.MotorOne.updateTargetPos(0);
  myShield.MotorTwo.updateTargetPos(0);
  delay(5000);
}

void loop() {
  // Check for Motor Status Flag
  int sFlag = digitalRead(MOTOR_SF_PIN);
  if(sFlag == LOW){
    // Oh Shoot! Board Reported a fault
    myShield.disableMotors();
    Serial.println("FAULT DETECTED!!!");
  }

  if (msgLength > 0) {
    if (offset==1) {
      digitalWrite(LED_BUILTIN,instruction[0]);
    }
    //printReceived();
    for (int i=0;i<msgLength;i++) {
      if(instruction[i] == 1){
        Serial.println("Recieved 1!");
        zeroZero();
      } else if(instruction[i] == 2){
        Serial.println("Recieved 2!");
        zeroOne();
      } else if(instruction[i] == 3){
        Serial.println("Recieved 3!");
        oneOne();
      } else if(instruction[i] == 4){
        Serial.println("Recieved 4!");
        oneZero();
      } else {
        zeroZero();
      }
    }
    msgLength = 0;
  }

  // Serial.print(myShield.MotorOne.getTurnsInRadians());
  // Serial.print("    ");
  // Serial.println(myShield.MotorTwo.getTurnsInRadians());

  // Update the Control Loop
  if(millis() >= lastSampleTime + desired_Ts){
    float Ts = millis() - lastSampleTime;
    lastSampleTime = millis();
    float velOne = myShield.MotorOne.controllerUpdate(Ts);
    float velTwo = myShield.MotorTwo.controllerUpdate(Ts);
    //Serial.print(velOne);
    //Serial.print("    ");
    //Serial.println(velTwo);
  }
}

void zeroZero(){
  myShield.MotorOne.updateTargetPos(0);
  myShield.MotorTwo.updateTargetPos(0);
}

void zeroOne(){
  myShield.MotorOne.updateTargetPos(0);
  myShield.MotorTwo.updateTargetPos(pi);
}

void oneZero(){
  myShield.MotorOne.updateTargetPos(pi);
  myShield.MotorTwo.updateTargetPos(0);
}

void oneOne(){
  myShield.MotorOne.updateTargetPos(pi);
  myShield.MotorTwo.updateTargetPos(pi);
}

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    Serial.print(instruction[0]);
  }
}

void request() {
  uint8_t reply = (uint8_t(instruction[0])+uint8_t(100));
  Wire.write(reply);
}

void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String(instruction[i])+"\t");
  }
}

void encoderOneISR(){
  int newA = digitalRead(MOTOR_ONE_ENCA);
  int newB = digitalRead(MOTOR_ONE_ENCB);
  // Help Debounce by not running the ISR if not enough time has passed
  if (micros() > (myShield.MotorOne.lastIntTime + minInterruptTime)) {

    if (newA == newB) {
      myShield.MotorOne.encoderTurns += 2;
    } else  {
      myShield.MotorOne.encoderTurns -= 2;
    }
  }
  myShield.MotorOne.lastIntTime = micros();
}

void encoderTwoISR(){
  int newA = digitalRead(MOTOR_TWO_ENCA);
  int newB = digitalRead(MOTOR_TWO_ENCB);
  // Help Debounce by not running the ISR if not enough time has passed
  if (micros() > (myShield.MotorTwo.lastIntTime + minInterruptTime)) {

    if (newA == newB) {
      myShield.MotorTwo.encoderTurns += 2;
    } else  {
      myShield.MotorTwo.encoderTurns -= 2;
    }
  }
  myShield.MotorTwo.lastIntTime = micros();
}