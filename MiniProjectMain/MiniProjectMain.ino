#include "RobotParams.h"

#define MOTOR_ONE_ENCA 2
#define MOTOR_ONE_ENCB 5

#define MOTOR_TWO_ENCA 3
#define MOTOR_TWO_ENCB 6

MotorShield myShield;

unsigned long desired_Ts = 10;
unsigned long lastSampleTime = 0;

// Function Prototypes
void encoderOneISR();
void encoderTwoISR();

void setup() {
  // put your setup code here, to run once:
  myShield.initShield();

  attachInterrupt(digitalPinToInterrupt(MOTOR_ONE_ENCA), encoderOneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_TWO_ENCA), encoderTwoISR, CHANGE);


  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  if(millis() >= lastSampleTime + desired_Ts){
    myShield.MotorOne.controllerUpdate(desired_Ts);
    myShield.MotorTwo.controllerUpdate(desired_Ts);
    lastSampleTime = millis();
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