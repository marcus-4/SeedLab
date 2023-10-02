#include "RobotParams.h"

MotorShield myShield;

unsigned long desired_Ts = 1;
unsigned long lastSampleTime = 0;

// Function Prototypes
void encoderOneISR();
void encoderTwoISR();

void setup() {
  // put your setup code here, to run once:
  myShield.initShield();

  attachInterrupt(digitalPinToInterrupt(MOTOR_ONE_ENCA), encoderOneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_TWO_ENCA), encoderTwoISR, CHANGE);

  myShield.MotorOne.desiredPos = (-2*pi);
  myShield.MotorTwo.desiredPos = (2*pi);

  Serial.begin(9600);

  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sFlag = digitalRead(MOTOR_SF_PIN);
  if(sFlag == LOW){
    // Oh Shoot! Board Reported a fault
    myShield.disableMotors();
    Serial.println("FAULT DETECTED!!!");
  }

  if(millis() < 5000){
    zeroZero();
  } else if(millis() >= 5000 && millis() < 10000){
    zeroOne();
  } else if(millis() >= 10000 && millis() < 15000){
    oneZero();
  } else {
    oneOne();
  }


  // Serial.print(myShield.MotorOne.getTurnsInRadians());
  // Serial.print("    ");
  // Serial.println(myShield.MotorTwo.getTurnsInRadians());

  if(millis() >= lastSampleTime + desired_Ts){
    float Ts = millis() - lastSampleTime;
    lastSampleTime = millis();
    float velOne = myShield.MotorOne.controllerUpdate(Ts);
    float velTwo = myShield.MotorTwo.controllerUpdate(Ts);
    Serial.print(velOne);
    Serial.print("    ");
    Serial.println(velTwo);
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