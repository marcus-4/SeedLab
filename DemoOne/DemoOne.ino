#include "Motor.h"
#include "I2CStuff.h"

// Function Prototypes
void leftEncoderISR();
void rightEncoderISR();
void turnToAngle();
void moveDistance();
bool readyToMove();

// Global Variables
unsigned long lastSampleTime = 0;  // Tracks the last time the controllers were updated

float desiredPos = 0;
float desiredTheta = pi;

float prevError = 0;
float integralError = 0;
float desiredForVelocity = 0; // meters per second.
//float desiredRotVelocity = 0;
float lastRads_L = 0;
float lastRads_R = 0;

Motor Motor_L = Motor(MOTOR_LEFT_DIR, MOTOR_LEFT_SPEED, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
Motor Motor_R = Motor(MOTOR_RIGHT_DIR, MOTOR_RIGHT_SPEED, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);


void setup() {
  // Initialize the motor shield
  pinMode(BOARD_ENB_PIN, OUTPUT);
  digitalWrite(BOARD_ENB_PIN, HIGH);
  pinMode(BOARD_SF_PIN, INPUT);

  // Initialize both motors
  Motor_L.init();
  Motor_R.init();

  Motor_L.setDesiredPosition(0);
  Motor_R.setDesiredPosition(0);

  Motor_L.flip();  // 50% chance the left motor needs to be flipped instead

  // All the I2C stuff is stored in a separate header file
  //initI2C();

  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENCA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENCA), rightEncoderISR, CHANGE);

  Serial.begin(9600);

  delay(2000);  // Keeps things from freaking out if power is only temporarily applied when we do not actually want to run the program

  lastSampleTime = millis();
  lastRads_L = Motor_L.getPosInRadians();
  lastRads_R = Motor_R.getPosInRadians();
}

void loop() {
  if(millis() >= lastSampleTime + desired_Ts){
    // Gather Information Needed for Feedback path
    float actualTs = (float)(millis() - lastSampleTime) / 1000.0f;
    lastSampleTime = millis();
    float rads_L = Motor_L.getPosInRadians();
    float rads_R = Motor_R.getPosInRadians();
    float angVel_L = (rads_L - lastRads_L) / actualTs;
    float angVel_R = (rads_R - lastRads_R) / actualTs;

    /*
      Rotation Control
    */

    float theta = r * ((rads_L - rads_R) / d);
    float thetaError = desiredTheta - theta;

    float derivativeError = (thetaError - prevError) / actualTs;
    prevError = thetaError;

    integralError += thetaError * actualTs;

    float desiredRotVelocity = (thetaError * KDP) + (derivativeError * KD) + (integralError * KI);
    

    /*
      Begin Velocity Controllers
    */

    Serial.println(thetaError);

    // Forward Velocity Calculation
    float botVelocity = r * ((angVel_L + angVel_R) / 2.0f);
    float velError = desiredForVelocity - botVelocity;

    // Rotational Velocity Calculation
    float botRotVelocity = r * ((angVel_L - angVel_R) / d);
    float rotError = desiredRotVelocity - botRotVelocity;


    // Proportional Controller Implementation
    float Va = KP * velError;
    float delVa = KP * rotError;
    
    // Split Va into left and right motor signals
    float V1 = (Va + delVa) / 2.0f;
    float V2 = (Va - delVa) / 2.0f;

    // Keep it slow to prevent slipping
    if (V1 > VOLT_CAP) V1 = VOLT_CAP;
    if (V1 < -VOLT_CAP) V1 = -VOLT_CAP;

    if (V2 > VOLT_CAP) V2 = VOLT_CAP;
    if (V2 < -VOLT_CAP) V2 = -VOLT_CAP;

    Motor_L.setVoltage(V1*FF_L);
    Motor_R.setVoltage(V2*FF_R);

    lastRads_L = rads_L;
    lastRads_R = rads_R;
  }
}

void leftEncoderISR() {
  int newA = digitalRead(MOTOR_LEFT_ENCA);
  int newB = digitalRead(MOTOR_LEFT_ENCB);

  // Help Debounce by not running the ISR if not enough time has passed
  if (micros() > (Motor_L.lastIntTime + minInterruptTime)) {
    if (newA == newB) {
      if (Motor_L.isFlipped) Motor_L.encoderTurns -= 2;
      if (!Motor_L.isFlipped) Motor_L.encoderTurns += 2;
    } else {
      if (Motor_L.isFlipped) Motor_L.encoderTurns += 2;
      if (!Motor_L.isFlipped) Motor_L.encoderTurns -= 2;
    }
  }
  Motor_L.lastIntTime = micros();
}

void rightEncoderISR() {
  int newA = digitalRead(MOTOR_RIGHT_ENCA);
  int newB = digitalRead(MOTOR_RIGHT_ENCB);

  // Help Debounce by not running the ISR if not enough time has passed
  if (micros() > (Motor_R.lastIntTime + minInterruptTime)) {
    if (newA == newB) {
      if (Motor_R.isFlipped) Motor_R.encoderTurns -= 2;
      if (!Motor_R.isFlipped) Motor_R.encoderTurns += 2;
    } else {
      if (Motor_R.isFlipped) Motor_R.encoderTurns += 2;
      if (!Motor_R.isFlipped) Motor_R.encoderTurns -= 2;
    }
  }
  Motor_R.lastIntTime = micros();
}