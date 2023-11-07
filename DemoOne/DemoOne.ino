#include "Motor.h"
#include "I2CStuff.h"

enum OperatingMode {
  DEBUG_PRINT_ENCODER,
  DEBUG_CONSTANT_VOLTAGE,
  DEBUG_CONSTANT_FORWARD_VELOCITY,
  DEBUG_CONSTANT_ROTATIONAL_VELOCITY,
  DEBUG_CONSTANT_FORWARD_POSITION,
  DEBUG_CONSTANT_ROTATIONAL_POSITION,
  DEBUG_CONSTANT_CIRCLE,
  DEMO_TYPE_ONE,
  DEMO_TYPE_TWO
};

enum DemoTypeTwoStates {
  SEARCHING,
  APPROACHING,
  CIRCLING
};


// Function Prototypes
void leftEncoderISR();
void rightEncoderISR();
float feetToMeters(float ft);

/*
  ---------- Global Variables ----------
*/

// Current Mode of the Bot
OperatingMode currentMode = DEBUG_CONSTANT_CIRCLE;

// Debug variables used in the various debug modes
const float desiredVoltage_DEBUG = 2;
const float desiredForVelocity_DEBUG = feetToMeters(0.5); // Meters per second
const float desiredRotVelocity_DEBUG = pi/4; // Radians per second
const float desiredPos_DEBUG = feetToMeters(2); // Meters
const float desiredRot_DEBUG = pi / 2.0f; // Radians
const float desiredCircleRadius_DEBUG = feetToMeters(1); // Meters radius
const float desiredCircleVelocity_DEBUG = feetToMeters(0.5);

// Tracks the last time the controllers were updated
unsigned long lastSampleTime = 0;

// Error values that need to be tracked between control loop updates
float prevError_ROT = 0;
float prevError_VA = 0;
float prevError_DEL_VA = 0;
float integralError_POS = 0;
float integralError_VA = 0;
float integralError_DEL_VA = 0;

// Each wheel's radian value from the previous control loop update
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

  Motor_L.setFlipped();

  // All the I2C stuff is stored in a separate header file
  initI2C();

  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENCA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENCA), rightEncoderISR, CHANGE);

  Serial.begin(9600);

  delay(2000);  // Keeps things from freaking out if power is only temporarily applied when we do not actually want to run the program

  lastSampleTime = millis();
  lastRads_L = Motor_L.getPosInRadians();
  lastRads_R = Motor_R.getPosInRadians();
}

void loop() {

  // Parameters that define the desired "goals" of the control loop
  static float desiredPos;
  static float desiredRot;
  static float desiredForVelocity;
  static float desiredRotVelocity;

  if(msgLength > 0){
    for(int i = 0; i < msgLength; i++){
      
    }
  }


  switch(currentMode){
    case DEBUG_CONSTANT_FORWARD_VELOCITY:
      desiredRot = 0;
      break;
    case DEBUG_CONSTANT_ROTATIONAL_POSITION:
      desiredRot = desiredRot_DEBUG;
      break;

    case DEBUG_PRINT_ENCODER:
      float rads_L = Motor_L.getPosInRadians();
      float rads_R = Motor_R.getPosInRadians();

      Serial.print(rads_L);
      Serial.print("    ");
      Serial.println(rads_R);
      return;
      break;
  }

  if (millis() >= lastSampleTime + desired_Ts) {
    /*
      ------------------ Gather Feedback Information ---------------------
    */

    float actualTs = (float)(millis() - lastSampleTime) / 1000.0f;
    lastSampleTime = millis();

    float rads_L = Motor_L.getPosInRadians();
    float rads_R = Motor_R.getPosInRadians();

    /*
      ------------- Positional Controllers (Find desired velocities) -----------------
    */

    // Position Error Calculations
    float currentRot = r * ((rads_L - rads_R) / d);
    float rotError = desiredRot - currentRot;
    integralError_POS += rotError * actualTs;
    float derivativeError_ROT = (rotError - prevError_ROT) / actualTs;
    prevError_ROT = rotError;


    switch (currentMode) {
      case DEBUG_CONSTANT_FORWARD_VELOCITY:
        desiredForVelocity = desiredForVelocity_DEBUG;
        desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
        break;

      case DEBUG_CONSTANT_ROTATIONAL_VELOCITY:
        desiredForVelocity = 0;
        desiredRotVelocity = desiredRotVelocity_DEBUG;
        break;

      case DEBUG_CONSTANT_FORWARD_POSITION:
        desiredForVelocity = desiredForVelocity_DEBUG;
        desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
        break;

      case DEBUG_CONSTANT_CIRCLE:
        desiredForVelocity = desiredCircleVelocity_DEBUG;
        desiredRotVelocity = -(desiredCircleVelocity_DEBUG / desiredCircleRadius_DEBUG);
        break;

      // PID Controller to find rotational velocity
      case DEBUG_CONSTANT_ROTATIONAL_POSITION:
      default:
        desiredForVelocity = 0;
        desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
        break;
    }

    /*
      ------------------ Begin Velocity Controllers (Find voltages from velocity) -----------------------
    */

    // Find the angular velocity of each wheel
    float angVel_L = (rads_L - lastRads_L) / actualTs;
    float angVel_R = (rads_R - lastRads_R) / actualTs;

    // Forward Velocity Error Calculations
    float botVelocity = r * ((angVel_L + angVel_R) / 2.0f);
    float velError = desiredForVelocity - botVelocity;
    integralError_VA += velError * actualTs;
    float derivativeError_VA = (velError - prevError_VA) / actualTs;
    prevError_VA = velError;

    // Rotational Velocity Error Calculations
    float botRotVelocity = r * ((angVel_L - angVel_R) / d);
    float rotVelError = desiredRotVelocity - botRotVelocity;
    integralError_DEL_VA += rotVelError * actualTs;
    float derivativeError_DEL_VA = (rotVelError - prevError_DEL_VA) / actualTs;
    prevError_DEL_VA = rotVelError;

    Serial.println(rotVelError,5);


    // PID Controllers to find voltages
    float Va = (KP_VA * velError) + (KI_VA * integralError_VA) + (KD_VA * derivativeError_VA);
    float delVa = (KP_DEL_VA * rotVelError) + (KI_DEL_VA * integralError_DEL_VA) + (KD_DEL_VA * derivativeError_DEL_VA);

    /*
      -------------------- End Controllers --------------------
    */

    // Split Va into left and right motor signals
    float V1 = (Va + delVa) / 2.0f;
    float V2 = (Va - delVa) / 2.0f;

    // Keep applied voltage to a minimum to avoid wheel slippage
    if (V1 > VOLT_CAP) V1 = VOLT_CAP;
    if (V1 < -VOLT_CAP) V1 = -VOLT_CAP;

    if (V2 > VOLT_CAP) V2 = VOLT_CAP;
    if (V2 < -VOLT_CAP) V2 = -VOLT_CAP;

    if(currentMode == DEBUG_CONSTANT_VOLTAGE){
      V1 = desiredVoltage_DEBUG;
      V2 = desiredVoltage_DEBUG;
    }

    Motor_L.setVoltage(V1 * (2 - FF_LEAN));
    Motor_R.setVoltage(V2 * FF_LEAN);

    // Update the lastRads variables for the next time the loop runs
    lastRads_L = rads_L;
    lastRads_R = rads_R;
  }
}

float feetToMeters(float ft) {
  return (ft * 0.3048);
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