#include "Motor.h"
#include "I2CStuff.h"

#define SEARCHING_ROT_VEL pi / 8
#define APPROACHING_FOR_VEL 0.4
#define ROT_ERROR_THRESHOLD 0.001
#define ROT_ERROR_THRESHOLD_90 0.2

enum BotState {
  DEBUG_PRINT_ENCODER,
  DEBUG_PRINT_I2C,
  DEBUG_STOP,
  DEBUG_CONSTANT_VOLTAGE,
  DEBUG_CONSTANT_FORWARD_VELOCITY,
  DEBUG_CONSTANT_ROTATIONAL_VELOCITY,
  DEBUG_CONSTANT_FORWARD_POSITION,
  DEBUG_CONSTANT_ROTATIONAL_POSITION,
  DEBUG_CONSTANT_CIRCLE,
  DEMO_START
};

// Function Prototypes
void leftEncoderISR();
void rightEncoderISR();
float feetToMeters(float ft);
void resetControlLoop();
void transitionNoise();

/* 
   -----------------------------------------
      Global Variables
   -----------------------------------------
*/

// Tracks the curent state of the FSM
BotState currentState = DEBUG_PRINT_ENCODER;

// Objects tracking the two motors
Motor Motor_L = Motor(MOTOR_LEFT_DIR, MOTOR_LEFT_SPEED, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB);
Motor Motor_R = Motor(MOTOR_RIGHT_DIR, MOTOR_RIGHT_SPEED, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB);

// Wrapper holding the data that is sent by the Raspberry Pi
RpiData recentRpiData;

// Debug parameters used in the various debug modes
const float desiredVoltage_DEBUG = 2;                          // Volts
const float desiredForVelocity_DEBUG = feetToMeters(0.5);      // Meters per second
const float desiredRotVelocity_DEBUG = pi / 4;                 // Radians per second
const float desiredPos_DEBUG = feetToMeters(2);                // Meters
const float desiredRot_DEBUG = (3.0f * (pi / 2.0f)) + FF_ROT;  // Radians
const float desiredCircleRadius_DEBUG = feetToMeters(1);       // Meters radius
const float desiredCircleVelocity_DEBUG = feetToMeters(0.5);   // Meters per second

const float circleRadius = feetToMeters(1.5);
const float desiredCircleVelocity = feetToMeters(1.6);

// Timer trackers
unsigned long lastSampleTime = 0;
unsigned long lastStateEndTime = 0;

// Error value trackers
float prevError_ROT = 0;
float prevError_VA = 0;
float prevError_DEL_VA = 0;
float integralError_POS = 0;
float integralError_VA = 0;
float integralError_DEL_VA = 0;

// Wheel radian trackers
float lastRads_L = 0;
float lastRads_R = 0;


/* 
   ----------------------------------------
    Setup
   ----------------------------------------
*/

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

  // Small Startup delay to prevent accidental motion
  delay(1000);

  // Take all initial values before the first loop
  lastSampleTime = millis();
  lastRads_L = Motor_L.getPosInRadians();
  lastRads_R = Motor_R.getPosInRadians();
}


/* 
   ------------------------------------
    Main Loop
   ------------------------------------
*/


void loop() {
  // ----------------- Check for Incoming I2C Data -------------------

  // Check if there is an available message
  if (msgLength > 0) {

    // Ensure that we recieved the expected 4 byte package
    if (msgLength == 4) {

      // Can the Rpi see an Aruko Marker
      recentRpiData.isSearching = instruction[1];

      // Convert signed 8-bit degree value to the marker angle in radians
      int8_t signedAngle = instruction[2];
      float degRpi = (float)signedAngle;
      recentRpiData.lastAngle = degRpi * (pi / 180.0f);  // Convert to radians

      // Convert marker distance from centimeters to Meters
      float distRpi = (float)instruction[3];
      recentRpiData.lastDistance = distRpi / 100.0f;
    }

    // Clear msgLength indicating we have recieved the message
    msgLength = 0;
  }

  //  --------------------- Pre-Control Loop Checks --------------------

  float desiredRot;

  // Each case statement is responsible for
  //      1.) Setting a desiredRot
  //      2.) Deciding when to change states
  //      3.) Any Miscellaneous behavior that runs outside the control loop

  switch (currentState) {

    case DEBUG_PRINT_ENCODER:
      desiredRot = 0;
      Serial.print("L: ");
      Serial.print(Motor_L.getPosInRadians(), 4);
      Serial.print(", R: ");
      Serial.println(Motor_R.getPosInRadians(), 4);
      break;

    case DEBUG_PRINT_I2C:
      desiredRot = 0;
      Serial.print("Is Searching: ");
      Serial.print(recentRpiData.isSearching);
      Serial.print(", Angle(Radians): ");
      Serial.print(recentRpiData.lastAngle);
      Serial.print(", Distance(Meters): ");
      Serial.println(recentRpiData.lastDistance);
      break;

    case DEBUG_CONSTANT_FORWARD_POSITION:
      desiredRot = 0;
      float currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
      if (currentDist >= desiredPos_DEBUG) {
        resetControlLoop();
        transitionNoise();
        resetControlLoop();
        currentState = DEBUG_STOP;
      }
      break;

    case DEBUG_CONSTANT_ROTATIONAL_POSITION:
      desiredRot = desiredRot_DEBUG;
      break;

    case DEBUG_CONSTANT_CIRCLE:
      desiredRot = 0;
      break;

    case DEMO_START:
      break;
  }

  /*
      ------------------- Begin Control Loop ------------------
  */

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

    static float desiredForVelocity;
    static float desiredRotVelocity;

    // Position Error Calculations
    float currentRot = r * ((rads_L - rads_R) / d);
    float rotError = desiredRot - currentRot;
    integralError_POS += rotError * actualTs;
    float derivativeError_ROT = (rotError - prevError_ROT) / actualTs;
    prevError_ROT = rotError;

    // Find Desired Forward Velocity
    switch (currentState) {
      case DEBUG_CONSTANT_FORWARD_POSITION:
      case DEBUG_CONSTANT_FORWARD_VELOCITY:
        desiredForVelocity = desiredForVelocity_DEBUG;
        break;

      case DEBUG_CONSTANT_CIRCLE:
        desiredForVelocity = desiredCircleVelocity_DEBUG;
        break;

      case DEBUG_PRINT_ENCODER:
      case DEBUG_PRINT_I2C:
      case DEBUG_CONSTANT_ROTATIONAL_VELOCITY:
      case DEBUG_CONSTANT_ROTATIONAL_POSITION:
      case DEBUG_STOP:
      default:
        desiredForVelocity = 0;
        break;
    }

    // Find Desired Rotational Velocity
    switch (currentState) {
      // Use PID equation to calculate Rotational Velocity
      case DEBUG_PRINT_ENCODER:
      case DEBUG_PRINT_I2C:
      case DEBUG_STOP:
      case DEBUG_CONSTANT_FORWARD_POSITION:
      case DEBUG_CONSTANT_FORWARD_VELOCITY:
      case DEBUG_CONSTANT_ROTATIONAL_POSITION:
        desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
        break;

      // Set an arbitrary rotational velocity
      case DEBUG_CONSTANT_ROTATIONAL_VELOCITY:
        desiredRotVelocity = desiredRotVelocity_DEBUG;
        break;

      // Find a constant rotational velocity from radius and forward velocity parameters
      case DEBUG_CONSTANT_CIRCLE:
        desiredRotVelocity = -(desiredCircleVelocity_DEBUG / desiredCircleRadius_DEBUG);
        break;

      default:
        desiredRotVelocity = 0;
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

    // Final check if we are in the constant voltage debug state
    if (currentState == DEBUG_CONSTANT_VOLTAGE) {
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

/* 
  -----------------------------------
    Extra Functions
  -----------------------------------
*/

float feetToMeters(float ft) {
  return (ft * 0.3048);
}

// Plays a tone indicating a change in states
void transitionNoise() {
}

// Cleaner function to be called in between steps in the state machine, ie reset to inital conditions
void resetControlLoop() {
  Motor_L.setVoltage(0);
  Motor_R.setVoltage(0);

  Motor_L.reset();
  Motor_R.reset();

  prevError_ROT = 0;
  prevError_VA = 0;
  prevError_DEL_VA = 0;
  integralError_POS = 0;
  integralError_VA = 0;
  integralError_DEL_VA = 0;

  lastRads_L = 0;
  lastRads_R = 0;

  lastSampleTime = millis();
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
