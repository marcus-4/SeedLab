#include "Motor.h"
#include "I2CStuff.h"

#define SEARCHING_ROT_VEL pi / 8
#define APPROACHING_FOR_VEL 0.4
#define ROT_ERROR_THRESHOLD 0.001
#define ROT_ERROR_THRESHOLD_90 0.2
#define BUZZER_DELAY 250

enum OperatingMode {
  DEBUG_PRINT_ENCODER,
  DEBUG_PRINT_I2C,
  DEBUG_CONSTANT_VOLTAGE,
  DEBUG_CONSTANT_FORWARD_VELOCITY,
  DEBUG_CONSTANT_ROTATIONAL_VELOCITY,
  DEBUG_CONSTANT_FORWARD_POSITION,
  DEBUG_CONSTANT_ROTATIONAL_POSITION,
  DEBUG_CONSTANT_CIRCLE,
  DEBUG_TEST_SEQUENCE,
  DEMO_TYPE_ONE,
  DEMO_TYPE_TWO
};

enum DemoState {
  SEARCHING,
  APPROACHING_TURN,
  APPROACHING_FORWARD,
  TURNING_90,
  CIRCLING,
  END
};



// Function Prototypes
void leftEncoderISR();
void rightEncoderISR();
float feetToMeters(float ft);
void resetControlLoop();

/*
  ---------- Global Variables ----------
*/

// Current States of the Bot
OperatingMode currentMode = DEMO_TYPE_TWO;
DemoState currentState = SEARCHING;

// Debug variables used in the various debug modes
const float desiredVoltage_DEBUG = 2;
const float desiredForVelocity_DEBUG = feetToMeters(0.5);      // Meters per second
const float desiredRotVelocity_DEBUG = pi / 4;                 // Radians per second
const float desiredPos_DEBUG = feetToMeters(2);                // Meters
const float desiredRot_DEBUG = (3.0f * (pi / 2.0f)) + FF_ROT;  // Radians
const float desiredCircleRadius_DEBUG = feetToMeters(1);       // Meters radius
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

unsigned long lastStateEndTime = 0;
const float circleRadius = feetToMeters(1.5);
const float desiredCircleVelocity = feetToMeters(0.7);

bool searchingRpi = true;
float distRpi = 0;
float angRpi = 0;
float desiredPos = 0;


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
  float desiredRot;
  float currentDist;

  Serial.println(currentState);

  if (msgLength > 0) {
    if (msgLength == 4) {
      //Serial.println("4 Bytes recieved");
      searchingRpi = instruction[1];

      int8_t signedAngle = instruction[2];
      angRpi = (float)signedAngle;
      angRpi = angRpi * (pi / 180.0f);  // Convert to radians

      distRpi = (float)instruction[3];
      distRpi = distRpi / 100.0f;  // Convert to meters
    }
    // Serial.print(searchingRpi);
    // Serial.print("    ");
    // Serial.print(distRpi);
    // Serial.print("    ");
    // Serial.println(angRpi);
    msgLength = 0;
  }


  switch (currentMode) {
    case DEBUG_CONSTANT_FORWARD_VELOCITY:
      desiredRot = 0;
      break;

    case DEBUG_CONSTANT_ROTATIONAL_POSITION:
      desiredRot = desiredRot_DEBUG;
      break;

    case DEBUG_CONSTANT_CIRCLE:
      desiredRot = 0;
      desiredPos = feetToMeters(2) * pi;
      currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
      if (currentDist >= (desiredPos + FF_CIRCLE_DIST)) {
        resetControlLoop();
        //currentMode = DEBUG_PRINT_I2C;
        tone(BUZZER_PIN, 220);
        delay(BUZZER_DELAY);
        noTone(BUZZER_PIN);
        resetControlLoop();
      }
      break;

      /*
      A test sequence to make sure the robot can handle sequential instructions
    */
      // case DEBUG_TEST_SEQUENCE:
      //   switch (currentState) {
      //     case SEARCHING:
      //       currentState = APPROACHING_TURN;
      //       desiredRot = (3.0f * (pi / 2.0f)) + FF_ROT;
      //       desiredPos = 0;
      //       lastStateEndTime = millis();
      //       break;

      //     case APPROACHING_TURN:
      //       desiredRot = (3.0f * (pi / 2.0f)) + FF_ROT;
      //       desiredPos = 0;

      //       if (millis() < lastStateEndTime + 2000) break;

      //       if (prevError_ROT < ROT_ERROR_THRESHOLD) {
      //         desiredRot = 0;
      //         desiredPos = 0;
      //         resetControlLoop();
      //         currentState = APPROACHING_FORWARD;
      //         tone(BUZZER_PIN, 220);
      //         delay(1500);
      //         noTone(BUZZER_PIN);
      //         resetControlLoop();
      //       }
      //       break;

      //     case APPROACHING_FORWARD:
      //       desiredRot = 0;
      //       desiredPos = feetToMeters(2);
      //       currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
      //       if (currentDist >= desiredPos) {
      //         desiredRot = 0;
      //         desiredPos = 0;
      //         resetControlLoop();
      //         currentState = TURNING_90;
      //         tone(BUZZER_PIN, 220);
      //         delay(1500);
      //         noTone(BUZZER_PIN);
      //         resetControlLoop();
      //         desiredRot = (pi / 2.0f);
      //         desiredPos = 0;
      //         lastStateEndTime = millis();
      //       }
      //       break;

      //     case TURNING_90:
      //       desiredRot = (pi / 2.0f);
      //       desiredPos = 0;

      //       if (millis() < lastStateEndTime + 2000) break;

      //       if (prevError_ROT < ROT_ERROR_THRESHOLD_90) {
      //         desiredRot = 0;
      //         desiredPos = 0;
      //         resetControlLoop();
      //         currentState = CIRCLING;
      //         tone(BUZZER_PIN, 220);
      //         delay(1500);
      //         noTone(BUZZER_PIN);
      //         resetControlLoop();
      //       }
      //       break;

      //     case CIRCLING:
      //       desiredRot = 0;
      //       desiredPos = feetToMeters(2) * pi;
      //       currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
      //       if (currentDist >= desiredPos) {
      //         desiredRot = 0;
      //         desiredPos = 0;
      //         resetControlLoop();
      //         currentState = END;
      //         tone(BUZZER_PIN, 220);
      //         delay(1500);
      //         noTone(BUZZER_PIN);
      //         resetControlLoop();
      //         lastStateEndTime = millis();
      //       }
      //       break;

      //     case END:
      //       desiredRot = 0;
      //       desiredPos = 0;
      //       break;
      //   }
      //   break;

    case DEMO_TYPE_ONE:
      switch (currentState) {
        case SEARCHING:
          if (!searchingRpi) {
            // The RPi has found the Ruko maker and sent a distance and angle.
            desiredPos = distRpi;
            desiredRot = angRpi + FF_RUKO_ANGLE;

            resetControlLoop();
            currentState = APPROACHING_FORWARD;
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();
            lastStateEndTime = millis();
            desiredRot = angRpi + FF_RUKO_ANGLE;
            desiredPos = distRpi - feetToMeters(0.5) - 0.08;
          }
          break;
        case APPROACHING_TURN:

          if (millis() < lastStateEndTime + 2000) break;

          if (prevError_ROT < ROT_ERROR_THRESHOLD) {
            desiredRot = 0;
            //desiredPos = distRpi;  // Update value here or keep original?
            resetControlLoop();
            currentState = APPROACHING_FORWARD;
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();
            lastStateEndTime = millis();
            desiredPos = distRpi - feetToMeters(0.5) - 0.04;
            Serial.println(distRpi);
          }
          break;
        case APPROACHING_FORWARD:
          desiredRot = 0;
          if (millis() < lastStateEndTime + 1000) break;

          float currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
          Serial.print(currentDist);
          Serial.print("     ");
          Serial.println(desiredPos);
          if (currentDist >= desiredPos) {
            resetControlLoop();
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            currentState = END;
          }
          break;
        case END:
          desiredPos = 0;
          desiredRot = 0;
          break;
      }
      break;

    case DEMO_TYPE_TWO:

      switch (currentState) {

        case TURNING_90:
          desiredRot = pi / 2.0f;
          if (millis() < lastStateEndTime + 500) break;

          Serial.println(prevError_ROT);

          if (abs(prevError_ROT) < ROT_ERROR_THRESHOLD_90) {
            desiredRot = 0;
            //desiredPos = distRpi;  // Update value here or keep original?
            resetControlLoop();
            currentState = CIRCLING;
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();
            lastStateEndTime = millis();
            desiredPos = 2 * circleRadius * pi + FF_CIRCLE_DIST;
            Serial.println(distRpi);
          }
          break;

        case CIRCLING:
          desiredRot = 0;
          desiredPos = 2 * circleRadius * pi + FF_CIRCLE_DIST;
          currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
          if (currentDist >= (desiredPos + FF_CIRCLE_DIST)) {
            resetControlLoop();
            currentState = END;
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();
          }
          break;

        case SEARCHING:
          if (!searchingRpi) {
            // The RPi has found the Ruko maker and sent a distance and angle.
            desiredPos = distRpi;
            desiredRot = angRpi + FF_RUKO_ANGLE;

            resetControlLoop();
            currentState = APPROACHING_FORWARD;
            tone(BUZZER_PIN, 220);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();
            lastStateEndTime = millis();
            desiredRot = angRpi + FF_RUKO_ANGLE;
            desiredPos = distRpi - feetToMeters(0.5) - 0.03;
          }
          break;

        case APPROACHING_FORWARD:
          desiredRot = 0;
          if (millis() < lastStateEndTime + 1000) break;

          float currentDist = C * (((lastRads_L + lastRads_R) / 2.0f) / (2 * pi));
          // Serial.print(currentDist);
          // Serial.print("     ");
          // Serial.println(desiredPos);
          if (currentDist >= desiredPos) {
            resetControlLoop();
            tone(BUZZER_PIN, 880);
            delay(BUZZER_DELAY);
            noTone(BUZZER_PIN);
            resetControlLoop();

            currentState = TURNING_90;
            desiredRot = pi / 2.0f;
            lastStateEndTime = millis();
          }

          break;

        case END:
          desiredPos = 0;
          desiredRot = 0;
          break;

        default:
          Serial.println("No State!!");
          break;
      }
      break;

    case DEBUG_PRINT_I2C:
      // Serial.print(searchingRpi);
      // Serial.print("    ");
      // Serial.print(distRpi);
      // Serial.print("    ");
      // Serial.println(angRpi);
      return;
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

    static float desiredForVelocity;
    static float desiredRotVelocity;

    // Position Error Calculations
    float currentRot = r * ((rads_L - rads_R) / d);
    float rotError = desiredRot - currentRot;
    integralError_POS += rotError * actualTs;
    float derivativeError_ROT = (rotError - prevError_ROT) / actualTs;
    prevError_ROT = rotError;

    //Serial.println(desiredRot);

    // Choose a method of finding desired velocity
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

      case DEBUG_CONSTANT_ROTATIONAL_POSITION:
        desiredForVelocity = 0;
        desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
        break;

      case DEBUG_TEST_SEQUENCE:
      case DEMO_TYPE_TWO:
      case DEMO_TYPE_ONE:
        switch (currentState) {
          case SEARCHING:
            desiredForVelocity = 0;
            desiredRotVelocity = SEARCHING_ROT_VEL;
            break;

          case APPROACHING_TURN:
            desiredForVelocity = 0;
            desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
            break;

          case APPROACHING_FORWARD:
            desiredForVelocity = APPROACHING_FOR_VEL;
            desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
            break;

          case TURNING_90:

            desiredForVelocity = 0;
            desiredRotVelocity = (rotError * KP_POS) + (derivativeError_ROT * KD_POS) + (integralError_POS * KI_POS);
            break;

          case CIRCLING:
            desiredForVelocity = desiredCircleVelocity;
            desiredRotVelocity = -(desiredCircleVelocity / circleRadius);
            break;

          case END:
            desiredForVelocity = 0;
            desiredRotVelocity = 0;
            break;
        }
        break;

      default:
        desiredForVelocity = 0;
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

    //Serial.println(rotVelError, 5);

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

    if (currentMode == DEBUG_CONSTANT_VOLTAGE) {
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