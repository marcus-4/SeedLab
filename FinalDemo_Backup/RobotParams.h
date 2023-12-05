/*
  Keeps constants, pin definitions, and controller parameters
*/

// Minimum interrupt time for the encoders (in microseconds)
const unsigned long minInterruptTime = 10;

// The desired update period for the control loop (in milliseconds)
const unsigned long desired_Ts = 25;
const float pi = 3.1415;
const float C = 0.15*pi; // Circumfrence of the wheels we are using, in Meters
const float r = 0.075; // radius of the wheels in meters
const float d = 0.214; // Distance between the two wheels in Meters
const float botLength = 0.28; // 11in

/*
  ------------------ Pin Definitions -------------------
*/

#define BUZZER_PIN 11

#define BOARD_ENB_PIN 4
#define BOARD_SF_PIN 12

#define MOTOR_LEFT_ENCA 2
#define MOTOR_LEFT_ENCB 5
#define MOTOR_LEFT_DIR 7 // M1DIR fixed to pin 7
#define MOTOR_LEFT_SPEED 9 // M1PWM fixed to pin 9

#define MOTOR_RIGHT_ENCA 3
#define MOTOR_RIGHT_ENCB 6
#define MOTOR_RIGHT_DIR 8 // M2DIR fixed to pin 8
#define MOTOR_RIGHT_SPEED 10 // M2PWM fixed to pin 10

/*
  -------------------- Control system parameters --------------------
*/

// PID Controller for forward velocity
#define KP_VA 20
#define KI_VA 40
#define KD_VA 0.2

// PID Controller for rotational velocity
#define KP_DEL_VA 2.6
#define KI_DEL_VA 5
#define KD_DEL_VA 0

// PID controller for angular position
#define KP_POS 1.2
#define KI_POS 0.04
#define KD_POS 0

/*
  -------------------- Miscellaneous Parameters ------------------
*/

#define BATTERY_VOLTAGE 8
#define VOLT_CAP 8
#define VOLT_MIN 0.3

// "Fudge Factors" that can consistently account for small errors
#define FF_LEAN 1.0 // Above one will lean right, below one will lean left

#define FF_DIST 0.02 // In Feet
#define FF_CIRCLE_DIST 0.015
#define FF_ROT 0.35
#define FF_RUKO_ANGLE 0

