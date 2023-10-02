const unsigned long minInterruptTime = 10; // In microseconds
const float pi = 3.1415;

#define MOTOR_ENB_PIN 4
#define MOTOR_SF_PIN 12

#define MOTOR_ONE_ENCA 2
#define MOTOR_ONE_ENCB 5
#define MOTOR_ONE_DIR 7
#define MOTOR_ONE_SPEED 9

#define MOTOR_TWO_ENCA 3
#define MOTOR_TWO_ENCB 6
#define MOTOR_TWO_DIR 8
#define MOTOR_TWO_SPEED 10

#define KP 3
#define KI 8
#define KIP 8
#define BATTERY_VOLTAGE 7.5

struct Motor{
  // Pins
  uint8_t dirPin;
  uint8_t speedPin;
  uint8_t encAPin;
  uint8_t encBPin;

  // Tracking Variables
  volatile long encoderTurns = 0;
  unsigned long lastIntTime;
  float integralError;
  float desiredPos;
  float lastPos;
  
  void initPins(uint8_t _dirPin, uint8_t _speedPin, uint8_t _encAPin, uint8_t _encBPin){
    dirPin = _dirPin;
    speedPin = _speedPin;
    encAPin = _encAPin;
    encBPin = _encBPin;

    pinMode(dirPin, OUTPUT);
    pinMode(speedPin, OUTPUT);
    pinMode(encAPin, INPUT);
    pinMode(encBPin, INPUT);

    digitalWrite(dirPin, HIGH);
    lastPos = getTurnsInRadians();
  }

  void updateTargetPos(float newPos){
    integralError = 0;
    desiredPos = newPos;
  }

  void setSpeed(int newSpeed){
    if(newSpeed > 255) newSpeed = 255;
    if(newSpeed < 0) newSpeed = 0;

    analogWrite(speedPin, newSpeed);
  }

  float controllerUpdate(float ts){
    float currentPos = getTurnsInRadians();
    float currentVelocity = (currentPos - lastPos) / (ts / 1000);

    float posError = desiredPos - currentPos;
    integralError += posError * (ts / 1000.0f);
    float desiredVel = KIP * posError + KI * integralError;

    float velError = desiredVel - currentVelocity;
    float setVoltage = KP * velError;

    if(setVoltage > 0){
      // Forward Direction
      digitalWrite(dirPin, HIGH);
    } else {
      // Backwards Direction
      digitalWrite(dirPin, LOW);
    }

    int setPWM = round(255.0f * (abs(setVoltage) / BATTERY_VOLTAGE));
    setSpeed(setPWM);
    lastPos = currentPos;
    return posError;
  }

  float getTurnsInRadians(){
    return (2*pi*(float)encoderTurns / 3200);
  }
};

struct MotorShield {
  Motor MotorOne;
  Motor MotorTwo;

  void initShield(){
    pinMode(MOTOR_ENB_PIN, OUTPUT);
    pinMode(MOTOR_SF_PIN, INPUT);
    digitalWrite(MOTOR_ENB_PIN, HIGH);

    MotorOne.initPins(MOTOR_ONE_DIR, MOTOR_ONE_SPEED, MOTOR_ONE_ENCA, MOTOR_ONE_ENCB);
    MotorTwo.initPins(MOTOR_TWO_DIR, MOTOR_TWO_SPEED, MOTOR_TWO_ENCA, MOTOR_TWO_ENCB);
  }

  void enableMotors(){
    digitalWrite(MOTOR_ENB_PIN, HIGH);
  }

  void disableMotors(){
    digitalWrite(MOTOR_ENB_PIN, LOW);
  }
};



