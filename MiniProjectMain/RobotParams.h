const unsigned long minInterruptTime = 10;
const float pi = 3.1415;

struct Motor{
  // Pin Variables
  uint8_t dirPin;
  uint8_t speedPin;
  uint8_t encAPin;
  uint8_t encBPin;

  // Other necissary variables
  volatile long encoderTurns = 0;
  unsigned long lastIntTime;

  void initPins(){
    pinMode(dirPin, OUTPUT);
    pinMode(speedPin, OUTPUT);
    pinMode(encAPin, INPUT);
    pinMode(encBPin, INPUT);
  }
  float turnsInRadians(){
    return (2*pi*(float)encoderTurns / 3200);
  }
};

struct Robot{
  // Physical properties of the robot build
  const float b = 0.215; // 21.5cm
  const float r = 0.075; // 7.5cm
  const float wheelCir = 2*r*pi;
  const uint8_t MOTOR_ENBPin = 4;


  Motor MOTOR_L;
  Motor MOTOR_R;

  // Robot Info Variables
  float xPos = 0;
  float yPos = 0;
  float phi = 0;
  float dL = 0;
  float dR = 0;

  void initRobot(){
    MOTOR_L.initPins();
    MOTOR_R.initPins();
    pinMode(MOTOR_ENBPin, OUTPUT);
    enableMotors();
  }

  void enableMotors(){
    digitalWrite(MOTOR_ENBPin, HIGH);
  }
  void disableMotors(){
    digitalWrite(MOTOR_ENBPin, LOW);
  }

  void updatePosition(){
    float newdL = (MOTOR_L.turnsInRadians() / (2*pi)) * wheelCir;
    float newdR = (MOTOR_R.turnsInRadians() / (2*pi)) * wheelCir;

    float xNew = xPos + cos(phi)*((dL - newdL) + (dR - newdR)) / 2;
    float yNew = yPos + sin(phi)*((dL - newdL) + (dR - newdR)) / 2;
    float phiNew = phi + (1 / b)*((dL - newdL) - (dR - newdR));

    dL = newdL;
    dR = newdR;
    xPos = xNew;
    yPos = yNew;
    phi = phiNew;
  }
};


