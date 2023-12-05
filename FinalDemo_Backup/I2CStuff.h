#include <Wire.h>

#define MY_ADDR 8

#define DIST_OFFSET 0.091  // Meters
#define ANGLE_OFFSET 0.0  // Radians

// Function Prototypes
void printReceived();
void receive();
void request();

struct RpiData {
  bool isSearching = true;
  uint8_t rawDistance = 0;
  int8_t rawAngle = 0;

  float getDist() {
    float tempDist = (float)rawDistance;
    // Convert marker distance from centimeters to Meters
    tempDist = tempDist / 100.0f;
    tempDist -= DIST_OFFSET;


    if(isSearching) tempDist = 0;

    return tempDist;
  }

  float getAngle() {
    float tempAngle = (float)rawAngle;
    tempAngle = tempAngle * (pi / 180.0f);
    tempAngle -= ANGLE_OFFSET;

    if(isSearching) tempAngle = 0;

    return tempAngle;
  }

  void printData() {
    Serial.print("Is Searching: ");
    Serial.print(isSearching);
    Serial.print(", Angle(Radians): ");
    Serial.print(getAngle());
    Serial.print(", Distance(Feet): ");
    Serial.print(getDist() / 0.308f);
    Serial.print(", Distance(Meters): ");
    Serial.println(getDist());
  }

  float calcDesiredRot(float perpDist, float botLength) {
    float desiredRot;
    if (!isSearching) {
      desiredRot = (atan(perpDist / (getDist() + botLength)) - getAngle());
    } else {
      desiredRot = (pi / 8);
    }

    return desiredRot;
  }
};

// I2C Variables
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = { 0 };
volatile uint8_t msgLength = 0;

// Wrapper holding the data that is sent by the Raspberry Pi
volatile RpiData recentRpiData;

void initI2C() {
  // Begin and set device address
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void request() {
  uint8_t reply = (uint8_t(instruction[0]) + uint8_t(100));
  Wire.write(reply);
}

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }

  if (msgLength == 4) {
    recentRpiData.isSearching = instruction[1];
    recentRpiData.rawAngle = instruction[2];
    recentRpiData.rawDistance = instruction[3];
  }
  msgLength = 0;
}

void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");
  for (int i = 0; i < msgLength; i++) {
    Serial.print(String(instruction[i]) + "\t");
  }
}