#include <Wire.h>

#define MY_ADDR 8

// Function Prototypes
void printReceived();
void receive();
void request();

// I2C Variables
volatile uint8_t offset = 0;
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;

void initI2C(){
  // Begin and set device address
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void request() {
  uint8_t reply = (uint8_t(instruction[0])+uint8_t(100));
  Wire.write(reply);
}

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
    //int8_t signedVersion = instruction[0];
    // Serial.print(msgLength);
    // Serial.print(" : ");
    // Serial.println(instruction[msgLength]);
    // Serial.print("     ");
    // Serial.println(signedVersion);
  }
  //Serial.println(msgLength);
}

void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Instruction received: ");
  for (int i=0;i<msgLength;i++) {
    Serial.print(String(instruction[i])+"\t");
  }
}