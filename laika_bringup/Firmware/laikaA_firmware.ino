#include <Wire.h>

#define I2C_ADDR 0x8

#define LDIR 0
#define LPWM 1
#define RDIR 2
#define RPWM 3
#define LENC 0
#define RENC 1

// Message Arrays
unsigned int incomingMsg[4];
unsigned long outgoingMsg[2];
bool dataReceived = false;

// Motor controller pins
int leftWheelDir = 8;
int leftWheelPwm = 9;
int rightWheelDir = 10;
int rightWheelPwm = 11;

// Encoder pins
const byte leftWheelEncoderPin = 3;
const byte rightWheelEncoderPin = 2;

// Wheel Encoder Counts
unsigned long leftCurrentEnc = 0;
unsigned long rightCurrentEnc = 0;

void leftWheelUpdate() {
  // Increment if encoder hits
  leftCurrentEnc++;
}
void rightWheelUpdate() {
  // Increment if encoder hits
  rightCurrentEnc++;
}

// Receive event 
// Gets four ints from I2C
void receiveEvent(int howMany) {
  if(howMany == 8){
    uint8_t buf[8];
    for(int i = 0; i < 8; i++){
      buf[i] = Wire1.read();
    }
    memcpy(&incomingMsg, buf, 8);
    dataReceived = true;
  }
  else{
    while(Wire1.available()) Wire.read();
  }
}

// Request event
// Sends two longs to I2C
void requestEvent(){
  outgoingMsg[LENC] = leftCurrentEnc;
  outgoingMsg[RENC] = rightCurrentEnc;

  Wire1.write((uint8_t *)&outgoingMsg, 2*sizeof(long));
}


void setup() {

  // Setup
  pinMode(leftWheelEncoderPin, INPUT);
  pinMode(leftWheelDir, OUTPUT);
  pinMode(leftWheelPwm, OUTPUT);
  pinMode(rightWheelEncoderPin, INPUT);
  pinMode(rightWheelDir, OUTPUT);
  pinMode(rightWheelPwm, OUTPUT);

  // I2C Setup
  Wire1.begin(I2C_ADDR);
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);

  // Interrupt setup
  attachInterrupt(digitalPinToInterrupt(leftWheelEncoderPin), leftWheelUpdate, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightWheelEncoderPin), rightWheelUpdate, FALLING);
}

void loop() {

  // Main loop
  if(dataReceived) {
    digitalWrite(leftWheelDir, incomingMsg[LDIR]); 
    digitalWrite(leftWheelPwm, incomingMsg[LPWM]);
    digitalWrite(leftWheelDir, incomingMsg[RDIR]); 
    digitalWrite(leftWheelPwm, incomingMsg[RPWM]);

    dataReceived = false;

  }
}