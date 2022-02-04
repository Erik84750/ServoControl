/*
This mod has the following changes:
1. use pwm.setPWM instead of pwm.writeMicroseconds. More accurate timing, less comuting
2. servo movement slowed to realistic, railroad-like, behaviour.

*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CMRI.h>
#include <Auto485.h>

#define CMRI_ADDR 1 // this also is the JMRI CMRI SMINI node address
// In JMRI this gives an address 1001 to the first servo on the PCA9685 address 0
// Configure a turnout as 1 bit and steady state
// Meaning: bit = 0 = (for example) thrown and bit = 1 = closed
#define DE_PIN 2
#define numServos 8 //The number of servos connected

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //setup the board address 0
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 24, 48, bus);

int Status[numServos]; //Create a table to hold the status of each turnout, signal, etc.
int Throw[numServos]; //Create a table to hold the throw value for each servo
int Close[numServos]; //Create a table to hold the close value for each servo
int i;
uint16_t pulselgt;
unsigned int servoMin[] = {300, 300, 200, 246, 246, 172, 246, 200, 200, 200}; //servo lower setting to suit individual servo
unsigned int servoMax[] = {450, 450, 500, 492, 492, 565, 492, 550, 550, 525}; //servo upper setting to suit individual servo
int previousServoPos[numServos];

void setup() {
  Serial.begin(9600);
  bus.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  pulselgt = 307; // about middle pos
  for (i = 0; i < numServos; i++) {
    previousServoPos[i] = 0; // update status to 1 = minimum angle
  }
  for (i = 0; i < numServos; i++) { // initiate servos to minimum angle
    pwm.setPWM(i, 0, pulselgt);
  }
}

void loop() {
  cmri.process();

  for (i = 0; i < numServos; i++) {
    Status[i] = (cmri.get_bit(i));
    if (Status[i] == 1 && previousServoPos[i] == 0) {  // decreasing angle
      for (pulselgt = servoMax[i]; pulselgt > servoMin[i]; pulselgt--) {
        pwm.setPWM(i, 0, pulselgt);
        delay(3);
        //Serial.print("decr pulselgt  = ");
        //Serial.println(pulselgt);
      }
      //Serial.print("final pulselgt decrease = ");
      //Serial.println(pulselgt);
      //Serial.println();
      previousServoPos[i] = 1; // update last servoposition to current servoposition
    }
    if (Status[i] == 0 && previousServoPos[i] == 1) {  // increasing angle
      for (pulselgt = servoMin[i]; pulselgt < servoMax[i]; pulselgt++) {
        pwm.setPWM(i, 0, pulselgt);
        delay(3);
        //Serial.print("incr pulselgt  = ");
        //Serial.println(pulselgt);
        //Serial.println();
        //Serial.println();
      }
      //Serial.print("final pulselgt increase = ");
      //Serial.println(pulselgt);
      previousServoPos[i] = 0; // update last servoposition to current servoposition
    }
  }
}
