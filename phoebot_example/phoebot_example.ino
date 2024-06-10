#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "Timer.h"

//------------------------------------------------------------------------------
// Variables
Timer moveTimer = Timer();

// See: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino
// MOTORS - Create the motor shield object with the default I2C address
Adafruit_MotorShield _AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* _motorL = NULL;
Adafruit_DCMotor* _motorR = NULL;

uint8_t _defForwardPWR = 120;
uint32_t wiggleTime = 1500;
uint8_t wiggleState = 0;

//------------------------------------------------------------------------------
// Setup
void setup(){
    // Start the serial
    Serial.begin(115200);
    Serial.println(F("Phoebot simple motor example\n"));

    // Start the motor shield object
    _AFMS.begin();  // create with the default frequency 1.6KHz
    // M1 is the right motor, M2 is the left motor
    _motorR = _AFMS.getMotor(1);
    _motorL = _AFMS.getMotor(2);
    // Set the speed to start, from 0 (off) to 255 (max  speed)
    _motorR->setSpeed(_defForwardPWR);
    _motorR->run(FORWARD);
    _motorR->run(RELEASE);
    _motorL->setSpeed(_defForwardPWR);
    _motorL->run(FORWARD);
    _motorL->run(RELEASE);

    moveTimer.start(0);
}

//------------------------------------------------------------------------------
// Main Loop
void loop(){
    // Update state variable based on time interval
    if(moveTimer.finished()){
        moveTimer.start(wiggleTime);

        if(wiggleState == 0){
            Serial.println(F("Turn left"));
        }
        else if(wiggleState == 1){
            Serial.println(F("Stop"));
        }
        else if(wiggleState == 2){
            Serial.println(F("Turn right"));
        }
        else if(wiggleState == 3){
            Serial.println(F("Stop"));
        }

        wiggleState++;
        if(wiggleState > 3){
            wiggleState = 0;
        }
    }

    // Do an action based on the state flag
    if(wiggleState == 0){
        turnLeft(_defForwardPWR);
    }
    else if(wiggleState == 1){
        stop();
    }
    else if(wiggleState == 2){
        turnRight(_defForwardPWR);
    }
    else if(wiggleState == 3){
        stop();
    }
}

//------------------------------------------------------------------------------
// Functions
void goForward(uint8_t inSpeed){
    _motorL->run(FORWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inSpeed);
    _motorR->setSpeed(inSpeed);
}

void goBack(uint8_t inSpeed){
    _motorL->run(BACKWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inSpeed);
    _motorR->setSpeed(inSpeed);
}

void turnLeft(uint8_t inSpeed){
    _motorL->run(BACKWARD);
    _motorR->run(FORWARD);
    _motorL->setSpeed(inSpeed);
    _motorR->setSpeed(inSpeed);
}

void turnRight(uint8_t inSpeed){
    _motorL->run(FORWARD);
    _motorR->run(BACKWARD);
    _motorL->setSpeed(inSpeed);
    _motorR->setSpeed(inSpeed);

}

void stop(){
    _motorL->run(RELEASE);
    _motorR->run(RELEASE);
    _motorL->setSpeed(0);
    _motorR->setSpeed(0);
}
