#include "ClawTINAH.h"

    
Claw::Claw(int clawOpenPosition, int clawClosePosition, int armUpPosition, int clawStepDelay,
int armStepDelay, int clawPin, int armPin) {
    //open the claw. As well as set up all the various constants.
    //ask TA about attaching multiple Servos
    p_clawOpenPosition = clawOpenPosition;
    p_clawClosePosition = clawClosePosition;
    p_armUpPosition = armUpPosition;
    p_clawStepDelay = clawStepDelay;
    p_armStepDelay = armStepDelay;

     //servos
    /*ServoTINAH p_claw;
    ServoTINAH p_arm;*/
    p_claw = new ServoTINAH;
    p_arm = new ServoTINAH;
    
    p_claw->attach(clawPin);
    p_arm->attach(armPin);

    // opening the claw and making sure the arm is initially retracted
    p_claw->write(p_clawOpenPosition);
    p_arm->write(p_armUpPosition);
}

Claw::~Claw() {
    delete p_claw;
    delete p_arm;
    p_claw = NULL;
    p_arm = NULL;
}

void Claw::retrieve(int armDownPosition){
    //retrieve motion
    p_armDownPosition = armDownPosition;


    
    // grabbing the agent
    this->armDown();
    this->clawClose();

    delay(1000);
    //dropping into the box
    this->armUp();
    this->clawOpen();
}

void Claw::clawSetUp(){

    // set up the servos to be at initial position
    // do not finish assembly to assemble.

     p_claw->write(p_clawOpenPosition);
    p_arm->write(p_armUpPosition);

    LCD.clear();
    LCD.home();
    LCD.print("ArmUp:"); LCD.print(p_armUpPosition); 
    delay(1000);
    LCD.setCursor(0,1);
    LCD.print("clawOpen"); LCD.print(p_clawOpenPosition);
    delay(1000);
     
}


void Claw::clawOpen() {
    //opening the claw
    moveServo(p_claw, p_clawClosePosition, p_clawOpenPosition, p_clawStepDelay);
}

void Claw::clawClose(){
    //closing the claw from opened position
    moveServo(p_claw, p_clawOpenPosition, p_clawClosePosition, p_clawStepDelay);
}

void Claw::armDown(){
    // moving the arm down
    moveServo(p_arm,p_armUpPosition, p_armDownPosition, p_armStepDelay);
}

void Claw::armUp(){
    //moving the arm up
    moveServo(p_arm, p_armDownPosition, p_armUpPosition, p_armStepDelay);
}

// internal invokable function
void moveServo(ServoTINAH *servo, int startPos, int endPos, int stepDelayMs) {
    int diff = endPos - startPos;
    
    int x = 0;
    while (diff != x) {
        x += (diff > 0) ? 1 : -1;
        servo->write(startPos + x);
        delay(stepDelayMs);
        LCD.print(startPos + x);
        LCD.setCursor(0,1);
        LCD.print(diff);
        delay(10);
        LCD.clear();
    }
    delay(2000);
}
