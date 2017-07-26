#pragma once
#include <phys253.h>          


class Claw {
public:
    Claw(int clawOpenPosition, int clawClosePosition, int armUpPosition, int clawStepDelay,
int armStepDelay, int clawPin, int armPin);
    void retrieve(int);
    void clawSetUp();
    
    
    Claw(const Claw &);
    Claw();
    ~Claw();
    

  
    
private:
    int p_clawOpenPosition;
    int p_clawClosePosition;
    int p_armUpPosition;
    int p_armDownPosition;
    int p_clawStepDelay;
    int p_armStepDelay;
    ServoTINAH *p_claw;
    ServoTINAH *p_arm;

    void clawOpen();
    void clawClose();
    void armDown();
    void armUp();
};

void moveServo(ServoTINAH *servo, int startPos, int endPos, int stepDelayMs);

