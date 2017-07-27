# pragma once
# include "RetrivalAgent.h"
# include "configuration.h"

//Method BreakDown:
/*

	I have to ask for all dynmically changed variables as inputs.
	
	ExecuteTetrivalAgent: Executes the required looping to catch all the animals.
		upon looping for 6 times, it will return a true value indicating the passing on
		to the next stage. Also make sure to lower the claw before handing over
		So that the claw is not in the way of the lifting mechanism.
		
		
	DetectCrosses: Will return a true when at least 3 QRDs to detect an threshold. IE 
		a cross has been detected
		For now I am going to make it so that only when 4 QRD detect the threshold.

	AlignClaw: This method will align the arm with agents by driving straight forward.
		Keep in mind that even if the motors are set to the same speed in code, the motors naturally
		turn differently. This will be a void method. 
	
	BackToTape: After grabbing the agents, turn the car so that it is re- aligned with the tape.
		Make it a boolean that once the middle two QRDs detect the threshold value, it has corrected 
		and can go back to driving.
		
	tapeFollow: void method that just makes the robot tape follow.  

	handOff: Exits the retrival Agent scope. Make the preparations to hand off the 
		controls to the 3rd stage of the robot.

*/


////Retrival Agent Arm
//#define armPin 9
//#define armUpPosition 150
//#define armDelay 15 
//#define handOffArmPosition 10
//
//// Retrival Agent Claw
//#define	clawPin 8
//#define clawClosePosition 0
//#define clawOpenPosition 130
//#define clawDelay 10


//Constructor:
RetrivalAgent::RetrivalAgent(int distances[6], int armDownPositions [6], int P, int D, int qRDThreshold, int mSpeed){
	
	// setting up Variables.
	newClaw = new Claw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN);
   // newNavigator = new Claw

   
    // PID tape following Values:
     int lastError = 0, recentError = 0;
    int q = 0, m = 0, con = 0;
    int proportional = P;
    int derivative = D;
    int threshold = qRDThreshold;
    int motorSpeed = motorSpeed; 
}


// Destructor
RetrivalAgent:: ~RetrivalAgent(){
    delete newClaw;
    newClaw = NULL;
}



// Maneuver function:
void RetrivalAgent:: maneuver(int leftTargetDistance, int rightTargetDistance,int leftConstant, int rightConstant, int minimumMotorSpeed){
    // start counting:
    Encoder encoders;
    int leftDifference = leftTargetDistance;
    int rightDifference = rightTargetDistance;

    while(leftDifference > 0 || rightDifference >0){
        leftDifference = leftTargetDistance- encoders.getDistanceLeftWheel();
        rightDifference = rightTargetDistance - encoders.getDistanceRightWheel();

        double leftSpeed = -(minimumMotorSpeed + leftDifference * leftConstant);
        double rightSpeed = minimumMotorSpeed + rightDifference * rightConstant; 

        motor.speed(leftMotor, leftSpeed);
        motor.speed(rightMotor, rightSpeed);
    }
    
}

// Detecting Crosses:
bool RetrivalAgent:: detectCross(){
    bool
        L = analogRead(leftQRDSensor) > threshold,
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold,
        R = analogRead(rightQRDSensor) > threshold; 
        
    return( (L&&CL&&CR&&R) ? true : false);
}


// tape Follow:
void tapeFollow(){
    // boolean values for PID tuning
    bool 
          L = analogRead(leftQRDSensor) > threshold,
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold,
        R = analogRead(rightQRDSensor) > threshold; 

        
    // Copied from Joel
    int error;
          if ( CL && CR )       error = 0;
        else if ( CL && !CR )    error = -1;
        else if ( !CL && CR )    error = 1;
        else{
           if( lastError > 0 )    error = 2;
           else                 error = -2;
        }
      
        if(!(error == lastError)){
          recentError = lastError;
          q=m;
          m=1;
        }
      
        int proportionalVal = proportional * error;
        int derivativeVal   = (int) (derivative * (float)(error - recentError) / (q + m) );
        con = proportionalVal + derivativeVal;
      
        m++;
        motor.speed(leftMotor, -motorSpeed + con);
        motor.speed(rightMotor, motorSpeed + con);
      
        lastError = error;
}


