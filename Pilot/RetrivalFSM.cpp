# include "RetrivalFSM.h"



#define MOTOR_SPEED_MIN 0
#define ALIGN_DIST 17.4
#define LEFT_CONST 2
#define RIGHT_CONST 5
#define DEBUG 


//PID parameters:
int proportional; 
int derivative; 
int threshold; 
int motorSpeed; 
 int lastError = 0, recentError = 0;
    int q = 0, m = 0, con = 0; 


void executeRetrivalFSM(int armDownPositions [6], int p, int d, int QRDthreshold, int MotorSpeed){
 
	g_CurrentState = S_TapeFollow;
	bool fsmDone = false;
	Claw newClaw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN); 
	int counter = 0;
	
	// PID: 
	
    proportional = p;
    derivative = d;
    threshold = threshold;
    motorSpeed = motorSpeed; 
    
	
	while(!fsmDone){
		switch(g_CurrentState){ 
			case S_TapeFollow:
				tapeFollow();
               
               lastError = 0;
               recentError = 0;
               
				if(detectCross()) {
					g_CurrentState = S_Forward; 
					
				}
				break;
				
			
			case S_Forward:
				forward();
				g_CurrentState = S_Retrieve;
				break;
					
			
			case S_Retrieve:
                if (counter >= 0 && counter <6){
                    newClaw.retrieve(armDownPositions [counter]);
                    counter++; 
                } 
				
				if(counter >=6){
					g_CurrentState = S_Exit; 
				}else{
					g_CurrentState = S_Reverse; 
				}
				break; 
			
			
			case S_Reverse:
					reverse();
					g_CurrentState = S_TapeFollow;
				break;
				
			case S_Exit: 
				fsmDone = true; 
				break; 
			
			default:
				break; 
		}
	
	}
	
 }
 

// check cross 

const bool detectCross(){
	bool 
	 L = analogRead(leftQRDSensor) > threshold,
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold,
        R = analogRead(rightQRDSensor) > threshold; 
      
      if((L||R)&&(CL&&CR)){
		return true;
      }else{
		return false; 
      }
	
}

// back to tape
const bool backOnTape(){
	//require testing**********************************************
	bool 
        CL = analogRead(centreLeftQRDSensor) > threshold,
        CR = analogRead(centreRightQRDSensor) > threshold;
        

	if( CL|| CR)
		return true;
	else 
		return false; 
	
}


//tape following
void  tapeFollow(){
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
 
 
 
 
 // move Forward
 
 void forward(){
 
	maneuver(ALIGN_DIST, ALIGN_DIST,RIGHT_CONST,LEFT_CONST, MOTOR_SPEED_MIN, false);
 }
 
 void reverse(){
	maneuver(ALIGN_DIST, ALIGN_DIST,RIGHT_CONST,LEFT_CONST, MOTOR_SPEED_MIN, true);
 }
 
 
 // maneuvering the robot. 
 
void maneuver(double leftTargetDistance, double rightTargetDistance,double leftConstant, double rightConstant, double minimumMotorSpeed, bool reverse){

    double leftDifference = leftTargetDistance;
    double rightDifference = rightTargetDistance; 
    Encoder encoders;
    
    while(leftDifference > 0 || rightDifference >0){
        leftDifference = leftTargetDistance- encoders.getDistanceLeftWheel();
        rightDifference = rightTargetDistance - encoders.getDistanceRightWheel();

        double leftSpeed; 
        double rightSpeed; 
		if( reverse){
			leftSpeed = (minimumMotorSpeed + leftDifference * leftConstant);
			rightSpeed = - minimumMotorSpeed + rightDifference * rightConstant; 
            
            if (backOnTape(){
                return; 
            }
		}else{
		 leftSpeed = -(minimumMotorSpeed + leftDifference * leftConstant);
		 rightSpeed = minimumMotorSpeed + rightDifference * rightConstant;
		}

        #ifdef DEBUG
		LCD.home();
		LCD.print("leftMotor:"), LCD.print(" "), LCD.print(encoders.getDistanceLeftWheel());
		LCD.setCursor(0,1);
        LCD.print("rightMotor:"), LCD.print(" "), LCD.print(encoders.getDistanceRightWheel());
       LCD.clear();
       #endif

        motor.speed(leftMotor, leftSpeed);
        motor.speed(rightMotor, rightSpeed);
    }
}
 
 
