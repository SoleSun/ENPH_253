# include "include/RetrivalFSM.h"

#define MOTOR_SPEED_MIN 139
#define ALIGN_DIST 18
#define LEFT_CONST 40
#define RIGHT_CONST 1
// #define DEBUG 7


//PID parameters:
int proportional; 
int derivative; 
int threshold; 
int motorSpeed; 
int lastError, recentError;
int q = 0, m = 0, con = 0; 


// agents servoPositions:
int armDownPositions[6] = {144, 140,146,144,140,146};


States g_CurrentState = S_TapeFollow; 
long cumulativeTime = 0; 


void executeRetrivalFSM(int p, int d, int QRDthreshold, int MotorSpeed){
    bool fsmDone = false;
    Claw newClaw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN); 
    int counter = 0;
  lastError = 0;
  recentError = 0; 
    
    // PID: 
    
    proportional = p;
    derivative = d;
    threshold = QRDthreshold;
    motorSpeed = MotorSpeed; 
    

  //test code: 

  LCD.clear();
        LCD.print(g_CurrentState);
        delay(1000);
        
    while(!fsmDone){
        switch(g_CurrentState){ 
            case S_TapeFollow:
                tapeFollow();
                if(detectCross()) {
                    // stopping the motor:
                    motor.speed(leftMotor, 0);
                    motor.speed(rightMotor, 0);
                    
                    LCD.clear();
                    LCD.home();
                    LCD.print("Cross Detected");
                    delay(2000);
                    
                    //g_CurrentState = S_Forward; 
                    
                }
                break;
                
            
            case S_Forward:

                 LCD.clear();
                 LCD.home();
                 LCD.print("Forward");
                 delay(2000);
                forward();
                g_CurrentState = S_Retrieve;
                 // stopping the motor:
                motor.speed(leftMotor, 0);
                motor.speed(rightMotor, 0);
                    
                break;
                    
            
            case S_Retrieve:
                LCD.clear();
                    LCD.home();
                    LCD.print("Retrieve");
                    delay(2000);
                    
                if (counter >= 0 && counter <6){
                    newClaw.retrieve(armDownPositions [counter]);
                    counter++; 
                } 
                
                if(counter >=6){
                    g_CurrentState = S_Exit; 
                }else{
                    g_CurrentState = S_Reverse; 
                }
                // stopping the motor:
                    motor.speed(leftMotor, 0);
                    motor.speed(rightMotor, 0);
                break; 
            
            
            case S_Reverse:
                LCD.clear();
                LCD.home();
                LCD.print("reverse");
                delay(2000);
                    
                reverse();
                g_CurrentState = S_TapeFollow;
                //reseting error values. 
                lastError = 0;
                recentError = 0;
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
      
      if((L&&R)&&(CL&&CR)){
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

    #ifdef DEBUG 

        
       LCD.clear();
        LCD.home();
        LCD.print(L);
        LCD.print(" ");
        LCD.print(CL);
        LCD.print(" ");
        LCD.print(CR);
        LCD.print("");
        LCD.print(R);
        
        delay(10);
        
    #endif
}
 
 
 
 
 // move Forward
 
 void forward(){
 
    maneuver(ALIGN_DIST, ALIGN_DIST,RIGHT_CONST,LEFT_CONST, MOTOR_SPEED_MIN, false);
 }
 
 void reverse(){
    maneuver(ALIGN_DIST, ALIGN_DIST,RIGHT_CONST,LEFT_CONST, MOTOR_SPEED_MIN, true);
 }
 
 
 // maneuvering the robot. 
 
void maneuver(int leftTargetDistance, int rightTargetDistance,int leftConstant, int rightConstant, int minimumMotorSpeed, bool reverse){
    Serial.begin(9600);
    int leftDifference = leftTargetDistance;
    int rightDifference = rightTargetDistance; 
    Encoder encoders;
    long startTime = millis();
    
    while(leftDifference > 0 || rightDifference >0){
        leftDifference = leftTargetDistance- encoders.getDistanceLeftWheel();
        rightDifference = rightTargetDistance - encoders.getDistanceRightWheel();

        // calculating the calibrated speeds. 
        int leftSpeed; 
        int rightSpeed; 
        if( reverse){
            leftSpeed = (minimumMotorSpeed + leftDifference * leftConstant);
            rightSpeed = - (minimumMotorSpeed + rightDifference * rightConstant); 
            
            if (backOnTape()){
                return; 
            }
        }else{
         leftSpeed = -(minimumMotorSpeed + leftDifference * leftConstant);
         rightSpeed = minimumMotorSpeed + rightDifference * rightConstant;
        }

        // if the difference is zero, set the speed to zero. 
        if(leftDifference <= 0){
            leftSpeed = 0;
        }else if(rightDifference <= 0){
            rightSpeed = 0; 
        }

    cumulativeTime += millis()- startTime;
        #ifdef DEBUG

          if(cumulativeTime >= 1000){
            LCD.clear();
            LCD.print("L: "), LCD.print(leftDifference),LCD.print(" "), LCD.print(encoders.getDistanceLeftWheel());
            LCD.setCursor(0,1);
            LCD.print("R:"), LCD.print(" "), LCD.print(rightDifference), LCD.print(" "), LCD.print(encoders.getDistanceRightWheel());
            cumulativeTime = 0; 
          }
              
       #endif

    //changing the speeds of the motors. 
    
        motor.speed(leftMotor, leftSpeed);
        motor.speed(rightMotor, rightSpeed);
    }
}
 
 

 
 
