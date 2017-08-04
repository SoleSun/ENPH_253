#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Zipline_Navigator.h"
#include "include/RetrievalFSM.h"

Zipline_Navigator::Zipline_Navigator (int thresh, int p, int d, int speed, int dist, int degree, int rightConstant, int leftConstant){
	thresholdVal = thresh;
	proportionalVal = p;
	derivativeVal = d;
	speedVal = speed;
	distToZipline = dist;
	degreeToTurn = degree;
	noOfCrossesEncountered = 0;
  rightConst = rightConstant;
  leftConst = leftConstant;
}

/**
 * Call this to navigate the robot to the cross bar that whose
 * tangent line leads straight to the zipline
 */
void Zipline_Navigator::tapeFollow() {
	int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;
  
	/* 
	Assuming the zipline navigator takes control after six crosses, 
	the minimum number of crosses to traverse is three
	*/
	const int minimumCrosses = 3;
	while (true){
	  
  		bool 
  		L = analogRead(leftQRDSensor) > thresholdVal,
  		CL = analogRead(centreLeftQRDSensor) > thresholdVal,
  		CR = analogRead(centreRightQRDSensor) > thresholdVal,
  		R = analogRead(rightQRDSensor) > thresholdVal; 

		if (  (CL && CR) && (R || L)  ){
			noOfCrossesEncountered++;
      delay(25);
			if (noOfCrossesEncountered == 3) return;
		}
      
  		int error;
  		if ( CL && CR )          error = 0;
  		else if ( CL && !CR )    error = -1;
  		else if ( !CL && CR )    error = 1;
  		else               		   error = (lastError > 0) ? 2 : -2;
  		
  		if(!(error == lastError)){
  		  recentError = lastError;
  		  q=m;
  		  m=1;
  		}
  	  
  		int proportional = proportionalVal * error,
  			derivative   = (int) (derivativeVal * (float)(error - recentError) / (q + m) );
  		con = proportional + derivative;
  	  
  		m++;
  		motor.speed(leftMotor, -speedVal + con);
  		motor.speed(rightMotor, speedVal + con);
  	  
  		lastError = error;

      LCD.print("L "); LCD.print(analogRead(centreLeftQRDSensor)); 
      LCD.print(" R "); LCD.print(analogRead(centreRightQRDSensor)); 
      LCD.setCursor(0,1); LCD.print("IR"); LCD.print(analogRead(OneKHzSensorPin));
      delay(15);

  	}
}

/**
 * Drives the robot straight to the zipline until it reaches
 * a predetermined distance 
 */
void Zipline_Navigator::driveToZipline() {
  maneuver(distToZipline, distToZipline, leftConst, rightConst, speedVal, false);
}

/**
 * Assume robot has successfully reached the zipline,
 * driving in a straight line. Now maneuver to have the 
 * box face the zipline
 */
void Zipline_Navigator::latch(bool drivingOnLeftSurface) {
	if (drivingOnLeftSurface){
    /* Turn Right */
    motor.speed(leftMotor, speedVal);    motor.speed(rightMotor, speedVal);
    delay(25);
	} else {
    /* Turn Left */
    motor.speed(leftMotor, -speedVal);    motor.speed(rightMotor, -speedVal);
    delay(25);    
	}

  /* Reverse direction and move backwards */
  maneuver (50, 50, leftConst, rightConst, speedVal, true);

  /* Lift the box*/
  for (int angle = 0; angle <= 180; angle++){
    RCServo0.write(angle);  RCServo1.write(180 - angle);
    delay(15);
  }

  /*Move forward */
  maneuver (50,50,leftConst, rightConst, speedVal, false);

  /* Lower the box*/
  for (int angle = 180; angle >= 0; angle--){
    RCServo0.write(angle);  RCServo1.write(180 - angle);
    delay(15);
  }

  return;
}

void Zipline_Navigator::Drive(bool drivingOnLeftSurface){
  
}

