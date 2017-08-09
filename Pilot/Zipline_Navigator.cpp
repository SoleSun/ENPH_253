#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Zipline_Navigator.h"
#include "include/RetrivalFSM.h"

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
  	}
}

/**
 * Drives the robot straight to the zipline until it reaches
 * a predetermined distance 
 */
void Zipline_Navigator::driveToZipline() {
  maneuver(distToZipline, distToZipline, leftConst, rightConst, speedVal, false);
  Claw newClaw(CLAWOPENPOSITION, CLAWCLOSEPOSITION, ARMUPPOSITION,CLAWDELAY, ARMDELAY, CLAWPIN, ARMPIN); 
  newClaw.stageThreeConfiguration();
}

/**
 * Assume robot has successfully reached the zipline,
 * driving in a straight line. Now maneuver to have the 
 * box face the zipline
 */
void Zipline_Navigator::latch(bool drivingOnLeftSurface) {
	if (drivingOnLeftSurface){
    /* Turn Right */
    motor.speed(leftMotor, 0);    motor.speed(rightMotor, speedVal);
    delay(degreeToTurn);
    /* Stop after turning */
	  motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);
	} else {
    /* Turn Left */
    motor.speed(leftMotor, 0);    motor.speed(rightMotor, -speedVal);
    delay(degreeToTurn);
    /* Stop after turning */
    motor.speed(leftMotor, 0);    motor.speed(rightMotor, 0);
    /* Claw is close to zipline. Move backwards and give the lift some space */
    maneuver (50, 50, leftConst, rightConst, speedVal, true);    
	}

  /* Lift the box with the animals */
  lift();

  /* Now slowly foward move */
  maneuver (15,15,leftConst, rightConst, speedVal, false);

  lower(); 
  
  return;
}

void Zipline_Navigator::lift(){
  const int initalizedangle = 177, servo0 = 15, servo2 = 63;
  int incrementstandard = initalizedangle - servo2,
      s0increment = (initalizedangle - servo0)/incrementstandard,
      s2increment = (initalizedangle - servo2)/incrementstandard;
  
  // 102 + 8 = 110 must be << 114(max increment for servo2)
  for (int i = 0; i <= 102; i++){
    if ( i > 0 && i < 102){
      RCServo0.write(initalizedangle - 40 - i);    RCServo1.write(initalizedangle - 8 - i);
    }
    else if (i == 0){
      RCServo0.write(initalizedangle - 37);        RCServo1.write(initalizedangle - 8);
    }
    else{
      RCServo0.write(servo0);      RCServo1.write(servo2);
    }
    delay(100);
  }

  return;
}

void Zipline_Navigator::lower(){
  const int initalizedangle = 177, servo0 = 15, servo2 = 63;
  int incrementstandard = initalizedangle - servo2,
      s0increment = (initalizedangle - servo0)/incrementstandard,
      s2increment = (initalizedangle - servo2)/incrementstandard;

  // 102 + 8 = 110 must be << 114(max increment for servo2)
  for (int i = 102; i >= 0; i--){
    if ( i > 0 && i < 102){
      RCServo0.write(initalizedangle - 40 - i);    RCServo1.write(initalizedangle - 8 - i);
    }
    else if (i == 0){
      RCServo0.write(initalizedangle - 37);        RCServo1.write(initalizedangle - 8);
    }
    else{
      RCServo0.write(servo0);      RCServo1.write(servo2);
    }
    delay(100);
  }

  return;
}

void Zipline_Navigator::Drive(bool drivingOnLeftSurface){
  tapeFollow();
  driveToZipline();
  latch(drivingOnLeftSurface);
}

