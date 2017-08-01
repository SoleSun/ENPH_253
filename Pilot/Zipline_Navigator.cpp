#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Zipline_Navigator.h"

Zipline_Navigator::Zipline_Navigator (int thresh, int p, int d, int speed, int dist, int degree){
	thresholdVal = thresh;
	proportionalVal = p;
	derivativeVal = d;
	speedVal = speed;
	distToZipline = dist;
	degreeToTurn = degree;
	noOfCrossesEncountered = 0;
}

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
			if (noOfCrossesEncountered == 3) return;
		}
      
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
  	  
  		int proportional = proportionalVal * error,
  			derivative   = (int) (derivativeVal * (float)(error - recentError) / (q + m) );
  		con = proportional + derivative;
  	  
  		m++;
  		motor.speed(leftMotor, -speedVal + con);
  		motor.speed(rightMotor, speedVal + con);
  	  
  		lastError = error;
		  delay(25);

  		if (stopbutton())
  		{
  			delay(100);
  			if (stopbutton())
  			{ 
  				LCD.clear(); LCD.home();
  				LCD.print("Exiting");
  				LCD.setCursor(0,1); LCD.print("Drive");
  				delay(500);
  				return;
  			}
  		}
  	}
}

void Zipline_Navigator::driveStraight() {
	while (true) {
	
	}
}
    
void Zipline_Navigator::maneuver() {
	Encoder angleCalculator = Encoder ();
	
}
