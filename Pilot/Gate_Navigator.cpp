#include <phys253.h>
#include <LiquidCrystal.h>
#include "Configuration.h"
#include "Encoder.h"
#include "Gate_Navigator.h"

Gate_Navigator::Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate) {
	thresholdVal = thresholdValue;
	proportionalVal = proportionalGain;
	derivativeVal = derivativeGain;
	speedVal = motorSpeed;
  distToGateVal = distanceToGate;
};

/*
 * @return true if the first cross has been detected 
 */
bool Gate_Navigator::Drive() {
  int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;
  const int minimumTimeToReachCrossBar = 5000;

  Encoder distCalculator = Encoder();
  
  LCD.clear(); LCD.home();
  LCD.print("Driving");
  delay(1000);
  
  unsigned long offset = millis();
  
  while (true){
	  
  	/* Sample the 1 khZ sensor to see if its should stop */
  	bool gateActive = digitalRead (OneKHzSensorPin);
    
    /* The distance that was travelled so far */
  	int averageDist = (distCalculator.getDistanceRightWheel() + distCalculator.getDistanceLeftWheel()) / 2;
    
  	if (gateActive && averageDist > distToGateVal) {
  		motor.speed(leftMotor, 0);
  		motor.speed(rightMotor, 0);
      LCD.clear(); LCD.home();
      LCD.print("Gate Active");
      LCD.setCursor(0,1);
      LCD.print("Stopping");
  	}
    else {
  		bool 
  		L = analogRead(leftQRDSensor) > thresholdVal,
  		CL = analogRead(centreLeftQRDSensor) > thresholdVal,
  		CR = analogRead(centreRightQRDSensor) > thresholdVal,
  		R = analogRead(rightQRDSensor) > thresholdVal; 

      if (  (CL && CR) && (R || L)  && (millis() - offset > minimumTimeToReachCrossBar)) {
        motor.speed(leftMotor, 0);
        motor.speed(rightMotor, 0);
        return true;
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

      LCD.clear(); LCD.home();
      LCD.print(speedVal); LCD.print(" "); LCD.print(proportionalVal); 
      LCD.print(" "); LCD.print(derivativeVal); LCD.print(" "); LCD.print(distToGateVal);
      LCD.setCursor(0,1);
      LCD.print("L "); LCD.print(CL); LCD.print(" R "); LCD.print(CR);
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
          return false;
        }
    }
  	}

  }
};
