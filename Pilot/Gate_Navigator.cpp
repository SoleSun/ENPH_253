#include <phys253.h>
#include <LiquidCrystal.h>
#include "Configuration.h"
#include "Encoder.h"
#include "Gate_Navigator.h"

Gate_Navigator::Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed) {
	threshold = thresholdValue;
	proportional = proportionalGain;
	derivative = derivativeGain;
	speed = motorSpeed;
};

/*
 * @return true if the first cross has been detected 
 */
bool Gate_Navigator::Drive() {
  int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;

  LCD.clear(); LCD.home();
  LCD.print("Driving");
  delay(1000);
  
  unsigned long offset = millis();
  
  while (true){
	  
  	/* Sample the 1 khZ sensor to see if its should stop */
  	bool gateActive = digitalRead (OneKHzSensorPin);
  	
  	if (gateActive) {
  		motor.speed(leftMotor, 0);
  		motor.speed(rightMotor, 0);
  	}
    else {
  		bool 
  		L = analogRead(leftQRDSensor) > threshold,
  		CL = analogRead(centreLeftQRDSensor) > threshold,
  		CR = analogRead(centreRightQRDSensor) > threshold,
  		R = analogRead(rightQRDSensor) > threshold; 

      if (  (CL && CR) && (R || L)  && (millis() - offset > 5000)) {
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
  	  
  		int proportional = proportional * error,
  			derivative   = (int) (derivative * (float)(error - recentError) / (q + m) );
  		con = proportional + derivative;
  	  
  		m++;
  		motor.speed(leftMotor, -speed + con);
  		motor.speed(rightMotor, speed + con);
  	  
  		lastError = error;

      LCD.clear(); LCD.home();
      LCD.print(speed); LCD.print(" "); LCD.print(proportional); 
      LCD.print(" "); LCD.print(derivative);
      LCD.setCursor(0,1);
      LCD.print("L "); LCD.print(CL); LCD.print(" R "); LCD.print(CR);
      
      delay(25);
  	}

  }
};
