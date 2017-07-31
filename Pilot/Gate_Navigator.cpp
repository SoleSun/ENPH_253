#include <phys253.h>
#include <LiquidCrystal.h>
#include "include/Configuration.h"
#include "include/Encoder.h"
#include "include/Gate_Navigator.h"

Gate_Navigator::Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate, int threshGate/*, int distanceAfterGate*/) {
	thresholdVal = thresholdValue;
	proportionalVal = proportionalGain;
	derivativeVal = derivativeGain;
	speedVal = motorSpeed;
  distToGateVal = distanceToGate;
  threshGateVal = threshGate;
  //distanceAfterGateVal = distanceAfterGate;
};

/*
 * @return true if the first cross has been detected 
 */
bool Gate_Navigator::Drive() {
  int lastError = 0, recentError = 0;
  int q = 0, m = 0, con = 0;
  const int minimumTimeToReachCrossBar = 8000;

  Encoder distCalculator = Encoder();
  
  LCD.clear(); LCD.home();
  LCD.print("Driving");
  delay(1000);

  unsigned long offset = millis(), stuck = millis();

  /* Has the robot stopped at least once before the gate */
  bool stoppedOnce = false;

  unsigned long noOfRightTicks = 0, noOfLeftTicks = 0;
  
  while (true){

    LCD.clear(); LCD.home();
    
    /* The distance that was travelled so far */
  	int averageDist = (distCalculator.getDistanceRightWheel() + distCalculator.getDistanceLeftWheel()) / 2;
      
    /* If the beacon is not flashing 1 KHz and the robot has already travelled its allotted safe distance */
  	if (analogRead (OneKHzSensorPin) > threshGateVal && averageDist > distToGateVal && !stoppedOnce) {
  		/* Stop the robot */
  		motor.speed(leftMotor, 0);
  		motor.speed(rightMotor, 0);
      stoppedOnce = true;

      while (analogRead(OneKHzSensorPin) > threshGateVal){
        LCD.print(averageDist); LCD.print(" "); LCD.print(analogRead (OneKHzSensorPin));
        LCD.setCursor(0,1);
        LCD.print("Stopping");
      }

      motor.speed(leftMotor, -255);
      motor.speed(rightMotor, 255);
      delay(30);
  	}
    else {
      /* Check the QRD sensors */
      bool 
        L = analogRead(leftQRDSensor) > thresholdVal,
        CL = analogRead(centreLeftQRDSensor) > thresholdVal,
        CR = analogRead(centreRightQRDSensor) > thresholdVal,
        R = analogRead(rightQRDSensor) > thresholdVal;
      
      if (  (CL && CR) && (R || L)  && (millis() - offset > minimumTimeToReachCrossBar)) {
        LCD.print("Cross"); LCD.setCursor(0,1); LCD.print("Detected");
        motor.speed(leftMotor, 0);
        motor.speed(rightMotor, 0);
        return true;
      }

      unsigned long
      newNoOfRightTicks =  distCalculator.getTicks(rightEncoder),
      newNoOfLeftTicks  = distCalculator.getTicks(leftEncoder);

      /* Ticks have not been generated for the last two seconds, accelerate*/
<<<<<<< HEAD
      if ((newNoOfRightTicks - noOfRightTicks < 2 || newNoOfLeftTicks - noOfLeftTicks < 2) && (stuck - millis() > 2000)){
=======
      if ((newNoOfRightTicks - noOfRightTicks < 5 || newNoOfLeftTicks - noOfLeftTicks < 5) && (stuck - millis() > 2000)){
>>>>>>> 3df4dbc030fe0c0648df8af69a0aaaaddf1a7d71
        motor.speed(-leftMotor, -255);
        motor.speed(rightMotor, 255);
        stuck = millis();
        delay(25);
      }
      
      noOfRightTicks = newNoOfRightTicks; noOfLeftTicks = newNoOfLeftTicks;
      
  		int error;
  		if ( CL && CR )       error = 0;
  		else if ( CL && !CR )    error = -1;
  		else if ( !CL && CR )    error = 1;
  		else{
  		   if( lastError > 0 )    error = 2;
  		   else                 error = -2;
  		}
  	  
<<<<<<< HEAD
  		if(error != lastError){
=======
  		if(!(error == lastError)){
>>>>>>> 3df4dbc030fe0c0648df8af69a0aaaaddf1a7d71
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

      LCD.print(speedVal); LCD.print(" "); LCD.print(proportionalVal); 
      LCD.print(" "); LCD.print(derivativeVal); LCD.print(" "); LCD.print(distToGateVal);
      LCD.setCursor(0,1);
      LCD.print("L "); LCD.print(CL); LCD.print(" R "); LCD.print(CR); LCD.print("IR"); LCD.print(analogRead(OneKHzSensorPin));
      delay(25);
  	}

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
<<<<<<< HEAD
};
=======
};
>>>>>>> 3df4dbc030fe0c0648df8af69a0aaaaddf1a7d71
