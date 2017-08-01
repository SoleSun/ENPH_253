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

/**
 * @return 10 if all four sensors have been activated; else follow truth table
 */
int Gate_Navigator::getErrorValue(int prevError) {

  /* 
   *  L    CL    CR    R    Err    Index
   *  0    0     0     0     4      0
   *  0    0     0     1     3      1  
   *  0    0     1     0     1      2
   *  0    0     1     1     2      3
   *  0    1     0     0    -1      4
   *  0    1     0     1     6      5
   *  0    1     1     0     0      6
   *  0    1     1     1     6      7
   *  1    0     0     0    -3      8
   *  1    0     0     1     5      9
   *  1    0     1     0     5      10
   *  1    0     1     1     5      11
   *  1    1     0     0    -2      12
   *  1    1     0     1     5      13
   *  1    1     1     0     6      14
   *  1    1     1     1     7      15
   */
   
  //const int errorValues [16] = {4,3,1,2,-1,6,0,6,-3,5,5,5,-2,5,6,7};
  
  bool 
    L = analogRead(leftQRDSensor) > thresholdVal,
    CL = analogRead(centreLeftQRDSensor) > thresholdVal,
    CR = analogRead(centreRightQRDSensor) > thresholdVal,
    R = analogRead(rightQRDSensor) > thresholdVal;

  if ( L && CL && CR && R) return 10;
  else if ( CL && CR )     return 0;
  else if ( CL && !CR )    return -1;
  else if ( !CL && CR )    return 1;
  else                     return (prevError > 0) ? 2 : -2;
}

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

//  unsigned long noOfRightTicks = 0, noOfLeftTicks = 0;
  
  while (true){

    LCD.clear(); LCD.home();
    
    /* The distance that was travelled so far */
  	int averageDist = (distCalculator.getDistanceRightWheel() + distCalculator.getDistanceLeftWheel()) / 2;
      
    /* If the beacon is not flashing 1 KHz and the robot has already travelled its allotted safe distance */
  	if (averageDist > distToGateVal && !stoppedOnce) {

      /* Stop the robot */
      motor.speed(leftMotor, 0);
      motor.speed(rightMotor, 0);
      stoppedOnce = true;
        
      /* Wait for the gate alarm to cycle at least once */
      /* If the analog value goes beyond the threshold, then the gate is off */
      while (analogRead(OneKHzSensorPin) < threshGateVal){ 
        /* If the gate is initially on, then it will catch on this loop. */
        LCD.clear(); LCD.home();
        LCD.print("Stop Dist: "); LCD.print(averageDist); 
        LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
      }
      while (analogRead(OneKHzSensorPin) > threshGateVal){ 
        /* If the gate is initially off, then it will catch on this one*/
        LCD.clear(); LCD.home();
        LCD.print("Stop Dist: "); LCD.print(averageDist); 
        LCD.setCursor(0,1); LCD.print("QSD: "); LCD.print(analogRead (OneKHzSensorPin));
      }

      motor.speed(leftMotor, -255);
      motor.speed(rightMotor, 255);
      delay(10);
  	}
    else {
      /* Check the QRD sensors */
      int error = getErrorValue (lastError); 

      /* If a crossbar has been detected and the robot has been following for some time */
      if (  error == 10 && (millis() - offset > minimumTimeToReachCrossBar)) {
        LCD.print("Cross"); LCD.setCursor(0,1); LCD.print("Detected");
        motor.speed(leftMotor, 0);
        motor.speed(rightMotor, 0);
        return true;
      }

//      unsigned long
//      newNoOfRightTicks =  distCalculator.getTicks(rightEncoder),
//      newNoOfLeftTicks  = distCalculator.getTicks(leftEncoder);
//
//      /* Ticks have not been generated for the last two seconds, accelerate*/
//      if ((newNoOfRightTicks - noOfRightTicks < 2 || newNoOfLeftTicks - noOfLeftTicks < 2) && (stuck - millis() > 2000)){
//        motor.speed(-leftMotor, -255);
//        motor.speed(rightMotor, 255);
//        stuck = millis();
//        delay(25);
//      }
//      
//      noOfRightTicks = newNoOfRightTicks; noOfLeftTicks = newNoOfLeftTicks;
  	  
  		if(error != lastError){
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
      LCD.print("L "); LCD.print(analogRead(centreLeftQRDSensor)); 
      LCD.print(" R "); LCD.print(analogRead(centreRightQRDSensor)); 
      LCD.print(" IR"); LCD.print(analogRead(OneKHzSensorPin));
      delay(25);
  	}

    if (stopbutton()) {
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
};
