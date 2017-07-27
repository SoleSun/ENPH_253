#include "TestProcedures.h"
#include "Configuration.h"
#include <phys253.h>
#include <LiquidCrystal.h>

TestProcedures::TestProcedures () {

}

TestProcedures::~TestProcedures () {

}

void TestProcedures::testGateSensors () {
	while(true) {
		int gateActive = analogRead (OneKHzSensorPin),
			 ziplineNear = analogRead (TenKHzSensorPin);

		LCD.clear(); LCD.home();
    LCD.print("1KHz "); LCD.print(gateActive); 
		LCD.setCursor(0,1);
		LCD.print("10KHz "); LCD.print(ziplineNear);
		delay(500);

		if (stopbutton())
		{
		  delay(100);
		  if (stopbutton())
		  { 
			LCD.clear(); LCD.home();
			LCD.print("Exiting");
			LCD.setCursor(0,1); LCD.print("Gate Test");
			delay(500);
			return;
		  }
		}
	}
}

void TestProcedures::testPID(int thresholdVal, int proportionalVal, int derivativeVal, int speedVal) {
	int lastError = 0, recentError = 0;
	int q = 0, m = 0, con = 0;
	
	LCD.clear(); LCD.home();
	LCD.print("Driving");
	delay(1000);
  
	while (true) {
		bool 
		L = analogRead(leftQRDSensor) > thresholdVal,
		CL = analogRead(centreLeftQRDSensor) > thresholdVal,
		CR = analogRead(centreRightQRDSensor) > thresholdVal,
		R = analogRead(rightQRDSensor) > thresholdVal; 

    if (L && CL && CR && R) {
      motor.speed(leftMotor, -speedVal + con);
      motor.speed(rightMotor, speedVal + con);  
    }
    else {
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
  		LCD.print(" "); LCD.print(derivativeVal);
  		LCD.setCursor(0,1);
  		LCD.print("L "); LCD.print(CL); LCD.print(" R "); LCD.print(CR);
  		delay(25);
  	}

    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
        { 
          motor.speed(leftMotor, 0); motor.speed(rightMotor, 0);
          LCD.clear(); LCD.home();
          LCD.print("Exiting");
          LCD.setCursor(0,1); LCD.print("Drive");
          delay(500);
          return;
        }
    }
	}
}

void TestProcedures::testMotors () {
  while (true) {

    LCD.clear(); LCD.home();
    LCD.print("Testing");
    LCD.setCursor(0,1); LCD.print("Motors");
    
    int motorSpeed = map (knob(6), 0, 1023, 0, 255);
    
    motor.speed(leftMotor, -motorSpeed);
    motor.speed(rightMotor, motorSpeed);
    
    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
        { 
          motor.speed(leftMotor, 0); motor.speed(rightMotor, 0);
          LCD.clear(); LCD.home();
          LCD.print("Exiting");
          LCD.setCursor(0,1); LCD.print("Drive");
          delay(500);
          return;
        }
    }
  }
}

void TestProcedures::testLifts() {
  int flag = 0;
  while (true){
    
    if(startbutton()) {
      delay(100);
      if (startbutton()){ 
        if (!flag) {  
          RCServo0.write(90);
          RCServo1.write(0);
          flag++;
        } else {
          RCServo0.write(0);
          RCServo1.write(180);
          flag--;
        }
      }
    }
    
    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
        { 
          motor.speed(leftMotor, 0); motor.speed(rightMotor, 0);
          LCD.clear(); LCD.home();
          LCD.print("Exiting");
          LCD.setCursor(0,1); LCD.print("Lift Test");
          delay(500);
          return;
        }
    }
  }
}

