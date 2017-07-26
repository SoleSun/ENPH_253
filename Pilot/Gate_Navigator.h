#pragma once 

class Gate_Navigator {
    private: 
		  int threshold, proportional, derivative, speed;
		
	  public:
  		Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed);
  		
  		//Primary method for line following 
  		bool Drive (); 
  		
  		//Getter methods
  		int getThresholdValue ();
  		
  		int getProportionalGainValue ();
  		
  		int getDerivativeGainValue();
  		
  		int getMotorSpeedValue();
};
