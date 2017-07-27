#pragma once 

class Gate_Navigator {
    private: 
		  int thresholdVal, proportionalVal, derivativeVal, speedVal, distToGateVal;
		
    public:
		Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate);
		
		//Primary method for line following 
		bool Drive (); 
		
		//Getter methods
		int getThresholdValue ();
		
		int getProportionalGainValue ();
		
		int getDerivativeGainValue();
		
		int getMotorSpeedValue();
};
