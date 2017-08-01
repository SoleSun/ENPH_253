#pragma once 

class Gate_Navigator {
    private: 
		  int thresholdVal, proportionalVal, derivativeVal, speedVal, distToGateVal, threshGateVal, distanceAfterGateVal;
		
    public:
		Gate_Navigator (int thresholdValue, int proportionalGain, int derivativeGain, int motorSpeed, int distanceToGate, int threshGate/*, int distanceAfterGate*/);
		
		//Primary method for line following 
		bool Drive (); 
		
		//Getter methods
		int getErrorValue(int prevError); 
};
