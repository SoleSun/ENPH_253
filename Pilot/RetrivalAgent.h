# pragma once
# include "claw.h"

//Method BreakDown:
/*
	ExecuteTetrivalAgent: Executes the required looping to catch all the animals.
		upon looping for 6 times, it will return a true value indicating the passing on
		to the next stage.
		
	DetectCrosses: Will return a true when at least 3 QRDs to detect an threshold 


*/


class RetrivalAgent{

	public:
	
	RetrivalAgent(int [] distances, int [] armDownPositions);
	bool executeRetrivalAgent();
	
	private: 
	bool detectCrosses();
	void alignClaw();
	
	
}
