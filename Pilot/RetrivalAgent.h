# pragma once
# include "ClawTINAH.h"
# include "configuration.h"
# include "Gate_Navigator.h"
# include "Encoder.h"

//Method BreakDown:
/*

	I have to ask for all dynmically changed variables as inputs.
	
	ExecuteTetrivalAgent: Executes the required looping to catch all the animals.
		upon looping for 6 times, it will return a true value indicating the passing on
		to the next stage. Also make sure to lower the claw before handing over
		So that the claw is not in the way of the lifting mechanism.
		
		
	DetectCrosses: Will return a true when at least 3 QRDs to detect an threshold. IE 
		a cross has been detected
		For now I am going to make it so that only when 4 QRD detect the threshold.

	AlignClaw: This method will align the arm with agents by driving straight forward.
		Keep in mind that even if the motors are set to the same speed in code, the motors naturally
		turn differently. This will be a void method. 
	
	BackToTape: After grabbing the agents, turn the car so that it is re- aligned with the tape.
		Make it a boolean that once the middle two QRDs detect the threshold value, it has corrected 
		and can go back to driving.
		
	tapeFollow: void method that just makes the robot tape follow.  

	handOff: Exits the retrival Agent scope. Make the preparations to hand off the 
		controls to the 3rd stage of the robot.

     Maneuver: Jason's simple algorithm to make sure the robot travels a certain way. THe 

*/




class RetrivalAgent{

	public:
		RetrivalAgent(int distances [6], int armDownPositionsm [6], int P, int D, int qRDThreshold, int mSpeed);
		bool executeRetrivalAgent();
		
		~RetrivalAgent();
	
	private: 
		bool detectCross();
		// for grabbing agent
		void maneuver(int leftTargetDistance, int RightTargetDistance,int LeftConstant, int RightConstant, int MinimumMotorSpeed);
		// maybe not needed.
		//void backToTape();

        
		// tape follow
		void tapeFollow();
		// handing off
		void handOff(int finalArmDownAngle); // might take a value for closing the claw. Might not.
	
		/* Variables*/
		Claw *newClaw;

        // PID stuff
		int proportional;
		int derivative;
		int threshold;
        int lastError, recentError;
        int q, m, con;
        int motorSpeed; 
		
	
	
	
	
	
	
	
};
