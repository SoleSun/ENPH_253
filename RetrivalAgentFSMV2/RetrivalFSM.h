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

	BackToTape: After grabbing the agents, turn the car so that it is re- aligned with the tape.
		Make it a boolean that once the middle two QRDs detect the threshold value, it has corrected 
		and can go back to driving.
		 

	handOff: Exits the retrival Agent scope. Make the preparations to hand off the 
		controls to the 3rd stage of the robot.

     Maneuver: Jason's simple algorithm to make sure the robot travels a certain way. THe 

*/

enum States{S_TapeFollow, S_Forward, S_Retrieve, S_Reverse, S_Exit};
States g_CurrentState;

 
void executeRetrivalFSM(int armDownPositions [6], int p, int d, int threshold, int motorSpeed);


//State 0: Right turn
void rightTurn();

// State 1: tapeFollow
const bool detectCross();

//State 2: forward offset
void forward();

// State 3: Retrieve
void retrieveAgent();

//Steate 4: 
void reverse();
const bool backOnTape();

//State 5:
void exit();

 void tapeFollow();
void maneuver(double leftTargetDistance, double rightTargetDistance,double leftConstant, double rightConstant, double startingMotorSpeed, bool reverse);