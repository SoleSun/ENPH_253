#include "Encoder.h"
#include "Configuration.h"

class Zipline_Navigator{
	private: 
		int thresholdVal; 
		int proportionalVal;
		int derivativeVal;
		int speedVal;
		int distToZipline;
		int degreeToTurn;
		int noOfCrossesEncountered;

		void tapeFollow ();
		void driveStraight(); 
		void maneuver ();
		
	public:
		Zipline_Navigator (int thresh, int p, int d, int speed, int dist, int degree);
		~Zipline_Navigator();
		void navigate ();
};
