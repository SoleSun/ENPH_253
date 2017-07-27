
#include <Arduino.h>

class TestProcedures {
	
	public:
		TestProcedures ();
		~TestProcedures();
		void testGateSensors();
		void testPID (int thresholdVal, int proportionalVal, int derivativeVal, int speedVal);
    void testMotors ();
    void testLifts ();
    void testMinMotor ();
	void testManeuver ();
};
