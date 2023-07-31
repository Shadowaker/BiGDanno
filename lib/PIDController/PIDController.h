#ifndef PIDCONTROLLER_H
# define PIDCONTROLLER_H

#include "Arduino.h"

#define VERBOSE   "verbose"
#define NOVERBOSE "noverbose"

class PIDController {
	private:
    	// Variables - long
    	unsigned long lastTime;

    	// Variables - double
    	double output;
    	double lastErr;
    	double timeChanged;

    	// Variables - double, error variables
    	double error;
    	double errSum;
    	double dErr;

    	// Variables - bool
    	bool doLimit;
    	bool init;

    	// Variables - double - tuining
    	double Kp;
    	double Ki;
    	double Kd;
    	double divisor;
    	double minOut;
    	double maxOut;
    	double setPoint;

	public:
    	// Constructor
    	PIDController();

    	// Methods - double
    	double compute(double input);

    	// Methods - void
    	void begin();
    	void tune(double _Kp, double _Ki, double _Kd);
    	void limit(double min, double max);
    	void setpoint(double newSetpoint);
    	void minimize(double newMinimize);

    	// Methods - double, getters
    	double getOutput();
};
#endif