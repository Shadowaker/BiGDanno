#include "PIDController.h"

PIDController::PIDController() : doLimit(true), init(true), Kp(1), Ki(1), Kd(1), divisor(10) {}

void PIDController::begin() {
	lastTime = millis();
}

void PIDController::setpoint(double newSetpoint) {
	setPoint = newSetpoint;
}

void PIDController::tune(double _Kp, double _Ki, double _Kd) {
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

void PIDController::limit(double min, double max) {
	minOut = min;
	maxOut = max;
	doLimit = true;
}

void PIDController::minimize(double newMinimize) {
	divisor = newMinimize;
}

// Getters
double PIDController::getOutput() {
	return output;
}


double PIDController::compute(double sensor) {
	// Return false if it could not execute;
	// This is the actual PID algorithm executed every loop();
	
	// Calculate time difference since last time executed
	unsigned long now = millis();
	double timeChange = (double)(now - lastTime);

	// Calculate error (P, I and D)
	double error = setPoint - sensor;
	errSum += error * timeChange;
	if (doLimit) {
	  errSum = constrain(errSum, minOut * 1.1, maxOut * 1.1); 
	}
	double dErr = (error - lastErr) / timeChange;
	
	// Calculate the new output by adding all three elements together
	double newOutput = (Kp * error);
	
	// If limit is specified, limit the output
	if (doLimit) {
	  output = constrain(newOutput, minOut, maxOut);
	} 
	else {
	  output = newOutput;  
	}
	// Update lastErr and lastTime to current values for use in next execution
	lastErr = error;
	lastTime = now;

	// Return the current output
	return output;
}