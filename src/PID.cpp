#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Set Kp, Ki and Kd to init values passed by controller.
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	// Set inital p, i and d error to zero.
	p_error = 0;
	i_error = 0;
	d_error = 0;

    // Previous cte.
    prev_cte = 0.0;

}

void PID::UpdateError(double cte) {
    p_error = cte;      // Proportional error
    i_error += cte;     // Integral error
    d_error = cte - prev_cte; // Differential error
    prev_cte = cte;
	
}

double PID::TotalError() {
    return ((Kp * p_error) + (Ki * i_error) + (Kd * d_error));
}




