#include "PID.h"
#include <cmath>		
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
   Kp = Kp_;		
   Ki = Ki_;		
   Kd = Kd_;		
   p_error = 0;		
   i_error = 0;		
   d_error = 0;		
   dpp = 0.005;		
   dpi = 0.0001;		
   dpd = 0.05;		
   controller = 0;		
   controllerSequence = 0;		
   best_err_initialized = false;

}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;		
    p_error = cte;		
    i_error += cte;
}

double PID::TotalError() {
return  -Kp * p_error - Ki * i_error - Kd * d_error;		  return 0.0;  // TODO: Add your total error calc here!
}		
double * PID::setGainValue(const int& controller) {		
    switch (controller) {		
        case 0:		
            return &Kp;		
        case 1:		
            return &Ki;		
        case 2:		
            return &Kd;		
        default:		
            std::cout<<"Gain sequence not in sync!"<<std::endl;			
    }		
}


double * PID::setDifferentialValue(const int& controller) {		
    switch (controller) {		
        case 0:		
            return &dpp;		
        case 1:		
            return &dpi;		
        case 2:		
            return &dpd;		
        default:		
            std::cout<<"Differential sequence not in sync!"<<std::endl;	
    }		
}		
void PID::twiddle(double err) {			

    double* K = setGainValue(controller);
    double* dp = setDifferentialValue(controller);

    switch (controllerSequence) {	
        case 0:		
            (*K) += (*dp);		
            controllerSequence++;		
            break;		
        case 1:		
            if (err < best_err) {		
                best_err = err;		
                (*dp) *= 1.1;			
                controller = (controller >= 2) ? 0 : (controller+1);		
                controllerSequence = 0;		
            }		
            else {		
                (*K) -= 2 * (*dp);		
                controllerSequence++;		
            }		
            break;		
        case 2:		
            if (err < best_err) {		
                best_err = err;		
                (*dp) *= 1.1;		
            }		
            else {		
                (*K) += (*dp);		
                (*dp) *= 0.9;		
            }		
            controller = (controller >= 2) ? 0 : (controller+1);		
            controllerSequence = 0;		
            break;		
        default:		
            std::cout<<"PID sequence not in sync!"<<std::endl;	
    }		
}
