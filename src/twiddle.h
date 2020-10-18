#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include "PID.h"

using namespace std;

class Twiddle {
  public:
  	/*
    * Constructor
    */
    Twiddle(double Kp0, double Kd0, double Ki0);

    /*
    * Destructor.
    */
    virtual ~Twiddle();
  
  	int state; // state0 -> not initialized (get initial best_err. state1 -> add dp. state2 check if step1 makes best_err better; if not, subract 2dp. state3 -> check if step2 makes best_err better otherwise, make dp smaller 
  	int param_idx;
  	vector<double> p; //params
  	vector<double> dp;
    double best_err;
    void run(double total_err);
}