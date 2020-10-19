#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

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
  
  	void run(double total_err);
  	vector<double> get_params();
  
  private:
  	int state; // state1 -> not initialized (get initial best_err and add dp. state1 -> check if state1 makes best_err better; if not, subract 2dp. state3 -> check if step2 makes best_err better otherwise, make dp smaller 
  	int param_idx;
  	vector<double> p; //params
  	vector<double> dp;
    double best_err;
  	double threshold;
    
};

#endif /* TWIDDLE_H */