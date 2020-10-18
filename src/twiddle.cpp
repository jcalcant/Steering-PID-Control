#include "twiddle.h"

#include <iostream>
#include <math.h>

Twiddle::Twiddle(double Kp0, double Kd0, double Ki0){
  state = 0;
  p = {Kp0,Kd0,Ki0};
  dp = {1.0,1.0,1.0};
  param_idx = 0;
}

void run(double total_err){
  if (state == 0){
    best_err = total_err;
    state=1;
    p[param_idx]+=dp[param_idx];
  }
  else if (state == 1){
    if (total_err < best_err){
      best_err = total_err;
      dp[param_idx]*=1.1;
      param_idx++;
      param_idx %= 3;
      p[param_idx]+=dp[param_idx];
    }
    else{
      p[param_idx]-=2*dp[param_idx];
      state=2;
    }
  }
  else if (state == 2){
    if (total_err < best_err){
      best_err = total_err;
      dp[param_idx]*=1.1;
      param_idx++;
      param_idx %= 3;
      p[param_idx]+=dp[param_idx];
      state = 1;
    }
    else{
      p[param_idx]+=dp[param_idx];
      dp[param_idx]*=0.9;
      param_idx++;
      param_idx %= 3;
      p[param_idx]+=dp[param_idx];
      state = 1;
    }
  }
}
