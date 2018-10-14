#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double K_p, double K_i, double K_d, double tolerance) {
  Kp = K_p;
  Ki = K_i;
  Kd = K_d;
  p = {K_p, K_i, K_d};
  dp = {0.1*K_p, 0.1*K_i, 0.1*K_d};
  tol = tolerance;
  is_initialized = true;
}

void PID::UpdateError(double cte) {
  if (is_initialized){
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    if (counter > num_settlesteps){
      acc_err += pow(cte,2);
    }  
  }
  else{
    cout << "Controller not initialized.\n";
  }
}

double PID::TotalError() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

double PID::TotalErrorTraining(){
  counter += 1;

  if(first_round && (counter % num_steps == 0)){
    best_err = acc_err/(double(num_steps - num_settlesteps));
    first_round = false;
    cout << "Best Error Estimate Initialized: " << best_err << "\n";
  }

  double sum_dp = 0.0;
  for (auto& n : dp){sum_dp += n;}  

  if ((counter % num_steps == 0) && !first_round){
    if (sum_dp > tol){
      //Average accumulated error
      acc_err /= double(num_steps - num_settlesteps);
      cout << "Best Error, Current Error: " << best_err << "\t" << acc_err << "\n";
      if(!first_pass){
        PID::Twiddle();
      }
      PID::TwiddleOp();
      first_pass = false;
      acc_err = 0.0;
      counter = 1;
      Kp = p[0];
      Ki = p[1];
      Kd = p[2];
    }
    else{
      cout <<"Gain optimization complete.\n";
    }
  }

  //To manually stop the simulation, print statement
  if (sum_dp <= tol){
    cout << "Stop Simulation - Gain Optimization Complete";
  }

  return -Kp*p_error - Ki*i_error - Kd*d_error;
}

void PID::Twiddle(){
  if (acc_err < best_err && op == 1){
    best_err = acc_err;
    dp[gain_counter] *= 1.1;
    PID::GainCounterUpdate(); 
  }
  else if (acc_err < best_err && op == 2){
    best_err = acc_err;
    dp[gain_counter] *= 1.1;
    op = 1;
    PID::GainCounterUpdate(); 
  }
  else if (acc_err >= best_err && op == 2){
    p[gain_counter] += dp[gain_counter];
    dp[gain_counter] *= 0.9;
    op = 1;
    PID::GainCounterUpdate();
  }
  else if (acc_err >= best_err && op == 1){
    op = 2;
  }

  cout << "Gain_counter: " << gain_counter << "\n";  
}

void PID::TwiddleOp(){ 
  switch(op){
    case 1: p[gain_counter] += dp[gain_counter];
	    break;
    case 2: p[gain_counter] -= 2*dp[gain_counter];
            break;
  }
}

void PID::GainCounterUpdate(){
  gain_counter = (gain_counter + 1)% p.size();
  if (dp[gain_counter] < 1.0e-10){
    gain_counter = (gain_counter + 1)% p.size();
    if (dp[gain_counter] < 1.0e-10){
      gain_counter = (gain_counter + 1)% p.size();
    }
  }
}
