#ifndef PID_H
#define PID_H

#include <vector>
using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  //Controller initialization state
  bool is_initialized = false;
  
  //Twiddle parameters
  int num_steps = 3000;
  int num_settlesteps = 200;
  vector<double> p;
  vector<double> dp;
  double best_err = 0.0;
  double acc_err = 0.0;
  double prev_err = 0.0;
  int counter = 0;
  int gain_counter = 0;
  int op = 1;
  bool first_round = true;
  bool first_pass = true;
  double tol = 0.0;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double K_p, double K_i, double K_d, double tolerance);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  /*
  * Calculate the total PID error for gain parameter optimization using Twiddle.
  */
  double TotalErrorTraining();

  /*
  * Gain parameter optimization using Twiddle
  */
  void Twiddle();
  void TwiddleOp();
  void GainCounterUpdate(); 
};

#endif /* PID_H */
