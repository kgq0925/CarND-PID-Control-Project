#ifndef PID_H
#define PID_H

class PID {
public:
  int step;
  int min_step;
  double best_error;
  double squared_error;
  double dKp;
  double dKi;
  double dKd;
  double min_tolerance;
  double max_tolerance;
  int twiddle_state;  //-1 decrease 0 start 1 increase
  int twiddle_param;  // 0 p 1 i 2 d

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();
};

#endif /* PID_H */
