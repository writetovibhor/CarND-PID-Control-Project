#ifndef PID_H
#define PID_H

#include <string>

typedef enum TW_PARAM_TYPE {
  TW_PARAM_TYPE_P = 0,
  TW_PARAM_TYPE_I = 1,
  TW_PARAM_TYPE_D = 2
} TW_PARAM_TYPE;

typedef enum TW_PARAM_PROBE_TYPE {
  TW_PARAM_PROBE_TYPE_INCREASE = 0,
  TW_PARAM_PROBE_TYPE_DECREASE = 1,
  TW_PARAM_PROBE_TYPE_DONE = 2
} TW_PARAM_PROBE_TYPE;

typedef struct TW_BEST_PARAMS {
  double best_Kp;
  double best_Ki;
  double best_Kd;
  double best_dp[3];
  TW_PARAM_TYPE tw_param_type;
  double best_error;
} TW_BEST_PARAMS;

class PID {
private:
  bool is_initialized_;
  bool do_twiddle_;
  double prev_cte_;
  double cte_sum_;
  double tw_total_error_;
  double tw_best_error_;
  double tw_dp_[3];
  bool tw_increase_param_;
  TW_PARAM_TYPE tw_param_type_;
  TW_PARAM_PROBE_TYPE tw_param_probe_type_;
  std::string type_name_;
public:
  int type;
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


  int tw_total_steps;
  int tw_step_count;

  TW_BEST_PARAMS best_params;

  bool found_best;
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
  void Init(TW_BEST_PARAMS best_params, bool do_twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  TW_PARAM_TYPE NextParamType();
};

#endif /* PID_H */
