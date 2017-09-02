#include "PID.h"
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(TW_BEST_PARAMS best_params, bool do_twiddle) {
  this->best_params = best_params;
  this->Kp = best_params.best_Kp;
  this->Ki = best_params.best_Ki;
  this->Kd = best_params.best_Kd;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  is_initialized_ = false;
  do_twiddle_ = do_twiddle;
  prev_cte_ = 0.0;
  cte_sum_ = 0.0;
  tw_total_steps = 2000;
  tw_step_count = 0;
  tw_total_error_ = 0.0;
  tw_best_error_ = best_params.best_error;
  tw_dp_[TW_PARAM_TYPE_P] = best_params.best_dp[TW_PARAM_TYPE_P];
  tw_dp_[TW_PARAM_TYPE_I] = best_params.best_dp[TW_PARAM_TYPE_I];
  tw_dp_[TW_PARAM_TYPE_D] = best_params.best_dp[TW_PARAM_TYPE_D];
  tw_param_type_ = best_params.tw_param_type;
  tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_INCREASE;
  type_name_ = (type == 0)?"Steering":"Throttle";
  found_best = false;
}

void PID::UpdateError(double cte) {
  if (!is_initialized_) {
    is_initialized_ = true;
    prev_cte_ = cte;
  }
  double diff_cte = cte - prev_cte_;
  prev_cte_ = cte;
  cte_sum_ += cte;

  p_error = Kp * cte;
  i_error = Ki * cte_sum_;
  d_error = Kd * diff_cte;

  if (do_twiddle_) {
    tw_step_count++;

    if ((tw_step_count % (2 * tw_total_steps)) > tw_total_steps) {
      tw_total_error_ += std::pow(cte, 2);
    }

    if ((tw_step_count % (2 * tw_total_steps)) == 0) {
      std::cout << type_name_ << " step " << tw_step_count << ", best error " << tw_best_error_ << std::endl;
      if ((tw_step_count > 2 * tw_total_steps) && (tw_total_error_ < tw_best_error_)) {
        tw_best_error_ = tw_total_error_;
        tw_dp_[tw_param_type_] *= 1.1;
        tw_param_type_ = NextParamType();
        tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_INCREASE;
        std::cout << type_name_ << " " << Kp << ", " << 
                                          Ki << ", " << 
                                          Kd << ", " << 
                                          tw_dp_[TW_PARAM_TYPE_P] << ", " << 
                                          tw_dp_[TW_PARAM_TYPE_I] << ", " << 
                                          tw_dp_[TW_PARAM_TYPE_D] << ", " << 
                                          ((tw_param_type_ == TW_PARAM_TYPE_P)?"TW_PARAM_TYPE_P":((tw_param_type_ == TW_PARAM_TYPE_I)?"TW_PARAM_TYPE_I":"TW_PARAM_TYPE_D")) << std::endl;
        found_best = true;
      }
      switch (tw_param_type_) {
        case TW_PARAM_TYPE_P:
          switch (tw_param_probe_type_) {
            case TW_PARAM_PROBE_TYPE_INCREASE:
              Kp += tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DECREASE;
              break;
            case TW_PARAM_PROBE_TYPE_DECREASE:
              Kp -= 2 * tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DONE;
              break;
            default:
              Kp += tw_dp_[tw_param_type_];
              tw_dp_[tw_param_type_] *= 0.9;
              tw_param_type_ = NextParamType();
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_INCREASE;
              break;
          }
          break;
        case TW_PARAM_TYPE_I:
          switch (tw_param_probe_type_) {
            case TW_PARAM_PROBE_TYPE_INCREASE:
              Ki += tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DECREASE;
              break;
            case TW_PARAM_PROBE_TYPE_DECREASE:
              Ki -= 2 * tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DONE;
              break;
            default:
              Ki += tw_dp_[tw_param_type_];
              tw_dp_[tw_param_type_] *= 0.9;
              tw_param_type_ = NextParamType();
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_INCREASE;
              break;
          }
          break;
        case TW_PARAM_TYPE_D:
          switch (tw_param_probe_type_) {
            case TW_PARAM_PROBE_TYPE_INCREASE:
              Kd += tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DECREASE;
              break;
            case TW_PARAM_PROBE_TYPE_DECREASE:
              Kd -= 2 * tw_dp_[tw_param_type_];
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_DONE;
              break;
            default:
              Kd += tw_dp_[tw_param_type_];
              tw_dp_[tw_param_type_] *= 0.9;
              tw_param_type_ = NextParamType();
              tw_param_probe_type_ = TW_PARAM_PROBE_TYPE_INCREASE;
              break;
          }
          break;
        default:
          break;
      }
      best_params.best_Kp = Kp;
      best_params.best_Ki = Ki;
      best_params.best_Kd = Kd;
      best_params.best_dp[TW_PARAM_TYPE_P] = tw_dp_[TW_PARAM_TYPE_P];
      best_params.best_dp[TW_PARAM_TYPE_I] = tw_dp_[TW_PARAM_TYPE_I];
      best_params.best_dp[TW_PARAM_TYPE_D] = tw_dp_[TW_PARAM_TYPE_D];
      best_params.tw_param_type = tw_param_type_;
      best_params.best_error = tw_best_error_;
    }
  }
}

double PID::TotalError() {
  return - p_error - i_error - d_error;
}

TW_PARAM_TYPE PID::NextParamType() {
  switch (tw_param_type_) {
    case TW_PARAM_TYPE_P:
      return TW_PARAM_TYPE_I;
    case TW_PARAM_TYPE_I:
      return TW_PARAM_TYPE_D;
    default:
      return TW_PARAM_TYPE_P;
  }
}