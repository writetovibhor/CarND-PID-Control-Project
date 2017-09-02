#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// const double max_speed = 50;

int main()
{
  uWS::Hub h;

  PID pid_steering;

  // Twiddle not used for throttle
  // PID pid_throttle;
  
  // TODO: Initialize the pid variable.

  pid_steering.type = 0;
  // pid_throttle.type = 1;

  bool enable_twiddle = false;
  bool enable_manual_config = false;

  TW_BEST_PARAMS best_steering_params;
  // TW_BEST_PARAMS best_throttle_params;

  // Input to Twiddle
  // best_steering_params.best_Kp = 0.1;
  // best_steering_params.best_Ki = -0.0001;
  // best_steering_params.best_Kd = 3.0;
  // best_steering_params.best_dp[TW_PARAM_TYPE_P] = 0.01;
  // best_steering_params.best_dp[TW_PARAM_TYPE_I] = 0.00001;
  // best_steering_params.best_dp[TW_PARAM_TYPE_D] = 0.1;
  // best_steering_params.tw_param_type = TW_PARAM_TYPE_D;
  // best_steering_params.best_error = std::numeric_limits<double>::max();

  // Result of Twiddle
  best_steering_params.best_Kp = 0.30923;
  best_steering_params.best_Ki = 0.000103916;
  best_steering_params.best_Kd = 4.60079;
  best_steering_params.best_dp[TW_PARAM_TYPE_P] = 0.0214359;
  best_steering_params.best_dp[TW_PARAM_TYPE_I] = 1.75384e-05;
  best_steering_params.best_dp[TW_PARAM_TYPE_D] = 0.214359;
  best_steering_params.tw_param_type = TW_PARAM_TYPE_D;
  best_steering_params.best_error = std::numeric_limits<double>::max();

  // best_throttle_params.best_Kp = 0.0;
  // best_throttle_params.best_Ki = 0.0;
  // best_throttle_params.best_Kd = 0.0;
  // best_throttle_params.best_dp[TW_PARAM_TYPE_P] = 1.0;
  // best_throttle_params.best_dp[TW_PARAM_TYPE_I] = 1.0;
  // best_throttle_params.best_dp[TW_PARAM_TYPE_D] = 1.0;
  // best_throttle_params.tw_param_type = TW_PARAM_TYPE_P;
  // best_throttle_params.best_error = std::numeric_limits<double>::max();

  pid_steering.Init(best_steering_params, enable_twiddle);
  // pid_throttle.Init(best_throttle_params, enable_twiddle);

  double squared_error = 0.0;
  int steps = 0;

  h.onMessage([&pid_steering, &enable_twiddle, &best_steering_params, &enable_manual_config, &squared_error, &steps](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // std::cout << "cte: " << cte << std::endl;
          if (enable_manual_config) {
            squared_error += std::pow(cte, 2);
            steps++;
            if (steps > 4000) {
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              ws.close();
              return;
            }
            std::cout << steps << "\t" << std::sqrt(squared_error / steps) << std::endl;
          }

          pid_steering.UpdateError(cte);
          double steer_value = pid_steering.TotalError();
          if (steer_value < -1) {
            steer_value = -1;
          }
          if (steer_value > 1) {
            steer_value = 1;
          }
          // pid_throttle.UpdateError((max_speed - speed) * cte);
          // double throttle_value = fabs(pid_throttle.TotalError());
          double throttle_value = 0.2;
          if (speed > 0.1) {
            throttle_value = 0.1 + 0.6 * ((10 - fabs(angle)) / 10);
          }
          // if (throttle_value > max_speed / 100) {
          //   throttle_value = max_speed / 100;
          // }
          // std::cout << "throttle: " << throttle_value << std::endl;
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          if (enable_twiddle) {
            // if ((pid_steering.tw_step_count < 10 * pid_steering.tw_total_steps) && (pid_throttle.tw_step_count < 10 * pid_throttle.tw_total_steps)) {
            if (!(pid_steering.found_best || (pid_steering.tw_step_count > pid_steering.tw_total_steps * 10) || (pid_steering.tw_step_count > 1000 && speed < 0.1))) {
              json msgJson;
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              // std::cout << msg << std::endl;
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } else {
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              best_steering_params = pid_steering.best_params;
              // best_throttle_params = pid_throttle.best_params;
              if (!pid_steering.found_best) {
                best_steering_params.tw_param_type = pid_steering.NextParamType();
              }
              // if (!pid_throttle.found_best) {
              //   best_throttle_params.tw_param_type = pid_throttle.NextParamType();
              // }
              pid_steering.Init(best_steering_params, enable_twiddle);
              // pid_throttle.Init(best_throttle_params, enable_twiddle);
              std::cout << "reset steering " << " " << best_steering_params.best_Kp << ", " << 
                                                best_steering_params.best_Ki << ", " << 
                                                best_steering_params.best_Kd << ", " << 
                                                best_steering_params.best_dp[TW_PARAM_TYPE_P] << ", " << 
                                                best_steering_params.best_dp[TW_PARAM_TYPE_I] << ", " << 
                                                best_steering_params.best_dp[TW_PARAM_TYPE_D] << ", " << 
                                                ((best_steering_params.tw_param_type == TW_PARAM_TYPE_P)?"TW_PARAM_TYPE_P":((best_steering_params.tw_param_type == TW_PARAM_TYPE_I)?"TW_PARAM_TYPE_I":"TW_PARAM_TYPE_D")) << std::endl;
              // std::cout << "reset throttle " << " " << best_throttle_params.best_Kp << ", " << 
              //                                   best_throttle_params.best_Ki << ", " << 
              //                                   best_throttle_params.best_Kd << ", " << 
              //                                   best_throttle_params.best_dp[TW_PARAM_TYPE_P] << ", " << 
              //                                   best_throttle_params.best_dp[TW_PARAM_TYPE_I] << ", " << 
              //                                   best_throttle_params.best_dp[TW_PARAM_TYPE_D] << ", " << 
              //                                   ((best_throttle_params.tw_param_type == TW_PARAM_TYPE_P)?"TW_PARAM_TYPE_P":((best_throttle_params.tw_param_type == TW_PARAM_TYPE_I)?"TW_PARAM_TYPE_I":"TW_PARAM_TYPE_D")) << std::endl;
            }
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
