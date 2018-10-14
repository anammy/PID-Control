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

int main()
{
  uWS::Hub h;

  PID pid_str, pid_thr;
  // TODO: Initialize the pid variable.
  //Initial guess for PID gains
  pid_thr.Init(0.0545, 0.0, 0.0999, 0.00001);
  pid_str.Init(0.165, 0.00410528 , 4.6355, 0.1);
  //Oct 8, 2018: - Initial Kp, Ki, Kd = [0.2, 0.004, 3.0]. Throttle set to 0.3. After Twiddle (forced end), Kp, Ki, Kd = [0.22, 0.004, 3.3] with dp = [0.0105225, 0.000172187, 0.175376] with best_err = 0.0992942. Seems to be have hit local minimum because Kp, Ki, and Kd weren't change. Possibly start with different initial values.
  //Oct 13, 2018: Initial Kp, Ki, Kp = [0.22, 0.004, 3.3]. Throttle set to 0.3 - fabs(steer_value). After twiddle (forced end), Kp, Ki, Kd = [0.257895, 0.00400236, 3.83484].
  //Oct 13, 2018: Initial steering Kp, Ki, Kd = [0.15, 0.004, 5.0, 0.1]. Throttle Kp, Ki, Kd = [0.05, 0.0, 0.1] kept constant. Steering gain optimization complete at Kp, Ki, Kd = [0.165, 0.00410528, 4.6355].
  //Oct 13, 2018: Steering Kp, Ki, Kd = [0.165, 0.00410528, 4.6355] kept constant. Initial throttle Kp, Ki, Kd = [0.05, 0.0, 0.1]. After twiddle (forced end), throttle Kp, Ki, Kd = [0.055, 0.0, 0.1001].
  //Oct 14, 2018: Steering Kp, Ki, Kd = [0.165, 0.00410528, 4.6355] kept constant. Initial throttle Kp, Ki, Kd = [0.05, 0.0, 0.1]. After twiddle (forced end due car crashing once another change was made), throttle Kp, Ki, Kd = [0.0545, 0, 0.0999].

  h.onMessage([&pid_str, &pid_thr](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
	  //Steering angle calculation
	  pid_str.UpdateError(cte);
	  steer_value = pid_str.TotalError();

	  //Throttle value calculation
	  pid_thr.UpdateError(fabs(cte));
	  throttle_value = pid_thr.TotalError();          

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
	  //std::cout << "Kp, Ki, Kd: " << pid_thr.Kp << "\t" << pid_thr.Ki << "\t" << pid_thr.Kd << "\n";
	  //std::cout << "dp = " << pid_thr.dp[0] << "\t" << pid_thr.dp[1] << "\t" << pid_thr.dp[2] << "\n";
	  //std::cout << "Throttle control value, Throttle: " << throttle_value << "\t" << (0.3 + throttle_value) << "\n";
	  //std::cout << "Best Error, Acc_Err: " << pid_str.best_err << "\t" << pid_str.acc_err << "\n";
	
          json msgJson;
          msgJson["steering_angle"] = steer_value;
	  //Original throttle value was 0.3
          msgJson["throttle"] = 0.3 + throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
