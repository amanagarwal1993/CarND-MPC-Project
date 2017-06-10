#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    string sdata = string(data).substr(0, length);
    std::cout << "DATA = " << sdata << "\n";
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // So now we have the current state of the vehicle.
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          v = v * 0.44704;
          
          int ptsize = j[1]["ptsx"].size();
          
          Eigen::VectorXd ptsx(ptsize);
          Eigen::VectorXd ptsy(ptsize);
          
          for (int i=0; i<j[1]["ptsx"].size(); i++) {
            double gx = j[1]["ptsx"][i];
            double gy = j[1]["ptsy"][i];
            ptsx[i] = ((gx - px)*cos(psi) + (gy - py)*sin(psi));
            ptsy[i] = ((gy - py)*cos(psi) - (gx - px)*sin(psi));
          };
          
          double steer_value;
          double throttle_value;
          
          /* Steps to follow:
                1. Get a reference trajectory which is to be followed for the next few points.
                2. Use constraints set actuations such that reference trajectory is followed.
           
                */
          
          Eigen::VectorXd coeffs;
          
          coeffs = polyfit(ptsx, ptsy, 3);
          
          double cte = polyeval(coeffs, px) - py;
          // coeffs[0] + coeffs[1]*px + coeffs[2]*(px * px) + coeffs[3]*(px * px * px)
          
          double epsi = psi - atan(coeffs[1] + coeffs[2]*px*2 + 3*coeffs[2]*(px * px));
          
          Eigen::VectorXd state(6);
          state[0] = px;
          state[1] = py;
          state[2] = psi;
          state[3] = v;
          state[4] = cte;
          state[5] = epsi;
          
          std::cout << state << "\n";
          
          auto result = mpc.Solve(state, coeffs);
          
          steer_value = result[0];
          throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          //Display the waypoints/reference line (yellow)
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          /*
          for (int i=0; i<6; i++) {
            double ax = result[2 + 2*i];
            double ay = result[3 + 2*i];
            mpc_x_vals.push_back(ax);
            mpc_y_vals.push_back(ay);
          }
          
          for (int i=0; i<6; i++) {
            double ax = result[2 + 2*i];
            double ay = result[3 + 2*i];
            next_x_vals.push_back(ax);
            next_y_vals.push_back(polyeval(coeffs, ay));
          }
          */
          msgJson["mpc_x"] = j[1]["ptsx"];
          msgJson["mpc_y"] = j[1]["ptsy"];
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          //std::cout << "yip9 \n";
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          std::cout << result[0] << std::endl;
          std::cout << result[1] << std::endl;
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          //std::cout << "yip10 \n";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          //std::cout << "yip11 \n";
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    else {
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
