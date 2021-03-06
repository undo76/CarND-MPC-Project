#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

Eigen::MatrixXd mapToCarCoordinates(const Eigen::MatrixXd &pts, double px,
                                    double py, double psi) {
  Eigen::MatrixXd transformed(2, pts.cols());
  double c_psi = cos(psi);
  double s_psi = sin(psi);
  auto t_x = Eigen::MatrixXd::Constant(1, pts.cols(), -px * c_psi - py * s_psi);
  auto t_y = Eigen::MatrixXd::Constant(1, pts.cols(), px * s_psi - py * c_psi);
  transformed.row(0) = 
      pts.row(0) * c_psi + pts.row(1) * s_psi + t_x;
  transformed.row(1) = 
      - pts.row(0) * s_psi + pts.row(1) * c_psi + t_y;
  return transformed;
}

vector<double> toVector(Eigen::VectorXd v) {
  vector<double> v2;
  v2.resize(v.size());
  Eigen::VectorXd::Map(&v2[0], v.size()) = v;
  return v2;
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
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
           
          // Latency correction in map coordinates.
          // Based on the current state, I project the
          // future state after latency seconds.
          const double latency = 0.1;
          v += throttle * latency;
          psi -= v * steering_angle * deg2rad(MAX_ANGLE) / Lf * latency;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;          

          // Convert landmark points to Eigen matrix (2 x N).
          Eigen::MatrixXd pts(2, ptsx.size());
          pts.row(0) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              ptsx.data(), ptsx.size());
          pts.row(1) = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
              ptsy.data(), ptsy.size());

          // Landmark points in car coordinates
          auto pts_c = mapToCarCoordinates(pts, px, py, psi);

          // Calculate polynomial coefficients of fitting curve of landmark
          // points in car coordinates. 
          Eigen::Vector4d coeffs = polyfit(pts_c.row(0), pts_c.row(1), 3);
 
          // Car position and orientation in car coordinates ([0, 0], obviously)
          double px_c = 0;
          double py_c = 0;
          double psi_c = 0.;

          // Calculate desired position
          auto fx = polyeval(coeffs, px_c);

          // Calculate cross track error.
          double cte = fx - py_c;

          // Calculate orientation error. 
          // At px_c = 0, the derivative of the polynomial 
          // is just c_1. 
          double epsi = psi_c - atan(coeffs[1]); 

          // Solve, using the current state (in car coordinates)
          Eigen::VectorXd state = Eigen::VectorXd(6);
          state << px_c, py_c, psi_c, v, cte, epsi;
          SolveResult sr = mpc.Solve(state, coeffs);

          // NOTE: Remember to divide by deg2rad(MAX_ANGLE) before you send the
          // steering value back. Otherwise the values will be in between
          // [-deg2rad(MAX_ANGLE), deg2rad(MAX_ANGLE] instead of [-1, 1].
          double steer_value = sr.delta / deg2rad(MAX_ANGLE);
          double throttle_value = sr.a;

          json msgJson;
  
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals = sr.x;
          vector<double> mpc_y_vals= sr.y;

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals = toVector(pts_c.row(0));
          vector<double> next_y_vals = toVector(pts_c.row(1));

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          std::cout << steer_value << ", " << throttle_value << std::endl;
          
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
    std::cerr << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cerr << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cerr << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
