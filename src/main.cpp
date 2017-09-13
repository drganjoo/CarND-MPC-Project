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
#include <chrono>

using namespace std::chrono;
using namespace std;
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

int main() {
  uWS::Hub h;
  MPC mpc;
  unsigned int iterations = 0;
//  double last_cte = 0.0;
//  std::chrono::system_clock::time_point last_call = system_clock::now();

//  bool initialized = false;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
//    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          iterations++;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // transform all points sent by the simulator into car coordinate system
          Eigen::VectorXd ptsx_transform(ptsx.size());
          Eigen::VectorXd ptsy_transform(ptsx.size());

          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx_transform[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
            ptsy_transform[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
          }

          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // figure out the cte and epsi
          // epsi = psi - psi_desired
          // psi_desired = atan(f'(x))
          // f'(x) = coeffs[1] + 2 * px * coeffs[2] + 3 * px * coeffs[3] * pow(coeffs[3], 2)
          // since psi = 0, px = 0 therefore we are left with:

          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          Result mpc_result = mpc.Solve(state, coeffs);

//          const double Lf = 2.67;
//          const double steering_to_degrees = deg2rad(25.0) * Lf;
//
//          double steer_value = vars[0] / steering_to_degrees;
//          double throttle_value = vars[1];

          double steer_value;
          double throttle_value;

//          if (!initialized) {
//            initialized = true;
//            last_call = system_clock::now();
//            last_cte = cte;
//            steer_value = 0;
//            throttle_value = 0;
//          }
//          else {
//            const double kp = 0.09, kd = 0.2;
//
//            double dt = duration_cast<milliseconds>(system_clock::now() - last_call).count() / 1000.0;
//            last_call = system_clock::now();
//
//            steer_value = kp * cte + kd * (cte - last_cte) / dt;
//            throttle_value = 0.3;
//
//            last_cte = cte;
//          }
//          if (steer_value > 1)
//            steer_value = 1;
//          else if (steer_value < -11)
//            steer_value = -1;

          steer_value = mpc_result.steering;
          throttle_value = mpc_result.throttle;

          const double steering_to_control = deg2rad(25.0) * Lf;
          steer_value /= steering_to_control;

          cout.precision(5);
//          cout << "CTE:\t" << cte
//               << " MPC steering (Radians):\t" << fixed << mpc_result.steering
//               << " MPC steering (Degrees):\t" << fixed << rad2deg(mpc_result.steering)
//               << "Steering:\t" << steer_value
//               << " Throttle:\t" << fixed << mpc_result.throttle << endl;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (size_t i = 0; i < mpc_result.x.size(); i++) {
            //mpc_x_vals.push_back(i);
            //mpc_y_vals.push_back(0);
            mpc_x_vals.push_back(mpc_result.x[i]);

            //cout << i << " = " << mpc_result.x[i] << endl;
            mpc_y_vals.push_back(mpc_result.y[i]);
          }

          //Display the waypoints/reference line
          auto num_points = 25;
          auto distance = 2.5;

          vector<double> next_x_vals(num_points);
          vector<double> next_y_vals(num_points);

          for (double  i = 0; i < num_points; i++) {
            next_x_vals.push_back(i * distance);
            next_y_vals.push_back(polyeval(coeffs, i * distance));
          }

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          //this_thread::sleep_for(chrono::milliseconds(100));
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
