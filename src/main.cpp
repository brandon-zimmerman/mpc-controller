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

int main(int argc, char *argv[]) {

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // default weights
  double cte_weight = 2000.0;
  double epsi_weight = 2000.0;
  double v_weight = 1.0;
  double delta_weight = 10.0;
  double a_weight = 10.0;
  double delta_gap_weight = 100000.0;
  double a_gap_weight = 10000.0;
  double latency_adjustment_sec = 0.045;


  if(argc>0 && argv[1] != NULL) cte_weight = atof(argv[1]);
  if(argc>1) epsi_weight = atof(argv[2]);
  if(argc>2) v_weight = atof(argv[3]);
  if(argc>3) delta_weight = atof(argv[4]);
  if(argc>4) a_weight = atof(argv[5]);
  if(argc>5) delta_gap_weight = atof(argv[6]);
  if(argc>6) a_gap_weight = atof(argv[7]);
  if(argc>7) latency_adjustment_sec = atof(argv[8]);

  mpc.init(cte_weight, epsi_weight, v_weight, delta_weight, a_weight, delta_gap_weight, a_gap_weight);

  h.onMessage([&mpc, &latency_adjustment_sec](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    const double Lf = 2.67;

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];

          // steering and throttle
          const double steer_value = j[1]["steering_angle"];
          const double throttle_value = j[1]["throttle"];


          // shift car reference angle 90 degree
          // doing this allows us to assume px, py, and psi to be zero, thereby
          // simplifying calculations
          for(unsigned int i = 0; i<ptsx.size(); i++)
          {
            const double shift_x = ptsx[i]-px;
            const double shift_y = ptsy[i]-py;

            ptsx[i] = (shift_x * cos(-psi) - shift_y * sin(-psi));
            ptsy[i] = (shift_x * sin(-psi) + shift_y * cos(-psi));

          }

          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          Eigen::VectorXd coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          const double cte = polyeval(coeffs, 0);
          const double epsi = -atan(coeffs[1]);

          const double lat_px = 0.0 + v * latency_adjustment_sec;
          const double lat_py = 0.0;
          const double lat_psi = 0.0 + v * (-steer_value) / Lf * latency_adjustment_sec;
          const double lat_v = v + throttle_value * latency_adjustment_sec;
          const double lat_cte = cte + v * sin(epsi) * latency_adjustment_sec;
          const double lat_epsi = epsi + v * (-steer_value) / Lf * latency_adjustment_sec;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          Eigen::VectorXd state(6);
          state << lat_px, lat_py, lat_psi, lat_v, lat_cte, lat_epsi;

          auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -vars[0] / deg2rad(25) * Lf;
          msgJson["throttle"] = vars[1];

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned int i = 2; i <  vars.size(); i++)
          {
            if(i%2 == 0)
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          const unsigned int poly_inc = 2.5;
          const unsigned int num_points = 25;
          for (unsigned int i = 1; i < num_points; i++)
          {
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
  if (h.listen("127.0.0.1", port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
