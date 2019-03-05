#include <algorithm>
#include <iterator>
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

#define N 50

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_s, pid_t;
  
  int n = 0;
  double err = 0.0, total_err = 0.0, best_err = 100000.0;
  double p_s[3] = {1.1147, 0.000342, 12.9026}, dp_s[3] = {0.01, 0.0001, 0.1};

  double p[3] = {0.179356, 0.000771561, 5.65027}, dp[3] = {0.01, 0.0001, 0.1};
  double best_err_p[3] = {p[0], p[1], p[2]};

// repeatedly tuning one parameter at a time, then update the initial value of that parameter before tuning another

// Reference: https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
// Steering: Reset all to zero, went off track soon after started. Initialized P to 0.1
// tuning P 0.11, 0, 0, -> tuning D 0.11, 0, 10.7309 -> tuning P 1.1147, 0, 10.73 -> tuning D 1.1147, 0, 11.6787
// -> tuning P 1.15, 0, 11.6787 -> tuning D 1.15, 0, 12.9026

// Throttle: tuning P 0.0331 (off track) -> tuning P 0.100787 (off track) -> tuning D 0.11, 0, 1.10639
// -> tuning P 0.168256,0, 1.10639 -> tuning D 0.168256,0,3.79715 -> tuning P 0.179356 -> tuning D 5.65027
// -> tuning I 0.000771561

  /* twiddle one parameter for any certain run of simulator, 
   * because the magnitude of errors for three parameters are different, 
   * so tracking one minimum error(best_err) is incorrect
   */
  int p_index = 1; //2;//1;//0;
  bool twiddle = true; //true;
  bool pre_first_run = true, pre_second_run = true; // flags to keep track of which step of twiddling algorithm we are executing
  bool reset_flags = false;
  
  
  /**
   * Initialize the pid variable.
   */
  pid_s.Init(p_s[0], p_s[1], p_s[2]);
  pid_t.Init(p[0], p[1], p[2]);
  
  h.onMessage([&pid_s, &pid_t, &n, &err, &total_err, &best_err, &p, &dp, &best_err_p, &p_index, &twiddle, &pre_first_run, &pre_second_run, &reset_flags]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, throttle_value;
          /**
           * Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid_s.UpdateError(cte);
          pid_t.UpdateError(fabs(cte));

          if(twiddle == true) // if twiddling is needed
          {
            n += 1;
          	if ( n > N ) // start twiddling
            {
              total_err += cte * cte;
              err = total_err / (n-N);
              // tune each parameter
              if(pre_first_run == true)
              {
                p[p_index] += dp[p_index];
                pre_first_run = false;
              }
              else
              {
                if(err < best_err)
                {
                  best_err = err;
                  best_err_p[p_index] = p[p_index];
                  dp[p_index] *= 1.1;
                  // done for current cycle of twiddling
                  reset_flags = true;
                  cout << "best_err: " << best_err << endl;
                  cout << "p_index " << p_index << " p[index] " << p[p_index] << " dp[index] " << dp[p_index] << endl;
                }
                else // previous addition of dp did not make error smaller
                {
                  if(pre_second_run == true)
                  {
                    p[p_index] -= 2 * dp[p_index];
                    pre_second_run = false;
                  }
                  else
                  {
                    if(err < best_err)
                    {
                      best_err = err;
                      best_err_p[p_index] = p[p_index];
                      dp[p_index] *= 1.1;
                      cout << "best_err: " << best_err << endl;
                      cout << "p_index " << p_index << " p[index] " << p[p_index] << " dp[index] " << dp[p_index] << endl;
                    }
                    else
                    {
                      p[p_index] += dp[p_index];
                      dp[p_index] *= 0.9;
                    }
                    // reset flags
                    reset_flags = true;
                  } // end of if(pre_second_run == true), else
                }
              } // end of if(pre_first_run == true), else

              if (reset_flags)
              {

                cout << "Twiddle #" << n << " err " << err << " dp " << dp[p_index] << endl;
                // p_index = (p_index + 1) % 3;
                pre_first_run = pre_second_run = true;
                reset_flags = false;
              }
              if ( n > 2 * N ) 
              {
                total_err = 0.0;
                //best_err = 100000.0;
                n = 0;

                copy(begin(best_err_p), end(best_err_p), begin(p));

                dp[0] = 0.01; dp[1] = 0.00001; dp[2] = 0.1; 
                
                pid_t.Init(best_err_p[0], best_err_p[1], best_err_p[2]);
                cout << "Reinit p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;              
              }
            }
            else // n <= N, wait till enough iterations have been executed
            {  
              // use current pid
            }
            
          }
          
          steer_value = pid_s.Response();
          throttle_value = 0.5 + pid_t.Response();

          // DEBUG
          /*std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;*/
          /*std::cout << "CTE: " << cte << " Throttle Value: " << throttle_value 
                    << std::endl;*/

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          //std::cout << msg << std::endl;

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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
