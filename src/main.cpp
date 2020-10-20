#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
//#include "twiddle.h"
//#include <vector>

// for convenience
using nlohmann::json;
using std::string;

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

  PID pid;
  PID throttle_pid;
  /**
   * TODO: Initialize the pid variable.
   */
  vector<double> p =  {0.21, 0.002009, 1.5001}; //initial steering PID parameters. manually tuned values were : 0.2,0.002,1.5. Now we are using twiddle tuned values
  pid.Init(p[0],p[1],p[2]); //Initialize PID 
  
  double previous_error = 99999.0; //initialize previous cte squared error for throttle control
  double throttle_val = 0.1; //initial throttle value
  int state = 0; // initial twiddle state
  
  vector<double> dp = {0.01,0.0001,0.01}; //initial delta values for twiddle
  int param_idx = 0; //initial twiddle parameter to optimize
  double total_error = 0.0; //initialize PID total error to be used in twiddle
  int num_iters = 0; //keep track of iterations
  int step_size = 400; //number of iterations to capture the cte after tweaking one parameter in twiddle
  double threshold = 0.001; //twiddle dp threshold
  double best_err = 9999; //initialize twiddle best error gotten so far
  bool done = false; //flag for determining if twiddle has finished
  h.onMessage([&pid,&previous_error,&throttle_val,&p,&dp,&param_idx,&total_error,&num_iters,&step_size,&threshold,&state,&best_err,&done](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
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
          //double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          bool twiddle1 = false; //flag to apply twiddle to steering PID
          //bool twiddle2 = false; //apply twiddle to throttle PID
          //bool pid2 = false;
          bool reset = false;
          //steering PID
          vector<double> params;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          if (steer_value > 1.0) steer_value = 1.0;
          else if (steer_value < -1.0) steer_value = -1.0; 
          
          //if (!pid2){
          
          // Basic throttle control
          double current_error = cte*cte;
          if (current_error < previous_error){
            if (speed < 12.0){
              throttle_val += 0.01;
            }
          }
          else if (throttle_val > 0.11){
            throttle_val -= 0.1*fabs(cte);
          }
          previous_error = current_error;
          //}
          
		  // Reset if cte exceeds 1.5 (getting out of the lane)
          if ((fabs(cte) > 1.5)||reset){
            throttle_val = 0.1;
            total_error = 0.0;
            steer_value = 0.0;
            pid.Init(p[0],p[1],p[2]);
            
            /*if (pid2){
            	throttle_pid.resetErrors();
            }
            */
            std::string reset_msg = "42[\"reset\",{}]";
    		ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_val;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
          num_iters++;
          //apply twiddle if needed
          if (twiddle1 && !done){
            //allow some time to accumulate the error
            if (num_iters%step_size == 0){
              total_error += cte*cte;
            }
            // after we have gathered enough error data we can determine if the changes in the parameters improved, made worse or didn't have effect in the performance
            if (num_iters%(2*step_size)==0){
              std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
              std::cout << "state: " << state << " param: " << param_idx << " p: " << p[0] << " " << p[1] << " " << p[2] <<" dp: " << dp[0] << " " << dp[1] << " " << dp[2] << std::endl;
              total_error/=step_size; //calculate the average error in the trajectory=2*step_size
              double sumdp = dp[0]+dp[1]+dp[2];
              if (sumdp > threshold){
                if (state == 0){ // Initial state of twiddle
                  best_err = total_error; //initialize best error that we will use as reference
                  state=1; //switch to next state
                  p[param_idx]+=dp[param_idx];
                }
                else if (state == 1){ //verify if increasing the parameter value improved performance
                  if (total_error < best_err){
                    best_err = total_error;
                    dp[param_idx]*=1.1;
                    param_idx++;  //switch to next parameter
                    param_idx %= 3; 
                    p[param_idx]+=dp[param_idx];
                  }
                  else{
                    p[param_idx]-=2*dp[param_idx];
                    state=2;
                  }
                }
                else if (state == 2){
                  if (total_error < best_err){
                    best_err = total_error;
                    dp[param_idx]*=1.1;
                    param_idx++;
                    param_idx %= 3;
                    p[param_idx]+=dp[param_idx];
                    state = 1;
                  }
                  else{
                    p[param_idx]+=dp[param_idx];
                    dp[param_idx]*=0.9;
                    param_idx++;
                    param_idx %= 3;
                    p[param_idx]+=dp[param_idx];
                    state = 1;
                  }
                }
                std::cout << "iter: " << num_iters << " best error: " << best_err << std::endl;
              }
              else{
                done = true;
                std::cout << "best params found: " << "Kp-> "<< p[0] << " Kd-> "<< p[1] << " Ki-> "<< p[2] << std::endl;
              }
              total_error=0.0;
              //pid.resetErrors();
              //pid.setParams(p);
              pid.Init(p[0],p[1],p[2]);
              //reset = true;
            }
          }
          else if (num_iters%(2*step_size)==0){
            pid.Init(p[0],p[1],p[2]);
          }
            
          //throttle PID (To be implemented later)
          /*if (pid2){
            throttle_pid.UpdateError(cte);
            throttle_val = throttle_pid.TotalError();

            if (throttle_val > 0.7) throttle_val = 0.7;
            else if (throttle_val < 0.15) throttle_val = 0.15; 
          }*/
          
          /*
          if (twiddle2){
            vector<double> th_params;
            throttle_tw.run(cte);
            th_params = throttle_tw.get_params();
            throttle_pid.setParams(th_params);
            
            cout << "throttle best params: " << "Kp-> "<< th_params[0] << " Kd-> "<< th_params[1] << " Ki-> "<< th_params[2] << endl;
          }*/
          
          
          // DEBUG
          /*if (!done){
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          }*/
          
          
            
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