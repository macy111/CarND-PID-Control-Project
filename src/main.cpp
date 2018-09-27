#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>
#include <float.h>
#include <numeric>
#include <unistd.h>


// for convenience
using json = nlohmann::json;
using namespace std;

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
			
  PID pid;
  // TODO: Initialize the pid variable.
	vector<double> k = {0.334155, 0, 4.80807};//the final choosed p,i,d coefficients
	vector<double> dk = {0.01, 0, 0.5};
	pid.Init(k[0], k[1], k[2]);
	double tol = 0.01;
	double best_error = DBL_MAX;
	double loss = 0.0;
	int count = 0;
	int n_run = 5000;
	int tune_index = 0;
	bool tune_k = false;// true:tune p,i,d value by twiddle
	bool is_initialized = false;
	bool is_back = false;
	
  h.onMessage([&pid,&tune_k,&k, &dk, &tol, &best_error, &loss, &count, &n_run, &tune_index, &is_initialized, &is_back](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
					if(tune_k){
						/**
						* Tune p,i,d coefficients through twiddle. 
						*	It's a little different from the code in the class video.
						* But the key idea is the same.
						*/
						if(count<n_run){
							count++;
							cout <<"current p:"<<k[0]<<" i:"<<k[1]<<" d:"<<k[2]<<endl;
							loss += cte * cte;
							cout<< count <<endl;
						}else{

							count = 0;
							loss = loss/n_run;
							if(!is_initialized){
								best_error = loss;
								is_initialized = true;
								k[tune_index] += dk[tune_index];
							}else{

								if(accumulate(dk.begin(), dk.end(), 0.0) > tol){

									if(loss < best_error){
										cout <<"Good! loss:"<<loss<<" best_error:"<<best_error<<" tune_index:"<<tune_index<<endl;
										cout <<"best p:"<<k[0]<<" i:"<<k[1]<<" d:"<<k[2]<<endl;
										best_error = loss;
										dk[tune_index] *= 1.1;
										is_back=false;
										tune_index++;
										//tune_index= (tune_index == 1 ? (tune_index+1):tune_index);//skip the tune of i (save time) 
										if(tune_index == 3){
											tune_index = 0;
										}
										k[tune_index] += dk[tune_index];
									}else{
										if(is_back){
											cout <<"Bad2! loss:"<<loss<<" best_error:"<<best_error<<" tune_index:"<<tune_index<<endl;
											k[tune_index] += dk[tune_index];
											dk[tune_index] *= 0.9;
											tune_index++;
											//tune_index= (tune_index == 1 ? (tune_index+1):tune_index);//skip the tune of i (save time) 
											if(tune_index == 3){
												tune_index = 0;
											}
											k[tune_index] += dk[tune_index];
											is_back=false;
										}else{
											cout <<"Bad1! loss:"<<loss<<" best_error:"<<best_error<<" tune_index:"<<tune_index<<endl;
											k[tune_index] -= 2*dk[tune_index];
											is_back = true;
										}
									}// loss err

									pid.Init(k[0], k[1], k[2]);
									loss = 0;
									cout<<"p:"<<k[0]<<" i:"<<k[1]<<" d:"<<k[2]<<" dk sum:"<< accumulate(dk.begin(), dk.end(), 0.0)<<endl;
									cout<<"dp:"<<dk[0]<<" di:"<<dk[1]<<" dd:"<<dk[2]<<" dk sum:"<< accumulate(dk.begin(), dk.end(), 0.0)<<endl;
								}else{
									tune_k = false;
								}//tol
								
							}//init 
							cout<<"please reset the simulate:"<<endl;
							sleep(5);//give some time to reset the car to the original state.
							cout<<"continue:"<<endl;
						}//count

					}

	        pid.UpdateError(cte);
					steer_value = pid.TotalError();
					//std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
					//steer_value = tanh(steer_value);	

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
