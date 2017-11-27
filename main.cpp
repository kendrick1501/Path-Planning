#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper_functions.h"
#include "ego_.h"

//#include <ctime>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<near_vehicle> near_vehicles;
  EGO_ car_; 

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // Reference velocity to target
  double ref_vel = 0.; //mph
  
  car_.lane = 1;
  car_.goal_lane = 1;
  car_.nav_time = 0;
  car_.nav_status = "EC"; // Efficient cruising
  
  car_.time_in_pl = 0;
  
  car_.prev_action = "KL"; // Keep lane
  car_.prev_cost = 0;
  car_.prev_potential_cost = 0;
  
  //clock_t tstart, tend; 
  //tstart = clock();

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&near_vehicles,&car_](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          ++car_.nav_time;
          // j[1] is the data JSON object
          
        	// Main car's localization Data
        	car_.x = j[1]["x"];
        	car_.y = j[1]["y"];
        	car_.s = j[1]["s"];
        	car_.d = j[1]["d"];
        	car_.yaw = j[1]["yaw"];
        	car_.speed = j[1]["speed"];
        	car_.speed /= 2.24; // MPH -> mps

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];

	        int prev_path_size = previous_path_x.size();
	
	        // Next (x, y) values
	        vector<double> next_xs;
        	vector<double> next_ys;

	        // Find ref_v to use
	        //double crit_distance = 15;
	        //double es_ = 0.;
	        //double es_dot = 0.;
	        //double target_vel = 49.5; //mph
	        
	        for (int i=0; i<sensor_fusion.size(); ++i){
	        
	          int detected_id = sensor_fusion[i][0];
	        	double detected_vx = sensor_fusion[i][3];
			      double detected_vy = sensor_fusion[i][4];
			      double detected_s = sensor_fusion[i][5];
			      float detected_d = sensor_fusion[i][6];
	        
	          // Check near by vehicles within a range of 100 meters
	          double delta_s = detected_s - car_.s;
	          if (fabs(delta_s)<100){ 
	            near_vehicle near_vehicle_tmp;
	            near_vehicle_tmp.id = detected_id;
	            		            
	            vector<int> detected_;
	            detected_ = find_veh(near_vehicles, detected_id); // Verify if the vehicle has been detected before
	            		            
	            if (detected_[0] == 0){        
	              near_vehicle_tmp.vx = detected_vx;
	              near_vehicle_tmp.vy = detected_vy;
	              near_vehicle_tmp.s = detected_s;
	              near_vehicle_tmp.d.push_back(detected_d);
	              near_vehicle_tmp.speed.push_back(distance(detected_vx,detected_vy,0.,0.));
	              near_vehicle_tmp.delta_s = delta_s;
	              near_vehicles.push_back(near_vehicle_tmp);            
	            }else if (detected_[0] == 1){           
	            	int indx = detected_[1]; // Detected vehicle index within vector<near_vehicle> near_vehicles
	            	int size_data = near_vehicles[indx].d.size();
	              if(size_data>600){ // Clear data when it has more than 600 samples
	              	vector<double> speed_tmp;
	              	vector<double> d_tmp;              	
	              	for(int i=size_data-150; i<size_data; ++i){
	              		speed_tmp.push_back(near_vehicles[indx].speed[i]);
	              		d_tmp.push_back(near_vehicles[indx].d[i]);
	              	}
	              	near_vehicles[indx].speed.clear();
	              	near_vehicles[indx].speed = speed_tmp;
	              	near_vehicles[indx].d.clear();
	              	near_vehicles[indx].d = d_tmp;
	              }
	              near_vehicles[indx].vx = detected_vx;
	              near_vehicles[indx].vy = detected_vy;
	              near_vehicles[indx].s = detected_s;
	              near_vehicles[indx].d.push_back(detected_d);
	              near_vehicles[indx].speed.push_back(distance(detected_vx,detected_vy,0.,0.));
	              near_vehicles[indx].delta_s = delta_s;
	            }
	          }						
					}
						
	        double delta_ref = navigation_fsm(near_vehicles, car_); // Behavior navigation function. Generates reference speed and goal lane for the path generation module.

			    ref_vel += delta_ref;
			    if(ref_vel > 49.5){
				  	ref_vel = 49.5;		
				 	}else if(ref_vel < 0){
						ref_vel = 0;
					}

	        // Assign current coordinates of the ego vehicle to the reference coordinates
	        double ref_x = car_.x;
	        double ref_y = car_.y;
	        double ref_yaw = car_.yaw;
	        
	        // Create a vector of waypoints coordinates 
        	vector<double> waypoints_x;
        	vector<double> waypoints_y;

	        // Check previous path size to initialize waypoints of current path
	        if(prev_path_size<2){
          
		        double prev_car_x = car_.x - cos(car_.yaw);
		        double prev_car_y = car_.y - sin(car_.yaw);

		        waypoints_x.push_back(prev_car_x);
		        waypoints_x.push_back(car_.x);

		        waypoints_y.push_back(prev_car_y);
		        waypoints_y.push_back(car_.y);
	        }else{

		        ref_x = previous_path_x[prev_path_size-1];
		        ref_y = previous_path_y[prev_path_size-1];

		        double prev_ref_x = previous_path_x[prev_path_size-2];
		        double prev_ref_y = previous_path_y[prev_path_size-2];
		        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

		        waypoints_x.push_back(prev_ref_x);
		        waypoints_x.push_back(ref_x);

		        waypoints_y.push_back(prev_ref_y);
		        waypoints_y.push_back(ref_y); 
	        }

	        // Generate 5 waypoints points evenly spaced in frenet coordinate s according to delta_s and goal lane.
	        double delta_s = 30 + round(25/(1 + exp(25 - 2.24 * car_.speed))); // The s-distance between waypoints is dynamically adjusted according to the actual speed of the ego vehicle
	        int n_waypoints = 5;
	        for(int i=0; i<n_waypoints; ++i){
		        vector<double> new_waypoint = getXY(car_.s + (i+1)*delta_s, (2 + 4*car_.lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
		        waypoints_x.push_back(new_waypoint[0]);
		        waypoints_y.push_back(new_waypoint[1]);
	        }
	
	        // Transform waypoints from world reference frame to local reference frame
	        for (int i=0; i<waypoints_x.size(); ++i){
		        double delta_x = waypoints_x[i] - ref_x;
		        double delta_y = waypoints_y[i] - ref_y;
		
		        double cyaw = cos(ref_yaw);
		        double syaw = sin(ref_yaw);
		        waypoints_x[i] = delta_x * cyaw + delta_y * syaw;
		        waypoints_y[i] = - delta_x * syaw + delta_y * cyaw;
	        }

	        // Calculate spline
	        tk::spline s;
	        s.set_points(waypoints_x, waypoints_y);

	        // Calculate y coordinate from the spline
	        double target_x = delta_s;
	        double target_y = s(target_x);
	        double target_distance = distance(target_x,target_y,0.,0.); // Calculating from the local reference frame origin*
	
	        // Initialize the new path with previous path points
	        for (int i=0; i<prev_path_size; ++i){
		        next_xs.push_back(previous_path_x[i]);
		        next_ys.push_back(previous_path_y[i]);
	        }
		
          // Add remaining points to the new path
          double delta_x_spline = 0;
          for (int i=0; i<50-prev_path_size; ++i){
	
	          double N = 112*target_distance/ref_vel;//112 = 2.24 MPH/mps / 0.02 ms
	          double x_spline = delta_x_spline + target_x/N;
	          double y_spline = s(x_spline);
	          delta_x_spline = x_spline;

	          // Express spline points in the world reference frame
	          double x_local = x_spline;
	          double y_local = y_spline;

	          double cyaw = cos(ref_yaw);
	          double syaw = sin(ref_yaw);
	          x_spline = ref_x + x_local * cyaw - y_local * syaw;
	          y_spline = ref_y + x_local * syaw  + y_local * cyaw;

	          next_xs.push_back(x_spline);
	          next_ys.push_back(y_spline);
          }

        	json msgJson;
        	msgJson["next_x"] = next_xs;
        	msgJson["next_y"] = next_ys;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } 
      
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
      
    }
    
    //tend = clock(); 
    //cout << "One cyle "<< (float)(tend - tstart)/CLOCKS_PER_SEC <<" second(s)."<< endl;
    //tstart = tend;
    
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
