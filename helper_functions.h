#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <math.h>
#include <vector>
#include <algorithm>
#include "ego_.h"

using namespace std;

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

//MyOWN

vector<int> find_veh(vector<near_vehicle> near_vehicles, int id);

double navigation_fsm(vector<near_vehicle>& near_vehicles, EGO_& car_);

vector<int> find_veh(vector<near_vehicle> near_vehicles, int id)
{

  vector<int> detected_;
  
  if (near_vehicles.size()>0){
    for (int i = 0; i<near_vehicles.size(); ++i){
      //cout << "iter: "<< i << " Near Vehicles id: " << near_vehicles[i].id << " Vehicle id: " << id <<endl;
      if(near_vehicles[i].id == id){
        detected_.push_back(1);
        detected_.push_back(i);
        return detected_;
      }
    }
  }
  detected_.push_back(0);
  detected_.push_back(99999);
  return detected_;
}

double navigation_fsm(vector<near_vehicle>& near_vehicles, EGO_& car_)
{
	
	double crit_distance = 20;
  double delta_ref = 0.35;
  
  //cout << "Current navigation status: " << car_.nav_status << endl;
  
	if((car_.nav_time > 5) && (near_vehicles.size()>0)){
		
		// Checking vehicles on the same lane
		vector<near_vehicle> same_lane_ahead;
		vector<near_vehicle> same_lane_behind;
		for (int i=0; i<near_vehicles.size(); ++i){
			double veh_d = near_vehicles[i].d.back();
			if((veh_d>4*car_.lane) && (veh_d<4*(car_.lane+1))){
				double delta_s = near_vehicles[i].delta_s;
				if (delta_s >= 0){
					same_lane_ahead.push_back(near_vehicles[i]);
				}else{
				  near_vehicles[i].delta_s = fabs(delta_s);
					same_lane_behind.push_back(near_vehicles[i]);
				}
			}
		}
		
		// Checking vehicles on adjacent lanes
		vector<string> valid_lanes;
		vector<near_vehicle> left_lane_ahead;
	  vector<near_vehicle> left_lane_behind;
	  vector<near_vehicle> right_lane_ahead;
	  vector<near_vehicle> right_lane_behind;
		
	  int adj_ = car_.lane - 1;
	  if(adj_>=0){
	    valid_lanes.push_back("adj_left");
	    
	    // Checking vehicles on adjacent left lane
	    for (int i=0; i<near_vehicles.size(); ++i){
		    double veh_d = near_vehicles[i].d.back();
		    if((veh_d>4*adj_) && (veh_d<4*(adj_+1))){
			    double delta_s = near_vehicles[i].delta_s;
			    if (delta_s >= 0){
				    left_lane_ahead.push_back(near_vehicles[i]);
			    }else{
			      near_vehicles[i].delta_s = fabs(delta_s);
				    left_lane_behind.push_back(near_vehicles[i]);
			    }
		    }
	    }
	  }
	  
	  adj_ = car_.lane + 1;
	  if(adj_<=2){
	    valid_lanes.push_back("adj_right");
	    
	    // Checking vehicles on adjacent right lane
	    for (int i=0; i<near_vehicles.size(); ++i){
		    double veh_d = near_vehicles[i].d.back();
		    if((veh_d>4*adj_) && (veh_d<4*(adj_+1))){
			    double delta_s = near_vehicles[i].delta_s;
			    if (delta_s >= 0){
				    right_lane_ahead.push_back(near_vehicles[i]);
			    }else{
			      near_vehicles[i].delta_s = fabs(delta_s);
				    right_lane_behind.push_back(near_vehicles[i]);
			    }
		    }
	    }
	  }
	  
	  // Checking vehicles on the furtherst lane
		vector<near_vehicle> further_lane_ahead;
	  vector<near_vehicle> further_lane_behind;
	
	  if(car_.lane==0){
	  	adj_ = 2;
	  }else if(car_.lane==2){
	  	adj_ = 0;
	  }
	  if((car_.lane==0) || (car_.lane==2)){   
	    for (int i=0; i<near_vehicles.size(); ++i){
		    double veh_d = near_vehicles[i].d.back();
		    if((veh_d>4*adj_) && (veh_d<4*(adj_+1))){
			    double delta_s = near_vehicles[i].delta_s;
			    if (delta_s >= 0){
				    further_lane_ahead.push_back(near_vehicles[i]);
			    }else{
			      near_vehicles[i].delta_s = fabs(delta_s);
				    further_lane_behind.push_back(near_vehicles[i]);
			    }
		    }
	    }
	  }
		
		if(same_lane_ahead.size()>0){ // Check if there is any vehicle ahead
			sort( same_lane_ahead.begin( ), same_lane_ahead.end( ), [ ]( const near_vehicle& lhs, const near_vehicle& rhs ){return lhs.delta_s < rhs.delta_s;});			
			if(same_lane_ahead[0].delta_s < 3 * crit_distance){ // Check for vehicles within 3 times the prescribed critical distance
			  if(car_.nav_status == "EC"){ // Check if navigation status is efficient cruising
			  	string action_ = eval_action(same_lane_ahead, same_lane_behind, left_lane_ahead, left_lane_behind, right_lane_ahead, right_lane_behind, further_lane_ahead, further_lane_behind, car_);
		      if(action_ == "CLL"){
		      	car_.prev_action = "CLL";
		      	car_.nav_status = "CLL";
		      	car_.prev_lane = car_.lane;
		        --car_.lane;
		      }else if(action_ == "CLR"){
		      	car_.prev_action = "CLR";
		      	car_.nav_status = "CLR";
		      	car_.prev_lane = car_.lane;
		        ++car_.lane;
		      }else{
		      	car_.nav_status = "KL";
		      	car_.prev_action = "KL";
		      	++car_.time_in_pl;		        	        
		        delta_ref = cruise_control(same_lane_ahead, car_);
		      }
			  }else if((car_.nav_status == "MTT") && (car_.lane != car_.goal_lane)){ // Check if navigating in Maneuver Through Traffic mode and the current lane is not the middle lane.
			  	if(car_.time_in_pl % 50 == 0){
			  		string action_ = eval_action(same_lane_ahead, same_lane_behind, left_lane_ahead, left_lane_behind, right_lane_ahead, right_lane_behind, further_lane_ahead, further_lane_behind, car_);
			  		cout << "Time in platooning: " << car_.time_in_pl << " / Navigation status: " << car_.nav_status << " / Previous action: " << car_.prev_action << endl;
				    if(action_ == "CLL"){
				    	car_.prev_action = "CLL";
				    	car_.nav_status = "MTT";
				    	car_.prev_lane = car_.lane;
				      --car_.lane;
				      car_.time_in_pl = 0;
				    }else if(action_ == "CLR"){
				    	car_.prev_action = "CLR";
				    	car_.nav_status = "MTT";
				    	car_.prev_lane = car_.lane;
				      ++car_.lane;
				      car_.time_in_pl = 0;
				    }else{
				    	car_.prev_action = "KL";
				    	car_.nav_status = "MTT";
				    	++car_.time_in_pl;		        	        
				      delta_ref = MTT_cruise_control(same_lane_ahead, car_);
				    }
			  	}else{
			  		car_.nav_status = "MTT";
				    ++car_.time_in_pl;		        	        
				    delta_ref = MTT_cruise_control(same_lane_ahead, car_);
			  	}
			  }else{
			    if((car_.time_in_pl > 0) && (car_.time_in_pl % 50 == 0)){ // When navigating in platooning mode, check for a better (closer to the limit speed) navigation option every 1 sec approx.
			    	string action_ = eval_action(same_lane_ahead, same_lane_behind, left_lane_ahead, left_lane_behind, right_lane_ahead, right_lane_behind, further_lane_ahead, further_lane_behind, car_);
			    	cout << "Time in platooning: " << car_.time_in_pl << " / Navigation status: " << car_.nav_status << " / Previous action: " << car_.prev_action << endl;
			    	if((car_.time_in_pl < 500) || (car_.lane == 1)){
			      	if(action_ == "CLL"){
					    	car_.prev_action = "CLL";
					    	car_.nav_status = "KL";
					    	car_.prev_lane = car_.lane;
					      --car_.lane;
					      car_.time_in_pl = 0;
					    }else if(action_ == "CLR"){
					    	car_.prev_action = "CLR";
					    	car_.nav_status = "KL";
					    	car_.prev_lane = car_.lane;
					      ++car_.lane;
					      car_.time_in_pl = 0;
					    }else{
					    	car_.prev_action = "KL";
					      car_.nav_status = "KL";
					      ++car_.time_in_pl;
					      delta_ref = cruise_control(same_lane_ahead, car_);
							}
		        }else{ // When navigating in platooning after 10 sec approx. enters the MTT mode to slowly increase the distance from the leading vehicle so as to enhance the possibility of finding a gap for a better navigation action
		        	if(car_.lane == 0){
				      	if(action_ == "CLR"){
						    	car_.prev_action = "CLR";
						    	car_.nav_status = "KL";
						    	car_.prev_lane = car_.lane;
						      ++car_.lane;
						      car_.time_in_pl = 0;
						    }else{
						    	car_.prev_action = "KL";
						      car_.nav_status = "MTT";
						      car_.time_in_pl = 0;
						      delta_ref = MTT_cruise_control(same_lane_ahead, car_);
								}
		        	}else{
				      	if(action_ == "CLL"){
				      		car_.prev_action = "CLL";
						    	car_.nav_status = "KL";
						    	car_.prev_lane = car_.lane;
						      --car_.lane;
						      car_.time_in_pl = 0;
						    }else{
						    	car_.prev_action = "KL";
						      car_.nav_status = "MTT";
						      car_.time_in_pl = 0;
						      delta_ref = MTT_cruise_control(same_lane_ahead, car_);
								}
		        	}
		        }
				  }else{
				    car_.nav_status = "KL";
	          ++car_.time_in_pl;
	          delta_ref = cruise_control(same_lane_ahead, car_);
				  }
				}		  
			}
		}else{
			car_.prev_action = "KL";
		  car_.nav_status = "EC";
		  car_.time_in_pl = 0;
		}
		
  }else{
  	car_.prev_action = "KL";
    car_.nav_status = "EC";
    car_.time_in_pl = 0;
  }

  return delta_ref;				  
}

#endif
