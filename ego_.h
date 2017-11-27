#ifndef EGO_H
#define EGO_H

#include <vector>
#include <math.h>

#include "helper_functions.h"

using namespace std;

struct near_vehicle{
  int id;
  double vx;
  double vy;
  double s;
  vector<double> d;
  vector<double> speed;
  double delta_s;
};

struct ego_action{
  string action_; // KL, CLL, CLR
  double cost_;
  double potential_cost_;
};

class EGO_ {
  
  public:
  
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    
    int lane;
    int prev_lane;
    int goal_lane;
    
    string nav_status; // EC, KL, MTT - > Manuever Through Traffic
    
    long int time_in_pl;
    double nav_time;
    
    string prev_action;
    double prev_cost;
    double prev_potential_cost;
  
    EGO_(); // Constructor
   
   ~EGO_(); // Destructor
};

EGO_::EGO_(){};

EGO_::~EGO_(){};

double cruise_control(vector<near_vehicle>& same_lane_ahead, EGO_& car_);

double MTT_cruise_control(vector<near_vehicle>& same_lane_ahead, EGO_& car_);

double traffic_av_speed(vector<near_vehicle>& lane_vehicles_ahead);

double cost_eff(double traffic_av_speed);

vector<double> cost_safety(vector<near_vehicle>& lane_vehicles_ahead, vector<near_vehicle>& lane_vehicles_behind, EGO_& car_, double t_min);

double cost_lane(string action_, EGO_& car_);

double cost_potential_advantage(vector<near_vehicle>& further_vehicles_ahead, vector<near_vehicle>& further_vehicles_behind, EGO_& car_);

double cost_lane_occupancy(vector<near_vehicle>& same_lane_ahead, EGO_& car_);

string eval_action(vector<near_vehicle>& same_lane_ahead, vector<near_vehicle>& same_lane_behind, vector<near_vehicle>& left_lane_ahead, vector<near_vehicle>& left_lane_behind, vector<near_vehicle>& right_lane_ahead, vector<near_vehicle>& right_lane_behind, vector<near_vehicle>& further_vehicles_ahead, vector<near_vehicle>& further_vehicles_behind, EGO_& car_);

double cruise_control(vector<near_vehicle>& same_lane_ahead, EGO_& car_){

	double crit_distance = 20;
  double es_ = 0.;
  double es_dot = 0.;
  double delta_ref = 0.224;

	// PID Speed controller
	double check_s = same_lane_ahead[0].s + 0 * (same_lane_ahead[0].speed.back() - car_.speed);
	es_ = (crit_distance - (check_s - car_.s));
	es_dot = same_lane_ahead[0].speed.back() - car_.speed;
	delta_ref = - 0.055*es_ + 0.125*es_dot;
	
	if(delta_ref>1.115){
  	delta_ref = 1.115 ;
  }else if(delta_ref<-1.125){
  	delta_ref = -1.125;
  }
	return delta_ref;
}

double MTT_cruise_control(vector<near_vehicle>& same_lane_ahead, EGO_& car_){

	double crit_distance = 20 + exp(car_.time_in_pl/1000 - 1); // Distance from the vehicle ahead is adjusted according to the elapse time in platooning navigation so as to increase the possibility of attaining a gap for changing lane
	double es_ = 0.;
  double es_dot = 0.;
  double delta_ref = 0.224;

	// PID Speed controller
	double check_s = same_lane_ahead[0].s + 0 * (same_lane_ahead[0].speed.back() - car_.speed);
	es_ = (crit_distance - (check_s - car_.s));
	es_dot = same_lane_ahead[0].speed.back() - car_.speed;
	delta_ref = - 0.025*es_ + 0.125*es_dot;
	
	if(delta_ref>1.115){
  	delta_ref = 1.115 ;
  }else if(delta_ref<-1.125){
  	delta_ref = -1.125;
  }
	return delta_ref;
}

double traffic_av_speed(vector<near_vehicle>& lane_vehicles_ahead){
  
  double av_speed = 0;
  if((lane_vehicles_ahead.size()>0) && (lane_vehicles_ahead[0].delta_s<55)){
    for (int i = 0; i<lane_vehicles_ahead.size(); ++i){
      if (lane_vehicles_ahead[i].speed.size()>100){
        for(vector<double>::iterator it = lane_vehicles_ahead[i].speed.end(); it > lane_vehicles_ahead[i].speed.end()-100; --it){
          av_speed += *it / 100;
        }
      }else{
        for(vector<double>::iterator it = lane_vehicles_ahead[i].speed.begin(); it != lane_vehicles_ahead[i].speed.end(); ++it){
          av_speed += *it / lane_vehicles_ahead[i].speed.size();
        }
      }
    }
    av_speed /= lane_vehicles_ahead.size();
    //cout << "Average speed on lane: " << av_speed * 2.24 << ". Vehicles ahead on lane: " << lane_vehicles_ahead.size() << endl;
  }else{
    av_speed = 49.5 / 2.24;
    //cout << "Average speed on lane: " << av_speed * 2.24 << ". Vehicles ahead on lane: " << 0 << endl;
  }
  return av_speed;
}

double cost_eff(double traffic_av_speed){ // Cost associated with the average speed on a lane

  double cost_ = -2/49.5 * traffic_av_speed + 1; 
  return cost_; 
}

vector<double> cost_safety(vector<near_vehicle>& lane_vehicles_ahead, vector<near_vehicle>& lane_vehicles_behind, EGO_& car_, double t_min){ // Cost associated with the safety of changing lanes

  vector<double> safety_;
  double ttc_ahead = 999999;
  double ttc_behind = 999999;
    
  if(lane_vehicles_ahead.size()>0){
  	sort(lane_vehicles_ahead.begin( ), lane_vehicles_ahead.end( ), [ ]( const near_vehicle& lhs, const near_vehicle& rhs ){return lhs.delta_s < rhs.delta_s;});
    double delta_s_left_ahead = lane_vehicles_ahead[0].delta_s + 1.5 * (lane_vehicles_ahead[0].speed.back() - car_.speed);
    //cout << "Vehicle ahead: " << lane_vehicles_ahead[0].id << ". Speed: " << lane_vehicles_ahead[0].speed.back() * 2.24 << ". Distance: " <<  delta_s_left_ahead << endl;
    if((lane_vehicles_ahead[0].speed.back()<car_.speed) && (fabs(delta_s_left_ahead)>15)){
      ttc_ahead = delta_s_left_ahead / (car_.speed - lane_vehicles_ahead[0].speed.back());
    }else if(fabs(delta_s_left_ahead)<15){
      ttc_ahead = -99999;
    }
  }
  ttc_ahead = 1-1/(1 + exp(-2*(ttc_ahead-t_min)));

  if(lane_vehicles_behind.size()>0) {
  	sort(lane_vehicles_behind.begin( ), lane_vehicles_behind.end( ), [ ]( const near_vehicle& lhs, const near_vehicle& rhs ){return lhs.delta_s < rhs.delta_s;});
    double delta_s_left_behind = lane_vehicles_behind[0].delta_s - 1.5 * (lane_vehicles_behind[0].speed.back() - car_.speed);
    //cout << "Vehicle behind: " << lane_vehicles_behind[0].id << ". Speed: " << lane_vehicles_behind[0].speed.back() * 2.24 << ". Distance: " <<  delta_s_left_behind << endl;
    if((lane_vehicles_behind[0].speed.back()>car_.speed) && (fabs(delta_s_left_behind)>15)){
      ttc_behind = delta_s_left_behind / (lane_vehicles_behind[0].speed.back() - car_.speed);
    }else if(fabs(delta_s_left_behind)<10){
      ttc_behind = -99999;
    }
  }
  ttc_behind = 1-1/(1 + exp(-2*(ttc_behind-t_min)));
  
  safety_.push_back(ttc_ahead);
  safety_.push_back(ttc_behind);
  
  return safety_;
}

double cost_lane(string action_, EGO_& car_){ // Cost associated with traveling within valid lanes

  double cost_ = 0;
  
  if((action_ == "CLL") && (car_.lane == 0)){
    cost_ = 1;
  }else if((action_ == "CLR") && (car_.lane == 2)){
    cost_ = 1;
  }
  
  return cost_;
}

double cost_potential_advantage(vector<near_vehicle>& further_vehicles_ahead, vector<near_vehicle>& further_vehicles_behind, EGO_& car_){ // Cost associated with the potential advantage to change to the middle lane if traveling on the inner or outer lanes.

	double WEIGHTS_[4] = {1.5, 0.5, 0.5, 2.5};
	double cost_ = 0;
	
	int size_ahead = further_vehicles_ahead.size();
	int size_behind = further_vehicles_behind.size();
	
	//cout << "Further ahead: " << size_ahead << " Further behind: " << size_behind << " Ego car lane: " << car_.lane <<endl;
	
	if((size_ahead>0) || (size_behind>0)){
		// Check cost of keeping lane
		double av_speed = traffic_av_speed(further_vehicles_ahead);
		double eff_ = cost_eff(av_speed * 2.24);
		
		vector<double> safety_;
		safety_ = cost_safety(further_vehicles_ahead, further_vehicles_behind, car_, 4.5);
		
		double occ_ = cost_lane_occupancy(further_vehicles_ahead, car_);
		
		//cout << "eff_: " << eff_ << " - " << "safety: " << safety_[0] << " / " << safety_[1] << endl;
		cost_ = WEIGHTS_[0] * eff_ + WEIGHTS_[1] * safety_[0] + WEIGHTS_[2] * safety_[1] + WEIGHTS_[3] * occ_;
  }else if((car_.lane==0) || (car_.lane==2)){
  	cost_ = -5;
  }  
  
  return cost_;
}

double cost_lane_occupancy(vector<near_vehicle>& same_lane_ahead, EGO_& car_){ // Cost associated with the traffic on a lane

	double cost_ = -1;
	
	double number_of_vehicles = (double) same_lane_ahead.size();
	if(number_of_vehicles>0){
		cost_ = 2/3 * number_of_vehicles - 1;	
	}

	return cost_;
}

string eval_action(vector<near_vehicle>& same_lane_ahead, vector<near_vehicle>& same_lane_behind, vector<near_vehicle>& left_lane_ahead, vector<near_vehicle>& left_lane_behind, vector<near_vehicle>& right_lane_ahead, vector<near_vehicle>& right_lane_behind, vector<near_vehicle>& further_vehicles_ahead, vector<near_vehicle>& further_vehicles_behind, EGO_& car_){

  double WEIGHTS_[7] = {5, 10, 15, 50, 0.5, 1.5, 1};
  vector<ego_action> actions_;
  ego_action action_;
			  
  // Check cost of keeping lane
  double av_speed = traffic_av_speed(same_lane_ahead);
  double eff_ = cost_eff(av_speed * 2.24);
  
  vector<double> safety_; double t_min = 2.;
  safety_ = cost_safety(same_lane_ahead, same_lane_behind, car_, t_min);
  
  double occ_ = cost_lane_occupancy(same_lane_ahead, car_);
  
  double cost_ = 1.25 * WEIGHTS_[0] * eff_ + 0 * WEIGHTS_[1] * safety_[0] + 0 * WEIGHTS_[2] * safety_[1] + WEIGHTS_[3] * cost_lane("KL", car_) + WEIGHTS_[5] * occ_;
  
  action_.action_ = "KL";
  action_.cost_ = cost_;
  action_.potential_cost_ = 0;
  actions_.push_back(action_);
  cout << "Action: " << action_.action_ << " / Cost: " << action_.cost_ << " / Safety: " << safety_[0] << " / " << safety_[1] << endl;
  
  // Check cost of changing left
  av_speed = traffic_av_speed(left_lane_ahead);
  eff_ = cost_eff(av_speed * 2.24);
  
  safety_ = cost_safety(left_lane_ahead, left_lane_behind, car_, t_min);
  double potential_advantage = cost_potential_advantage(further_vehicles_ahead, further_vehicles_behind, car_);
  
  occ_ = cost_lane_occupancy(left_lane_ahead, car_);
  
  cost_ = WEIGHTS_[0] * eff_ + WEIGHTS_[1] * safety_[0] + WEIGHTS_[2] * safety_[1] + WEIGHTS_[3] * cost_lane("CLL", car_) + WEIGHTS_[4] * potential_advantage + WEIGHTS_[5] * occ_ + WEIGHTS_[6] * car_.prev_potential_cost;
  
  action_.action_ = "CLL";
  action_.cost_ = cost_;
  action_.potential_cost_ = potential_advantage;
  actions_.push_back(action_);
  cout << "Action: " << action_.action_ << " / Cost: " << action_.cost_ << " / Safety: " << safety_[0] << " / " << safety_[1] << " / Potential advantage: " << potential_advantage << endl;
  
  // Check cost of changing right
  av_speed = traffic_av_speed(right_lane_ahead);
  eff_ = cost_eff(av_speed * 2.24);
   
  safety_ = cost_safety(right_lane_ahead, right_lane_behind, car_, t_min);
  potential_advantage = cost_potential_advantage(further_vehicles_ahead, further_vehicles_behind, car_);
  
  occ_ = cost_lane_occupancy(right_lane_ahead, car_);
  
  cost_ = WEIGHTS_[0] * eff_ + WEIGHTS_[1] * safety_[0] + WEIGHTS_[2] * safety_[1] + WEIGHTS_[3] * cost_lane("CLR", car_) + WEIGHTS_[4] * potential_advantage + WEIGHTS_[5] * occ_ + WEIGHTS_[6] * car_.prev_potential_cost;
  
  action_.action_ = "CLR";
  action_.cost_ = cost_;
  action_.potential_cost_ = potential_advantage;
  actions_.push_back(action_);
  cout << "Action: " << action_.action_ << " / Cost: " << action_.cost_ << " / Safety: " << safety_[0] << " / " << safety_[1] << " / Potential advantage: " << potential_advantage << endl;
  
  // Choosing action with less cost with penalty to succesive lane change in oposite direction
  sort( actions_.begin( ), actions_.end( ), [ ]( const ego_action& lhs, const ego_action& rhs ){return lhs.cost_ < rhs.cost_;});
  car_.prev_potential_cost = potential_advantage;
  if(actions_[0].cost_>0) actions_[0].action_ = "KL"; 
  if(((car_.prev_action == "CLR") && (actions_[0].action_ == "CLL")) || ((car_.prev_action == "CLL") && (actions_[0].action_ == "CLR"))) actions_[0].action_ = "KL";
  
  cout << "\nOptimal action: " << actions_[0].action_ << endl;
  return actions_[0].action_;

}

#endif
