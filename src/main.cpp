#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


using std::vector;


class Trajectory
{
public:
	Trajectory()
	{
		timesteps_ = 0;
		trajectory_cost_ = 0.0;
	}

	void setXYSD(vector<double> _x, vector<double> _y, vector<double> _s, vector<double> _d)
	{
		this->clearTrajectory();
		x_ = _x;
		y_ = _y;
		s_ = _s;
		d_ = _d;
	};

	void clearTrajectory()
	{
		x_.clear();
		y_.clear();
		s_.clear();
		d_.clear();
	}

	int getTrajectorySize()
	{
		return x_.size();
	}

	void setTimesteps(unsigned int _timesteps)
	{
		timesteps_ = _timesteps;
	}

	void setPathControlPoints(vector<double> _control_points)
	{
		control_points_ = _control_points;
	}


	vector<double> x_ ;
	vector<double> y_;
	vector<double> s_;
	vector<double> d_;
	unsigned int timesteps_;

	tk::spline sp_;
	vector<double> control_points_;

	double trajectory_cost_;


};

class EgoCar
{
public:
	EgoCar()
	{
		car_x_ = 0;
		car_y_ = 0;
		car_planned_s_ = 0;
		car_d_ = 0;
		car_yaw_ = 0;
		prev_path_size_ = 0;
		target_speed_ = 0;
		follow_object_speed_ = 0;
	}

	void setEgoPose(double _car_x, double _car_y, double _car_yaw)
	{
		car_x_ = _car_x;
		car_y_ = _car_y;
		car_yaw_ = _car_yaw;
	}

	void setEgoFrenet(double _car_s, double _car_planned_s, double _car_d)
	{
		car_s_ = _car_s;
		car_planned_s_ = _car_planned_s;
		car_d_ = _car_d;
	}

	void setEgoCarSpeed(double _car_speed)
	{
		car_speed_ = _car_speed;
	}

	void setEgoPrevPath(vector<double> _prev_path_x, vector<double> _prev_path_y)
	{
			prev_path_x_  = _prev_path_x;
			prev_path_y_  = _prev_path_y;

			// I implizietely assume that x size always == y size
			// I should check and return an error if not
			prev_path_size_ = prev_path_x_.size();
	}



	void setEgoLane(int _lane)
	{
		lane_ = _lane;
	}

	vector<Trajectory> ego_trajectories_;
	vector<double> prev_path_x_;
	vector<double> prev_path_y_;

	int prev_path_size_;

	double car_x_;
	double car_y_;
	double car_s_;
	double car_planned_s_;
	double car_d_;
	double car_yaw_;
	double car_speed_;
	double target_speed_;
	double follow_object_speed_;
	int lane_;

};

class Object : private Trajectory
{
public:
	Object(int _id, double _x, double _y, double _vx, double _vy, double _s_pos, double _d_pos) : Trajectory()
	{
		id_ = _id;
		x_ = _x;
		y_ = _y;
		vx_ = _vx;
		vy_ = _vx;
		s_pos_ = _s_pos;
		d_pos_ =_d_pos;


		lane_ = floor(d_pos_/4);



		object_speed_mps_ = (sqrt(vx_*vx_+vy_*vy_));
		object_speed_mph_ = object_speed_mps_*2.236934; // mps to mph
		object_future_s_ = 0;
	};

	void estimateTrajectory(int timesteps)
	{
		object_future_s_ = s_pos_; // current s position
		object_future_s_+=((double)timesteps*0.02*object_speed_mps_);

		//std::cout << "----- Object ID: " << id_ << std::endl;

		for(unsigned int tstep = 0; tstep < timesteps; tstep++)
		{
			this->s_.push_back(s_pos_+((double)tstep*0.02*object_speed_mps_));
			this->d_.push_back(d_pos_+((double)tstep*0.02*object_speed_mps_));
			//std::cout << s_[tstep] << std::endl;
		}



	}

	void setObjectData(double _x, double _y, double _vx, double _vy, double _s_pos, double _d_pos, double est_time_elapesed)
	{

		double prev_x = x_;
		double prev_y = y_;

		x_ = _x;
		y_ = _y;
		vx_ = _vx;
		vy_ = _vx;

		// when setObjectData is called, the object is already initialized with the first measurement
		double dx = x_-prev_x;
		double dy = y_-prev_y;

		traveled_distance_ = sqrt(dx*dx+dy*dy);
		speed_from_traveled_distance_ = traveled_distance_*est_time_elapesed; // but simulator does not return after one timestep!


		s_pos_ = _s_pos;
		d_pos_ =_d_pos;
		lane_ = floor(d_pos_/4);
		object_speed_mps_ = (sqrt(vx_*vx_+vy_*vy_));
		object_speed_mph_ = object_speed_mps_*2.236934; // mps to mph
		object_future_s_ = 0;
	}

	int id_;
	int lane_;
	double vx_;
	double vy_;
	double s_pos_;
	double d_pos_;
	double object_speed_mps_;
	double object_speed_mph_;

	double speed_from_traveled_distance_;

	double object_future_s_;

	double x_;
	double y_;
	double traveled_distance_;
};




class EgoPlanner
{
public:
	EgoPlanner(vector<double> _map_waypoints_x,
					  vector<double> _map_waypoints_y,
					  vector<double> _map_waypoints_s,
					  vector<double> _map_waypoints_dx,
					  vector<double> _map_waypoints_dy,
					  unsigned int _max_planning_timesteps)
	{

		map_waypoints_x_  = _map_waypoints_x;
		map_waypoints_y_  = _map_waypoints_y;
		map_waypoints_s_  = _map_waypoints_s;
		map_waypoints_dx_ = _map_waypoints_dx;
		map_waypoints_dy_ = _map_waypoints_dy;
		max_planning_timesteps_ = _max_planning_timesteps;
		system_state_ = 0;
		next_trajectory_ = 1; // WE know that car starts in lane 1 TODO: Change this
		sensor_objects_initialized_ = false;
	}


	void calculateCosts()
	{

		for(unsigned trajectory_lane_index = 0; trajectory_lane_index < ego_car_.ego_trajectories_.size(); trajectory_lane_index++)
		{
			ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ = 0;


			if(inhibit_action_counter_>0) // Inhibit lane
			{
				if(trajectory_lane_index != ego_car_.lane_)
				{
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 100;
				}
				inhibit_action_counter_--;
			}

			// Because it was not possible to set the speed of the ego car properly to the preceding one
			// I inhibit lane changes while car is decreasing speed - whith the current approach the car gets very slow sometimes
			// In this situations a lane change sit not safe
			if(inhibit_action_decreasing_speed_ == true)
			{
				if(trajectory_lane_index != ego_car_.lane_)
				{
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 100;
				}
			}

			// penalize lane changes

			if(trajectory_lane_index != ego_car_.lane_)
			{
				// Penalize double lane changes
				if(abs(trajectory_lane_index - ego_car_.lane_)>1)
				{
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 22.5;
				}
				else
				{
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 2.5;
				}

				// Penalize lane changes even more, if lane change occured in the last interation
				if(system_state_ == CHANGE_RIGHT || system_state_ == CHANGE_LEFT)
				{
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 10;
				}
			}


			// First search for objects in the ego lane
			// sort it according to their s value
			// only calculate cost for the nearest object


			for(unsigned int i=0; i < sensor_objects_.size(); i++)
			{

				if(sensor_objects_[i].object_future_s_ > ego_car_.car_s_           // check if the sensor object is infront of the car
				  && sensor_objects_[i].object_future_s_ < (ego_car_.car_planned_s_ + 100)  // and it is not too far in the future
				  && (sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) < 60
				  && (sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) > 0
				  && sensor_objects_[i].lane_ == trajectory_lane_index)  // and if the car is in the planned trajectory
				{

					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ +=8.4;

				}


				// Collision check
				if(sensor_objects_[i].lane_ == ego_car_.lane_ &&
				   sensor_objects_[i].object_future_s_ > ego_car_.car_s_
				   && (sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) < 20)
				{

					std::cout << "ObjectID: " << sensor_objects_[i].id_ << " lane: " << sensor_objects_[i].lane_ << " ego lane " << ego_car_.lane_ << std::endl;

					std::cout << "----------- TO CLOSE -------------------- " << std::endl;
					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ +=10;


					std::cout <<" Follow Object speed mph: " << sensor_objects_[i].object_speed_mph_
							<< " Follow Object speed mps: " << sensor_objects_[i].object_speed_mps_
							<< " Speed from traveled distance: " << sensor_objects_[i].speed_from_traveled_distance_  << std::endl;


					follow_car_ = true;
				}




				// Check for collision at current car pos +/- range left and right from car
				if((sensor_objects_[i].s_pos_ - ego_car_.car_s_) < 20
					&& (sensor_objects_[i].s_pos_ - ego_car_.car_s_) > -20
					&& sensor_objects_[i].lane_ == trajectory_lane_index
					&& ego_car_.lane_ != trajectory_lane_index)
				{
					std::cout << "object spos: " << sensor_objects_[i].s_pos_ << " ego car spos: " << ego_car_.car_s_ << " diff: " << sensor_objects_[i].s_pos_ - ego_car_.car_s_ << std::endl;

					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 15;
				}

			}
		}



//			// penalize lane changes -> so the car keeps the lane if nothing else is around
//			if(trajectory_lane_index != ego_car_.lane_)
//			{
//				// Penalize double lane changes
//				if(abs(trajectory_lane_index - ego_car_.lane_)>1)
//				{
//					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 22.5;
//				}
//				else
//				{
//					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 2.5;
//				}
//
//				// Penalize lane changes even more, if lane change occured in the last interation
//				if(system_state_ == CHANGE_RIGHT || system_state_ == CHANGE_LEFT)
//				{
//					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 10;
//				}
//			}
//
//			// Check each sesor object
//			for(unsigned int i=0; i < sensor_objects_.size(); i++)
//			{
//
//				// if the sensor object is ahead of us on long sight
//				// Penalize changes into lanes with cars upcoming on the long run
//				// --> change into lane without an obstacle on the long run
//				if((sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) >= 30
//					&& (sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) <= 60
//					&& sensor_objects_[i].object_future_s_ > ego_car_.car_planned_s_
//					&& sensor_objects_[i].lane_ == trajectory_lane_index)
//				{
//					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 5;
//				}
//
//				// if the sensor object is in the same lane
//				if(sensor_objects_[i].lane_ == trajectory_lane_index)
//				{
//					// some car is ahead of us in the lane somewhere
//					if((sensor_objects_[i].object_future_s_ > ego_car_.car_s_)
//						&& (sensor_objects_[i].object_future_s_ - ego_car_.car_planned_s_) < 30
//						&& sensor_objects_[i].s_pos_ > ego_car_.car_s_)
//					 {
//						ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 10;
//
//						if(ego_car_.lane_ == trajectory_lane_index)
//						{
//							follow_car_ = true;
//							ego_car_.follow_object_speed_ = sensor_objects_[i].object_speed_mph_;
//
//							std::cout <<" Follow Object speed mph: " << sensor_objects_[i].object_speed_mph_
//									<< " Follow Object speed mps: " << sensor_objects_[i].object_speed_mps_
//									<< " Speed from traveled distance: " << sensor_objects_[i].speed_from_traveled_distance_  << std::endl;
//
//						}
//					 }
//				}
//
//
//
//				// Check for collision at current car pos +/- range left and right from car
//				if((sensor_objects_[i].s_pos_ - ego_car_.car_s_) < 10
//					&& sensor_objects_[i].lane_ == trajectory_lane_index
//					&& ego_car_.lane_ != trajectory_lane_index)
//				{
//					ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ += 15;
//				}



//				if(sensor_objects_[i].lane_ == ego_car_.lane_)
//				{
//
//
//					 // check s values grater than mine and s gap
//					 if((sensor_objects_[i].object_future_s_ > ego_car_.car_s_) && ((sensor_objects_[i].object_future_s_ - ego_car_.car_s_) < 30))
//					 {
//						 too_close = true;
//
//						 if(ego_car_.lane_ > 0)
//						 {
//							ego_car_.lane_ = 0;
//						 }
//
//					 }
//				}

//			}
//		}


	}

	void updateEgoState()
	{
		double tmp_lane_ = 0;
		follow_car_ = false;

		// Check Ego Car lane
		std::cout << "------------ Ego Car lane: " << ego_car_.lane_ << std::endl;
		tmp_lane_ = floor(ego_car_.car_d_/4);
		std::cout << "-------- Car lane index:  " << tmp_lane_ << std::endl;

		//Is the car at the moment in a
		if(system_state_ == CHANGE_RIGHT || system_state_ == CHANGE_LEFT)
		{
			std::cout << "Lane change in progress ... " << std::endl;

			if(tmp_lane_ == next_trajectory_) // Did the car arrive in the new lane?
			{
				ego_car_.lane_ = next_trajectory_;
				system_state_ = KEEP_LANE;
			}
		}
		else
		{



			calculateCosts();


			double min_const = 999;

			for(unsigned trajectory_lane_index = 0; trajectory_lane_index < ego_car_.ego_trajectories_.size(); trajectory_lane_index++)
			{
				std::cout << "Trajectory cost lane " << trajectory_lane_index << " : " << ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_ << std::endl;

				if(min_const > ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_)
				{
					//TODO:
					next_trajectory_ = trajectory_lane_index;

					min_const = ego_car_.ego_trajectories_[trajectory_lane_index].trajectory_cost_;
				}
			}



			if(ego_car_.lane_ == next_trajectory_)
			{
			  system_state_ = KEEP_LANE;
			}

			if(ego_car_.lane_ > next_trajectory_)
			{
				system_state_ = CHANGE_LEFT;
				std::cout << ">>>>> Lane change left" << std::endl;
				inhibit_action_counter_ = 50;
			}

			if(ego_car_.lane_ < next_trajectory_)
			{
				system_state_= CHANGE_RIGHT;
				std::cout << ">>>>> Lane change right" << std::endl;
				inhibit_action_counter_ = 50;
			}

			std::cout << "Minimum trajectory costs: " << min_const <<  " Next trajectory: " <<  next_trajectory_ << std::endl;


			//std::cout << "Target speed: " << ego_car_.target_speed_ << " car_speed: " << ego_car_.car_speed_
			//		  << " Follow car speed: " << ego_car_.follow_object_speed_ <<  std::endl;

			  if(follow_car_ == true)// && ego_car_.car_speed_ > ego_car_.follow_object_speed_)
			  {
				  ego_car_.target_speed_ -= 2.0;
				  inhibit_action_decreasing_speed_ = true;
	//			  if(ego_car_.target_speed_< ego_car_.follow_object_speed_)
	//			  {
	//				  ego_car_.target_speed_ = ego_car_.follow_object_speed_;
	//			  }
			  }
			  else
			  {
				  if(ego_car_.car_speed_ < speed_limits[ego_car_.lane_] && follow_car_ == false)
				  {
					  ego_car_.target_speed_ += 2.0;
					  inhibit_action_decreasing_speed_ = false;
					  if(ego_car_.target_speed_> speed_limits[ego_car_.lane_])
					  {
						  ego_car_.target_speed_ = speed_limits[ego_car_.lane_];
					  }
				  }
			  }
		}

		  // TODO: UNCOMMENT
		  //ego_car_.lane_ = next_trajectory_;

	}

	void generateObjectTrajectories(unsigned int elapsed_time_steps)
	{
		for(unsigned int i=0; i < sensor_objects_.size(); i++)
		{
			sensor_objects_[i].estimateTrajectory(elapsed_time_steps);
		}
	}

	void genererateEgoTrajectories()
	{


		  vector<double> control_points_x; // control points
		  vector<double> control_points_y; // control points

		  // temporary local reference coordinate system
		  double ref_x = ego_car_.car_x_;
		  double ref_y = ego_car_.car_y_;
		  double ref_yaw = deg2rad(ego_car_.car_yaw_);

		  ego_car_.ego_trajectories_.clear();

		  double max_last_points = 0;

		  // Generate 3 trajectories: keep_lane, change left, change right, or change right right/change left left
		  for(int lane = 0; lane < 3; lane++)
		  {

			  Trajectory temp_lane = Trajectory();
			  ego_car_.ego_trajectories_.push_back(temp_lane);

			  // If therer are not enough points in the previous path
			  // I need to calculate the position of the car at a previous timestep
			  // --> This position is estimated behind the car in the last know orientation
			  // of the car
			  // --> Two points as starting control points are neccessary to force the
			  // estimated splien into the last know direction of the car
			  // --> This keeps the trajectory smooth


			  std::cout << "Prev path size: " << ego_car_.prev_path_size_ << std::endl;

			  if(ego_car_.prev_path_size_ < 2)
			  {
				  double prev_car_x = ego_car_.car_x_ - cos(ego_car_.car_yaw_);
				  double prev_car_y = ego_car_.car_y_ - sin(ego_car_.car_yaw_);

				  control_points_x.push_back(prev_car_x);
				  control_points_x.push_back(ego_car_.car_x_);

				  control_points_y.push_back(prev_car_y);
				  control_points_y.push_back(ego_car_.car_y_);
			  }
			  else
			  {

				  ref_x = ego_car_.prev_path_x_[ego_car_.prev_path_size_ -1];
				  ref_y = ego_car_.prev_path_y_[ego_car_.prev_path_size_ -1];

				  double ref_x_prev = ego_car_.prev_path_x_[ego_car_.prev_path_size_ -2];
				  double ref_y_prev = ego_car_.prev_path_y_[ego_car_.prev_path_size_ -2];

				  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

				  control_points_x.push_back(ref_x_prev);
				  control_points_x.push_back(ref_x);

				  control_points_y.push_back(ref_y_prev);
				  control_points_y.push_back(ref_y);
			  }


			  // In Frenet add evenly 30m spaced points ahead of the starting reference
			  vector<double> next_wp0 = getXY(ego_car_.car_planned_s_+30,(2+4*lane),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);
			  vector<double> next_wp1 = getXY(ego_car_.car_planned_s_+60,(2+4*lane),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);
			  vector<double> next_wp2 = getXY(ego_car_.car_planned_s_+90,(2+4*lane),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);

			  control_points_x.push_back(next_wp0[0]);
			  control_points_x.push_back(next_wp1[0]);
			  control_points_x.push_back(next_wp2[0]);

			  control_points_y.push_back(next_wp0[1]);
			  control_points_y.push_back(next_wp1[1]);
			  control_points_y.push_back(next_wp2[1]);

			  for(int i=0; i < control_points_x.size(); i++)
			  {
				  // shift car reference angel to 0 degrees
				  double shift_x = control_points_x[i]-ref_x;
				  double shift_y = control_points_y[i]-ref_y;

				  control_points_x[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				  control_points_y[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

			  }

			  // create a spline
			  //tk::spline s;


			  // set (x,y) points to the spline
	//		  std::cout << "Num control poijnts: " << control_points_x.size() << std::endl;

			  ego_car_.ego_trajectories_[lane].sp_.set_points(control_points_x, control_points_y);

			  //s.set_points(ptsx,ptsy);

	//          std::cout << "------ previous path points" << std::endl;
			  for(int i=0; i < ego_car_.prev_path_size_; i++)
			  {
	//        	  std::cout << "X: " << previous_path_x[i] << " Y: " <<previous_path_y[i] << std::endl;

				  ego_car_.ego_trajectories_[lane].x_.push_back(ego_car_.prev_path_x_[i]);
				  ego_car_.ego_trajectories_[lane].y_.push_back(ego_car_.prev_path_y_[i]);
			  }



			  // Calculate how to break up spline points so that we travel at our desired reference velocity

			  double target_x = 30.0;
			  double target_y = ego_car_.ego_trajectories_[lane].sp_(target_x);
			  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

			  double x_add_on = 0;

			  // Fill up the rest of our path planner after filling it with prevous points, here we will alway output 50 points

	  //        std::cout << "--------------- interpolating spline:" << std::endl;
			  for(int i=1; i<=max_planning_timesteps_-ego_car_.prev_path_size_; i++)
			  {
				  double N = (target_dist/(.02*(ego_car_.target_speed_/2.24)));
				  double x_point = x_add_on+(target_x)/N;
				  double y_point = ego_car_.ego_trajectories_[lane].sp_(x_point);


	  //      	  std::cout << "X: " << x_point << " Y: " << y_point << std::endl;

				  x_add_on = x_point;

				  double x_ref = x_point;
				  double y_ref = y_point;

				  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
				  y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

				  x_point = x_point + ref_x;
				  y_point = y_point + ref_y;


				  ego_car_.ego_trajectories_[lane].x_.push_back(x_point);
				  ego_car_.ego_trajectories_[lane].y_.push_back(y_point);
			  }


			  ego_car_.ego_trajectories_.rbegin()[0].x_;
			  ego_car_.ego_trajectories_.rbegin()[1].x_;

			  double yaw_trajectory_end = atan2(ego_car_.ego_trajectories_[lane].y_.rbegin()[0]-ego_car_.ego_trajectories_[lane].y_.rbegin()[1],
							  	  	  	  	  	ego_car_.ego_trajectories_[lane].x_.rbegin()[0]-ego_car_.ego_trajectories_[lane].x_.rbegin()[1]);




			//  std::cout << "car_prev_planned_s: " << ego_car_.car_planned_s_ << std::endl;
			  	// TODO:

			  vector<double> sd;

			  sd  = getFrenet(ego_car_.ego_trajectories_[lane].x_.rbegin()[0],  ego_car_.ego_trajectories_[lane].y_.rbegin()[0], yaw_trajectory_end, map_waypoints_x_,map_waypoints_y_);

			//  std::cout << "NEW car_prev_planned_s: " << sd[0] << "   yaw: " << yaw_trajectory_end << std::endl;
			  ego_car_.car_planned_s_ = sd[0];


			  control_points_x.clear();
			  control_points_y.clear();
		  }

	}

	vector<Object> sensor_objects_;
	bool sensor_objects_initialized_;

	EgoCar ego_car_;
	unsigned int max_planning_timesteps_;

	vector<double> speed_limits;
	unsigned int next_trajectory_;
	unsigned int system_state_;
	bool follow_car_;
	double inhibit_action_counter_;
	double inhibit_action_decreasing_speed_;

	enum
	{
		KEEP_LANE,
		CHANGE_RIGHT,
		CHANGE_LEFT
	};

private:

	vector<double> map_waypoints_x_;
	vector<double> map_waypoints_y_;
	vector<double> map_waypoints_s_;
	vector<double> map_waypoints_dx_;
	vector<double> map_waypoints_dy_;



};




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "./data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  if (in_map_.is_open()) {
	  std::cout << "Opened file" << std::endl;
  }
  else
  {
	  std::cout << "Error open file" << std::endl;
	  exit(1);
  }

  string line;
  while (getline(in_map_, line)) {

	//std::cout << line << std::endl;
	std::istringstream iss(line);
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

//    std::cout << x << std::endl;
//    std::cout << y << std::endl;
//    std::cout << s << std::endl;
//

    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }


  EgoPlanner ego_environment(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, 50);
  ego_environment.ego_car_.setEgoLane(1); // We know that we start in lane 1
  ego_environment.ego_car_.target_speed_ = 0.624;
  ego_environment.speed_limits.push_back(45.5); // lane 0
  ego_environment.speed_limits.push_back(45.5); // lane 1
  ego_environment.speed_limits.push_back(45.5); // lane 2


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_environment]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode)
{
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Add Objects to new enviroment


		 // Define the actual (x,y) points we will use for the planner
		 vector<double> next_x_vals;
		 vector<double> next_y_vals;

		 double car_planned_s;

          //TODO:

		  if(previous_path_x.size() > 0)
		  {
			  car_planned_s = end_path_s;
		  }
		  else
		  {
			  car_planned_s = car_s;
		  }

		  // Check how much points of last trajectory are driven by the car - each missing point is one timestep of 0.02
		  double estimated_timesteps_elapsed = ego_environment.max_planning_timesteps_ - previous_path_x.size();


		  for(int i=0; i < sensor_fusion.size(); i++)
		  {

//	          std::cout << "-------------- Sensor fusion size: " << sensor_fusion.size() << std::endl;
//	          std::cout << "ID: " << sensor_fusion[i][0] << std::endl;
//	          std::cout << "X: " << sensor_fusion[i][1] << std::endl;
//	          std::cout << "Y: " << sensor_fusion[i][2] << std::endl;
//	          std::cout << "VX: " << sensor_fusion[i][3] << std::endl;
//	          std::cout << "VY: " << sensor_fusion[i][4] << std::endl;
//	          std::cout << "S: " << sensor_fusion[i][5] << std::endl;
//	          std::cout << "D: " << sensor_fusion[i][6] << std::endl << std::endl;



//			  if(ego_environment.sensor_objects_initialized_ == false)
//			  {
				  Object tmp_sensor_object = Object(sensor_fusion[i][0],
				  					  	  	  	       sensor_fusion[i][1],
				  									   sensor_fusion[i][2],
				  									   sensor_fusion[i][3],
				  									   sensor_fusion[i][4],
				  									   sensor_fusion[i][5],
				  									   sensor_fusion[i][6]);

				  ego_environment.sensor_objects_.push_back(tmp_sensor_object);
//			  }
//			  else
//			  {
//				  // There are always 12 objects transmitted without changing order and ID
//				  // This does not work with real sensor fusion data where IDs jump and number of objects is constantly changeing
//
//					  ego_environment.sensor_objects_[i].setObjectData( sensor_fusion[i][1],
//																	   sensor_fusion[i][2],
//																	   sensor_fusion[i][3],
//																	   sensor_fusion[i][4],
//																	   sensor_fusion[i][5],
//																	   sensor_fusion[i][6],
//																	   estimated_timesteps_elapsed);
//
//					  std::cout << "------- Object: " << std::endl;
//					  std::cout << "ID: " << ego_environment.sensor_objects_[i].id_ << std::endl;
//					  std::cout << "Lane: " << ego_environment.sensor_objects_[i].lane_ << std::endl;
//					  std::cout << "spos: " << ego_environment.sensor_objects_[i].s_pos_ << std::endl;
//					  std::cout << "dpos: " << ego_environment.sensor_objects_[i].d_pos_ << std::endl;
//					  std::cout << "x: " << ego_environment.sensor_objects_[i].x_ << std::endl;
//					  std::cout << "y: " << ego_environment.sensor_objects_[i].y_ << std::endl;
//					  std::cout << "vx: " << ego_environment.sensor_objects_[i].vx_ << std::endl;
//					  std::cout << "vy: " << ego_environment.sensor_objects_[i].vy_ << std::endl;
//					  std::cout << "predicted v: " << ego_environment.sensor_objects_[i].speed_from_traveled_distance_ << std::endl;
//					  std::cout << "Speed mph: " << ego_environment.sensor_objects_[i].object_speed_mph_ << std::endl;
//					  std::cout << "Speed mps: " << ego_environment.sensor_objects_[i].object_speed_mps_ << std::endl;
//
//
//
//			  }



		  }
		  ego_environment.sensor_objects_initialized_ = true; // after first run


		  ego_environment.ego_car_.setEgoCarSpeed(car_speed);
		  ego_environment.ego_car_.setEgoPose(car_x, car_y, car_yaw);
		  ego_environment.ego_car_.setEgoFrenet(car_s, car_planned_s, car_d);
		  ego_environment.ego_car_.setEgoPrevPath(previous_path_x, previous_path_y);

		  ego_environment.genererateEgoTrajectories();
		  ego_environment.generateObjectTrajectories(previous_path_x.size()); // The amount of missing waypoints == timesteps run by simulator
		  ego_environment.updateEgoState();



		  ego_environment.sensor_objects_.clear();

		  std::cout << "EgoCar lane: " << ego_environment.ego_car_.lane_ << " Next trajectory index: " << ego_environment.next_trajectory_ << std::endl;

		  //TODO: CHECK IF I SHOULD USE next_trajectory istead of CAR

          msgJson["next_x"] = ego_environment.ego_car_.ego_trajectories_[ego_environment.next_trajectory_].x_;
          msgJson["next_y"] = ego_environment.ego_car_.ego_trajectories_[ego_environment.next_trajectory_].y_;

          std::cout <<"-----------------------------------------"<< std::endl;


          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
