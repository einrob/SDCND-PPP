#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


class Trajectory
{
public:
	Trajectory()
	{
		timesteps_ = 0;
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

	tk::spline sp;
	vector<double> control_points_;



};

class EgoCar : private Trajectory
{
public:
	EgoCar()
	{
		car_x_ = 0;
		car_y_ = 0;
		car_s_ = 0;
		car_d_ = 0;
		car_yaw_ = 0;
		prev_path_size_ = 0;
		ref_speed_ = 0;
	}

	void setEgoPose(double _car_x, double _car_y, double _car_yaw)
	{
		car_x_ = _car_x;
		car_y_ = _car_y;
		car_yaw_ = _car_yaw;
	}

	void setEgoFrenet(double _car_s, double _car_d)
	{
		car_s_ = _car_s;
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
	double car_d_;
	double car_yaw_;
	double car_speed_;
	double ref_speed_;
	int lane_;

};

class Object : private Trajectory
{
public:
	Object(int _id, int _lane, double _vx, double _vy, double _s_pos, unsigned int _timesteps) : Trajectory()
	{
		id_ = _id;
		lane_ = _lane;
		vx_ = _vx;
		vy_ = _vx;
		s_pos_ = _s_pos;
	};

	void estimateTrajectory()
	{

	}

private:

	int id_;
	int lane_;
	double vx_;
	double vy_;
	double s_pos_;
};


class EnvirmonmentModel
{
public:
	EnvirmonmentModel(vector<double> _map_waypoints_x,
					  vector<double> _map_waypoints_y,
					  vector<double> _map_waypoints_s,
					  vector<double> _map_waypoints_dx,
					  vector<double> _map_waypoints_dy)
	{

		map_waypoints_x_  = _map_waypoints_x;
		map_waypoints_y_  = _map_waypoints_y;
		map_waypoints_s_  = _map_waypoints_s;
		map_waypoints_dx_ = _map_waypoints_dx;
		map_waypoints_dy_ = _map_waypoints_dy;
	}

	void genererateEgoTrajectories()
	{


		  vector<double> ptsx; // control points
		  vector<double> ptsy; // control points

		  double ref_x = ego_car_.car_x_;
		  double ref_y = ego_car_.car_y_;
		  double ref_yaw = deg2rad(ego_car_.car_yaw_);


		  Trajectory keep_lane;
		  ego_car_.ego_trajectories_.clear();
		  ego_car_.ego_trajectories_.push_back(keep_lane);

		  // If therer are not enough points in the previous path
		  // I need to calculate the position of the car at a previous timestep
		  // --> This position is estimated behind the car in the last know orientation
		  // of the car
		  // --> Two points as starting control points are neccessary to force the
		  // estimated splien into the last know direction of the car
		  // --> This keeps the trajectory smooth

		  if(ego_car_.prev_path_size_ < 2)
		  {
			  double prev_car_x = ego_car_.car_x_ - cos(ego_car_.car_yaw_);
			  double prev_car_y = ego_car_.car_y_ - sin(ego_car_.car_yaw_);



			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(ego_car_.car_x_);

			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(ego_car_.car_y_);
		  }
		  else
		  {
			  ref_x = ego_car_.prev_path_x_[ego_car_.prev_path_size_ -1];
			  ref_y = ego_car_.prev_path_y_[ego_car_.prev_path_size_ -1];

			  double ref_x_prev = ego_car_.prev_path_x_[ego_car_.prev_path_size_ -2];
			  double ref_y_prev = ego_car_.prev_path_y_[ego_car_.prev_path_size_ -2];

			  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);

			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
		  }


		  // In Frenet add evenly 30m spaced points ahead of the starting reference
		  vector<double> next_wp0 = getXY(ego_car_.car_s_+30,(2+4*ego_car_.lane_),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);
		  vector<double> next_wp1 = getXY(ego_car_.car_s_+60,(2+4*ego_car_.lane_),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);
		  vector<double> next_wp2 = getXY(ego_car_.car_s_+90,(2+4*ego_car_.lane_),map_waypoints_s_, map_waypoints_x_,map_waypoints_y_);

		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);

		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);

		  for(int i=0; i < ptsx.size(); i++)
		  {
			  // shift car reference angel to 0 degrees
			  double shift_x = ptsx[i]-ref_x;
			  double shift_y = ptsy[i]-ref_y;

			  ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
			  ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

		  }

		  // create a spline
		  tk::spline s;

		  // set (x,y) points to the spline
		  s.set_points(ptsx,ptsy);

//          std::cout << "------ previous path points" << std::endl;
		  for(int i=0; i < ego_car_.prev_path_size_; i++)
		  {
//        	  std::cout << "X: " << previous_path_x[i] << " Y: " <<previous_path_y[i] << std::endl;

			  ego_car_.ego_trajectories_[0].x_.push_back(ego_car_.prev_path_x_[i]);
			  ego_car_.ego_trajectories_[0].y_.push_back(ego_car_.prev_path_y_[i]);

			 // next_x_vals.push_back(ego_car_.prev_path_x_[i]);
			  //next_y_vals.push_back(ego_car_.prev_path_y_[i]);
		  }

		  // Calculate how to break up spline points so that we travel at our desired reference velocity

		  double target_x = 30.0;
		  double target_y = s(target_x);
		  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

		  double x_add_on = 0;

		  // Fill up the rest of our path planner after filling it with prevous points, here we will alway output 50 points

//          std::cout << "--------------- interpolating spline:" << std::endl;
		  for(int i=1; i<=50-ego_car_.prev_path_size_; i++)
		  {
			  double N = (target_dist/(.02*(ego_car_.ref_speed_/2.24)));
			  double x_point = x_add_on+(target_x)/N;
			  double y_point = s(x_point);


//        	  std::cout << "X: " << x_point << " Y: " << y_point << std::endl;

			  x_add_on = x_point;

			  double x_ref = x_point;
			  double y_ref = y_point;

			  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
			  y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

			  x_point = x_point + ref_x;
			  y_point = y_point + ref_y;


			  ego_car_.ego_trajectories_[0].x_.push_back(x_point);
			  ego_car_.ego_trajectories_[0].y_.push_back(y_point);

			//  next_x_vals.push_back(x_point);
			//  next_y_vals.push_back(y_point);

		  }

	}

	vector<Object> sensor_objects_;
	EgoCar ego_car_;

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


  EnvirmonmentModel ego_environment(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  ego_environment.ego_car_.setEgoLane(1); // We know that we start in lane 1
  ego_environment.ego_car_.ref_speed_ = 49.5;

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






          int lane = ego_environment.ego_car_.lane_;
          double ref_vel; // m/s
          int prev_size = previous_path_x.size();

		// Define the actual (x,y) points we will use for the planner
		vector<double> next_x_vals;
		vector<double> next_y_vals;

          //TODO:

		  if(prev_size > 0)
		  {
			  car_s = end_path_s;
		  }

		  bool too_close = false;


		  for(int i=0; i < sensor_fusion.size(); i++)
		  {
			  float d = sensor_fusion[i][6];

			  // Check if car is im my lane
			  if(d< (2+4*ego_environment.ego_car_.lane_+2) && d > (2+4*ego_environment.ego_car_.lane_-2))
			  {
				  double vx = sensor_fusion[i][3];
				  double vy = sensor_fusion[i][4];
				  double check_speed = sqrt(vx*vx+vy*vy);
				  double check_car_s = sensor_fusion[i][5];



				  check_car_s+=((double)prev_size*0.02*check_speed);

				  // check s values grater than mine and s gap
				  	 if((check_car_s > car_s) && ((check_car_s -car_s) < 30))
				  	 {
				  		 too_close = true;

				  		 if(ego_environment.ego_car_.lane_ > 0)
				  		 {
				  			ego_environment.ego_car_.lane_ = 0;
				  		 }

				  	 }

			  }
		  }


		  if(too_close)
		  {
			  ref_vel -= 0.624;
		  }
		  else
		  {
			  if(ref_vel < 49.5)
			  {
				  ref_vel += 0.624;

			  }

		  }






		  ego_environment.ego_car_.setEgoCarSpeed(car_speed);
		  ego_environment.ego_car_.setEgoPose(car_x, car_y, car_yaw);
		  ego_environment.ego_car_.setEgoFrenet(car_s, car_d);
		  ego_environment.ego_car_.setEgoPrevPath(previous_path_x, previous_path_y);
		  ego_environment.genererateEgoTrajectories();





//		 //-------------
//
//
//
//          vector<double> ptsx;
//          vector<double> ptsy;
//
//          double ref_x = car_x;
//          double ref_y = car_y;
//          double ref_yaw = deg2rad(car_yaw);
//
//          if(prev_size < 2)
//          {
//        	  double prev_car_x = car_x - cos(car_yaw);
//        	  double prev_car_y = car_y - sin(car_yaw);
//
//        	  ptsx.push_back(prev_car_x);
//        	  ptsx.push_back(car_x);
//
//        	  ptsy.push_back(prev_car_y);
//        	  ptsy.push_back(car_y);
//          }
//          else
//          {
//        	  ref_x = previous_path_x[prev_size-1];
//        	  ref_y = previous_path_y[prev_size-1];
//
//        	  double ref_x_prev = previous_path_x[prev_size-2];
//        	  double ref_y_prev = previous_path_y[prev_size-2];
//
//        	  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
//
//        	  ptsx.push_back(ref_x_prev);
//        	  ptsx.push_back(ref_x);
//
//        	  ptsy.push_back(ref_y_prev);
//        	  ptsy.push_back(ref_y);
//          }
//
////          std::cout << "YAW: " << ref_yaw << std::endl;
////          std::cout << "----- first two path points: " << std::endl;
////          std::cout << "WP1 x:" << ptsx[0] << ", y: " << ptsy[0] <<std::endl;
////          std::cout << "WP2 x:" << ptsx[1] << ", y: " << ptsy[1] <<std::endl;
//
//
//
//          // In Frenet add evenly 30m spaced points ahead of the starting reference
//          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x,map_waypoints_y);
//
////          std::cout << "----- Spline control points: " << std::endl;
////          std::cout << next_wp0[0] << " " << next_wp0[1] << std::endl;
////          std::cout << next_wp1[0] << " " << next_wp1[1] << std::endl;
////          std::cout << next_wp2[0] << " " << next_wp2[1] << std::endl;
//
//
//          ptsx.push_back(next_wp0[0]);
//          ptsx.push_back(next_wp1[0]);
//          ptsx.push_back(next_wp2[0]);
//
//          ptsy.push_back(next_wp0[1]);
//          ptsy.push_back(next_wp1[1]);
//          ptsy.push_back(next_wp2[1]);
//
////          std::cout << "Points transforrmed to CCS: " << std::endl;
//
//          for(int i=0; i < ptsx.size(); i++)
//          {
//        	  // shift car reference angel to 0 degrees
//        	  double shift_x = ptsx[i]-ref_x;
//        	  double shift_y = ptsy[i]-ref_y;
//
//        	  ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
//        	  ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
//
// //       	  std::cout <<  "X: " << ptsx[i] << "," << ptsy[i] << std::endl;
//
//
//          }
//
//          // create a spline
//          tk::spline s;
//
//          // set (x,y) points to the spline
//          s.set_points(ptsx,ptsy);
//
//
////
//          // Start with all of the previous path points from the last time
//
////          std::cout << "------ previous path points" << std::endl;
//          for(int i=0; i < previous_path_x.size(); i++)
//          {
////        	  std::cout << "X: " << previous_path_x[i] << " Y: " <<previous_path_y[i] << std::endl;
//        	  next_x_vals.push_back(previous_path_x[i]);
//        	  next_y_vals.push_back(previous_path_y[i]);
//          }
//
//          // Calculate how to break up spline points so that we travel at our desired reference velocity
//
//          double target_x = 30.0;
//          double target_y = s(target_x);
//          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
//
//          double x_add_on = 0;
//
//          // Fill up the rest of our path planner after filling it with prevous points, here we will alway output 50 points
//
//
////          std::cout << "--------------- interpolating spline:" << std::endl;
//          for(int i=1; i<=50-previous_path_x.size(); i++)
//          {
//        	  double N = (target_dist/(.02*(ref_vel/2.24)));
//        	  double x_point = x_add_on+(target_x)/N;
//        	  double y_point = s(x_point);
//
//
////        	  std::cout << "X: " << x_point << " Y: " << y_point << std::endl;
//
//        	  x_add_on = x_point;
//
//        	  double x_ref = x_point;
//        	  double y_ref = y_point;
//
//        	  x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
//              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
//
//              x_point = x_point + ref_x;
//              y_point = y_point + ref_y;
//
//              next_x_vals.push_back(x_point);
//              next_y_vals.push_back(y_point);
//
//          }
//
////          	  std::cout << "-------------------------------------------- next waypoints " << std::endl;
////
////          	 for(int i = 0; i<next_x_vals.size(); i++)
////          	 {
////          		 std::cout <<"X: " << next_x_vals[i] << " Y: "  << next_y_vals[i] << std::endl;
////          	 }
//
//
//




          msgJson["next_x"] = ego_environment.ego_car_.ego_trajectories_[0].x_;
          msgJson["next_y"] = ego_environment.ego_car_.ego_trajectories_[0].y_;




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
