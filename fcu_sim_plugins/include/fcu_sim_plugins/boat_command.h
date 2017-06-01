#ifndef FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_COMMAND_H_
#define FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_COMMAND_H_

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Joy.h"

#include <cmath>

/*
 * +++++++++++++++++++++++++++++++++++++++++
 * Eventually, this node should also take in
 * waypoints and command the boat based on
 * those, using some inertial subscriber (I
 * guess the UAVbook can be of some help here)
 *
 * Before that happens, a quaternion overhaul
 * will be required.
 * +++++++++++++++++++++++++++++++++++++++++
 */
#define MAX_ROTATION 0.75

class boat_commander {
public:
	boat_commander();
	~boat_commander();

private:
	ros::NodeHandle* 	nh_;
	ros::Subscriber 	joy_sub_;
	ros::Publisher 		boat_pub_;
	ros::Timer 			update_timer;
	ros::Time 			start_time;
	ros::Time 			now_time;
	geometry_msgs::Pose pose_cmd;

	void joyCallback(const sensor_msgs::Joy& msg);
	void update(const ros::TimerEvent& e);

	bool	use_joy_;
	bool 	rotate_roll_;
	bool 	rotate_pitch_;
	double 	update_rate_;
	double 	linear_multiplier_;
	double 	angular_multiplier_;
	double 	roll_omega_;
	double 	pitch_omega_;
};

#endif
