#ifndef FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_COMMAND_H_
#define FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_COMMAND_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"

#include <cmath>

// NOT BEING USED / COMPILED CURRENTLY ========================================

class move {
public:
	move();
	void update(const ros::TimerEvent& e);
private:
	ros::NodeHandle nh_;
	ros::Publisher boat_pub_;
	ros::Timer update_timer;
	ros::Time start_time;
	ros::Time now_time;
	std_msgs::Float32MultiArray cmd;
	double update_rate;
	double omega;
};

#endif
