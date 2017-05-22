/*
 * boat_controller.h
 *
 *  Created on: May 15, 2017
 *      Author: andrew
 */

#ifndef FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_CONTROLLER_H_
#define FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_CONTROLLER_H_

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <fcu_sim_plugins/common.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#include <iostream>

#define PI 3.14159265

namespace gazebo {

class BoatControllerPlugin : public ModelPlugin {
public:
	BoatControllerPlugin();
	~BoatControllerPlugin();
	void commandCallback(const std_msgs::Float32MultiArray& msg);

protected:
	void Reset();
	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo& _info); // Called by the world update start event

private:
	ros::NodeHandle* 	nh_;
	ros::Subscriber 	boat_cmd_sub_;
	ros::Publisher 		boat_pose_pub_;

	// Pointers to gazebo items
	physics::LinkPtr			link_;
	physics::JointControllerPtr joint_controller_;
	physics::JointPtr			pitch_joint_;
	physics::JointPtr			roll_joint_;
	physics::JointPtr			yaw_joint_;
	physics::ModelPtr			model_;
	physics::WorldPtr			world_;

	// Pointer to the update event connection
	event::ConnectionPtr		updateConnection_;

	std::string	namespace_;

	// Time
	double previous_time_;
	double current_time_;

	// Commands
	double pitch_desired_;
	double pitch_actual_;
	double roll_desired_;
	double roll_actual_;
	double yaw_desired_;
	double yaw_actual_;

	// Filters on Axes
	std::unique_ptr<FirstOrderFilter<double>> pitch_filter_;
	std::unique_ptr<FirstOrderFilter<double>> roll_filter_;
	std::unique_ptr<FirstOrderFilter<double>> yaw_filter_;

	double time_constant_;
};

} // namespace gazebo

#endif /* FCU_SIM_FCU_SIM_PLUGINS_INCLUDE_FCU_SIM_PLUGINS_BOAT_CONTROLLER_H_ */
