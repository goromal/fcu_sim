/*
 * boat_controller.cpp
 *
 *  Created on: May 16, 2017
 *      Author: andrew
 */
#include "fcu_sim_plugins/boat_controller.h"

namespace gazebo {

BoatControllerPlugin::BoatControllerPlugin() : ModelPlugin()
{
	nh_ = new ros::NodeHandle();
	boat_cmd_sub_ = nh_->subscribe("/boat/set_pose", 1,
									&BoatControllerPlugin::commandCallback, this);
	//boat_pose_pub_ = nh_->advertise<geometry_msgs::Pose>("/boat/pose", 1);

	previous_time_ = 0.0;
	current_time_ = 0.0;
	time_constant_ = 0.25;

	// Initialize Commands
	yaw_desired_ = 0.0;
	pitch_desired_ = 0.0;
	roll_desired_ = 0.0;

	// Initialize Filtered Values
	yaw_actual_ = 0.0;
	pitch_actual_ = 0.0;
	roll_actual_ = 0.0;
}

BoatControllerPlugin::~BoatControllerPlugin()
{
	event::Events::DisconnectWorldUpdateBegin(updateConnection_);
	if (nh_)
	{
		nh_->shutdown();
		delete nh_;
	}
}

void BoatControllerPlugin::commandCallback(const std_msgs::Float32MultiArray& msg)
{
	pitch_desired_ = msg.data[0];
	roll_desired_ = msg.data[1];
	yaw_desired_ = msg.data[2];
}

void BoatControllerPlugin::Reset()
{
	previous_time_ = 0.0;
}

void BoatControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// Gazebo Integration
	model_ = _model;
	world_ = model_->GetWorld();
	namespace_.clear();

	if (_sdf->HasElement("namespace"))
	{
	    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
	    std::cout << "The namespace is " << namespace_ << std::endl; // -----------------
	}
	else
		gzerr << "[GimbalPlugin] Please specify a namespace";

	Reset();
}

GZ_REGISTER_MODEL_PLUGIN(BoatControllerPlugin)

} // namespace gazebo

