#include "fcu_sim_plugins/boat_command.h"

boat_commander::boat_commander() : update_rate_(10.0)
{
	// Load parameters from yaml file
	nh_ = new ros::NodeHandle(ros::this_node::getNamespace());

	use_joy_ = nh_->param<bool>("use_joy", false);
	rotate_roll_ = nh_->param<bool>("rotate_roll", true);
	rotate_pitch_ = nh_->param<bool>("rotate_pitch", true);
	linear_multiplier_ = nh_->param<double>("linear_multiplier", 2.0);
	angular_multiplier_ = nh_->param<double>("angular_multiplier", 0.2);
	roll_omega_ = nh_->param<double>("roll_omega", 1.75);
	pitch_omega_ = nh_->param<double>("pitch_omega", 1.2);

	// Initialize pose command
	pose_cmd.position.x = -10.0; 	// desired x position
	pose_cmd.position.y = 0.0;		// desired y position
	pose_cmd.position.z = 0.0;		// desired z position
	pose_cmd.orientation.x = 0.0;	// desired pitch angle
	pose_cmd.orientation.y = 0.0;	// desired roll angle
	pose_cmd.orientation.z = 0.0;	// desired yaw angle
	pose_cmd.orientation.w = 0.0;	// will go unused (for now)

	// Determine how commands will be generated
	if (use_joy_)
		joy_sub_ = nh_->subscribe("/joy_throttled", 1, &boat_commander::joyCallback, this);
	else
		update_timer = nh_->createTimer(ros::Duration(1.0 / update_rate_), &boat_commander::update, this);

	// Initialize time parameters and publish initial position command
	start_time = ros::Time::now();
	now_time = start_time;

	boat_pub_ = nh_->advertise<geometry_msgs::Pose>("boat_command", 1);
	boat_pub_.publish(pose_cmd);
}

boat_commander::~boat_commander()
{
	if (nh_)
	{
		nh_->shutdown();
		delete nh_;
	}
}

void boat_commander::joyCallback(const sensor_msgs::Joy& msg)
{
	pose_cmd.position.x += linear_multiplier_ * msg.axes[3];
	pose_cmd.position.y += -1.0 * linear_multiplier_ * msg.axes[4];
	pose_cmd.position.z += linear_multiplier_ * msg.axes[1];

	// Handle bounded rotation commands
	if (msg.buttons[0] == 1) // Pitch
	{
		if (pose_cmd.orientation.x + angular_multiplier_ <= MAX_ROTATION)
			pose_cmd.orientation.x += angular_multiplier_;
		else
			pose_cmd.orientation.x = MAX_ROTATION;
	}
	else if (msg.buttons[3] == 1)
	{
		if (pose_cmd.orientation.x - angular_multiplier_ >= -MAX_ROTATION)
			pose_cmd.orientation.x -= angular_multiplier_;
		else
			pose_cmd.orientation.x = -MAX_ROTATION;
	}
	if (msg.buttons[4] == 1) // Roll
	{
		if (pose_cmd.orientation.y + angular_multiplier_ <= MAX_ROTATION)
			pose_cmd.orientation.y += angular_multiplier_;
		else
			pose_cmd.orientation.y = MAX_ROTATION;
	}
	else if (msg.buttons[5] == 1)
	{
		if (pose_cmd.orientation.y - angular_multiplier_ >= -MAX_ROTATION)
			pose_cmd.orientation.y -= angular_multiplier_;
		else
			pose_cmd.orientation.y = -MAX_ROTATION;
	}
	if (msg.buttons[2] == 1) // Yaw
	{
		if (pose_cmd.orientation.z + angular_multiplier_ <= MAX_ROTATION)
			pose_cmd.orientation.z += angular_multiplier_;
		else
			pose_cmd.orientation.z = MAX_ROTATION;
	}
	else if (msg.buttons[1] == 1)
	{
		if (pose_cmd.orientation.z - angular_multiplier_ >= -MAX_ROTATION)
			pose_cmd.orientation.z -= angular_multiplier_;
		else
			pose_cmd.orientation.z = -MAX_ROTATION;
	}

	boat_pub_.publish(pose_cmd);
}

void boat_commander::update(const ros::TimerEvent& e)
{
	static double time = 0.0;
	now_time = ros::Time::now();
	time = (now_time - start_time).toSec();

	pose_cmd.position.y = 15.0 * cos(0.75 * time); // =================

	if (rotate_roll_)
		pose_cmd.orientation.y = MAX_ROTATION * sin(roll_omega_ * time);
	if (rotate_pitch_)
		pose_cmd.orientation.x = MAX_ROTATION * sin(pitch_omega_ * time);

	boat_pub_.publish(pose_cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boat_cmd");
	boat_commander boat_cmdr;

	ros::spin();
	return 0;
}
