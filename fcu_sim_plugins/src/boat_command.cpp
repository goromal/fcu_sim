#include "fcu_sim_plugins/boat_command.h"

move::move() : update_rate(100.0), omega(1.0/5.0)
{
	cmd.data.clear();
	cmd.data.push_back(0.0);
	cmd.data.push_back(0.0);
	cmd.data.push_back(0.0);
	cmd.data.push_back(0.0);

	boat_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/boat/set_pose", 1);
	update_timer = nh_.createTimer(ros::Duration(1.0 / update_rate), update);
	start_time = ros::Time::now();
	now_time = ros::Time::now();
}

void move::update(const ros::TimerEvent& e)
{
	static double time = 0.0;
	now_time = ros::Time::now();
	time = now_time - start_time;
	cmd.data[0] = sin(time * omega);

	boat_pub_.publish(cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "boat_cmd");
	move boat;

	ros::spin();
	return 0;
}
