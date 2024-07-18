#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "poppy_ros/ik.h"
#include "math.h"

float _fps = 10.0;

double deg2rad(double angle)
{
	return -angle / 180.0 * M_PI;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ik_client");
	
	if (argc != 4)
	{
		ROS_INFO("usage: rosrun poppy_ros ik_client x y z");
		ROS_INFO("ex.: rosrun poppy_ros ik_client 12.07 7.05 18.4");
		return 1;
	}
	
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<poppy_ros::ik>("ik");
	
	ros::Publisher jointCmdPublisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	
	poppy_ros::ik srv;
	srv.request.request.linear.x = atof(argv[1]);
	srv.request.request.linear.y = atof(argv[2]);
	srv.request.request.linear.z = atof(argv[3]);
	
	srv.request.request.angular.x = 0.0;
	srv.request.request.angular.y = 0.0;
	srv.request.request.angular.z = 0.0;
	
	// create a loop rate
	ros::Rate loopRate(_fps);
	
	if (client.call(srv))
	{
		ROS_INFO("IK: (x,y,z) --> (J1...J6)");
		ROS_INFO("(%f, %f, %f) --> (%f, %f, %f, %f, %f, %f)", atof(argv[1]), atof(argv[2]), atof(argv[3]), srv.response.response.linear.x, srv.response.response.linear.y, srv.response.response.linear.z, srv.response.response.angular.x, srv.response.response.angular.y, srv.response.response.angular.z);
		
		
		
		std::vector<double> jointCmdArray = {deg2rad(srv.response.response.linear.x), deg2rad(srv.response.response.linear.y), deg2rad(srv.response.response.linear.z),
										deg2rad(srv.response.response.angular.x), deg2rad(srv.response.response.angular.y), deg2rad(srv.response.response.angular.z)};
		
		std::vector<std::string> jointCmdNameArray = {"m1", "m2", "m3", "m4", "m5", "m6"};
										
		while (ros::ok())
		{
			sensor_msgs::JointState jointCmdMsg;
			jointCmdMsg.header.stamp = ros::Time::now();
			jointCmdMsg.header.seq++;
			jointCmdMsg.position = jointCmdArray;
			jointCmdMsg.name = jointCmdNameArray;
			
			jointCmdPublisher.publish(jointCmdMsg);
			ros::spinOnce();
			loopRate.sleep();
		}			
			
	}
	else
	{
		ROS_ERROR("Failed to call the service ik!");
		return 1;
	}
	
	return 0;
}