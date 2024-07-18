#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "poppy_ros/ik.h"
#include "Kinematics.h"

// 0s 17 0 13.5

//_vQiValues.push_back(10.0);_vQiValues.push_back(-20.0);_vQiValues.push_back(30.0);_vQiValues.push_back(20.0);_vQiValues.push_back(-40.0);
// 12.07 7.05 18.4

//_vQiValues.push_back(15.0);_vQiValues.push_back(-30.0);_vQiValues.push_back(35.0);_vQiValues.push_back(30.0);_vQiValues.push_back(-30.0);
// 8.93 10.36 17.56


Kinematics _kinematics;
cv::Mat _oXTarget(3, 1, CV_64F, 0.0);

bool ik(poppy_ros::ik::Request &request, poppy_ros::ik::Response &response)
{
	ROS_INFO("================================================");
	ROS_INFO("IK for (%f, %f, %f) in progress...", request.request.linear.x, request.request.linear.y, request.request.linear.z);
	_oXTarget.at<double>(0,0) = request.request.linear.x;
	_oXTarget.at<double>(1,0) = request.request.linear.y;
	_oXTarget.at<double>(2,0) = request.request.linear.z;
	
	_kinematics.inverseKinematics(_oXTarget);
	
	std::vector<double> l_vQi = _kinematics.getQiValues();
		
	response.response.linear.x = l_vQi[0]; response.response.linear.y = l_vQi[1]; response.response.linear.z = l_vQi[2];
	response.response.angular.x = l_vQi[3]; response.response.angular.y = l_vQi[4]; response.response.angular.z = 0.0;
	
	ROS_INFO("IK: (x,y,z) --> (J1...J6)");
	ROS_INFO("(%f, %f, %f) --> (%f, %f, %f, %f, %f, %f)", request.request.linear.x, request.request.linear.y, request.request.linear.z, response.response.linear.x, response.response.linear.y, response.response.linear.z, response.response.angular.x, response.response.angular.y, response.response.angular.z);
	ROS_INFO("-----------------------------------------------------------------------------------------------------------");
	
	return true;
}


int main(int argc, char** argv)
{
	// args
	if (argc != 3 && argc != 4)
	{
		ROS_INFO("usage: rosrun poppy_ros ik_server <dhParamsFile> <ikParamsFile> <dhDatabaseFile>");
		ROS_INFO("ex.: rosrun poppy_ros ik_server /home/ros/_eeng/_eeng4/2link_dh.yaml /home/ros/_eeng/_eeng4/2link_ik.yaml /home/ros/_eeng/_eeng4/2link_database.yaml");
		ROS_INFO("ex.: rosrun poppy_ros ik_server /home/ros/_eeng/_eeng4/poppy_dh.yaml /home/ros/_eeng/_eeng4/poppy_ik.yaml /home/ros/_eeng/_eeng4/poppy_database.yaml");
		return 1;
	}
	if (argc >= 2)
	{
		_kinematics.loadDHParameters(argv[1]);
	}
	if (argc >= 3)
	{
		_kinematics.loadIKParameters(argv[2]);
	}
	if (argc == 4)
	{
		_kinematics.loadKinematicDatabase(argv[3]);
	}
	
	//_kinematics.createKinematicDatabase(1000, "/home/ros/_eeng/_eeng4/poppy_db.yaml", -180, 180, 0, 10);
	cv::Mat l_oCurrentEndEffectorPosition(4, 1, CV_64F, 0.0f); 
	l_oCurrentEndEffectorPosition = _kinematics.computeCurrentEndEffectorPosition();
	ROS_INFO("Current End-effector position = (%f, %f, %f)", l_oCurrentEndEffectorPosition.at<double>(0, 0), l_oCurrentEndEffectorPosition.at<double>(1, 0), l_oCurrentEndEffectorPosition.at<double>(2, 0) );
	
	// ROS
	ros::init(argc, argv, "ik_server");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("ik", ik);
	
	ROS_INFO("IK server launched...");
	
	ros::spin();
	
	return 0;
}