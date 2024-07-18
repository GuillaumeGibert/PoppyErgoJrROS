#include <signal.h>

#include "DynamixelHandler.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// Global variables
float _fps = 10.0f; // Hz
ros::Publisher _jointPositionPublisher;
ros::Publisher _jointTorquePublisher;
DynamixelHandler _oDxlHandler;
std::string _poppyDxlPortName = "/dev/ttyUSB0";
float _poppyDxlProtocol = 2.0;
int _poppyDxlBaudRate = 1000000;
int _nbJoints = 6;
float _minJointCmd = 0;
float _maxJointCmd = 1023;
float _minJointAngle = -180.0f;
float _maxJointAngle = 180.0f;

 

int convertAnglesToJointCmd(float fJointAngle)
{
	// y = ax + b
	float a =  (_maxJointCmd-_minJointCmd) / (_maxJointAngle - _minJointAngle);
	float b = _minJointCmd - a * _minJointAngle;
	float jointCmd = a * fJointAngle + b;
	return (int)jointCmd;
}

void goToHomePosition()
{
	std::vector<uint16_t> l_vTargetJointPosition;
	for (int l_joint = 0; l_joint < _nbJoints; l_joint++)
		l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}

void customSigIntHandler(int sig)
{
	ROS_INFO("===Stopping Poppy node===");
	

	// shutdown ROS
	ros::shutdown();
}


void jointCmdCallback(const geometry_msgs::Twist::ConstPtr& jointCmd)
{
	std::vector<uint16_t> l_vTargetJointPosition;
	
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->linear.x));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->linear.y));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->linear.z));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->angular.x));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->angular.y));
	l_vTargetJointPosition.push_back(convertAnglesToJointCmd(jointCmd->angular.z));
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}


int main(int argc, char** argv)
{
	// create a node called poppy_ros
	ros::init(argc, argv, "poppy_ros", ros::init_options::NoSigintHandler);
	//ros::init(argc, argv, "autopilot");
	
	// create a node handle
	ros::NodeHandle nh;
	
	// override the default sigint handler (must be set after the first node handler is created)
	signal(SIGINT, customSigIntHandler);
	
	// create a publisher to joint_position topic
	_jointPositionPublisher = nh.advertise<geometry_msgs::Twist>("joint_position", 1);
	
	// create a publisher to joint_torque topic
	_jointTorquePublisher = nh.advertise<geometry_msgs::Twist>("joint_torque", 1);
	
	// create a subscriber to cmd_joint topic
	ros::Subscriber cmdJointSubscriber = nh.subscribe("joint_cmd", 1, jointCmdCallback);
	
	// create a loop rate
	ros::Rate loopRate(_fps);
	
	// create a Twist message
	geometry_msgs::Twist jointPositionMsg;
	geometry_msgs::Twist jointTorqueMsg;
		
	std::cout << "===Initialization of the Dynamixel Motor communication====" << std::endl;
	_oDxlHandler.setDeviceName(_poppyDxlPortName);
	_oDxlHandler.setProtocolVersion(_poppyDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_poppyDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	std::cout << std::endl;
	
	//goToHomePosition();
	
	ROS_INFO("===Launching Poppy node===");
	
	// loop until Ctrl+C is pressed or ROS connectivity issues
	while(ros::ok())
	{
		//===RETRIEVE Dynamixel Motor positions====
		std::vector<uint16_t> l_vCurrentJointPosition;
		bool bIsReadSuccessfull = _oDxlHandler.readCurrentJointPosition(l_vCurrentJointPosition);
		
		// stores them into a msg
		if (bIsReadSuccessfull)
		{
			jointPositionMsg.linear.x = l_vCurrentJointPosition[0];
			jointPositionMsg.linear.y = l_vCurrentJointPosition[1];
			jointPositionMsg.linear.z = l_vCurrentJointPosition[2];
			jointPositionMsg.angular.x = l_vCurrentJointPosition[3];
			jointPositionMsg.angular.y = l_vCurrentJointPosition[4];
			jointPositionMsg.angular.z = l_vCurrentJointPosition[5];
		}
	
		// publish the Twist message to the joint_position topic
		_jointPositionPublisher.publish(jointPositionMsg);
		
		//===RETRIEVE Dynamixel Motor torques====
		std::vector<uint16_t> l_vCurrentJointTorque;
		bIsReadSuccessfull = _oDxlHandler.readCurrentJointTorque(l_vCurrentJointTorque);
		
		// stores them into a msg
		if (bIsReadSuccessfull)
		{
			jointTorqueMsg.linear.x = l_vCurrentJointTorque[0];
			jointTorqueMsg.linear.y = l_vCurrentJointTorque[1];
			jointTorqueMsg.linear.z = l_vCurrentJointTorque[2];
			jointTorqueMsg.angular.x = l_vCurrentJointTorque[3];
			jointTorqueMsg.angular.y = l_vCurrentJointTorque[4];
			jointTorqueMsg.angular.z = l_vCurrentJointTorque[5];
		}
	
		// publish the Twist message to the joint_position topic
		_jointTorquePublisher.publish(jointTorqueMsg);
		
		// spin once to let the process handle callback ad key stroke
		ros::spinOnce();
		
		// sleep the right amout of time to comply with _fps 
		loopRate.sleep();
	}
	
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
	
	return 0;
}
