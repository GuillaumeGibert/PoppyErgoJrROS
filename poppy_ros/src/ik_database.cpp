#include "ros/ros.h"

#include "Kinematics.h"

Kinematics _kinematics;
int _minRevoluteJoint = -180;
int _maxRevoluteJoint = 180;
int _minPrismaticJoint = 0;
int _maxPrismaticJoint = 10;


int main(int argc, char** argv)
{
	int nbSamples = 0;
	std::string databaseFilename = "";
	
	// args
	if (argc != 4)
	{
		ROS_INFO("usage: rosrun poppy_ros ik_database <dhParamsFile> <nbSample> <databaseFilename>");
		ROS_INFO("ex.: rosrun poppy_ros ik_database /home/ros/_eeng/_eeng4/2link_dh.yaml 1000 /home/ros/_eeng/_eeng4/2link_database.yaml");
		ROS_INFO("ex.: rosrun poppy_ros ik_database /home/ros/_eeng/_eeng4/poppy_dh.yaml 1000 /home/ros/_eeng/_eeng4/poppy_database.yaml");
		return 1;
	}
	if (argc >= 2)
	{
		_kinematics.loadDHParameters(argv[1]);
	}
	if (argc >= 3)
	{
		nbSamples = atoi(argv[2]);
	}
	if (argc == 4)
	{
		databaseFilename = argv[3];
	}
	
	
	std::cout << "IK database in construction..." ;
	
	_kinematics.createKinematicDatabase(nbSamples, databaseFilename, _minRevoluteJoint, _maxRevoluteJoint, _minPrismaticJoint, _maxPrismaticJoint);
	
	std::cout << "  DONE!" << std::endl;
	
	return 0;
}