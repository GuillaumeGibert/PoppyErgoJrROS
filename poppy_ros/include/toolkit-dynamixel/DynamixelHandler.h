#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"    

#include "DynamixelAX12A.h"

#define ADDR_XL320_TORQUE_ENABLE 24
#define ADDR_XL320_LED_ON 25

#define ADDR_XL320_D_GAIN 27
#define ADDR_XL320_I_GAIN 28
#define ADDR_XL320_P_GAIN 29

#define ADDR_XL320_GOAL_POSITION 30
#define ADDR_XL320_GOAL_VELOCITY 32
#define ADDR_XL320_GOAL_TORQUE 35

#define ADDR_XL320_PRESENT_POSITION 37
#define ADDR_XL320_PRESENT_VELOCITY 39
#define ADDR_XL320_PRESENT_LOAD 41
#define ADDR_XL320_PRESENT_VOLTAGE 45
#define ADDR_XL320_PRESENT_TEMPERATURE 46

#define ADDR_XL320_HARDWARE_ERROR_STATUS 50

#define NB_JOINTS 6


class DynamixelHandler
{
	
public:
	DynamixelHandler();
	~DynamixelHandler();

public:
	//scan();
	//ping();
	//reboot();
	bool openPort();
	void closePort();
	bool setBaudRate(int);
	void setDeviceName(std::string);
	void setProtocolVersion(float);
	bool enableTorque(bool);
	
	bool readCurrentJointPosition(std::vector<uint16_t>& vCurrentJointPosition);
	bool readCurrentJointTorque(std::vector<uint16_t>& vCurrentJointTorque);
	bool sendTargetJointPosition(std::vector<uint16_t>& vTargetJointPosition);

	

private:
	std::string m_sDeviceName;
	float m_fProtocolVersion;
	int m_i32BaudRate;
	
	dynamixel::PortHandler* m_pPortHandler;
	dynamixel::PacketHandler* m_pPacketHandler;
	
	bool m_bIsDeviceNameSet;
	bool m_bIsProtocolVersionSet;
	bool m_bIsPortOpened;
	bool m_bIsBaudRateSet;
	
	int m_i32DxlCommunicationResult;             // Communication result
	uint8_t m_ui8DxlError;                          // Dynamixel error
	std::vector<uint16_t> m_vDxlCurrentPosition;              // Present position

};