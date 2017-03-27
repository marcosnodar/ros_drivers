#include <stdint.h>

#include <stdio.h>
#include "ros/ros.h"
#include "mraa.hpp"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#define ERROR(fmt, args...) fprintf(stdout, "E: %s - " fmt, __func__, ##args)
#define DEBUG(fmt, args...) fprintf(stdout, "D: %s - " fmt, __func__, ##args)
#define NUM_JOINTS 4
using namespace ros;

class MotorControl{
	public:
		MotorControl(ros::NodeHandle & node);
		~MotorControl();	
		void update(double radps[4]);

	private:
		mraa::I2c* i2c;

};
