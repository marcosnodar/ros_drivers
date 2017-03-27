
#include "MotorControl.hpp"
#include <iostream>
#include <mraa/i2c.h>
#include <ros/console.h>
#include <math.h>
using namespace std;


#define DEBUG(fmt, args...) fprintf(stdout, "D: %s - " fmt, __func__, ##args)
#define ERROR(fmt, args...) fprintf(stdout, "E: %s - " fmt, __func__, ##args)


void MotorControl::update(double radps[4])
{
	double secs;
	mraa::Result status;
	uint8_t data[5];
	double conv = 255/(5*2*M_PI);
	int i;

	data[0] = 1;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;	
	data[5] = 0;	

	/* wheel 0 & 1 must have same direction. 
	wheel 3 & 4 must have same direction */

	if(radps[0] * radps[1] < 0)
		ROS_ERROR("Inconsistent commands for wheels 0 & 1\n");

	if(radps[2] * radps[3] < 0)
		ROS_ERROR("Inconsistent commands for wheels 3 & 4\n");

	data[5] &= ~(3<<2);	
	if(radps[0] <0 && radps[1] < 0)
	{
	/* right wheels backward */
		data[5] |= 1<<2;
	}
	
	data[5] &= ~(3);
        if(radps[2] < 0 && radps[3] < 0)
        {
        /* left wheels backward */
		data[5] |= 1;	
        }


	/* unit = 5/255th of an rps = 2*Pi*5/255 radian per s*/ 
	/* we receive the command in Radian per second */
	/* if the command is in deg per s */
	/* unit = 6/255*360 deg per s. */
 

	for(i=0; i<NUM_JOINTS ; i++)
	{
		data[i+1] = (uint8_t) abs(radps[i] * conv);
	//	data[i+1] = (uint8_t) abs(radps[i]);
		ROS_INFO("Motor %d - PWM %x\n", i, data[i+1]);
	}
	data[2] = 0; // disable wheel 2 for now
        status = i2c->write(data, 6);
        if(status != MRAA_SUCCESS)
                        DEBUG("err %d could not set I2C data\n", status);



}

MotorControl::MotorControl(ros::NodeHandle & node)
{
	mraa::Result status;

	DEBUG("I2C driver init\n");
	ROS_DEBUG("I2C driver init\n");

	try{
		i2c = new mraa::I2c(3);
		if(i2c == NULL)
			ERROR("Cannot get I2C\n");
		else
		{
			DEBUG("Got I2C\n");
				
		}	

	}
	catch(std::invalid_argument ia)
	{
		ERROR("cannot create mraa::i2c object : %s\n", ia.what());
	}

	status = i2c->address(0x42);
	if(status != MRAA_SUCCESS)
			DEBUG("err %d could not set I2C address\n", status);
		
}

MotorControl::~MotorControl()
{
	DEBUG("Destruct I2C\n");
	if(i2c) 
		delete i2c;
}
