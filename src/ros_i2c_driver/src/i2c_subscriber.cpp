#include "ros/ros.h"
#include "MotorControl.hpp"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <ros/console.h>

#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#define DEBUG(fmt, args...) fprintf(stdout, "D: %s - " fmt, __func__, ##args)
#define ERROR(fmt, args...) fprintf(stdout, "E: %s - " fmt, __func__, ##args)

MotorControl *motor_control = NULL;
//#template <unsigned int NUM_JOINTS = 2>
#define NUM_JOINTS 4
class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
cmd[0] = 0;
cmd[1] = 0;
cmd[2] = 0;
cmd[3] = 0;

vel[0] = 0;
vel[1] = 0;
vel[2] = 0;
vel[3] = 0;

pos[0] = 0;
pos[1] = 0;
pos[2] = 0;
pos[3] = 0;
   // connect and register the joint state interface
	   hardware_interface::JointStateHandle state_handle_a("wheel_0_joint", &pos[0], &vel[0], &eff[0]);
	   jnt_state_interface.registerHandle(state_handle_a);

	   hardware_interface::JointStateHandle state_handle_b("wheel_1_joint", &pos[1], &vel[1], &eff[1]);
	   jnt_state_interface.registerHandle(state_handle_b);
	   
	hardware_interface::JointStateHandle state_handle_c("wheel_2_joint", &pos[2], &vel[2], &eff[2]);
	   jnt_state_interface.registerHandle(state_handle_c);

	   hardware_interface::JointStateHandle state_handle_d("wheel_3_joint", &pos[3], &vel[3], &eff[3]);
	   jnt_state_interface.registerHandle(state_handle_d);

	   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
	   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("wheel_0_joint"), &cmd[0]);
	   jnt_pos_interface.registerHandle(pos_handle_a);

	   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("wheel_1_joint"), &cmd[1]);
	   jnt_pos_interface.registerHandle(pos_handle_b);

	   hardware_interface::JointHandle pos_handle_c(jnt_state_interface.getHandle("wheel_2_joint"), &cmd[2]);
	   jnt_pos_interface.registerHandle(pos_handle_c);

	   hardware_interface::JointHandle pos_handle_d(jnt_state_interface.getHandle("wheel_3_joint"), &cmd[3]);
	   jnt_pos_interface.registerHandle(pos_handle_d);
	  
	   registerInterface(&jnt_pos_interface);


   // connect and register the joint velocity interface
	   hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle("wheel_0_joint"), &vel[0]);
	   jnt_vel_interface.registerHandle(vel_handle_a);

	   hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle("wheel_1_joint"), &vel[1]);
	   jnt_vel_interface.registerHandle(vel_handle_b);
	   
	hardware_interface::JointHandle vel_handle_c(jnt_state_interface.getHandle("wheel_2_joint"), &vel[2]);
	   jnt_vel_interface.registerHandle(vel_handle_c);

	   hardware_interface::JointHandle vel_handle_d(jnt_state_interface.getHandle("wheel_3_joint"), &vel[3]);
	   jnt_vel_interface.registerHandle(vel_handle_d);
	   registerInterface(&jnt_vel_interface);

	}

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read()
  {
    std::ostringstream os;
    for (unsigned int i = 0; i < NUM_JOINTS - 1; ++i)
    {
      os << vel[i] << ", ";
    }
    os << vel[NUM_JOINTS - 1];
    ROS_INFO_STREAM("Commands for joints: " << os.str());
  }

  void write()
  {
    if (running_)
    {
      for (unsigned int i = 0; i < NUM_JOINTS; ++i)
      {
        // Note that pos_[i] will be NaN for one more cycle after we start(),
        // but that is consistent with the knowledge we have about the state
        // of the robot.
        pos[i] += vel[i]*getPeriod().toSec(); // update position
       // vel[i] = cmd[i]; // might add smoothing here later
      }
	motor_control->update(vel);
    }
    else
    {
      std::fill_n(pos, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
      std::fill_n(vel, NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());
    }
  }

  bool start_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = true;
    return true;
  }

  bool stop_callback(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/)
  {
    running_ = false;
    return true;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];
  bool running_;
};

void robotCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

double linear_velocity = msg->linear.x, angular_velocity = msg->angular.z;

//motor_control->update(, angular_velocity);

}



int main(int argc, char **argv)
{

	
	ros::init(argc, argv, "motor_control");
	ros::NodeHandle node;
	MyRobot robot;
	motor_control = new MotorControl(node);
	controller_manager::ControllerManager cm(&robot, node);
	
	ros::Subscriber sub = node.subscribe("robot_cmd", 1000, robotCmdCallback);
	ros::Rate rate(1.0 / robot.getPeriod().toSec());
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while(ros::ok())
	{
		robot.read();
		cm.update(robot.getTime(), robot.getPeriod());
		robot.write();
		rate.sleep();
	}	

	spinner.stop();

	delete motor_control;
	return 0;
}
