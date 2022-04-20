/*
 * robotnik_trajectory_pad
 * Copyright (c) 2013, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <vector>
//#include <torso_pad/enable_disable.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <robotnik_trajectory_pad/CartesianEuler.h>

#include <XmlRpcValue.h>

#define MAX_NUM_OF_BUTTONS			16
#define MAX_NUM_OF_AXES				8
#define MAX_NUM_OF_BUTTONS_PS3		19
#define MAX_NUM_OF_AXES_PS3			20

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_NUM_OF_AXES			8

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_ANGULAR		0	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0


#define DEFAULT_JOY			"/joy"

#define DEFAULT_HZ			50.0

#define CARTESIAN_CONTROL			1
#define JOINTBYJOINT_CONTROL		2

#define POSITION_CONTROL    1
#define VELOCITY_CONTROL    2

#define MAX_AXIS_VELOCITY			0.1 // m/s
#define MAX_JOINT_VELOCITY			0.1 // rad/s

using namespace std;

//! Class to save the state of the buttons
class Button{
	int iPressed;
	bool bReleased;
	
	public:
	
	Button(){
		iPressed = 0;
		bReleased = false;
	}
	//! Set the button as 'pressed'/'released'
	void Press(int value){		
		if(iPressed and !value){
			bReleased = true;
			
		}else if(bReleased and value)
			bReleased = false;
			
		iPressed = value;
			
	}
	
	int IsPressed(){
		return iPressed;
	}
	
	bool IsReleased(){
		bool b = bReleased;
		bReleased = false;
		return b;
	}
};

////////////////////////////////////////////////////////////////////////
//                               		                                //
////////////////////////////////////////////////////////////////////////
class RobotnikTrajectoryPad
{
	public:
	
	RobotnikTrajectoryPad();
	
	void ControlLoop();
	
	private:
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	char * StateToString(int state);
	int SwitchToState(int new_state);
	
	void PublishState();
	//! Enables/Disables the joystick
	//bool EnableDisable(robotnik_msgs::enable_disable::Request &req, robotnik_msgs::enable_disable::Response &res );
	void Update();
	
	//! Changes the current control group
	void ChangeGroup(int next);
	//! Gets all the joints of every group
	void GetJointsGroup();
	//! Sets the internal control type (CARTESIAN, JOINTBYJOINT)
	void SetControlType(int type);
	//! Sets the planner control mode (POSITION, VELOCITY)
	void SetControlMode(string mode);
	//! Increase/Decrease the joint number
	void SetJoint(int next);
	
private:	
	
	ros::NodeHandle pnh_; // Private node handle
	ros::NodeHandle nh_; 

	//! Axis numbers 
	int axis_x, axis_y, axis_z, axis_pitch, axis_roll, axis_yaw, axis_move_joint;
	
	double l_scale_, a_scale_;
	double current_speed_lvl;
	//! Set the max speed sent to the robot
	double max_axis_velocity, max_joint_velocity;
	
	//! Desired component's freq
	double desired_freq_;
	//! Robot control type (CARTESIAN, JOINT BY JOINT)
	int control_type;
    //! Robot control mode (POSITION, VELOCITY)
    int control_mode;

	// TOPICS
	// PUBLISHER
	//! It will publish to cartesian command
	ros::Publisher cartesian_pub_;
	//! It will publish to joint by joint command
	ros::Publisher jointbyjoint_pub_;
	//! Topic to publish the state
	ros::Publisher state_pub_;
	//! Name of the topic to publish
	string cmd_topic_cartesian;
	string cmd_topic_jointbyjoint;
	
	// SUBSCRIBER
	//! they will be suscribed to the joysticks
	ros::Subscriber joy_sub_;
	//! Subscribes to the planner node state
	ros::Subscriber planner_state_sub_;
	//! topic name for the state
	string topic_planner_state_;
	
	//! // Name of the joystick's topic
	string  joy_topic_;	

	// JOYSTICK
	//! Current number of buttons of the joystick
	int num_of_buttons_;
	int num_of_axes_;
	
	//! Vector to save the axis values
	std::vector<float> fAxes;
	//! Vector to save and control the axis values
	std::vector<Button> vButtons;
	//! Button's number associated to every functionality
	int button_dead_man_, button_euler_mode_, button_change_group_right_, button_change_group_left_, button_set_control_mode_,
	 button_set_control_type_, button_increase_joint_, button_decrease_joint_;
	int button_control_bhand_, button_control_bhand_grasp_, button_control_bhand_release_, button_control_bhand_mode1_, button_control_bhand_mode2_;
	int button_control_wsg50_, button_control_wsg50_grasp_, button_control_wsg50_release_;
	
	//! Number of the button for increase or decrease the speed max of the joystick	
	int button_speed_up_, button_speed_down_;
		
	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;

protected:
	//! Sends an action to the BHand
	void sendBHandAction(int action);
	//! Sends a grasp action to the WSG50
	void sendWSG50GraspAction();
	//! Sends a release action to the WSG50
	void sendWSG50ReleaseAction();
};


RobotnikTrajectoryPad::RobotnikTrajectoryPad():
  pnh_("~")
{	
	control_type = CARTESIAN_CONTROL;
	control_mode = POSITION_CONTROL;

	current_speed_lvl = 0.1;
	// JOYSTICK CONFIG
	pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	pnh_.param("num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES);
	pnh_.param("desired_freq", desired_freq_, DEFAULT_HZ);
	
	if(num_of_axes_ > MAX_NUM_OF_AXES){
		num_of_axes_ = MAX_NUM_OF_AXES;
		ROS_INFO("RobotnikTrajectoryPad::RobotnikTrajectoryPad: Limiting the max number of axes to %d", MAX_NUM_OF_AXES);
	}
	if(num_of_buttons_ > MAX_NUM_OF_BUTTONS){
		num_of_buttons_ = MAX_NUM_OF_BUTTONS;
		ROS_INFO("RobotnikTrajectoryPad::RobotnikTrajectoryPad: Limiting the max number of buttons to %d", MAX_NUM_OF_BUTTONS);
	}
	
	pnh_.param("topic_joy", joy_topic_, std::string(DEFAULT_JOY));	
	
	// MOTION CONF
	pnh_.param("cmd_topic_cartesian", cmd_topic_cartesian, std::string("/rt_traj_planner/commands/cartesian_euler"));
	pnh_.param("cmd_topic_jointbyjoint", cmd_topic_jointbyjoint, std::string("/rt_traj_planner/commands/jointbyjoint"));
	
	pnh_.param("button_dead_man", button_dead_man_, button_dead_man_);
	pnh_.param("button_speed_up", button_speed_up_, button_speed_up_);
	pnh_.param("button_speed_down", button_speed_down_, button_speed_down_); 
	pnh_.param("button_euler_mode", button_euler_mode_, button_euler_mode_); 
	pnh_.param("button_change_group_right", button_change_group_right_, button_change_group_right_); 
	pnh_.param("button_change_group_left", button_change_group_left_, button_change_group_left_); 
	pnh_.param("button_set_control_mode", button_set_control_mode_, button_set_control_mode_); 
	pnh_.param("button_set_control_type", button_set_control_type_, button_set_control_type_); 
	pnh_.param("button_increase_joint", button_increase_joint_, button_increase_joint_); 
	pnh_.param("button_decrease_joint", button_decrease_joint_, button_decrease_joint_); 
	
	
	pnh_.param("axis_x", axis_x, axis_x); 
	pnh_.param("axis_y", axis_y, axis_y); 
	pnh_.param("axis_z", axis_z, axis_z); 
	pnh_.param("axis_pitch", axis_pitch, axis_pitch); 
	pnh_.param("axis_roll", axis_roll, axis_roll); 
	pnh_.param("axis_yaw", axis_yaw, axis_yaw); 
	pnh_.param("axis_move_joint", axis_move_joint, axis_move_joint); 
	
	pnh_.param("max_axis_velocity", max_axis_velocity, MAX_AXIS_VELOCITY); 
	pnh_.param("max_joint_velocity", max_joint_velocity, MAX_JOINT_VELOCITY); 
	
	//ROS_INFO("axis_linear_speed_ = %d, axis_angular = %d", axis_linear_speed_, axis_angular_position_);
	//ROS_INFO("max_linear_speed = %lf, max_angular_speed = %lf", max_linear_speed_, max_angular_position_);

	
	
	XmlRpc::XmlRpcValue list;

	
	
	//ROS_INFO("RobotnikTrajectoryPad num_of_buttons_ = %d, axes = %d, topic controller: %s, hz = %.2lf", num_of_buttons_, num_of_axes_, cmd_topic_vel.c_str(), desired_freq_);	
	
	for(int i = 0; i < MAX_NUM_OF_BUTTONS_PS3; i++){
		Button b;
		vButtons.push_back(b);
	}
	
	for(int i = 0; i < MAX_NUM_OF_AXES_PS3; i++){
		fAxes.push_back(0.0);
	}
	
	//
  	// Publish through the node handle Twist type messages to the guardian_controller/command topic
  	this->cartesian_pub_ = pnh_.advertise<robotnik_trajectory_pad::CartesianEuler>(this->cmd_topic_cartesian, 1);
  	//this->jointbyjoint_pub_ = pnh_.advertise<robotnik_trajectory_pad::JointByJoint>(this->cmd_topic_jointbyjoint, 1);
	
	//
	// Publishes the state
	//state_pub_ = pnh_.advertise<robotnik_trajectory_pad::State>("state", 1);
	
 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to rescuer_controller/command)
	joy_sub_ = pnh_.subscribe<sensor_msgs::Joy>(joy_topic_, 1, &RobotnikTrajectoryPad::joyCallback, this);
		
	
}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */
void RobotnikTrajectoryPad::Update(){
	PublishState();
}



/*! \fn void RobotnikTrajectoryPad::ControlLoop()
 *  Controls the actions and states
*/
void RobotnikTrajectoryPad::ControlLoop(){
	
	double desired_linear_speed = 0.0, desired_angular_position = 0.0;	
	robotnik_trajectory_pad::CartesianEuler cartesian_msg;
	//robotnik_trajectory_pad::JointByJoint jointbyjoint_msg;
	
	ros::Rate r(desired_freq_);   

    while(ros::ok()) {
		
		Update();
			
		if(bEnable){
			
			// Changing internal control type
			if(vButtons[button_set_control_type_].IsReleased()){
				if(control_type == CARTESIAN_CONTROL)
					SetControlType(JOINTBYJOINT_CONTROL);
				else
					SetControlType(CARTESIAN_CONTROL);
			}
							
			if(vButtons[button_dead_man_].IsPressed()){
				
				// Depending the type we'll use some buttons or others
				// 
				// CARTESIAN-EULER CONTROL
				if(control_type == CARTESIAN_CONTROL){
					cartesian_msg.x = cartesian_msg.y = cartesian_msg.z = cartesian_msg.pitch = cartesian_msg.roll = 0.0;
							
					// EULER MODE
					if(vButtons[button_euler_mode_].IsPressed()){
                        cartesian_msg.x = 0; 
                        cartesian_msg.y = 0;
                        cartesian_msg.z = 0;

						cartesian_msg.pitch = max_joint_velocity * current_speed_lvl * fAxes[axis_pitch];
						cartesian_msg.roll = max_joint_velocity * current_speed_lvl * fAxes[axis_roll];
						cartesian_msg.yaw = max_joint_velocity * current_speed_lvl * fAxes[axis_yaw];

					}else{
					// CARTESIAN MODE
						cartesian_msg.x = max_axis_velocity * current_speed_lvl * fAxes[axis_x];
						cartesian_msg.y = max_axis_velocity * current_speed_lvl * fAxes[axis_y];
						cartesian_msg.z = max_axis_velocity * current_speed_lvl * fAxes[axis_z];
                        cartesian_msg.pitch = 0.0;
                        cartesian_msg.roll = 0.0;
                        cartesian_msg.yaw = 0.0;
                    }
					cartesian_pub_.publish(cartesian_msg);
					
				}
				
				// INCREASE/DECREASE MAX SPEED
				if(vButtons[button_speed_up_].IsReleased()){
					current_speed_lvl += 0.1;
					if(current_speed_lvl > 1.0)
						current_speed_lvl = 1.0;
				}
				if(vButtons[button_speed_down_].IsReleased()){
					current_speed_lvl -= 0.1;
					if(current_speed_lvl < 0.0)
						current_speed_lvl = 0.0;
				}
				
				
			}else if(vButtons[button_dead_man_].IsReleased()){
				
			}
		}
		
		ros::spinOnce();
		r.sleep();
	}
    	
}


/*! \fn void RobotnikTrajectoryPad::SetControlType(int type)
 *  Sets the internal control type (CARTESIAN, JOINTBYJOINT)
*/
void RobotnikTrajectoryPad::SetControlType(int type){
	control_type = type;
	//ROS_INFO("RobotnikTrajectoryPad::SetControlType: Setting type to %d", type);
}


	
///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_trajectory_pad");
	RobotnikTrajectoryPad pad;
	
	pad.ControlLoop();
	
}

