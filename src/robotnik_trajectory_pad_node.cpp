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
#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       1
#define DEFAULT_AXIS_ANGULAR		1
#define DEFAULT_AXIS_LINEAR_Z       1	
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		0.05
#define DEFAULT_SCALE_LINEAR_Z      1.0 

//Used only with ps4
#define AXIS_PTZ_TILT_UP  0
#define AXIS_PTZ_TILT_DOWN  1
#define AXIS_PTZ_PAN_LEFT  2
#define AXIS_PTZ_PAN_RIGHT  3

#define MAX_AXIS_STEP			0.1 // m/s
#define MAX_JOINT_STEP			0.1 // rad/s

using namespace std;


////////////////////////////////////////////////////////////////////////
//                               		                                //
////////////////////////////////////////////////////////////////////////
class RobotnikTrajectoryPad
{
	public:
	
	RobotnikTrajectoryPad();
	
	void Update();

	private:
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	int linear_x_, linear_y_, linear_z_, angular_;
	double l_scale_, a_scale_, l_scale_z_; 
	//! It will publish into cartesian_euler (for the robot) 
	ros::Publisher pad_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;
	//! Name of the topic where it will be publishing the cartesianeuler moves
	std::string cartesian_topic_name_;

	double current_step;
	//! Pad type
	std::string pad_type_;
	//! Number of the DEADMAN button
	int dead_man_button_;
	//! Number of the button for increase or decrease the speed max of the joystick	
	int speed_up_button_, speed_down_button_;
	double max_axis_step_;
	double max_joint_step_;
	int button_euler_mode_;
	int button_output_1_, button_output_2_;
	int output_1_, output_2_;
	bool bOutput1, bOutput2;
	//! button to start the homing service
	int button_home_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
    //! Pointer to a vector for controlling the event when pushing directional arrows (UNDER AXES ON PX4!)
    bool bRegisteredDirectionalArrows[4];

	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; // 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; // 	
	//! Flag to enable/disable the communication with the publishers topics
	bool bEnable;
	//! Flag to track the first reading without the deadman's button pressed.
	bool last_command_;
	//! Client of the sound play service
	//  sound_play::SoundClient sc;
	//! Pan & tilt increment (degrees)
	double pan_increment_, tilt_increment_;
	//! Zoom increment (steps)
	int zoom_increment_;
	//! Add a dead zone to the joystick that controls scissor and robot rotation (only useful for xWam) 
	std::string joystick_dead_zone_;
	//! Flag to enable the ptz control via axes
	bool ptz_control_by_axes_;
};

RobotnikTrajectoryPad::RobotnikTrajectoryPad():
  pnh_("~")
{	
	
	current_step = 0.01;
	cartesian_topic_name_="cartesian_move";

	//JOYSTICK PAD TYPE
	pnh_.param<std::string>("pad_type",pad_type_,"ps3");
	// 
	pnh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	// MOTION CONF
	pnh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
  	pnh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
	pnh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);
	pnh_.param("axis_angular", angular_, DEFAULT_AXIS_ANGULAR);
	pnh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	pnh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
	pnh_.param("scale_linear_z", l_scale_z_, DEFAULT_SCALE_LINEAR_Z);
	pnh_.param("cartesian_topic_name", cartesian_topic_name_, cartesian_topic_name_);
	pnh_.param("button_dead_man", dead_man_button_, dead_man_button_);
	pnh_.param("button_speed_up", speed_up_button_, speed_up_button_);  //4 Thrustmaster
	pnh_.param("button_speed_down", speed_down_button_, speed_down_button_); //5 Thrustmaster
	pnh_.param("button_euler_mode", button_euler_mode_, button_euler_mode_); //euler or cartesian
	pnh_.param<std::string>("joystick_dead_zone", joystick_dead_zone_, "true");
	
	pnh_.param("max_axis_step", max_axis_step_, MAX_AXIS_STEP); 
	pnh_.param("max_joint_step", max_joint_step_, MAX_JOINT_STEP); 
	
	ROS_INFO("KukaPad num_of_buttons_ = %d", num_of_buttons_);	
	for(int i = 0; i < num_of_buttons_; i++){
		bRegisteredButtonEvent[i] = false;
		ROS_INFO("bREG %d", i);
	}

	for(int i = 0; i < 3; i++){
	  bRegisteredDirectionalArrows[i] = false;
	}

	// Publish through the node handle CartesianEuler type messages to the topic
	pad_pub_ = nh_.advertise<robotnik_trajectory_pad::CartesianEuler>(cartesian_topic_name_, 1);
	
	// Listen through the node handle sensor_msgs::Joy messages from joystick 
	// (these are the references that we will sent to summit_xl_controller/command)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotnikTrajectoryPad::padCallback, this);
	
	// Diagnostics
	updater_pad.setHardwareID("None");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cartesian_topic_name_.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));


	bEnable = false;	// Communication flag disabled by default
	last_command_ = true;
	
}


/*
 *	\brief Updates the diagnostic component. Diagnostics
 * 		   Publishes the state
 *
 */
void RobotnikTrajectoryPad::Update(){
	updater_pad.update();
}


void RobotnikTrajectoryPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	robotnik_trajectory_pad::CartesianEuler cartesian_msg;

	cartesian_msg.x = 0.0;
	cartesian_msg.y = 0.0;
	cartesian_msg.z = 0.0;
	cartesian_msg.pitch = 0.0;
	cartesian_msg.roll = 0.0;
	cartesian_msg.yaw = 0.0;
	
	bEnable = (joy->buttons[dead_man_button_] == 1);

	// Actions dependant on dead-man button
 	if (joy->buttons[dead_man_button_] == 1) {
		//ROS_ERROR("SummitXLPad::padCallback: DEADMAN button %d", dead_man_button_);
		//Set the current velocity level
		if ( joy->buttons[speed_down_button_] == 1 ){

			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_step > 0.001){
		  			current_step = current_step - 0.01;
					bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Step: %f%%", current_step*100.0);
					char buf[50]="\0";
 					int percent = (int) (current_step*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		 }
		//ROS_ERROR("SummitXLPad::padCallback: Passed SPEED DOWN button %d", speed_down_button_);
		if (joy->buttons[speed_up_button_] == 1){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_step <= 0.9){
					current_step = current_step + 0.01;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Step: %f%%", current_step*100.0);
  					char buf[50]="\0";
					int percent = (int) (current_step*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		
		// EULER MODE
		if(joy->buttons[button_euler_mode_] == 1){
			cartesian_msg.x = 0.0; 
			cartesian_msg.y = 0.0;
			cartesian_msg.z = 0.0;

			cartesian_msg.pitch = current_step * a_scale_*joy->axes[linear_x_];
			cartesian_msg.roll = current_step * a_scale_*joy->axes[linear_y_];
			cartesian_msg.yaw = current_step * a_scale_*joy->axes[linear_z_];

		}else{
		// CARTESIAN MODE
			cartesian_msg.x = current_step * l_scale_*joy->axes[linear_x_];
			cartesian_msg.y = current_step * l_scale_*joy->axes[linear_y_];
			cartesian_msg.z = current_step * l_scale_*joy->axes[linear_z_];
			cartesian_msg.pitch = 0.0;
			cartesian_msg.roll = 0.0;
			cartesian_msg.yaw = 0.0;
		}
		pad_pub_.publish(cartesian_msg);
		
	}
	else {
		cartesian_msg.x = 0.0;
		cartesian_msg.y = 0.0;
		cartesian_msg.z = 0.0;
		cartesian_msg.pitch = 0.0;
		cartesian_msg.roll = 0.0;
		cartesian_msg.yaw = 0.0;
	}
	
	sus_joy_freq->tick();	// Ticks the reception of joy events
	
	// Publish 
	// Only publishes if it's enabled
	if(bEnable){
		//if (ptzEvent) ptz_pub_.publish(ptz);
		pad_pub_.publish(cartesian_msg);
		pub_command_freq->tick();
		last_command_ = true;
	}
		
		
	if(!bEnable && last_command_){
		//if (ptzEvent) ptz_pub_.publish(ptz);
		
		cartesian_msg.x = 0.0;
		cartesian_msg.y = 0.0;
		cartesian_msg.z = 0.0;
		cartesian_msg.pitch = 0.0;
		cartesian_msg.roll = 0.0;
		cartesian_msg.yaw = 0.0;
		
		pad_pub_.publish(cartesian_msg);
		
		pub_command_freq->tick();
		last_command_ = false;
	}	
		
		

	/*		
		
	ros::spinOnce();
	r.sleep();
*/
    	
}

///////////////////////// MAIN /////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotnik_trajectory_pad");
	RobotnikTrajectoryPad robotnik_trajectory_pad;

	ros::Rate r(50.0);

	while( ros::ok() ){
		// UPDATING DIAGNOSTICS
		robotnik_trajectory_pad.Update();
		ros::spinOnce();
		r.sleep();
		}
}

