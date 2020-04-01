#include <ros/ros.h>
#include "ros/master.h"
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include "phantom_omni/PhantomButtonEvent.h"
#include "phantom_omni/OmniFeedback.h"
#include "phantom_omni/LockState.h"
#include <pthread.h>
// #include <vector.h>

int calibrationStyle;

struct OmniState {
	hduVector3Dd position;  //3x1 vector of position
	hduVector3Dd velocity;  //3x1 vector of velocity
	hduVector3Dd inp_vel1; //3x1 history of velocity used for filtering velocity estimate
	hduVector3Dd inp_vel2;
	hduVector3Dd inp_vel3;
	hduVector3Dd out_vel1;
	hduVector3Dd out_vel2;
	hduVector3Dd out_vel3;
	hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
	hduVector3Dd pos_hist2;
	hduVector3Dd rot;
	hduVector3Dd joints;
	hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
	float thetas[7];
	int buttons[2];
	int buttons_prev[2];
	bool lock;
	bool lock_grey;
	bool lock_white;
	hduVector3Dd lock_pos;
};

class PhantomROS {

public:
	ros::NodeHandle n;
	ros::Publisher joint_pub;

	ros::Publisher button_pub;
	ros::Publisher lock_state_pub;
	ros::Subscriber haptic_sub;
	std::string omni_name;
	std::string omni_frame_name;

	OmniState *state;

	void init(OmniState *s) {

		n.param<std::string>("omni_frame_name", omni_frame_name, "omni_rotation");

		ros::param::param(std::string("~omni_name"), omni_name,
				std::string("omni1"));

                // Publish joint states for robot_state_publisher,
                // and anyone else who wants them.
		std::ostringstream joint_topic;
		joint_topic << omni_name << "_joint_states";
		joint_pub = n.advertise<sensor_msgs::JointState>(joint_topic.str(), 1);

		// Publish button state on NAME_button.
		std::ostringstream button_topic;
		button_topic << omni_name << "_button";
		button_pub = n.advertise<phantom_omni::PhantomButtonEvent>(button_topic.str(), 100);

		// Publish button state on NAME_lock_state.
		std::ostringstream lock_state_topic;
		lock_state_topic << omni_name << "_lock_state";
		lock_state_pub = n.advertise<phantom_omni::LockState>(lock_state_topic.str(), 100);
		
	
		// Subscribe to NAME_force_feedback.
		std::ostringstream force_feedback_topic;
		force_feedback_topic << omni_name << "_force_feedback";
		haptic_sub = n.subscribe(force_feedback_topic.str(), 100,
				&PhantomROS::force_callback, this);

		state = s;
		state->buttons[0] = 0;
		state->buttons[1] = 0;
		state->buttons_prev[0] = 0;
		state->buttons_prev[1] = 0;
		hduVector3Dd zeros(0, 0, 0);
		state->velocity = zeros;
		state->inp_vel1 = zeros;  //3x1 history of velocity
		state->inp_vel2 = zeros;  //3x1 history of velocity
		state->inp_vel3 = zeros;  //3x1 history of velocity
		state->out_vel1 = zeros;  //3x1 history of velocity
		state->out_vel2 = zeros;  //3x1 history of velocity
		state->out_vel3 = zeros;  //3x1 history of velocity
		state->pos_hist1 = zeros; //3x1 history of position
		state->pos_hist2 = zeros; //3x1 history of position
		state->lock = false;
		state->lock_grey = false;
		state->lock_white = false;
		state->lock_pos = zeros;

	}

	/*******************************************************************************
	 ROS node callback.
	 *******************************************************************************/
	void force_callback(const phantom_omni::OmniFeedbackConstPtr& omnifeed) {
		////////////////////Some people might not like this extra damping, but it
		////////////////////helps to stabilize the overall force feedback. It isn't
		////////////////////like we are getting direct impedance matching from the
		////////////////////omni anyway
		// ROS_INFO_STREAM("force_callback is called")	;
		// state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
		// state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
		// state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];
		// ROS_INFO_STREAM("Lock state: "<< state->lock);
		if (state->lock_grey == true) {
			// ROS_INFO_STREAM("in if lock true");
			state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
			state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
			state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];
			// state->lock_pos[0] = omnifeed->position.x;
			// state->lock_pos[1] = omnifeed->position.y;
			// state->lock_pos[2] = omnifeed->position.z;
		}
		else{
			state->force[0] = 0;
			state->force[1] = 0;
			state->force[2] = 0;
		}
			state->lock_pos[0] = omnifeed->position.x;
			state->lock_pos[1] = omnifeed->position.y;
			state->lock_pos[2] = omnifeed->position.z;
		// rewrite to stiffnes and damping callback and use omniposition,lockposition to
		// calculate the force here:
		// state->force = 0.04 * (state->lock_pos - state->position)- 0.001 * state->velocity;
		// state->lock_pos[0] = omnifeed->position.x;
		// state->lock_pos[1] = omnifeed->position.y;
		// state->lock_pos[2] = omnifeed->position.z;
	}

	void publish_omni_state() {
		sensor_msgs::JointState joint_state;
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);
		joint_state.name[0] = "waist";
		joint_state.position[0] = -state->thetas[1];
		joint_state.name[1] = "shoulder";
		joint_state.position[1] = state->thetas[2];
		joint_state.name[2] = "elbow";
		joint_state.position[2] = state->thetas[3];
		joint_state.name[3] = "wrist1";
		joint_state.position[3] = -state->thetas[4] + M_PI;
		joint_state.name[4] = "wrist2";
		joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
		joint_state.name[5] = "wrist3";
		joint_state.position[5] = -state->thetas[6] - M_PI;
		joint_pub.publish(joint_state);

		if ((state->buttons[0] != state->buttons_prev[0])
				or (state->buttons[1] != state->buttons_prev[1])) {

			if ((state->buttons[0] == state->buttons[1])
					and (state->buttons[0] == 1)) {
				state->lock = !(state->lock);
			}

		if ((state->buttons[0] != state->buttons_prev[0]) 
				and (state->buttons[0] == 1)) {
				state->lock_grey = !(state->lock_grey);
				}

		if ((state->buttons[1] != state->buttons_prev[0]) 
				and (state->buttons[1] == 1)) {
				state->lock_white = !(state->lock_white);
				}

			phantom_omni::PhantomButtonEvent button_event;
			button_event.header.stamp = ros::Time::now();
			button_event.grey_button = state->buttons[0];
			button_event.white_button = state->buttons[1];
			state->buttons_prev[0] = state->buttons[0];
			state->buttons_prev[1] = state->buttons[1];
			button_pub.publish(button_event);	
		}

		// publish LockState, current position and locked position
		phantom_omni::LockState lock_state_msg;
		lock_state_msg.header.stamp = ros::Time::now();
		lock_state_msg.header.frame_id = omni_frame_name;
		lock_state_msg.lock = state->lock;
		lock_state_msg.lock_grey = state->lock_grey;
		lock_state_msg.lock_white = state->lock_white;
			
		// lock_state_msg.lock_position = state->lock_pos;
		lock_state_msg.lock_position.x = state->lock_pos[0]/1000; // from [mm] to [m]
		lock_state_msg.lock_position.y = state->lock_pos[1]/1000;
		lock_state_msg.lock_position.z = state->lock_pos[2]/1000;
		// lock_state_msg.current_position = state->position;
		lock_state_msg.current_position.x = state->position[0]/1000;
		lock_state_msg.current_position.y = state->position[1]/1000;
		lock_state_msg.current_position.z = state->position[2]/1000;

		lock_state_pub.publish(lock_state_msg);
		// When experiment runs toggle back to :

		std::vector<std::string> v;
		std::string node_name = "/simple_experiment";
		if(!ros::master::getNodes(v))
        {
          ROS_INFO("Error");
        }   
         else
        {  
           for (int i = 0; i < v.size(); i++){
				if (node_name == v[i]){
					state->lock_white = false;
					lock_state_msg.lock_white = state->lock_white;
					// ROS_INFO_STREAM("in State");
		   }
        }


		}
		

	}
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
	OmniState *omni_state = static_cast<OmniState *>(pUserData);
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
	  ROS_DEBUG("Updating calibration...");
	    hdUpdateCalibration(calibrationStyle);
	  }
	hdBeginFrame(hdGetCurrentDevice());
	//Get angles, set forces
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
	hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);

	hduVector3Dd vel_buff(0, 0, 0);
	vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
			+ omni_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
	omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
			+ .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
			- (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
					- 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
	omni_state->pos_hist2 = omni_state->pos_hist1;
	omni_state->pos_hist1 = omni_state->position;
	omni_state->inp_vel3 = omni_state->inp_vel2;
	omni_state->inp_vel2 = omni_state->inp_vel1;
	omni_state->inp_vel1 = vel_buff;
	omni_state->out_vel3 = omni_state->out_vel2;
	omni_state->out_vel2 = omni_state->out_vel1;
	omni_state->out_vel1 = omni_state->velocity;

	// if (omni_state->lock == true) {
	// 	omni_state->force = 0.07 * (omni_state->lock_pos - omni_state->position)
	// 			- 0.001 * omni_state->velocity;
	// }

	hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

	//Get buttons
	int nButtons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during main scheduler callback");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}

	float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
			omni_state->joints[2] - omni_state->joints[1], omni_state->rot[0],
			omni_state->rot[1], omni_state->rot[2] };
	for (int i = 0; i < 7; i++)
		omni_state->thetas[i] = t[i];
	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
		calibrationStyle = HD_CALIBRATION_AUTO;
		ROS_INFO("HD_CALIBRATION_AUTO..");
	}
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
	  do {
		hdUpdateCalibration(calibrationStyle);
		ROS_INFO("Calibrating.. (put stylus in well)");
		if (HD_DEVICE_ERROR(error = hdGetError())) {
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			break;
		}
	} while (hdCheckCalibration() != HD_CALIBRATION_OK);
	ROS_INFO("Calibration complete.");
	}
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
	  ROS_INFO("Please place the device into the inkwell for calibration.");
	}
}

void *ros_publish(void *ptr) {
	PhantomROS *omni_ros = (PhantomROS *) ptr;
	int publish_rate;
	omni_ros->n.param(std::string("publish_rate"), publish_rate, 100);
	ros::Rate loop_rate(publish_rate);
	ros::AsyncSpinner spinner(2);
	spinner.start();

	while (ros::ok()) {
		omni_ros->publish_omni_state();
		loop_rate.sleep();
	}
	return NULL;
}

int main(int argc, char** argv) {
	////////////////////////////////////////////////////////////////
	// Init Phantom
	////////////////////////////////////////////////////////////////
	HDErrorInfo error;
	HHD hHD;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
		return -1;
	}

	ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		ROS_ERROR("Failed to start the scheduler"); //, &error);
		return -1;
	}
	HHD_Auto_Calibration();

	////////////////////////////////////////////////////////////////
	// Init ROS
	////////////////////////////////////////////////////////////////
	ros::init(argc, argv, "omni_haptic_node");
	OmniState state;
	PhantomROS omni_ros;

	omni_ros.init(&state);
	hdScheduleAsynchronous(omni_state_callback, &state,
			HD_MAX_SCHEDULER_PRIORITY);

	////////////////////////////////////////////////////////////////
	// Loop and publish
	////////////////////////////////////////////////////////////////
	pthread_t publish_thread;
	pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
	pthread_join(publish_thread, NULL);

	ROS_INFO("Ending Session....");
	hdStopScheduler();
	hdDisableDevice(hHD);

	return 0;
}

