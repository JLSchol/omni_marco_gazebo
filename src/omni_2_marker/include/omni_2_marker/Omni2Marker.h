#ifndef OMNI_2_MARKER_H
#define OMNI_2_MARKER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h> // can weg?
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

// rviz
#include <visualization_msgs/Marker.h>

//Openhaptics
#include "phantom_omni/PhantomButtonEvent.h"
#include "phantom_omni/LockState.h"

//math
#include <vector> 



class Omni2Marker
{

	public:
		
		// Public parameters
		bool publish_on_;

		// Constructor
        Omni2Marker();

        // Public methods
		void run();

	private: 
		// ROS Parameters
		ros::NodeHandle nh_;

		ros::Subscriber joint_State_sub_;
		ros::Subscriber button_event_sub_;
		ros::Subscriber lock_state_sub_;
		std::string joint_state_topic_name_;
		std::string button_event_topic_name_;
		std::string lock_state_topic_name_;
		

		ros::Publisher marker_pub_;
		ros::Publisher marker_transform_pub_;
		std::string marker_topic_name_;
		std::string marker_trans_topic_name_;

		std::string base_frame_name_;
		std::string ee_frame_name_;	

		double scale_marker_deviation_;

		// message type parameters
		sensor_msgs::JointState jointstate_msg_;
		phantom_omni::PhantomButtonEvent button_msg_; 
		phantom_omni::LockState lockstate_msg_;
		visualization_msgs::Marker marker_;
		geometry_msgs::TransformStamped marker_transform_;
		tf::TransformListener TF_listener_;
		

		// Private parameters
		tf::StampedTransform base_to_ee_;
		tf::StampedTransform base_to_marker_;

		
		// visualization_msgs::Marker::CUBE cube_shape_;

        // Private methods

        // Initialize inside constructor
        void getParameters();
        void initializeSubscribers();
        void initializePublishers();

        // callback
        void CB_getJointStates(const sensor_msgs::JointState& jointstate_message);
		void CB_getButtonEvent(const phantom_omni::PhantomButtonEvent& button_message);
		void CB_getLockState(const phantom_omni::LockState& lockstate_message);


        void getTF();

        
		void findDeviationFromLockPosition(std::vector<double> &deviation_from_lock);
		void addMarkerTransform(const std::vector<double> &deviation_from_lock);
		void fillMarkerMsg();
		void fillMarkerTransformMsg();
		// void setOmni2Center(); 	// set omni to center with ff of workspace

};

#endif