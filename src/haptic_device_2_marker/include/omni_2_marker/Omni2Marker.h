#ifndef OMNI_2_MARKER_H
#define OMNI_2_MARKER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

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
		void getTF(tf2_ros::Buffer& buffer);
		const geometry_msgs::Vector3 vectorRotation(
							geometry_msgs::Quaternion q, geometry_msgs::Vector3 v);

	private: 
		// ROS Parameters
		ros::NodeHandle nh_;

		// ros::Subscriber joint_State_sub_;
		ros::Subscriber button_event_sub_;
		ros::Subscriber lock_state_sub_;
		// std::string joint_state_topic_name_;
		std::string button_event_topic_name_;
		std::string lock_state_topic_name_;
		

		ros::Publisher marker_pub_;
		// ros::Publisher marker_transform_pub_;
		std::string marker_topic_name_;
		std::string marker_trans_topic_name_;

		std::string robot_reference_frame_name_;
		std::string HD_frame_name_;
		std::string ee_frame_name_;
		std::string virtual_marker_;	

		double scale_marker_deviation_;

		// message type parameters
		// sensor_msgs::JointState jointstate_msg_;
		phantom_omni::PhantomButtonEvent button_msg_; 
		phantom_omni::LockState lockstate_msg_;
		visualization_msgs::Marker marker_;
		// geometry_msgs::TransformStamped marker_transform_;


		
		

		// Private parameters
		geometry_msgs::TransformStamped ee_in_base_;
		geometry_msgs::TransformStamped HD_to_base_trans_;
		geometry_msgs::TransformStamped marker_in_base_;

        // Initialize inside constructor
        void getParameters();
        void initializeSubscribers();
        void initializePublishers();


        // callbacks
        // void CB_getJointStates(const sensor_msgs::JointState& jointstate_message);
		void CB_getButtonEvent(const phantom_omni::PhantomButtonEvent& button_message);
		void CB_getLockState(const phantom_omni::LockState& lockstate_message);

		// private members
		void findDeviationFromLockPosition(std::vector<double> &deviation_from_lock);
		void addMarkerTransform(const std::vector<double> &deviation_from_lock);
		void fillMarkerMsg(geometry_msgs::TransformStamped& trans);
		void fillMarkerTransformMsg(visualization_msgs::Marker& marker, 
												geometry_msgs::TransformStamped& trans);
};

#endif