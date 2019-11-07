#ifndef STIFFNESS_LEARNING_H
#define STIFFNESS_LEARNING_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>




class StiffnessLearning
{

	public:
		
		// Public parameters

		// Constructor
        StiffnessLearning();

        // Public methods
		void run();


	private: 
		// ROS Parameters
		ros::NodeHandle nh_;

		ros::Subscriber marker_sub_;
		ros::Publisher stiffness_pub_;

		std::string marker_trans_topic_name_;
		std::string stiffness_command_topic_name_;

		std::string base_frame_name_;
		std::string ee_frame_name_;	

		// TF
		tf::StampedTransform base_to_ee_;
		tf::StampedTransform base_to_marker_;

		// message type parameters
		geometry_msgs::TransformStamped marker_transform_;

		// tune parameters
		float stiffness_min_;
		float stiffness_max_;
		float lambda_min_;
		float lambda_max_;
		float window_length_;

        // Private methods

        // Initialize inside constructor
        void getParameters();
        void initializeSubscribers();
        void initializePublishers();

        // callback
        void CB_getMarkerTransform(const geometry_msgs::TransformStamped& marker_transform_message);
};

#endif