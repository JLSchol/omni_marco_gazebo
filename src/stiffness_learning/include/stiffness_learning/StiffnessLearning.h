#ifndef STIFFNESS_LEARNING_H
#define STIFFNESS_LEARNING_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Dense>




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
		tf::TransformListener TF_listener_;
		tf::StampedTransform base_to_ee_;
		tf::StampedTransform base_to_marker_;
		

		// message type parameters
		geometry_msgs::TransformStamped marker_transform_;
		std_msgs::Float32MultiArray stiffness_matrix_;

		//other things
		std::vector< std::vector<float> > data_matrix_;

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
		void initializeStiffnessMsg();

        // callback
        void CB_getMarkerTransform(const geometry_msgs::TransformStamped& marker_transform_message);
		
		

		void getErrorSignal(std::vector<float>& vect);
			void getTF();
		void populateDataMatrix(std::vector<float>& error_signal, std::vector< std::vector<float> >& data_matrix_);
		void getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix_, Eigen::Matrix3f& covariance_matrix);
		void getEigenValues();
		void getEigenVector();
		void getStiffnessEig();
		void setStiffnessMatrix();

		void fillStiffnessMsg();

};

#endif