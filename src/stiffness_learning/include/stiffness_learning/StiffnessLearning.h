#ifndef STIFFNESS_LEARNING_H
#define STIFFNESS_LEARNING_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>



class StiffnessLearning
{

	public:
		// CLASS CONSTRUCTOR
        StiffnessLearning();

		// CLASS PUBLIC METHODS
		void run();
		void getTF(tf2_ros::Buffer& buffer);


	private: 
		// CLASS PRIVATE ATTRIBUTES
		// ROS Parameters
		ros::NodeHandle nh_;
		ros::Publisher stiffness_pub_;
		ros::Publisher covariance_pub_;
		// topic names
		std::string stiffness_command_topic_name_;
		std::string covariance_command_topic_name_;
		// tf frame names
		std::string ee_frame_name_;	
		std::string virtual_marker_name_;
		// tf2_ros messages from geometry
		geometry_msgs::TransformStamped ee_in_base_;
		geometry_msgs::TransformStamped base_to_ee_;
		geometry_msgs::TransformStamped marker_in_ee_frame_;
		// other message type parameters
		std_msgs::Float32MultiArray covariance_matrix_MA_;
		std_msgs::Float32MultiArray stiffness_matrix_MA_;
		//other things
		std::vector< std::vector<float> > data_matrix_;
		// tune parameters
		float stiffness_min_;
		float stiffness_max_;
		float lambda_min_;
		float lambda_max_;
		float window_length_;

		// CLASS PRIVATE METHODS
        // Initialize inside constructor
        void getParameters();
        void initializePublishers();
		std_msgs::Float32MultiArray initialize2DMultiArray(
											int height, int width, int offset);
		std::vector<float> getErrorSignal();

		void populateDataMatrix(std::vector<float>& error_signal, 
								std::vector< std::vector<float> >& data_matrix_);
		void getCovarianceMatrix(std::vector< std::vector<float> >& data_matrix_, 
									Eigen::Matrix3f& covariance_matrix);
		void getStiffnessEig(Eigen::EigenSolver<Eigen::Matrix3f> &eigen_solver, 
								Eigen::Vector3f &stiffness_diagonal);
		void setStiffnessMatrix(Eigen::EigenSolver<Eigen::Matrix3f> &eigen_solver,
								Eigen::Vector3f &stiffness_diagonal,
								Eigen::Matrix3f &K_matrix);
		void fill2DMultiArray(Eigen::Matrix3f matrix, 
										std_msgs::Float32MultiArray& multi_array);
};

#endif