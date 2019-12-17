#ifndef STIFFNESS_LEARNING_H
#define STIFFNESS_LEARNING_H

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>

#include "stiffness_learning/VectorValue.h"
#include "stiffness_learning/EigenPairs.h"

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
		ros::Publisher covariance_pub_;
		ros::Publisher eigen_pair_pub_;
		ros::Publisher stiffness_pub_;		
		// topic names
		std::string stiffness_command_topic_name_;
		std::string covariance_command_topic_name_;
		std::string eigen_pair_topic_name_;
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
		stiffness_learning::EigenPairs eigen_pair_;
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

		std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeEigenVectorsAndValues(
												Eigen::Matrix3f &stiffness_matrix);
			bool checkRightHandednessMatrix(
									Eigen::Matrix3f &eigen_vectors);
			std::pair<Eigen::Matrix3f, Eigen::Vector3f> shuffelEigenPairs(
													Eigen::Matrix3f &eigen_vectors, 
													Eigen::Vector3f &eigen_values);
													
		Eigen::Vector3f getStiffnessEig(std::pair<Eigen::Matrix3f, 
												Eigen::Vector3f>& vector_value_pair);

		Eigen::Matrix3f getStiffnessMatrix(
					std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair,
												Eigen::Vector3f &stiffness_diagonal);

		void fill2DMultiArray(Eigen::Matrix3f matrix, 
										std_msgs::Float32MultiArray& multi_array);
		stiffness_learning::EigenPairs setEigenPairMessage(std::pair<Eigen::Matrix3f, 
													Eigen::Vector3f>& vector_value_pair);
};

#endif