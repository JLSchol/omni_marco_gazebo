#ifndef ELLIPSOID_VISUALIZATION_H
#define ELLIPSOID_VISUALIZATION_H

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>



class EllipsoidVisualization
{

	public:
		
		// Public parameters

		// Constructor
        EllipsoidVisualization();

        // Public methods
		void run();
        

	private: 
		// ROS Parameters
		ros::NodeHandle nh_;

        // sub/pubs
		ros::Subscriber stiffness_sub_;
		ros::Publisher marker_pub_;
        ros::Publisher marker_pub_arrow_;

		// TF
		tf::TransformListener TF_listener_;
		tf::StampedTransform base_to_ee_;

        // topic/tf names
        std::string stiffness_topic_name_;
		std::string marker_topic_name_;

        std::string base_frame_name_;
		std::string ee_frame_name_;	

		// message type parameters
		visualization_msgs::Marker ellipsoid_;
        visualization_msgs::Marker arrow_;
        std_msgs::Float32MultiArray stiffness_MultiArray_;
        // std_msgs::Float32MultiArray stiffness_array_msgs;
        
        // tune parameters
		float stiffness_min_;
		float stiffness_max_;
		float lambda_min_;
		float lambda_max_;
		float window_length_;


        // Initialize inside constructor
        void getParameters();
        void initializeSubscribers();
        void initializePublishers();

        void getTF();
        Eigen::Matrix3f getStiffnessMatrix();
        
        std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeEigenValuesAndVectors(Eigen::Matrix3f stiffness_matrix);

        void setEllipsoidMsg(std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair);
            tf2::Quaternion computeRotation(std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair);
            Eigen::Vector3f computeScale(std::pair<Eigen::Matrix3f, Eigen::Vector3f>& vector_value_pair);
            
        visualization_msgs::Marker setArrowMsg(Eigen::Matrix3f M, Eigen::Vector3f& scales, int vector_i);

        // callback
        void CB_getStiffnessArray(const std_msgs::Float32MultiArray& stiffness_array_msgs);
};

#endif