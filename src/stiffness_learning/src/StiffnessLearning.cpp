#include "stiffness_learning/StiffnessLearning.h"

StiffnessLearning::StiffnessLearning():
nh_("~")
{
    ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();
}  

void StiffnessLearning::getParameters()
{
    // input output topic names
    nh_.param<std::string>("marker_topic_name", marker_trans_topic_name_, "/marker_transform");
    nh_.param<std::string>("stiffness_topic_name", stiffness_command_topic_name_, "/stiffness_command");  
    // TF frame names
    nh_.param<std::string>("base_frame_name", base_frame_name_, "base_marco"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "end_effector"); 
    // Tune parameters
    nh_.param<float>("stiffness_min", stiffness_min_, 100); 
    nh_.param<float>("stiffness_max", stiffness_max_, 500); 
    nh_.param<float>("lambda_min", lambda_min_, 5); 
    nh_.param<float>("lambda_max", lambda_max_, 25); 
    nh_.param<float>("window_length", window_length_, 200); 
}

void StiffnessLearning::initializeSubscribers()
{
    marker_sub_ = nh_.subscribe(marker_trans_topic_name_, 1, &StiffnessLearning::CB_getMarkerTransform, this);
}

void StiffnessLearning::initializePublishers()
{
    stiffness_pub_ = nh_.advertise<std_msgs::Int8MultiArray>(stiffness_command_topic_name_,1);
}

void StiffnessLearning::CB_getMarkerTransform(const geometry_msgs::TransformStamped& marker_transform_message)
{
    marker_transform_ = marker_transform_message;
}



void StiffnessLearning::run()
{

  ROS_INFO_STREAM("shit is aan het runnen");



//   marker_pub_.publish(marker_);


}




int main( int argc, char** argv )
{

	ros::init(argc, argv, "stiffness_learning");

	StiffnessLearning node;

  
  while (ros::ok())
  {

    ros::Rate loop_rate(30);

    node.run();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

}