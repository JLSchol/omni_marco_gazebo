#include "stiffness_ellipsoid/ellipsoid_visualization.h"

EllipsoidVisualization::EllipsoidVisualization():
nh_("~")
{
    ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();
}  

void EllipsoidVisualization::getParameters()
{
    // pub/sub names
    nh_.param<std::string>("input_topic_name", stiffness_topic_name_, "/stiffness_command"); 
    nh_.param<std::string>("output_topic_name", marker_topic_name_, "/ellipsoid_visualization"); 
    // TF frame names
    nh_.param<std::string>("base_frame_name", base_frame_name_, "base_footprint"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "wrist_ft_tool_link"); 
}

void EllipsoidVisualization::initializeSubscribers()
{
    stiffness_sub_ = nh_.subscribe(stiffness_topic_name_, 1, &EllipsoidVisualization::CB_getStiffnessArray, this);
}

void EllipsoidVisualization::initializePublishers()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_name_,1);
}

void EllipsoidVisualization::CB_getStiffnessArray(const std_msgs::Float32MultiArray& stiffness_array_msgs)
{
    stiffness_MultiArray_ = stiffness_array_msgs; // als cb klaar msgs weg?!?! o.O
    // ROS_INFO_STREAM(stiffness_MultiArray_);
}

Eigen::Matrix3f EllipsoidVisualization::getStiffnessMatrix() 
{       
    float dstride0 = stiffness_MultiArray_.layout.dim[0].stride; //9 just total elements
    float dstride1 = stiffness_MultiArray_.layout.dim[1].stride; //3
    float h = stiffness_MultiArray_.layout.dim[0].size;
    float w = stiffness_MultiArray_.layout.dim[1].size;

    Eigen::Matrix3f stiffness_matrix   = Eigen::Matrix3f::Zero();
    for(int i=0; i<h; ++i){
        for(int j; j<w; ++j){
            // multiarray(i,j,k) = data[data_offset + dim[1]stride*i + dim[2]stride*j + k]
            stiffness_matrix(i,j) = stiffness_MultiArray_.data[dstride1*i+j];
        }
    }
    return stiffness_matrix;
}

std::pair<Eigen::Matrix3f, Eigen::Vector3f> EllipsoidVisualization::computeEigenValuesAndVectors(Eigen::Matrix3f stiffness)
{
    Eigen::Vector3f eigen_values  = Eigen::Vector3f::Identity();
    Eigen::Matrix3f eigen_vectors = Eigen::Matrix3f::Zero();
    
    // Compute eigen values and eigen vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(stiffness);

    if (eigensolver.info() == Eigen::Success)
    {
        eigen_values  = eigensolver.eigenvalues();
        eigen_vectors = eigensolver.eigenvectors();
    }
    else
    {
        ROS_WARN_THROTTLE(1, "Failed to compute eigen vectors/values. Is the stiffness matrix correct?");
    }

    return std::make_pair(eigen_vectors, eigen_values);
}

void EllipsoidVisualization::run()
{
    if(stiffness_MultiArray_.data.empty()){
        return; // no data, no muney
    }
    
    getTF();
    Eigen::Matrix3f stiffness_matrix = getStiffnessMatrix();
    std::pair<Eigen::Matrix3f, Eigen::Vector3f> stiffnessEigenVectorsAndValues = computeEigenValuesAndVectors(stiffness_matrix);
    // ROS_INFO_STREAM(stiffnessEigenVectorsAndValues.first);
    // ROS_INFO_STREAM(stiffnessEigenVectorsAndValues.second);
    // computeRotation(std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pair);
    // computeScale();

    setMarkerMsg();

    marker_pub_.publish(ellipsoid_);


}

void EllipsoidVisualization::setMarkerMsg()
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    ellipsoid_.header.frame_id = base_frame_name_; // verancer nog
    ellipsoid_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    ellipsoid_.ns = "Ellipsoid"; // TODO: make variable
    ellipsoid_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    ellipsoid_.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    ellipsoid_.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    ellipsoid_.pose.position.x = base_to_ee_.getOrigin().x();//1;
    ellipsoid_.pose.position.y = base_to_ee_.getOrigin().y();//1;
    ellipsoid_.pose.position.z = base_to_ee_.getOrigin().z();//1;
    // marker_.pose.position = base_to_marker_.getOrigin();
    ellipsoid_.pose.orientation.x = 0;//base_to_ee_.getRotation().x();
    ellipsoid_.pose.orientation.y = 0;//base_to_ee_.getRotation().y();
    ellipsoid_.pose.orientation.z = 0;//base_to_ee_.getRotation().z();
    ellipsoid_.pose.orientation.w = 1;//base_to_ee_.getRotation().w();
    // marker_.pose.orientation = base_to_ee_.getRotation();
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    ellipsoid_.scale.x = 0.1;
    ellipsoid_.scale.y = 0.2;
    ellipsoid_.scale.z = 0.30;

    // Set the color -- be sure to set alpha to something non-zero!
    ellipsoid_.color.r = 1.0f;
    ellipsoid_.color.g = 0.3f;
    ellipsoid_.color.b = 1.0f;
    ellipsoid_.color.a = 0.3;

    ellipsoid_.lifetime = ros::Duration();
}

void EllipsoidVisualization::getTF()
{
    if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    {
        TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    }
}

int main( int argc, char** argv )
{

	ros::init(argc, argv, "stiffness_ellipsoid");

	EllipsoidVisualization node;

  
    while (ros::ok())
    {

        ros::Rate loop_rate(30);

        node.run();

        ros::spinOnce();
        loop_rate.sleep();
    }

}