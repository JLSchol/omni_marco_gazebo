#include "omni_2_marker/Omni2Marker.h"

Omni2Marker::Omni2Marker():
nh_("~")
{
    // ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();
}  

void Omni2Marker::getParameters()
{
    // nh_.param<std::string>("joint_state_topic_name", joint_state_topic_name_, "/omni1_joint_states"); 
    nh_.param<std::string>("button_event_topic_name", button_event_topic_name_, "/omni1_button"); 
    nh_.param<std::string>("lock_state_topic_name", lock_state_topic_name_, "/omni1_lock_state");

    nh_.param<std::string>("marker_topic_name", marker_topic_name_, "/marker_visualization"); 
    // nh_.param<std::string>("marker_trans_topic_name", marker_trans_topic_name_, "/marker_transform"); 

    nh_.param<std::string>("robot_reference_frame", robot_reference_frame_name_, "base_footprint"); // nodig?
    nh_.param<std::string>("HD_frame_name", HD_frame_name_, "omni_rotation"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "wrist_ft_tool_link"); 
    nh_.param<std::string>("virtual_marker_name", virtual_marker_, "virtual_marker"); 

    nh_.param<double>("scale_marker_deviation", scale_marker_deviation_, 1.0);  
}

void Omni2Marker::initializeSubscribers()
{
    // joint_State_sub_ = nh_.subscribe(joint_state_topic_name_, 1, &Omni2Marker::CB_getJointStates, this);
    button_event_sub_ = nh_.subscribe(button_event_topic_name_, 1, &Omni2Marker::CB_getButtonEvent, this);
    lock_state_sub_ = nh_.subscribe(lock_state_topic_name_, 1, &Omni2Marker::CB_getLockState, this);
}

void Omni2Marker::initializePublishers()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_name_,1);
    // marker_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(marker_trans_topic_name_,1);
}

// void Omni2Marker::CB_getJointStates(const sensor_msgs::JointState& jointstate_message)
// {
//     jointstate_msg_ = jointstate_message;
// }

void Omni2Marker::CB_getButtonEvent(const phantom_omni::PhantomButtonEvent& button_message)
{
    button_msg_ = button_message;
}

void Omni2Marker::CB_getLockState(const phantom_omni::LockState& lockstate_message)
{
    lockstate_msg_ = lockstate_message;
}

void Omni2Marker::getTF(tf2_ros::Buffer& buffer)
{
    // Read as buffer.lookupTransform(target_frame, source_frame, when?, wait ..sec untill available):
    // Where the transform can mean two things: 
    //1) find a transform that maps a pose(point/rot) defined in source frame to the target frame
    // OR:
    //2) find the pose(point/rot) of the source_frame as seen from the target frame
    try{
        ee_in_base_ = buffer.lookupTransform(robot_reference_frame_name_,ee_frame_name_ ,  
                                ros::Time(0),ros::Duration(0.25) ); 
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());;
        } 
    try{
        HD_to_base_trans_ = buffer.lookupTransform(robot_reference_frame_name_,HD_frame_name_ ,
                                ros::Time(0), ros::Duration(0.25) );
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        } 
}

void Omni2Marker::run()
{  
    if ( lockstate_msg_.lock_grey == true)
    {
        this->publish_on_ = true;  // set boolean to true if both buttons are pressed
    }

    //sms
    if (publish_on_) // naar 0900 9292
    {
        // Find transform 
        std::vector<double> deviation_from_lock;
        findDeviationFromLockPosition(deviation_from_lock);
        addMarkerTransform(deviation_from_lock);

        // fill messages for visualizatoin
        fillMarkerMsg(marker_in_base_);

        // publish messages
        marker_pub_.publish(marker_);
        // marker_transform_pub_.publish(marker_in_base_);
    }
}

void Omni2Marker::findDeviationFromLockPosition(std::vector<double> &deviation_from_lock)
{
    // This is in the omni frame!! (y up, x right, z  towards)
    deviation_from_lock.push_back(lockstate_msg_.current_position.x - lockstate_msg_.lock_position.x);
    deviation_from_lock.push_back(lockstate_msg_.current_position.y - lockstate_msg_.lock_position.y);
    deviation_from_lock.push_back(lockstate_msg_.current_position.z - lockstate_msg_.lock_position.z);
}

void Omni2Marker::addMarkerTransform(const std::vector<double> &deviation_from_lock)
{
    geometry_msgs::Vector3 deviation, rotated_deviation;

    // scale the deviations of the omni
    deviation.x = scale_marker_deviation_ * deviation_from_lock[0];
    deviation.y = scale_marker_deviation_ * deviation_from_lock[1];
    deviation.z = scale_marker_deviation_ * deviation_from_lock[2];

    // The vector that is defined in the omni frame is mapped (rotated) to the base frame
    rotated_deviation = vectorRotation(HD_to_base_trans_.transform.rotation,deviation);

    marker_in_base_.header.stamp = ros::Time::now();
    marker_in_base_.header.frame_id = robot_reference_frame_name_;
    marker_in_base_.child_frame_id = virtual_marker_;
    marker_in_base_.transform.translation.x = ee_in_base_.transform.translation.x + rotated_deviation.x;
    marker_in_base_.transform.translation.y = ee_in_base_.transform.translation.y + rotated_deviation.y;
    marker_in_base_.transform.translation.z = ee_in_base_.transform.translation.z + rotated_deviation.z;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0); // no rotation wrt the robot_reference_frame_name_!
    marker_in_base_.transform.rotation.x = quat.x(); 
    marker_in_base_.transform.rotation.y = quat.y(); // no rotation wrt the omni_frame
    marker_in_base_.transform.rotation.z = quat.z(); // no rotation wrt the omni_frame
    marker_in_base_.transform.rotation.w = quat.w(); // no rotation wrt the omni_frame

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(marker_in_base_);
}

void Omni2Marker::fillMarkerMsg(geometry_msgs::TransformStamped& trans)
{
    // Set the frame ID and timestamp. Also parent frame
    marker_.header = trans.header;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = trans.child_frame_id; 
    marker_.id = 0;

    // Set the marker type.  
    marker_.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker_.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = trans.transform.translation.x;
    marker_.pose.position.y = trans.transform.translation.y;
    marker_.pose.position.z = trans.transform.translation.z;
    marker_.pose.orientation.x = trans.transform.rotation.x;
    marker_.pose.orientation.y = trans.transform.rotation.y;
    marker_.pose.orientation.z = trans.transform.rotation.z;
    marker_.pose.orientation.w = trans.transform.rotation.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_.scale.x = 0.05;
    marker_.scale.y = 0.05;
    marker_.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();
}

const geometry_msgs::Vector3 Omni2Marker::vectorRotation(geometry_msgs::Quaternion q, geometry_msgs::Vector3 v)
{
    geometry_msgs::Vector3 vr;
    vr.x =  v.x*q.w*q.w + 2*v.z*q.w*q.y - 2*v.y*q.w*q.z + v.x*q.x*q.x + 
            2*v.y*q.x*q.y + 2*v.z*q.x*q.z - v.x*q.y*q.y - v.x*q.z*q.z;
    vr.y =  v.y*q.w*q.w - 2*v.z*q.w*q.x + 2*v.x*q.w*q.z - v.y*q.x*q.x + 
            2*v.x*q.x*q.y + v.y*q.y*q.y + 2*v.z*q.y*q.z - v.y*q.z*q.z;
    vr.z =  v.z*q.w*q.w + 2*v.y*q.w*q.x - 2*v.x*q.w*q.y - v.z*q.x*q.x + 
            2*v.x*q.x*q.z - v.z*q.y*q.y + 2*v.y*q.y*q.z + v.z*q.z*q.z;
    return vr;   
}



int main( int argc, char** argv )
{
    // initialize ros node and class
    ros::init(argc, argv, "omni_2_marker");
    Omni2Marker node;

    node.publish_on_ = false;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener TF_listener(tfBuffer);

    while (ros::ok()) 
    {
        ros::Rate loop_rate(30); //publish_frequency_
        node.getTF(tfBuffer);
        node.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
}