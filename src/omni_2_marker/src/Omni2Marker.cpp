#include "omni_2_marker/Omni2Marker.h"

Omni2Marker::Omni2Marker():
nh_("~")
{
    ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_(tfBuffer_);
}  

void Omni2Marker::getParameters()
{
    // nh_.param<std::string>("joint_state_topic_name", joint_state_topic_name_, "/omni1_joint_states"); 
    nh_.param<std::string>("button_event_topic_name", button_event_topic_name_, "/omni1_button"); 
    nh_.param<std::string>("lock_state_topic_name", lock_state_topic_name_, "/omni1_lock_state");

    nh_.param<std::string>("marker_topic_name", marker_topic_name_, "/marker_visualization"); 
    nh_.param<std::string>("marker_trans_topic_name", marker_trans_topic_name_, "/marker_transform"); 

    nh_.param<std::string>("base_frame_name", base_frame_name_, "base_footprint"); // nodig?
    nh_.param<std::string>("HD_frame_name", HD_frame_name_, "omni_rotation"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "wrist_ft_tool_link"); 

    nh_.param<double>("scale_marker_deviation", scale_marker_deviation_, 1.0);  
}

void Omni2Marker::initializeSubscribers()
{
    // joint_State_sub_ = nh_.subscribe(joint_state_topic_name_, 1, &Omni2Marker::CB_getJointStates, this);
    button_event_sub_ = nh_.subscribe(button_event_topic_name_, 1, &Omni2Marker::CB_getButtonEvent, this);
    lock_state_sub_ = nh_.subscribe(lock_state_topic_name_, 1, &Omni2Marker::CB_getLockState, this);
    // tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformListener tfListener_(tfBuffer_);
    // tf2_ros::Buffer tfBuffer_;
    // tfListener_(tfBuffer_);
}

void Omni2Marker::initializePublishers()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_name_,1);
    marker_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(marker_trans_topic_name_,1);
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

void Omni2Marker::getTF()
{
    // if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    // {
    //     TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    // }

    // if (TF_listener_.waitForTransform(ee_frame_name_, HD_frame_name_, ros::Time(0), ros::Duration(0.25)))
    // {
    //     TF_listener_.lookupTransform(HD_frame_name_, ee_frame_name_,ros::Time(0), HD_to_ee_);
    // }
    // tf::Quaternion quat;
    // quat = HD_to_ee_.getRotation();
    // quat.normalize();
    // HD_to_ee_.setRotation(quat);
    // geometry_msgs::TransformStamped HD_to_ee_;
    // tf2_ros::Buffer tfBuffer_;
    // tf2_ros::TransformListener tfListener_(tfBuffer_);
    try{
        HD_to_ee_ = tfBuffer_.lookupTransform(ee_frame_name_, HD_frame_name_,
                                ros::Time(0)) //,ros::Duration(0.25) );
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            // ros::Duration(1.0).sleep();
        // continue;
        } 
}

void Omni2Marker::findDeviationFromLockPosition(std::vector<double> &deviation_from_lock)
{
    // This is in the omni frame!!
    deviation_from_lock.push_back(lockstate_msg_.current_position.x - lockstate_msg_.lock_position.x);
    deviation_from_lock.push_back(lockstate_msg_.current_position.y - lockstate_msg_.lock_position.y);
    deviation_from_lock.push_back(lockstate_msg_.current_position.z - lockstate_msg_.lock_position.z);
}

void Omni2Marker::addMarkerTransform(const std::vector<double> &deviation_from_lock)
{
    // define (map) the frame of the omni to the reference frame that you want to use to control the marker
    // TODO : Allow for input rotation matrix that specifies the marco_base wrt the omniframe
    // fix this part of the code
    // double y = -deviation_from_lock[0]; // x_direction_omni = -y_direction_marco_base;
    // double z = deviation_from_lock[1];  // y_direction_omni = z_direction_marco_base;
    // double x = -deviation_from_lock[2]; // z_direction_omni = -x_direction_marco_base;

    // get omni frame

    // This is in the omni_frame!!, therfore we can add it to the HD_to_ee_
    // tf::Vector3 deviation; 
    // deviation.setX(scale_marker_deviation_ * deviation_from_lock[0]);
    // deviation.setY(scale_marker_deviation_ * deviation_from_lock[1]);
    // deviation.setZ(scale_marker_deviation_ * deviation_from_lock[2]);


    // add the tranformation to the base frame by adding the deviation + the endeffector
    // endeffector (wrt2base) + the deviation (wrt2base)
    // base_to_marker_.setOrigin( base_to_ee_.getOrigin() + deviation );                                 
    // base_to_marker_.setRotation( tf::Quaternion(0, 0, 0, 1) ); // no frame rotation as seen from base
    geometry_msgs::Vector3 deviation;
    deviation.x = scale_marker_deviation_ * deviation_from_lock[0];
    deviation.y = scale_marker_deviation_ * deviation_from_lock[1];
    deviation.z = scale_marker_deviation_ * deviation_from_lock[2];


    HD_to_marker_.header.stamp = ros::Time::now();
    HD_to_marker_.header.frame_id = HD_frame_name_;
    HD_to_marker_.child_frame_id = "virtual_marker_transform";
    HD_to_marker_.transform.translation.x = HD_to_ee_.transform.translation.x + deviation.x;
    HD_to_marker_.transform.translation.y = HD_to_ee_.transform.translation.y + deviation.y;
    HD_to_marker_.transform.translation.z = HD_to_ee_.transform.translation.z + deviation.z;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    // geometry_msgs::Quaternion quat;
    HD_to_marker_.transform.rotation.x = quat.x(); // no rotation wrt the omni_frame
    HD_to_marker_.transform.rotation.y = quat.y(); // no rotation wrt the omni_frame
    HD_to_marker_.transform.rotation.z = quat.z(); // no rotation wrt the omni_frame
    HD_to_marker_.transform.rotation.w = quat.w(); // no rotation wrt the omni_frame

    // HD_to_marker_.setOrigin( HD_to_ee_.getOrigin() + deviation );                                 
    // HD_to_marker_.setRotation( tf::Quaternion(0, 0, 0, 1) ); // no frame rotation as seen from HD

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(HD_to_marker_);


    // tf::StampedTransform EE_to_marker;
    // EE_to_marker.setOrigin(deviation );                                 
    // EE_to_marker.setRotation( tf::Quaternion(0, 0, 0, 1) ); // no frame rotation as seen from HD
    // When incorporating rotation, this should be set to the rotation of the omni

    
    // The Virtual_marker pose is tranferred via topics rather than using tf
    // no need for broadcasting but if so:
    // tf::TransformBroadcaster br;
    // ROS_INFO_STREAM(HD_to_marker_.getOrigin().x());
    // ROS_INFO_STREAM(HD_to_marker_.getOrigin().y());
    // ROS_INFO_STREAM(EE_to_marker.getOrigin().z());
    // ROS_INFO_STREAM("---------------------------------------------");
    // br.sendTransform(tf::StampedTransform(HD_to_marker_, ros::Time::now(), HD_frame_name_, "virtual_marker_transform"));
}

void Omni2Marker::fillMarkerMsg(geometry_msgs::TransformStamped& trans)
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    // marker_.header.frame_id = reference_frame_name; // verancer nog
    // marker_.header.stamp = ros::Time::now();
    marker_.header = trans.header;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = trans.child_frame_id; // TODO: make variable
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

// void Omni2Marker::fillMarkerTransformMsg(visualization_msgs::Marker& marker,tf::StampedTransform& trans)
// {
//     // const tf::StampedTransform dummy_transform = trans;

//     // transform
//     // tf::transformStampedTFToMsg(dummy_transform,marker_transform_);
        
//     marker_transform_.header.stamp = ros::Time::now();
//     marker_transform_.header.frame_id = marker.header.frame_id;
//     marker_transform_.child_frame_id = "virtual_marker"; // TO DO: make variable
// }

void Omni2Marker::run()
{  
  getTF();

  if ( (button_msg_.grey_button && button_msg_.white_button) == true)
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

      // fill messages
      fillMarkerMsg(HD_to_marker_);
    //   fillMarkerTransformMsg(marker_,HD_to_marker_);

      // publish messages
      marker_pub_.publish(marker_);
      marker_transform_pub_.publish(HD_to_marker_);
  }
}



int main( int argc, char** argv )
{
	// initialize ros node and class
    ros::init(argc, argv, "omni_2_marker");
	Omni2Marker node;
    // tfListener_(tfBuffer_);
    node.publish_on_ = false;
// test_class amf = test_class();
// est_class amf;
    // spin
    while (ros::ok()) 
    {
        ros::Rate loop_rate(30); //publish_frequency_
        node.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
}