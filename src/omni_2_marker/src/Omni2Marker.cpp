#include "omni_2_marker/Omni2Marker.h"

Omni2Marker::Omni2Marker():
nh_("~")
{
    ROS_INFO_STREAM("----------------------------------");
    getParameters();
    initializeSubscribers();
    initializePublishers();
}  

void Omni2Marker::getParameters()
{
    nh_.param<std::string>("joint_state_topic_name", joint_state_topic_name_, "/omni1_joint_states"); 
    nh_.param<std::string>("button_event_topic_name", button_event_topic_name_, "/omni1_button"); 
    nh_.param<std::string>("lock_state_topic_name", lock_state_topic_name_, "/omni1_lock_state");

    nh_.param<std::string>("marker_topic_name", marker_topic_name_, "/marker_visualization"); 
    nh_.param<std::string>("marker_trans_topic_name", marker_trans_topic_name_, "/marker_transform"); 

    nh_.param<std::string>("base_frame_name", base_frame_name_, "base"); 
    nh_.param<std::string>("ee_frame_name", ee_frame_name_, "wrist2"); 

    nh_.param<double>("scale_marker_deviation", scale_marker_deviation_, 1.0);  
}

void Omni2Marker::initializeSubscribers()
{
    joint_State_sub_ = nh_.subscribe(joint_state_topic_name_, 1, &Omni2Marker::CB_getJointStates, this);
    button_event_sub_ = nh_.subscribe(button_event_topic_name_, 1, &Omni2Marker::CB_getButtonEvent, this);
    lock_state_sub_ = nh_.subscribe(lock_state_topic_name_, 1, &Omni2Marker::CB_getLockState, this);
}

void Omni2Marker::initializePublishers()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_name_,1);
    marker_transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(marker_trans_topic_name_,1);
}

void Omni2Marker::CB_getJointStates(const sensor_msgs::JointState& jointstate_message)
{
    jointstate_msg_ = jointstate_message;
}

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
    if (TF_listener_.waitForTransform(ee_frame_name_, base_frame_name_, ros::Time(0), ros::Duration(0.25)))
    {
        TF_listener_.lookupTransform(base_frame_name_, ee_frame_name_,ros::Time(0), base_to_ee_);
    }
}

void Omni2Marker::findDeviationFromLockPosition(std::vector<double> &deviation_from_lock)
{
    deviation_from_lock.push_back(lockstate_msg_.current_position.x - lockstate_msg_.lock_position.x);
    deviation_from_lock.push_back(lockstate_msg_.current_position.y - lockstate_msg_.lock_position.y);
    deviation_from_lock.push_back(lockstate_msg_.current_position.z - lockstate_msg_.lock_position.z);
}

void Omni2Marker::addMarkerTransform(const std::vector<double> &deviation_from_lock)
{
    // define (map) the frame of the omni to the reference frame that you want to use to control the marker
    double y = -deviation_from_lock[0]; // x_direction_omni = -y_direction_marco_base;
    double z = deviation_from_lock[1];  // y_direction_omni = z_direction_marco_base;
    double x = -deviation_from_lock[2]; // z_direction_omni = -x_direction_marco_base;

    tf::Vector3 deviation; 
    deviation.setX(scale_marker_deviation_ * x);
    deviation.setY(scale_marker_deviation_ * y);
    deviation.setZ(scale_marker_deviation_ * z);

    // add the tranformation to the base frame by adding the deviation + the endeffector
    // endeffector (wrt2base) + the deviation (wrt2base)
    base_to_marker_.setOrigin( base_to_ee_.getOrigin() + deviation );                                 
    base_to_marker_.setRotation( tf::Quaternion(0, 0, 0, 1) ); // no frame rotation as seen from base

    // no need for broadcasting but if so:
    // tf::TransformBroadcaster br;
    // br.sendTransform(tf::StampedTransform(base_to_marker_, ros::Time::now(), base_frame_name_, "virtual_marker"));
}

void Omni2Marker::fillMarkerMsg()
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_.header.frame_id = base_frame_name_; // verancer nog
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = "virtual_marker";
    marker_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker_.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = base_to_marker_.getOrigin().x();
    marker_.pose.position.y = base_to_marker_.getOrigin().y();
    marker_.pose.position.z = base_to_marker_.getOrigin().z();
    // marker_.pose.position = base_to_marker_.getOrigin();
    marker_.pose.orientation.x = base_to_marker_.getRotation().x();
    marker_.pose.orientation.y = base_to_marker_.getRotation().y();
    marker_.pose.orientation.z = base_to_marker_.getRotation().z();
    marker_.pose.orientation.w = base_to_marker_.getRotation().w();
    // marker_.pose.orientation = base_to_ee_.getRotation();
    
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

void Omni2Marker::fillMarkerTransformMsg()
{
    const tf::StampedTransform dummy_transform = base_to_marker_;
    // new_transform = &base_to_marker_;

    tf::transformStampedTFToMsg(dummy_transform,marker_transform_);
    // base_to_marker_.transformStampedTFToMsg

    marker_transform_.header.stamp = marker_.header.stamp;
    marker_transform_.header.frame_id = marker_.header.frame_id;
    marker_transform_.child_frame_id = "virtual_marker";
    // marker_transform_.transform 
}

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
      std::vector<double> deviation_from_lock;
      findDeviationFromLockPosition(deviation_from_lock);
      addMarkerTransform(deviation_from_lock);
      fillMarkerMsg();
      fillMarkerTransformMsg();
      marker_pub_.publish(marker_);
      marker_transform_pub_.publish(marker_transform_);
  }
}



int main( int argc, char** argv )
{
	ros::init(argc, argv, "omni_2_marker");
	Omni2Marker node;
    node.publish_on_ = false;

  while (ros::ok()) // fix publish rate to 30 hz
  {
    ros::Rate loop_rate(30); //publish_frequency_
    node.run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}