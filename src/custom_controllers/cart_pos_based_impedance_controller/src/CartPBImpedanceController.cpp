#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace cart_pos_based_impedance_controller_ns
{

class CartPBImpedanceController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
// class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh)
    {
        // When the controller is loaded, this code get executed
        std::string my_joint;
        if(!controller_nh.getParam("joint",my_joint))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        joint_ = hw->getHandle(my_joint); // throws on failure
        command_ = joint_.getPosition(); // set current joint  state to goal state

        //load gain using gains set on parameter server
        if(!controller_nh.getParam("gain",gain_))
        {
            ROS_ERROR("Could not find the gain parameter value");
            return false;
        }

        //Start subscriber that listents the the topic command
        sub_command_ = controller_nh.subscribe<std_msgs::Float64>("command",1,&CartPBImpedanceController::setCommandCB, this);
        
        return true;
    }

    void update(const ros::Time& time, const ros::Duration& period)
    {
        double error = command_ - joint_.getPosition();
        double commanded_position = error * gain_;
        joint_.setCommand(commanded_position);
    }

    void setCommandCB(const std_msgs::Float64ConstPtr& msg)
    {
        command_ = msg->data; // probeer of dit wat anders mag zijn.
    }

    void starting(const ros::Time& time){ } // to be defined
    void stopping(const ros::Time& time){ } // to be defined

private:
    hardware_interface::JointHandle joint_; // Joint resource
    double gain_;                           // loaded from the config file
    double command_;                        // set point to 
    ros::Subscriber sub_command_;
}; // end class: CartPBImpedanceController



PLUGINLIB_EXPORT_CLASS(cart_pos_based_impedance_controller_ns::CartPBImpedanceController, 
                        controller_interface::ControllerBase) // plugin for controllerBase type

} // end of namespace: cart_pos_based_impedance_controller_ns 
