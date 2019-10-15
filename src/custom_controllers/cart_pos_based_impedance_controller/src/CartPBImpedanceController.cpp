#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
// Boost
#include <boost/thread.hpp>
// URDF
#include <urdf/model.h>
// Eigen
// #include <Eigen/Geometry>
//KDL
#include <kdl_parser/kdl_parser.hpp>

namespace cart_pos_based_impedance_controller_ns
{

class CartPBImpedanceController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
// class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
   

{
public:

    bool init(hardware_interface::PositionJointInterface* hw, 
                                    ros::NodeHandle& root_nh,
                                    ros::NodeHandle& controller_nh)
    {
         controller_nh_ = controller_nh;
        
        // Controller name
        name_ = getLeafNamespace(controller_nh_);
        ROS_INFO_STREAM("NAME = " << name_);

        // Name list of controlled joints
        joint_names_ = getStrings(controller_nh_, "joints");
        if (joint_names_.empty())
        {
            return false;
        }
        const unsigned int n_joints = joint_names_.size();
        
        // URDF joints
        boost::shared_ptr<urdf::Model> urdf = getUrdf(root_nh, "robot_description");
        if (!urdf)
        {
            return false;
        }
        std::vector<UrdfJointConstPtr> urdf_joints =
            getUrdfJoints(*urdf, joint_names_);
        if (urdf_joints.empty())
        {
            return false;
        }
        assert(n_joints == urdf_joints.size());

        // Create kdl tree
        // KDL::Tree kdl_tree(const std::string "arm_1_joint");
        // if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree))
        // {
        //     ROS_INFO_STREAM("IN IF LOOP");
        //     ROS_ERROR_NAMED(name_, "Could not create KDL Tree from URDF");
        //     return false;
        // }
        // ROS_INFO_STREAM("BEFOR NAMED KDL INFO");
        // ROS_INFO_NAMED(name_, "KDL tree loaded with %d joints and %d segments",
        //          kdl_tree.getNrOfJoints(), kdl_tree.getNrOfSegments());
        // std::string base_frame, ee_frame;
        // controller_nh_.getParam("base_frame", base_frame);
        // controller_nh_.getParam("ee_frame", ee_frame);
        // if (!kdl_tree.getChain(base_frame, ee_frame, kdl_chain_))
        // {
        //     ROS_ERROR_NAMED(name_, "Could not create KDL chain from base frame '%s' to "
        //                         "end effector frame '%s'",
        //                     base_frame.c_str(), ee_frame.c_str());
        //     return false;
        // }



        // When the controller is loaded, this code get executed
        std::string my_joint;
        if(!controller_nh.getParam("joint",my_joint))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        joint_ = hw->getHandle(my_joint); // throws on failure
        command_ = joint_.getPosition();  // set current joint  state to goal state

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
    std::string name_;
    double gain_;                           // loaded from the config file
    double command_;                        // set point to 
    ros::Subscriber sub_command_;
    ros::NodeHandle controller_nh_;
    std::vector<std::string> joint_names_;


    // get controller name
    std::string getLeafNamespace(const ros::NodeHandle& nh)
    {
        const std::string complete_ns = nh.getNamespace();
        ROS_INFO_STREAM("COMPLETE NAME:"<< complete_ns);
        std::size_t id = complete_ns.find_last_of("/");
        return complete_ns.substr(id + 1);
    }


    // get joint names
    std::vector<std::string> getStrings(const ros::NodeHandle& nh,
                                        const std::string& param_name)
    {
        using namespace XmlRpc;
        XmlRpcValue xml_array;
        if (!nh.getParam(param_name, xml_array))
        {
            ROS_ERROR_STREAM("Could not find '" << param_name
                                                << "' parameter (namespace: "
                                                << nh.getNamespace() << ").");
            return std::vector<std::string>();
        }
        if (xml_array.getType() != XmlRpcValue::TypeArray)
        {
            ROS_ERROR_STREAM("The '" << param_name
                                    << "' parameter is not an array (namespace: "
                                    << nh.getNamespace() << ").");
            return std::vector<std::string>();
        }

        std::vector<std::string> out;
        for (int i = 0; i < xml_array.size(); ++i)
        {
            if (xml_array[i].getType() != XmlRpcValue::TypeString)
            {
            ROS_ERROR_STREAM(
                "The '" << param_name
                        << "' parameter contains a non-string element (namespace: "
                        << nh.getNamespace() << ").");
            return std::vector<std::string>();
            }
            out.push_back(static_cast<std::string>(xml_array[i]));
        }
        return out;
    }


    // get urdf model pointer
    boost::shared_ptr<urdf::Model> getUrdf(const ros::NodeHandle& nh,
                                       const std::string& param_name)
    {
        boost::shared_ptr<urdf::Model> urdf(new urdf::Model);

        std::string urdf_str;
        // Check for robot_description in proper namespace
        if (nh.getParam(param_name, urdf_str))
        {
            if (!urdf->initString(urdf_str))
            {
            ROS_ERROR_STREAM("Failed to parse URDF contained in '"
                            << param_name << "' parameter (namespace: "
                            << nh.getNamespace() << ").");
            return boost::shared_ptr<urdf::Model>();
            }
        }
        // Check for robot_description in root
        else if (!urdf->initParam("robot_description"))
        {
            ROS_ERROR_STREAM("Failed to parse URDF contained in '" << param_name
                                                                << "' parameter");
            return boost::shared_ptr<urdf::Model>();
        }
        return urdf;
    }


    // get joints from urdf model pointer
    typedef boost::shared_ptr<const urdf::Joint> UrdfJointConstPtr;
    std::vector<UrdfJointConstPtr>
    getUrdfJoints(const urdf::Model& urdf,
                const std::vector<std::string>& joint_names)
    {
        std::vector<UrdfJointConstPtr> out;
        for (unsigned int i = 0; i < joint_names.size(); ++i)
        {
            UrdfJointConstPtr urdf_joint = urdf.getJoint(joint_names[i]);
            if (urdf_joint)
            {
            out.push_back(urdf_joint);
            }
            else
            {
            ROS_ERROR_STREAM("Could not find joint '" << joint_names[i]
                                                        << "' in URDF model.");
            return std::vector<UrdfJointConstPtr>();
            }
        }
        return out;
    }
    
}; // end class: CartPBImpedanceController



PLUGINLIB_EXPORT_CLASS(cart_pos_based_impedance_controller_ns::CartPBImpedanceController, 
                        controller_interface::ControllerBase) // plugin for controllerBase type

} // end of namespace: cart_pos_based_impedance_controller_ns 
