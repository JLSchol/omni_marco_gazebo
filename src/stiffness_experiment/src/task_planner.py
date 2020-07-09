#!/usr/bin/env python

import rospy
import moveit_commander
import numpy
import math
from math import pi
import copy

from tf2_ros import TransformListener, Buffer, LookupException
from tf2_ros import ConnectivityException, ExtrapolationException

from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseStamped, Point
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.msg import LinkStates

from moveit_commander.conversions import pose_to_list

from tf.transformations import *


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

            elif type(goal) is PoseStamped:
                return all_close(goal.pose, actual.pose, tolerance)

            elif type(goal) is Pose:
                return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True


class EEFPathPlanning():

    def __init__(self):
        rospy.init_node('EEF_Path_Planning', anonymous=False)

        # move it configuration
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "marco_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        eef_link = move_group.get_end_effector_link()

        #tf
        self._tfBuffer = Buffer()
        listener = TransformListener(self._tfBuffer)

        # subsriber topics
        link_state_sub = rospy.Subscriber('/gazebo/link_states',LinkStates, self.linkStateCB)
        # rospy.wait_for_message('/gazebo/link_states',LinkStates)

        # publish topics
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                            DisplayTrajectory,
        #                                            queue_size=20)
        marker_pub = rospy.Publisher('visualization_marker_path',
                                                   MarkerArray,
                                                   queue_size=20)
        pose_waypoint = rospy.Publisher('waypoints_pose',
                                                   PoseArray,
                                                   queue_size=20)
        door_angles = rospy.Publisher('door_angles',
                                                   Float64,
                                                   queue_size=20)

        # print "============ Printing robot Start state"
        # print(robot.get_current_state())

        # class variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher
        self.eef_link = eef_link
        # print("EE_link: {}".format(self.eef_link))

        self.link_state_sub = link_state_sub
        self.link_states = LinkStates()

        self.marker_pub = marker_pub
        self.pose_waypoint = pose_waypoint

        
        self.waypoints=[]


        # self.setHingeJointAngle = setHingeJointAngle
        # gazebo service for manipulating the door by setting the joint angle
        # rospy.wait_for_service('/gazebo/set_model_configuration')
        # setHingeJointAngle = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration ,persistent=True)
        # try:
        # self.door_angle_service(x, y)
        # except rospy.ServiceException as exc:
        # print("Service did not process request: " + str(exc))
        # SetModelConfiguration message:
        # string model_name
        # string urdf_param_name
        # string[] joint_names
        # float64[] joint_positions
        # bool success
        # string status_message

    def listenToTransform(self, targetFrame, sourceFrame):    
        trans = []  
        try:        # lookup_transform(from this frame, to this frame)
            trans = self._tfBuffer.lookup_transform(targetFrame, sourceFrame, 
                                                            rospy.Time(), rospy.Duration(1))            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return trans


    def linkStateCB(self,message):
        self.link_states = message


    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)

    def visualize_Trajectory(self):

        markerArray = MarkerArray()
        i=0

        for pose in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            marker.pose.position.z = pose.position.z
            
            markerArray.markers.append(marker)
            i=i+1

        # Publish the MarkerArray
        self.marker_pub.publish(markerArray)


    def go_to_joint_state(self,joint_goal=False):
        # set std joint state if not defined
        if joint_goal==False:
            joint_goal = self.move_group.get_current_joint_values()
            joint_goal[0] = 0.2
            joint_goal[1] = -0
            joint_goal[2] = 0
            joint_goal[3] = 0.29
            joint_goal[4] = 0
            joint_goal[5] = 0
            joint_goal[6] = 0

        self.move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def get_oven_link_state(self):
        # get hinge position in worldframe from topic /gazebo/link_states
        hinge_x = []
        hinge_y = []
        hinge_z = []
        # print(self.link_states)
        # print(type(self.link_states))
        rospy.wait_for_message('/gazebo/link_states',LinkStates)        
        for i,link_name in enumerate(self.link_states.name):
            # print("loop {}, name {}".format(i,link_name))
            if link_name == 'microwave_oven::door_link':
                # print "printing pose of door link"
                # print(self.link_states.pose[i])
                hinge_x = self.link_states.pose[i].position.x # in gazebo world frame (should be same as base_footprint)
                hinge_y = self.link_states.pose[i].position.y
                hinge_z = self.link_states.pose[i].position.z
                break
        return hinge_x, hinge_y, hinge_z

    def get_wrist_at_handle_pose(self, hinge_x, hinge_y, hinge_z, r_hinge_to_handle, l_handle_to_wrist, height_handle, theta):

        # get wrist_ft_tool_link pose from tf for the height and pose
        # add wait for transform
        wrist_ft_tool_link = self.listenToTransform("base_footprint",self.eef_link) 
        print("ee: {}".format(self.eef_link))
        x_ee = wrist_ft_tool_link.transform.translation.x
        y_ee = wrist_ft_tool_link.transform.translation.y
        z_ee = wrist_ft_tool_link.transform.translation.z

        qx = wrist_ft_tool_link.transform.rotation.x
        qy = wrist_ft_tool_link.transform.rotation.y
        qz = wrist_ft_tool_link.transform.rotation.z
        qw = wrist_ft_tool_link.transform.rotation.w
        q_ee = [qx,qy,qz,qw]
        print "printing ee quat list"
        # print(q_ee)
        # print(z_ee)

        # calc wrist position 
        # y, and x use the axis definition of the world frame
        # These equation describe the motion of EE wrt the hinge by varying theat [0->pi/2]
        add_y = lambda theta,r,l: -r*math.cos(theta) + l*math.cos(pi/2 - theta)
        add_x = lambda theta,r,l: -r*math.sin(theta) - l*math.sin(pi/2 - theta)

        # add the difference between wrist_ft_tool_link and door handle to the hinge position in world frame
        x = hinge_x + add_x(theta,r_hinge_to_handle,l_handle_to_wrist)
        y = hinge_y + add_y(theta,r_hinge_to_handle,l_handle_to_wrist)
        # find the rotation of the endeffector in quats around world z axis
        q_rot = quaternion_from_euler(0,0,-theta) # rote around z-axis with -theta
        # apply the rotation to endeffector (in world frame thus pre multiply)
        q = quaternion_multiply(q_rot,q_ee)

        # set pose messages
        wpose = Pose()
        wpose.position.x = x 
        wpose.position.y = y 
        wpose.position.z = hinge_z + height_handle 
        print(wpose.position.z)
        wpose.orientation = Quaternion(x=q[0],y=q[1],z=q[2],w=q[3])

        return wpose


    def cart_door_plan(self,hinge_to_handle_radius,door_to_ee_length,height_handle):

        # get hinge position in worldframe from topic /gazebo/link_states
        hinge_x, hinge_y, hinge_z = self.get_oven_link_state()

        # init pose array message
        waypoints_array = PoseArray()
        waypoints_array.header.frame_id = "base_footprint" # or base_footprint (should be exactly the same but tf troubles)
        # wrist_at_handle = first waypoint at theta is 0
        # wpose = self.get_wrist_at_handle_pose(hinge_x, hinge_y, hinge_z, hinge_to_handle_radius, door_to_ee_length, height_handle, 0)
        # wpose_handle = Pose( position=Point(x=0.797,y=-0.183,z=(height_handle+)),
        #                      orientation=Quaternion(x=1.000,y=-0.016,z=-0.002,w=-0.011) )
        # wpose_handle = wpose
        # self.waypoints.append(copy.deepcopy(wpose))

        # create trajectory based on angle steps
        start = 0 # rad
        end = pi/2 # rad
        step = 0.045 # rad no idea how big???
        angles = []

        for theta in numpy.arange(start, end, step): # loop over angles
            # calculate wrist pose angle theta
            wpose = self.get_wrist_at_handle_pose(hinge_x, hinge_y, hinge_z, hinge_to_handle_radius, door_to_ee_length, height_handle, theta)
            # append for waypoints
            self.waypoints.append(copy.deepcopy(wpose))
            waypoints_array.poses.append(wpose)
            # append angle
            angles.append(theta)

        # compute plan from waypoints
        (plan, fraction) = self.move_group.compute_cartesian_path(
                                           self.waypoints,   # waypoints to follow
                                           0.01,        # eef_step resolution of 1 cm
                                           0.0)         # jump_threshold
        # Note: We are just planning and publishing plan, not moving
        self.pose_waypoint.publish(waypoints_array)
        return plan, fraction, angles, wpose.orientation


def main():

    pl = EEFPathPlanning()
    print "============ Press `Enter` to go to start position ============"
    raw_input()
    # joint_start_pos = [0.6717, -0.8694, -3.3071, 2.1338, -0.5232, -1.5398, 1.2745]
    joint_start_pos = [2.4, -1.27, 1.43, 2.17, -0.59, -1.19, 0.59]
    test_bool = pl.go_to_joint_state(joint_start_pos)



    #########################       FIRST PART      ################################
    #########################       Approach door   ################################
    # assumes torso_lift_joint = 0.3 and gripper_joint 0.37 


    # print "============ Press `Enter` to approach door ============"

    # move to door handle
    # print "============ Press `Enter` to approach door ============"

    raw_input()
    # joint_angles_at_door = [1.35, -0.18, 1.46, 0.35, -1.28, -0.58, -0.37]
    # # joint_angles_at_door = [1.28, -0.12, 0.81, 0.33, -1.82, -0.62, -0.54]
    # test_bool = pl.go_to_joint_state(joint_angles_at_door)

    #########################       SECOND PART     ################################
    #########################       Open door       ################################
    # 0.335 approximate lengt of radius hinge to handle    
    # 0.15 no idea whatbut should be distance between ee_link and the door
    print "============ Press `Enter` to approach and open door ============"
    # need to sleep otherwise it goes wrong
    raw_input()
    cart_plan, fraction2, door_angles, ee_quats = pl.cart_door_plan(0.325,0.23,0.15)
    pl.visualize_Trajectory()
    print("press enter to excecute")
    
    # pl.display_trajectory_publisher.publish()
    raw_input()
    pl.execute_plan(cart_plan)

    # eind_js = [2.43,0.04,0.95,1.04,-2.06,1.26,0.47]

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pl.visualize_Trajectory()
        # self.display_trajectory_publisher.publish()
        # pl.display_trajectory_publisher.publish()
        print('...')
        # publish joint angle door

        

        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass