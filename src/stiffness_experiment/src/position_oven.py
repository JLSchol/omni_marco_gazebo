#!/usr/bin/env python
import rospy
# from rospy import 
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from gazebo_msgs.msg import LinkStates, ModelState
from gazebo_msgs.srv import SetModelConfiguration, SetModelState
from itertools import izip as zip

def setData(TypeClass,dat):
    x = TypeClass
    x.data = dat
    return x

class PositionOven(object):
    def __init__(self):
        rospy.init_node("position_oven",anonymous=False)
    
        self.door_angles = []
        self.oven_pose = []

        # self._fix_oven = Publisher("fix_oven_gazebo", String, queue_size=1)

        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState ,persistent=True)
        self.set_model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration ,persistent=True)




    def setModelState(self,model_name,pose,twist,ref_frame):
        # convert list to Pose
        arrToPose = lambda p: Pose(position=Point(p[0],p[1],p[2]),orientation=Quaternion(p[3],p[4],p[5],p[6]))
        # convert list to Twist
        arrToTwist = lambda p: Twist(linear=Vector3(p[0],p[1],p[2]),angular=Vector3(p[3],p[4],p[5]))
        
        # fill msgs
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = arrToPose(pose)
        model_state.twist = arrToTwist(twist)
        model_state.reference_frame = ref_frame

        # call service
        self.set_model_state(model_state)





    def setModelConfig(self, model_name, urdf_param_name, joint_names, joint_positions):
        self.set_model_config( model_name, urdf_param_name, joint_names, joint_positions )



    def run(self):

        model = 'microwave_oven'
        # poseRPY = [1.16461, -0.068784, 0.675001, -0.0, 0, -1.57302]
        # poseQuat = [1.16461, -0.068784, 0.675000, -0.0, 0, -0.7071068, 0.7071068] # goeie
        poseQuat = [1.13461, -0.138784, 0.675000, -0.0, 0, -0.7071068, 0.7071068]
        # poseQuat = [1.16461, -0.068784, 1.0, -0.0, 0, -0.7071068, 0.7071068]
        twist = [0,0,0,0,0,0]
        ref_frame = 'world'

        urdf_param_name = 'microwave_oven'
        joint_names = ['door_joint']
        door_joint = 0 #=ditch
        # door_joint = -1.5708 #=open
        # door_joint = -1.3708 
        joint_positions = [door_joint]

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.setModelState(model,poseQuat,twist,ref_frame)
            self.setModelConfig(model, urdf_param_name, joint_names, joint_positions)

            r.sleep()



if __name__ == "__main__":
    try:
        node = PositionOven()
        node.run()

    except rospy.ROSInterruptException:
        pass