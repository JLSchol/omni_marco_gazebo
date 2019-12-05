#!/usr/bin/env python
from rospy import init_node, is_shutdown, Rate, loginfo, sleep, get_param
from rospy import ROSInterruptException, Time

#import messages
from geometry_msgs.msg import TransformStamped

# import numpy as np
from PyKDL import Rotation 
from tf2_ros import StaticTransformBroadcaster
from tf_conversions import transformations





class hapticDeviceRotation():
    def __init__(self):
        init_node("haptic_device_rotation", anonymous=True)
        self._getParameters()

    
    def _getParameters(self):
        self._HDFrame = get_param("~HD_frame_name") #from launchfile
        self._robotBaseFrame = get_param("~robot_base_frame_name") #from launchfile
        rotMatrixString = get_param("~rot_matrix_array")
        self._rotMatrixArray = self._getMatrixList(rotMatrixString)

    def _getMatrixList(self, matrixString):
        matrixList = matrixString.split(" ")
        matrixListFloats = [float(char) for char in matrixList]
        return matrixListFloats

    def _setTransform(self, parentName, childName, rot):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = Time.now()
        static_transformStamped.header.frame_id = parentName
        static_transformStamped.child_frame_id = childName

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0
        
        # these vecotrs are the columns!! of the rotation matrix
        #rot = Rotation(Vector(0,0,-1),Vector(-1,0,0),Vector(0,1,0)) 
        # rot = Rotation(0,-1,0,0,0,1,-1,0,0)
        rot = Rotation(rot[0],rot[1],rot[2],rot[3],rot[4],rot[5],rot[6],rot[7],rot[8])
        quat = rot.GetQuaternion()

        # quat = Quaternion.setRPY(float(eulerAngles[0]),float(eulerAngles[1]),float(eulerAngles[2]))
        # quat = transformations.quaternion_from_euler(
        #            float(eulerAngles[0]),float(eulerAngles[1]),float(eulerAngles[2]))
        static_transformStamped.transform.rotation.x = quat[0]#-0.5#quat[0]
        static_transformStamped.transform.rotation.y = quat[1]#0.5#quat[1]
        static_transformStamped.transform.rotation.z = quat[2]#0.5#quat[2]
        static_transformStamped.transform.rotation.w = quat[3]#0.5#quat[3]

        return static_transformStamped 
            

    def run(self):
        rosRate = Rate(30)
        broadcaster = StaticTransformBroadcaster()
        while not is_shutdown():
            loginfo("hoi")
            # staticTransform = self._setTransform(self._HDFrame,self._robotBaseFrame,[-1.5707963,0,1.5707963])
            staticTransform = self._setTransform(self._HDFrame,self._robotBaseFrame,self._rotMatrixArray) 
            broadcaster.sendTransform(staticTransform)

            rosRate.sleep()    
        
        
        
if __name__ == "__main__":
	
	try:
		node = hapticDeviceRotation()
		node.run()
	except ROSInterruptException:
		pass