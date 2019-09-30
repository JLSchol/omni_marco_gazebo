#!/usr/bin/env python


#import rospy
from rospy import init_node, is_shutdown, Time, get_param, Rate, sleep, ROSInterruptException
from geometry_msgs.msg import Pose, Quaternion, Point

from tf2_ros import TransformBroadcaster, TransformStamped



class sendMockEndEffector():
    def __init__(self):
        init_node("mock_End_Effector", anonymous=True)
        self._getParameters()


    def _getParameters(self):
        self._parentFrame = get_param("~ParentName")
        self._childFrame = get_param("~ChildName")

        rawEEPoseString = get_param("~EEPose")
        self._markerPose = self._getMarkerPose(rawEEPoseString) 


    def _getMarkerPose(self, poseString):
        poseList = poseString.split(" ")
        poseListFloats = [float(char) for char in poseList]

        point = Point( poseListFloats[0],poseListFloats[1],poseListFloats[2] )
        quaternion = Quaternion( poseListFloats[3],poseListFloats[4],poseListFloats[5],poseListFloats[6] )

        return Pose(position=point, 
                    orientation=quaternion)


    def run(self):
        rosRate = Rate(30)
        while not is_shutdown():

            self._broadcastTransform(self._markerPose, self._parentFrame, self._childFrame) #map2center
           
        rosRate.sleep()


    def _broadcastTransform(self, Pose, parentName, childName):
        br = TransformBroadcaster()
        tr = TransformStamped()

        tr.header.stamp = Time.now() 
        tr.header.frame_id = parentName
        tr.child_frame_id = childName
        tr.transform.translation = Pose.position
        tr.transform.rotation = Pose.orientation

        br.sendTransform(tr)


        

if __name__ == "__main__":
	
	try:
		node = sendMockEndEffector()
		node.run()
	except ROSInterruptException:
		pass
