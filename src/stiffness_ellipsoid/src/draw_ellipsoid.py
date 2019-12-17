#!/usr/bin/env python


from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, sleep
from rospy import ROSInterruptException, Time
from tf2_ros import TransformBroadcaster

#import messages
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from visualization_msgs.msg import Marker

import PyKDL


import numpy as np


class DrawEllipsoid(object):
    def __init__(self):
        init_node("draw_ellipsoid", anonymous=False)
        self._publisher = Publisher("/ellipsoid_visualization", Marker, queue_size=50)
        self._subscribers = Subscriber("/covariance_matrix", Float32MultiArray, self._callback)
        self._getParameters()
        self._multiArray = []

    def _getParameters(self):
        pass

    def _callback(self, message):
        self._multiArray = message
        

    def run(self):

        publishRate = 30 # Hz
        rosRate = Rate(publishRate)
        while not is_shutdown():
            if not self._multiArray:  
                continue

            covMat = self._convertToMatrix(self._multiArray)
            covMatNpTp = np.transpose(np.array(covMat)) 

            (eigValues,eigVectors) = np.linalg.eig(covMatNpTp)

            # loginfo("eigValues:")
            # loginfo("\n" + str(eigValues))
            # loginfo("eigVectors:")
            # loginfo("\n" + str(eigVectors))

            # The eigensolver returns random order of vector value pairs
            # Assuming that the eigenvectors represent a rotation, total of six orders exist 
            # The rotation matrix is either a reflection (determinant is -1) which reverses "handedness" 
            # or a rotation (determinant is 1) which preserves "handedness".
            # Therefore, only 3 are correct and associated with a rotation.
            rightHanded = self._checkRightHandedNessMatrix(eigVectors)
            if rightHanded == False:
                (eigValues,eigVectors) = self._shuffleEig(rightHanded,eigValues,eigVectors)

            quaternion = self._getRotation(eigVectors)

            scaledEigenValues = self._getScaledEigenvalues(eigValues,0.05,0.25)

            ellipsoid = self._createEllipsoidMsg([0,0,0],quaternion,scaledEigenValues)

            self._publisher.publish(ellipsoid)
            # self._broadcastEllipsoidAxis(quaternion)


            loginfo(10*"---")
            rosRate.sleep()



    def _convertToMatrix(self,multiArray):
        if not multiArray:  
            return
        # multiArray.layout.dim[0].stride =9
        # multiArray.layout.dim[1].stride =3
        dstride1 = multiArray.layout.dim[1].stride
        # dimensions
        h = multiArray.layout.dim[0].size
        w = multiArray.layout.dim[1].size
        dataVector = multiArray.data
        # create matrix
        matrix = []
        matrix = [[dataVector[i*dstride1 + j] for i in range(h)] for j in range(w)]

        return matrix


    def _checkRightHandedNessMatrix(self,matrix):
        v1 = np.array(matrix[:,0])
        v2 = np.array(matrix[:,1])
        v3 = np.array(matrix[:,2])
        loginfo(np.dot(np.cross(v1,v2),v3))
        rightHanded = False
        if np.dot(np.cross(v1,v2),v3)>0: #1>0
            rightHanded = True
            

        loginfo(rightHanded)
        return rightHanded


    def _shuffleEig(self,flag,eigenValues,eigenVectors):
        # Shuffel 2 arbritary vectors will always result in a rotation instead of an reflection
        shuffleSequence = [0,2,1] # in stead of [0,1,2]
        i = np.argsort(shuffleSequence)
        eigenValues = eigenValues[i]
        eigenVectors = eigenVectors[:,i]

        loginfo("We are swapping!!")

        return eigenValues, eigenVectors


    def _getRotation(self,eigVectors):
        rot = PyKDL.Rotation (eigVectors[0,0],eigVectors[0,1],eigVectors[0,2],
                            eigVectors[1,0],eigVectors[1,1],eigVectors[1,2],
                            eigVectors[2,0],eigVectors[2,1],eigVectors[2,2] )
        quat = rot.GetQuaternion()

        # normalize
        length = np.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
        quaternions = [np.array(q)/length for q in quat]

        return quaternions


    def _getScaledEigenvalues(self,eigenValues,lambdaMin,lambdaMax):

        lambdaVec = np.zeros(3)
        for i,eig in enumerate(eigenValues):
            if np.abs(eig) < 10**(-6):
                eig = 0

            lambda_ = np.sqrt(eig)

            if lambda_ <= lambdaMin:
                lambda_ = lambdaMin
            elif lambda_>= lambdaMax:
                lambda_ = lambdaMax

            lambdaVec[i] = lambda_

        return lambdaVec

    
    def _createEllipsoidMsg(self,positions,quaternions,scales):
        marker = Marker()
        marker.header.frame_id ="/wrist_ft_tool_link"
        marker.header.stamp = Time.now()
        marker.ns = "ellipsoid_py"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = positions[0]
        marker.pose.position.y = positions[1]
        marker.pose.position.z = positions[2]

        marker.pose.orientation.x = quaternions[0]
        marker.pose.orientation.y = quaternions[1]
        marker.pose.orientation.z = quaternions[2]
        marker.pose.orientation.w = quaternions[3]

        #2times radius and sqrt(2) from the eigdecomposition
        marker.scale.x = 2*np.sqrt(2)*scales[0] 
        marker.scale.y = 2*np.sqrt(2)*scales[1]
        marker.scale.z = 2*np.sqrt(2)*scales[2]         

        marker.color.a = 0.3
        marker.color.r = 1
        marker.color.g = 0.3
        marker.color.b = 1

        return marker


    def _broadcastEllipsoidAxis(self,q):
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = Time.now()
        t.header.frame_id = "/wrist_ft_tool_link"
        t.child_frame_id = "ellipsoidAxis"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)




if __name__ == "__main__":
    try:
        node = DrawEllipsoid()
        node.run()

    except ROSInterruptException:
        pass