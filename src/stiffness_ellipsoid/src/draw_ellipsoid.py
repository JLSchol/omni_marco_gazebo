#!/usr/bin/env python


from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, sleep
from rospy import ROSInterruptException, Time

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
            covMatNp = np.array(covMat)
            covMatNpTp = np.transpose(np.array(covMat)) 
            
            (eigValues,eigVectors) = np.linalg.eig(covMatNpTp)
            loginfo("eigValues:")
            loginfo("\n" + str(eigValues))
            loginfo("eigVectors:")
            loginfo("\n" + str(eigVectors))

            quaternion = self._getRotation(eigVectors)
            # loginfo(quaternion)

            scaledEigenValues = self._getScaledEigenvalues(eigValues,0.05,0.25)

            ellipsoid = self._createEllipsoidMsg([0,0,0],quaternion,scaledEigenValues)

            self._publisher.publish(ellipsoid)
            loginfo(ellipsoid)

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

    def _getRotation(self,eigVectors):

        # from internet:
        # eigx_n=PyKDL.Vector(eigVectors[0,0],eigVectors[0,1],eigVectors[0,2])
        # eigy_n=-PyKDL.Vector(eigVectors[1,0],eigVectors[1,1],eigVectors[1,2]) # WAAROM DEZE MIN
        # eigz_n=PyKDL.Vector(eigVectors[2,0],eigVectors[2,1],eigVectors[2,2])
        # eigx_n.Normalize()
        # eigy_n.Normalize()
        # eigz_n.Normalize()
        # rot = PyKDL.Rotation (eigx_n,eigy_n,eigz_n)

        # anders
        # eigx_n=PyKDL.Vector(eigVectors[0,0],eigVectors[1,0],eigVectors[2,0])
        # eigy_n=PyKDL.Vector(eigVectors[0,1],eigVectors[1,1],eigVectors[2,1])
        # eigz_n=PyKDL.Vector(eigVectors[0,2],eigVectors[1,2],eigVectors[2,2])

        rot = PyKDL.Rotation (eigVectors[0,0],eigVectors[0,1],eigVectors[0,2],
                            eigVectors[1,0],eigVectors[1,1],eigVectors[1,2],
                            eigVectors[2,0],eigVectors[2,1],eigVectors[2,2] )
        # rot = PyKDL.Rotation (eigVectors[0,0],eigVectors[0,2],eigVectors[0,1],
        #                     eigVectors[1,0],eigVectors[1,2],eigVectors[1,1],
        #                     eigVectors[2,0],eigVectors[2,2],eigVectors[2,1] )
        # rot = PyKDL.Rotation (eigVectors[0,0],eigVectors[1,0],eigVectors[2,0],
        #             eigVectors[0,1],eigVectors[1,1],eigVectors[2,1],
        #             eigVectors[0,2],eigVectors[1,2],eigVectors[2,2] )


        
        quat = rot.GetQuaternion()
        # loginfo("quaternion; \n"+ str(quat))
        # loginfo(eigVectors[0,1])

        return quat

    def _getScaledEigenvalues(self,eigenValues,lambdaMin,lambdaMax):

        lambdaVec = np.zeros(3)
        for i,eig in enumerate(eigenValues):
            if np.abs(eig) < 10**(-6):
                eig = 0

            lambda_ = np.sqrt(eig)
            loginfo(lambda_)

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
        loginfo("quaternion; \n"+ str(quaternions))
        # marker.pose.orientation.x = -0.147#quaternions[0]
        # marker.pose.orientation.y = -0.354#quaternions[1]
        # marker.pose.orientation.z = 0.354#quaternions[2]
        # marker.pose.orientation.w = 0.854#quaternions[3] # WAAROM DEZE MIN

        marker.pose.orientation.x = quaternions[0]
        marker.pose.orientation.y = quaternions[1]
        marker.pose.orientation.z = quaternions[2]
        marker.pose.orientation.w = quaternions[3]

        marker.scale.x = 2*np.sqrt(2)*scales[0] #2times radius and sqrt(2) from the eigdecomposition
        marker.scale.y = 2*np.sqrt(2)*scales[1]
        marker.scale.z = 2*np.sqrt(2)*scales[2]

        marker.color.a = 0.3
        marker.color.r = 1
        marker.color.g = 0.3
        marker.color.b = 1
        return marker







if __name__ == "__main__":

    # init_node("draw_ellipsoid", anonymous=False)

    try:

        node = DrawEllipsoid()
        node.run()

    except ROSInterruptException:
        pass