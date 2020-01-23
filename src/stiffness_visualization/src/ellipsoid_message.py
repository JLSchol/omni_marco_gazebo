#!/usr/bin/env python

from rospy import Time

#import messages
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from visualization_msgs.msg import Marker
#import custom messages
from stiffness_commanding.msg import EigenPairs, VectorValue

from PyKDL import Rotation

import numpy as np
from numpy import array, transpose, sqrt, zeros

class EllipsoidMessage(object):
    def __init__(self):
        pass

    def checkRightHandedNessMatrix(self,matrix):
        v1 = np.array(matrix[:,0])
        v2 = np.array(matrix[:,1])
        v3 = np.array(matrix[:,2])
        # loginfo(np.dot(np.cross(v1,v2),v3))
        rightHanded = False
        if np.dot(np.cross(v1,v2),v3)>0: #1>0
            rightHanded = True
            
        # loginfo(rightHanded)
        return rightHanded


    def shuffleEig(self,flag,eigenValues,eigenVectors):
        # Shuffel 2 arbritary vectors will always result in a rotation instead of an reflection
        shuffleSequence = [0,2,1] # in stead of [0,1,2]
        i = np.argsort(shuffleSequence)
        eigenValues = eigenValues[i]
        eigenVectors = eigenVectors[:,i]

        return eigenValues, eigenVectors


    def eigDecompositionToMatrix(self, eigVectorMatrix, eigValueVector):
        originalMatrix = eigVectorMatrix* np.diag(eigValueVector) * np.transpose(eigVectorMatrix)
        return originalMatrix


    def EigenPairMsgsToMatrixVector(self,eigenPairMsg): 
        matrix = []
        vector = []
        for pair in eigenPairMsg.pairs:
            matrix.append(pair.eigen_vector)
            vector.append(pair.eigen_value)
            
        matrix = transpose(array(matrix))
        vector = array(vector)

        return matrix, vector


    def MultiArrayMsgToMatrix(self,multiArray): # checked
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


    def getQuatFromMatrix(self,matrix):
        rot = Rotation(matrix[0,0],matrix[0,1],matrix[0,2],
                        matrix[1,0],matrix[1,1],matrix[1,2],
                        matrix[2,0],matrix[2,1],matrix[2,2] )
        quat = rot.GetQuaternion()

        # normalize
        length = sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2)
        quaternions = [array(q)/length for q in quat]

        return quaternions


    def getEllipsoidScales(self,eigenValues,lambdaMin,lambdaMax): #works

        lambdaVec = zeros(3)
        ellipsoid_axis_scale=[]

        for i,eigenValue in enumerate(eigenValues):

            # round to zero when close to zero
            if abs(eigenValue) < 10**(-6):
                eigenValue = 0

            # eigenvalues are variance need standard deviation
            lambda_ = sqrt(eigenValue)

            # saturate lambda if below or above wiggle threshold
            if lambda_ <= lambdaMin:
                lambda_ = lambdaMin
            elif lambda_>= lambdaMax:
                lambda_ = lambdaMax
            
            # scale the axis of the ellipsoid
            #2 = diameter is two time the radius
            #sqrt(2) = is term from the eigendecomposition
            ellipsoid_axis_scale.append(2*sqrt(2)*lambda_)

        return ellipsoid_axis_scale
        
    
    def getEllipsoidMsg(self,frame_id,marker_ns,marker_id,positions,quaternions,scales,rgba):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = Time.now()
        marker.ns = marker_ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = positions[0]
        marker.pose.position.y = positions[1]
        marker.pose.position.z = positions[2]

        marker.pose.orientation.x = quaternions[0]
        marker.pose.orientation.y = quaternions[1]
        marker.pose.orientation.z = quaternions[2]
        marker.pose.orientation.w = quaternions[3]

        marker.scale.x = scales[0] 
        marker.scale.y = scales[1]
        marker.scale.z = scales[2]         

        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]

        return marker


