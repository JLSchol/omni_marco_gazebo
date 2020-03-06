#!/usr/bin/env python

from rospy import Time
from tf2_ros import TransformBroadcaster
#import messages
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from visualization_msgs.msg import Marker
#import custom messages
from stiffness_commanding.msg import EigenPairs, VectorValue

from PyKDL import Rotation
from tf.transformations import *

import numpy as np
from numpy import array, transpose, sqrt, zeros

class EllipsoidMessage(object):
    def __init__(self):
        pass

    def checkRightHandedNessMatrix(self,matrix):
        # print(matrix)
        # print(type(matrix))
        v1 = np.array(matrix[:,0])
        v2 = np.array(matrix[:,1])
        v3 = np.array(matrix[:,2])
        # loginfo(np.dot(np.cross(v1,v2),v3))
        rightHanded = False
        if np.dot(np.cross(v1,v2),v3)>0: #1>0
            rightHanded = True
            
        # loginfo(rightHanded)
        return rightHanded


    def shuffleEig(self,eigenValues,eigenVectors,sequence):
        # Shuffel 2 arbritary vectors will always result in a rotation instead of an reflection
        # sequence = [0,2,1] # in stead of [0,1,2]
        # sequence=np.argsort(sequence)
        # print("sequence {}".format(sequence))
        # print("eigenValues {}".format(eigenValues))
        eigenValues = np.array(eigenValues)
        eigenVectors = np.array(eigenVectors)

        eigenValues = eigenValues[sequence]
        # print("newEigenVaues {}".format(eigenValues))
        eigenVectors = eigenVectors[:,sequence]


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

    def relativeRotationQuat(self,q1,q2):
        q1Inv = q1
        q1Inv[3] = -q1Inv[3]
        qr = quaternion_multiply(q2,q1Inv)
        return qr

    def innerProductOfUnitQuaternion(self,q1,q2):
        # angle4 = np.arccos(abs(np.dot(q1[0:2],q2[0:2])))
        rad = np.arccos(min(abs(np.dot(q1,q2)),1.0))
        return rad

    def normOfDifferences(self,q1,q2):
        # dimensionless
        distance = min(np.linalg.norm(q1-q2),np.linalg.norm(q1+q2))
        return distance


    def absoluteAngle(self,q1,q2,deg='deg'):
        qr = self.relativeRotationQuat(q1,q2)
        # angle = 2*np.arctan2(np.linalg.norm(qr[0:2]),qr[3])
        angle2 = 2*np.arccos(qr[3])
        # angle3 = 2*np.arcsin(np.linalg.norm(qr[0:2]))
        # dist = self.normOfDifferences(q1,q2)
        innerProd = self.innerProductOfUnitQuaternion(q1,q2)
        # innerProdx2 = innerProd*2
        absoluteAngle = abs(np.pi - (2*innerProd))

        #  shortest shortest angle
        # Mathf.Acos(  Mathf.Min( Mathf.Abs(Quaternion.Dot(a, b)), 1f) ) * 2.0 * 57.2957801818848  );
        # print("initial angle: {} rad, {} deg".format(angle,np.degrees(angle)))
        print("initial angle2: {} rad, {} deg".format(angle2,np.degrees(angle2)))
        # print("initial angle3: {} rad, {} deg".format(angle3,np.degrees(angle3)))
        # distance metrics phi1 and phi2 from http://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf
        # print("distance: {} [-]".format(dist))
        # print("2Xinnerproduct: {} rad, {} deg".format(innerProdx2,np.degrees(innerProdx2)))
        print("new angle innerprod: {} rad, {} deg".format(absoluteAngle,np.degrees(absoluteAngle)))


        newAngle = self.transformAngle(angle2)
        print("new angle: {} rad, {} deg".format(newAngle,np.degrees(newAngle)))

        if deg == 'deg':
            newAngle = np.degrees(newAngle)
            absoluteAngle = np.degrees(absoluteAngle)
        elif deg=='rad':
            pass
        else:
            print("{} is not a valid option. Use 'deg' or 'rad'. \nOutput is given in rad".format(deg))

        # return newAngle
        return absoluteAngle

    def transformAngle(self,radian):

        quarter = 1*np.pi/2
        half = 2*np.pi/2
        threeQuarter = 3*np.pi/2
        whole = 4*np.pi/2

        correction = []
        sign = -1 if radian > 0 else 1
        absRad = abs(radian)
        # print("absrad: {}".format(absRad))

        if absRad >= 0 and absRad <= quarter:
            correction = 0
        elif absRad>quarter and absRad<=threeQuarter:
            correction = half
        elif absRad>=threeQuarter and absRad<=whole:
            correction = whole
        else:
            print("abs({}) is larger than 2 PI".format(radian))

        # print("sign: {}, correction: {}".format(sign,correction))
        newRad = radian + sign*correction
        # print("new rad: {}".format(newRad))
        return newRad

    def getShuffleSequence(self,scaleExp,scaleUser,eigValues,eigVectors):
        # print("desired: {}".format([0,2,1]))
        shuffleSequence = ['x','y','z']
        print("Values: {}".format(eigValues))
        print("Vectors:\n {}\n {}\n {}".format(eigVectors[0][:],eigVectors[1][:],eigVectors[2][:]))
        # try to find the suffle sequence
        maxIndex = lambda x: x.index(max(x))
        shuffleImax = maxIndex(scaleExp)
        shuffleVmax = maxIndex(scaleUser)

        # check if shuffle is necessary 
        if shuffleImax == shuffleVmax:
            print("greatest principial axis is alligned, no shuffle needed")
            shuffleSequence = [0,1,2]
            print(shuffleSequence)
            return [0,1,2]

        # start creating shuffle sequence
        shuffleSequence[shuffleImax] = shuffleVmax # ['x',maxValue,'z']
        print(shuffleSequence)

        # get random guess voor shuffleSequence ['x',maxValue,'z']
        shuffleIRandom = []
        shuffleVRandom = []
        if shuffleImax != 0:
            shuffleIRandom = 0 
            if shuffleVmax != 0:
                shuffleVRandom = 0
            else:
                shuffleVRandom = 1 # or 2
        else:
            shuffleIRandom = 1 # or 2
            if shuffleVmax != 0:
                shuffleVRandom = 0
            else:
                shuffleVRandom = 1 # or 2
        shuffleSequence[shuffleIRandom] = shuffleVRandom # ['x',maxValue,randomValue] (on random index)
        print(shuffleSequence)

        # get remaining index value for shuffleSequence ['x',maxValue,randomValue]
        shuffleIRemain = 3 - shuffleImax - shuffleIRandom
        shuffleVRemain = 3 - shuffleVmax - shuffleVRandom
        shuffleSequence[shuffleIRemain] = shuffleVRemain # [remainginValue,maxValue,randomValue] (on remaining index)
        print(shuffleSequence)

        # Check if found sequence provides an actual rotation, otherwise swap -> should give rotation!
        (shuffledValues, shuffledVectors) = self.shuffleEig(eigValues, eigVectors, shuffleSequence)
        validRotation = self.checkRightHandedNessMatrix(shuffledVectors)
        print("validRotation? {}".format(validRotation))

        if not validRotation:
            # swap the guessed and remaining index value pair in the shuffle sequence
            shuffleSequence[shuffleIRemain] = shuffleVRandom
            shuffleSequence[shuffleIRandom] = shuffleVRemain
            print(shuffleSequence)

        return shuffleSequence


    def broadcastEllipsoidAxis(self,position,quaternion,parentID,childID):
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = Time.now()
        t.header.frame_id = parentID
        t.child_frame_id = childID
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        br.sendTransform(t)

if __name__ == "__main__":

    ############ shuffle testing ############
    scaleUser = [0.1,0.1,0.5]   # sequence: [1,2,0] or [0,2,1]
    # scaleUser = [0.1,0.5,0.1] # sequence: [0,1,2] or [2,1,0]
    # scaleUser = [0.5,0.1,0.1] # sequence: [2,0,1] or [1,0,2]

    # scaleUser = [0.1,0.1,0.5] # should be [0.1 0.5 0.1]
    scaleExp = [0.08,0.4,0.08]

    Vec = [ [0,0,1],
            [1,0,0],
            [0,1,0]]
    print(Vec[0][2])
    Val = [1,1,5]

    # find index sequence that maps user to experiment
    EM = EllipsoidMessage()
    sequence = EM._getShuffleSequence(scaleExp,scaleUser,Val,Vec)
    (newValues, newVectors) = EM.shuffleEig(Val, Vec, sequence)
    validRotation = EM.checkRightHandedNessMatrix(newVectors)
    print(validRotation)
    print(newValues)
    print(newVectors)
    

    # desiredSequence = [0,2,1] # of [1,2,0]
    # print(scaleUser[desiredSequence]) # [0.1 0.5 0.1]

    ############ absoluteAngle ############
    # q1 = quaternion_from_euler(np.pi/4,0,0)
    # q2 = quaternion_from_euler(np.pi/2,0,0)
    # print(q1)
    # print(q2)
    # EM = EllipsoidMessage()
    # qr = EM.absoluteAngle(q1,q2,"rad")
    # print(EM.transformAngle(qr))
