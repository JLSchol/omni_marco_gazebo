#!/usr/bin/env python

from rospy import Time, init_node, is_shutdown
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
    ######################### Ros Interface, message conversions etc  ###################################  
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
            #sqrt(2) = is term from the eigendecomposition?
            ellipsoid_axis_scale.append(2.0*sqrt(2.0)*lambda_)

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


    ######################### General Rotation matrix/quat operations  ###################################  
    def getQuatFromMatrix(self,matrix):
        rot = Rotation(matrix[0,0],matrix[0,1],matrix[0,2],
                        matrix[1,0],matrix[1,1],matrix[1,2],
                        matrix[2,0],matrix[2,1],matrix[2,2] )
        quat = rot.GetQuaternion()

        # normalize
        quaternions = self.normalizeList(quat)

        return quaternions

    def matrixFromQuat(self,q):
        T = quaternion_matrix(q)
        return T[0:3,0:3]

    def checkRightHandedNessMatrix(self,matrix):
        v1 = []
        v2 = []
        v3 = []
        if isinstance(matrix,list):
            v1 = np.array(matrix[:,0])
            v2 = np.array(matrix[:,1])
            v3 = np.array(matrix[:,2])
        else:
            v1 = matrix[:,0]
            v2 = matrix[:,1]
            v3 = matrix[:,2]
        rightHanded = False
        if np.dot(np.cross(v1,v2),v3)>0: #1>0
            rightHanded = True
        return rightHanded


    def normalizeList(self,quat):
        # normalize
        norm = sqrt(sum([q**2 for q in quat]))
        normalizedQuat = [array(q)/norm for q in quat]
        return normalizedQuat

    def conjugateQuat(self,quat): # same as inverse for unit quat
        qConj = list(quat)
        qConj[0] = -quat[0]
        qConj[1] = -quat[1]
        qConj[2] = -quat[2]
        return qConj

    def inverseQuat(self,quat):
        qInv = list(quat)
        qInv[3] = -quat[3]
        return qInv

    def relativeRotationQuat(self,q1,q2):
        # from q1 to q2
        q1Inv = self.inverseQuat(q1)
        qr = quaternion_multiply(q2,q1Inv)
        return qr

    def rotateQuatOverOneAxis(self,quat,axis,angle):
        # can only be used for rotation over single x OR y OR Z axis, not combined axis
        # rotates quat [x,y,z,w] over axis [1,0,0](xaxis) with angle in rad
        qRot = quaternion_from_euler(axis[0]*angle,axis[1]*angle,axis[2]*angle)
        newQuat = quaternion_multiply(quat,qRot)
        return newQuat

    def slerp(self, v0, v1, t_array):
        """Spherical linear interpolation."""
        # >>> slerp([1,0,0,0], [0,0,0,1], np.arange(0, 1, 0.001))
        t_array = np.array(t_array)
        v0 = np.array(v0)
        v1 = np.array(v1)
        dot = np.sum(v0 * v1)

        if dot < 0.0:
            v1 = -v1
            dot = -dot
        
        DOT_THRESHOLD = 0.9995
        if dot > DOT_THRESHOLD:
            result = v0[np.newaxis,:] + t_array[:,np.newaxis] * (v1 - v0)[np.newaxis,:]
            return (result.T / np.linalg.norm(result, axis=1)).T
        
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * t_array
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return (s0[:,np.newaxis] * v0[np.newaxis,:]) + (s1[:,np.newaxis] * v1[np.newaxis,:])

    def angleBetween2UnitVec(self,v1,v2):
        dotProduct = np.dot(v1,v2)
        angle = np.arccos(round(dotProduct,4))
        return angle

    def innerProductOfUnitQuaternion(self,q1,q2):
        # from http://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf (phi3)
        # almost similair as the absolute angle accourding to unity: 
        # Mathf.Acos(  Mathf.Min( Mathf.Abs(Quaternion.Dot(a, b)), 1f) ) * 2.0 * 57.2957801818848  );
        # angle4 = np.arccos(abs(np.dot(q1[0:2],q2[0:2])))
        rad = np.arccos(min(abs(np.dot(q1,q2)),1.0))
        return rad


######################### Specific function to fix orientation problem  ###################################  
    def getCharacteristicAxis(self,scalesExperiment,scalesUser):
        minValueExp = min(scalesExperiment)
        maxValueExp = max(scalesExperiment)
        minValueUser = min(scalesUser)
        maxValueUser = max(scalesUser)

        minListExp = [x for x in scalesExperiment if x == minValueExp]
        maxListExp = [x for x in scalesExperiment if x == maxValueExp]
        minListUser = [x for x in scalesUser if x == minValueUser]
        maxListUser = [x for x in scalesUser if x == maxValueUser]

        axis = []
        if len(minListExp) == 2 and len(minListUser) == 2: # werkt long
            # print("beiden Sigaar") 
            axis = 'long'
        elif len(minListExp) == 2 and len(maxListUser) == 2: # werkt niet short and long
            print("Dismiss trial incorrect orientation")
            axis = 'long'
        elif len(minListExp) == 2 and len(minListUser) == 1: # werkt long
            print("exp sigaar, user ovaal")
            axis = 'long'
        elif len(minListExp) == 2 and len(minListUser) == 3: # werkt nooit
            print("Dismiss trial incorrect orientation")
            axis = 'long'

        elif len(maxListExp) == 2 and len(maxListUser) == 2: # werkt short
            print("beden pannenkoek")
            axis = 'short'
        elif len(maxListExp) == 2 and len(minListUser) == 2: # werkt niet long and short
            print("Dismiss trial incorrect orientation")
            axis = 'long'
        elif len(maxListExp) == 2 and len(maxListUser) == 1: # werkt short
            print("exp pannenkoek, user ovaal")
            axis = 'short'
        elif len(maxListExp) == 2 and len(minListUser) == 3: # werkt nooit
            print("Dismiss trial incorrect orientation")
            axis = 'short'

        elif len(maxListExp) == 1 and len(minListUser) == 2: # werkt long
            print("exp ovaal, user Sigaar")
            axis = 'long'
        elif len(maxListExp) == 1 and len(maxListUser) == 2: # werkt short
            print("exp ovaal, user pannenkoek")
            axis = 'short'
        elif len(maxListExp) == 1 and len(minListUser) == 1: # werkt long 
            print("exp ovaal, user ovaal")
            axis = 'long'
        elif len(maxListExp) == 1 and len(minListUser) == 3: # werkt nooit
            print("Dismiss trial incorrect orientation")
            axis = 'long'

        else:
            print("no more cases possible -> otherwise error in code/logic")

        return axis


    def axisSwap(self,expScales,userEigVec,userEigVal,lambda_min,lambda_max ,axis='long'):
        # Swaps the axis of input orientation(eigVectors) by shuffeling vector,value,scales
        # along largest (long) axis or smalles (short) axis
        # get user scales:
        userScales = self.getEllipsoidScales(userEigVal, lambda_min, lambda_max)
        print("userScales: {}, expScales: {}".format(userScales,expScales))
        shuffleSeq = self.getShuffleSequence(expScales,userScales,userEigVal,userEigVec,axis)
        newUserVal, newUserVec = self.shuffleEig(userEigVal, userEigVec, shuffleSeq)
        newUserScales = self.shuffleList(userScales,shuffleSeq)
        if not self.checkRightHandedNessMatrix(newUserVec):
            logfatal("not a valid rotation, could NOT find a solution")
        swappedAxisQuat = self.getQuatFromMatrix(newUserVec)
        return swappedAxisQuat, newUserScales, newUserVec, newUserVal,   

    def closestQuaternionProjection(self,q,qTarget,scales,axis='long'):
        # get indices from long and short axis other2
        longOrShortIndex, other1Index, other2Index = self._getAxisIndicesOfEllipses(scales,axis)
        print("short or long?: {} at: {}, other1Index: {}, small_2: {}".format(axis,longOrShortIndex,other1Index,other2Index))

        # Check if axis is not pointing the opposite way (180 deg)
        qCorrected = self._flipAxis180Degrees(q,qTarget,longOrShortIndex)
        projectionOfOther1Vec, other1Vec = self._projectUnitVectorOnPlane(qCorrected,qTarget,longOrShortIndex,other1Index)
        projectionOfOther2Vec, other2Vec = self._projectUnitVectorOnPlane(qCorrected,qTarget,longOrShortIndex,other2Index)

        # get angle between axis
        angle = [0,0]
        angle[0] = self.angleBetween2UnitVec(projectionOfOther1Vec,other1Vec)
        angle[1] = self.angleBetween2UnitVec(projectionOfOther2Vec,other2Vec)

        # rotate userEllips quaternionion to experimentEllips quaternions with found projection angle
        axisOfRotation = [0,0,0]
        axisOfRotation[longOrShortIndex] = 1
        qRoted = [[0,0,0,0],[0,0,0,0]]
        qRoted[0] = self.rotateQuatOverOneAxis(qCorrected,axisOfRotation,angle[0])
        qRoted[1] = self.rotateQuatOverOneAxis(qCorrected,axisOfRotation,angle[1])

        for iterator,otherAxisIndex in enumerate([other1Index,other2Index]):
            if not self._isAxisAlligned(qRoted[iterator],qTarget,longOrShortIndex,otherAxisIndex):
                # angle[iterator] = -angle[iterator] # or other rule??
                # find new rotation with correct angle of rotaton
                qRoted[iterator] = self.rotateQuatOverOneAxis(qCorrected,axisOfRotation,-angle[iterator])

        for iterator,otherAxisIndex in enumerate([other1Index,other2Index]):
            if not self._isAxisAlligned(qRoted[iterator],qTarget,longOrShortIndex,otherAxisIndex):
                print("nog steeds niet alligned probeer wat anders")

        # find angle in de middle of the two found rotations
        qExpNew = self.slerp(qRoted[0],qRoted[1],[0.5])[0]
        return qExpNew, qCorrected

    # renamed change check 
    def _getAxisIndicesOfEllipses(self,scales,axis='long'):
        if axis == 'long':
            largeI = scales.index(max(scales))
            _, smallI1, smallI2 = self._getRemainingIndices(largeI)
            return largeI, smallI1, smallI2
            
        elif axis == 'short':
            smallI = scales.index(min(scales))
            _, largeI1, largeI2 = self._getRemainingIndices(smallI)
            return smallI, largeI1, largeI2
        else:
            print("{} is not a valid 2nd input arg. Try: 'long', 'short' or leave empty.".format(axis))

    def _getRemainingIndices(self,index):
        if index == 0:
            return index, 1, 2
        elif index == 1:
            return index, 0, 2 
        elif index == 2:
            return index, 0, 1 
        else:
            print("Wrong input in method: ellipsoid_message.getSmallIndices()")

    def _flipAxis180Degrees(self,q,qTarget,flipAxisIndex): 
        m = self.matrixFromQuat(q)
        mTarget = self.matrixFromQuat(qTarget)
        dotProduct = np.dot(m[:,flipAxisIndex],mTarget[:,flipAxisIndex])
        if dotProduct < 0:
            # 180 rotation around other axis
            indexFlipAxis = self._getRemainingIndices(flipAxisIndex)[1]
            rotationAxis = [0,0,0]
            rotationAxis[indexFlipAxis] = 1
            q = self.rotateQuatOverOneAxis(q,rotationAxis,np.pi)
        return q

    def _projectUnitVectorOnPlane(self,q,qTarget,normalOfPlaneAxisIndex,toBeProjectedAxisIndex):
        m = self.matrixFromQuat(q)
        mTarget = self.matrixFromQuat(qTarget)

        normalOfPlane = np.array(mTarget[:,normalOfPlaneAxisIndex])
        V = np.array(m[:,toBeProjectedAxisIndex])

        VProjected = V - (np.dot(V,normalOfPlane))*normalOfPlane
        VProjectedUnit = self.normalizeList(VProjected)

        return VProjectedUnit, mTarget[:,toBeProjectedAxisIndex]

    def _isAxisAlligned(self,q,qTarget,normalOfPlaneAxisIndex,toBeProjectedAxisIndex):
        VProjectedUnit, VTarget = self._projectUnitVectorOnPlane(q,qTarget,normalOfPlaneAxisIndex,toBeProjectedAxisIndex)
        dotProduct = np.dot(VProjectedUnit,VTarget)
        threshold = 0.93
        if round(dotProduct,2) < threshold:
            print("projection is not alligned as: dotProduct={} < threshold={}".format(round(dotProduct,2),threshold))
            return False
        else:
            return True

    # remove some print statements, eigValues is not Used except that it needs to be an input for another function
    # Also, adjust formula to rotate also over the shortest axis
    def getShuffleSequence(self,scaleExp,scaleUser,eigValues,eigVectors,axis='long'):
        # function returns a sequence list with indexes that can be used to shuffle the eigValues,eigVectors positions
        # and thereby checks if the relocated eigVectors is an valid rotation 

        # Star with unknown shuffleSequence
        shuffleSequence = ['x','y','z']

        # Alling the first long or short axis accouding axis='specification'
        minIndex = lambda x: x.index(min(x))
        maxIndex = lambda x: x.index(max(x))

        shuffleImaxOrMin = []
        shuffleVmaxOrMin = []
        if axis == 'long':
            shuffleImaxOrMin = maxIndex(scaleExp)
            shuffleVmaxOrMin = maxIndex(scaleUser)
        elif axis == 'short':
            shuffleImaxOrMin = minIndex(scaleExp)
            shuffleVmaxOrMin = minIndex(scaleUser)
        else: 
            print("{} is not an option. Try: 'long', 'short' or leave 5e input empty".format(axis))
            return


        # check if shuffle is necessary 
        if shuffleImaxOrMin == shuffleVmaxOrMin:
            print("specified principial axis are alligned, no shuffle needed")
            shuffleSequence = [0,1,2]
            print("shuffleSequence is {}".format(shuffleSequence))
            return shuffleSequence

        # start creating shuffle sequence based of alligning the largest or smallest scales of user and exp
        shuffleSequence[shuffleImaxOrMin] = shuffleVmaxOrMin  # e.g.) ['x',maxOrMinValue,'z']

        # set a random guess voor one of the oterh two in shuffleSequence ['x',maxOrMinValue,'z']
        shuffleIRandom = []
        shuffleVRandom = []
        if shuffleImaxOrMin != 0:   
            shuffleIRandom = 0 
            if shuffleVmaxOrMin != 0:
                shuffleVRandom = 0
            else:
                shuffleVRandom = 1 # or 2
        else:
            shuffleIRandom = 1 # or 2
            if shuffleVmaxOrMin != 0:
                shuffleVRandom = 0
            else:
                shuffleVRandom = 1 # or 2
        shuffleSequence[shuffleIRandom] = shuffleVRandom # ['x',maxOrMinValue,randomValue] (on random index)

        # get remaining index value for shuffleSequence ['x',maxOrMinValue,randomValue]
        shuffleIRemain = 3 - shuffleImaxOrMin - shuffleIRandom
        shuffleVRemain = 3 - shuffleVmaxOrMin - shuffleVRandom
        shuffleSequence[shuffleIRemain] = shuffleVRemain # [remainginValue,maxOrMinValue,randomValue] (on remaining index)
        print("shuffleSequence is {}".format(shuffleSequence))

        # Check if found sequence provides an actual rotation matrix, otherwise swap 
        shuffledValues, shuffledVectors = self.shuffleEig(eigValues, eigVectors, shuffleSequence)
        validRotation = self.checkRightHandedNessMatrix(shuffledVectors)
        print("validRotation? {}".format(validRotation))

        if not validRotation:
            # swap the guessed and remaining index value pair in the shuffle sequence to achieve a valid rotations
            shuffleSequence[shuffleIRemain] = shuffleVRandom
            shuffleSequence[shuffleIRandom] = shuffleVRemain
            print("new shuffleSequence is {}".format(shuffleSequence))

        return shuffleSequence

    # rename shuffleEigVectorValues, Split function in matrix and vector
    # rewrite function
    def shuffleEig(self,eigenValues,eigenVectors,sequence):
        # Changes the eigValues/ eigVectors pairs its index positions from a sequence list
        # sequence =[2,0,1]; eigenValues [1, 2, 3]; eigenVectors[V1, V2, V3]
        # returns [3, 1, 2], [V3, V1, V2] as an np array
        newValues = array(self.shuffleList(eigenValues,sequence))
        newVectors = array(self.shuffle2DList(eigenVectors,sequence,'col'))
        return newValues, newVectors

    def shuffleList(self,lijst,sequence):
        lijst = np.array(lijst)
        newList = lijst[sequence]
        return newList.tolist()

    def shuffle2DList(self,matrix,sequence,axis='col'):
        npMatrix = np.array(matrix)
        if axis == 'col':
            npMatrix = npMatrix[:,sequence]
        elif axis == 'row':
            npMatrix = npMatrix[sequence,:]
        else:
            print("{} is not a valid input try: 'col' or 'row' ".format(axis))
            return
        return npMatrix.tolist()


    # rename absoluteAngleBetweenEllipsoids, test function (is angle always positive?)
    def absoluteAngleBetweenEllipsoids(self,q1,q2,deg='deg'):
        # goal: find distance metric to compare ellipsoids
        # 2 distance metrics for rotations(see  phi2 and 3) from http://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf
        
        # 1) not used because lack of physical interpretation and how to correct for symmetry ellipsoid
        # phi2 = self.normOfDifferences(q1,q2)
        # print("distance: {} [-]".format(phi2))

        # 2) use innerproduct of unit quaternion as metric
        # AND scale range times 2 to give it a physical meaningfull angle (the absolute angle between rotations)
        # AND correct for symmetry of the ellipsoid
        innerProd = self.innerProductOfUnitQuaternion(q1,q2) # phi 3
        absoluteAngle = 2*innerProd
        if absoluteAngle > np.pi/2: # summetry correction
            absoluteAngle = np.pi - absoluteAngle      # 89 degrees = 180 - 91
        # print("new angle innerprod: {} rad, {} deg".format(absoluteAngle,np.degrees(absoluteAngle)))

        # 3) or use own derived method with same result -.-'
        # qr = self.relativeRotationQuat(q1,q2)
        # angle2 = 2*np.arccos(qr[3])
        # newAngle = self.transformAngle(angle2)
        # print("new angle: {} rad, {} deg".format(newAngle,np.degrees(newAngle)))

        if deg == 'deg':
            # newAngle = np.degrees(newAngle)
            absoluteAngle = np.degrees(absoluteAngle)
        elif deg=='rad':
            pass
        else:
            print("{} is not a valid option. Use 'deg' or 'rad'. \nOutput is given in rad".format(deg))

        # return newAngle
        return absoluteAngle

    def volumeEllipsoid(self,scales):
        # scales are diameter
        # V = 4*pi/3 * r1*r2*r3
        return (4.0*np.pi/3.0 * (0.5**3)*scales[0]*scales[1]*scales[2])

    def distanceEllipsoid(self,scales):
        principleAxisLength = [0.5*scale for scale in scales]
        distance = np.linalg.norm(principleAxisLength)
        return distance

#################### Outdated/ not used/ not effective methods relating simple_experiment.py  #################### 
    def rotateTillClosest(self,q,qTarget,axis):
        # method did not work for orientation problem
        angle = np.deg2rad(1)
        oneDegRotationQuat = quaternion_from_euler(axis[0]*angle,axis[1]*angle,axis[2]*angle)
        prevAngle = self.absoluteAngleBetweenEllipsoids(qTarget,q,'deg')
        counter = 0
        smallestAngle = 360 # initialize large
        smallestQuat = []
        while counter<=360:
            counter+=1
            q = quaternion_multiply(q,oneDegRotationQuat)
            angle = self.absoluteAngleBetweenEllipsoids(qTarget,q,'deg')

            if abs(angle)<abs(smallestAngle):
                smallestAngle = angle
                smallestQuat = list(q)
            print("counter: {};    angle: {};    smallest Angle: {};    smallestQuat {}".format(counter,angle,smallestAngle,smallestQuat))
            prevAngle = angle

        return smallestQuat, smallestAngle

    def zeroQuaternion(self, quat, boolList):
        # zero the quaternion using the boolList
        newQuat = list(quat)
        for index,booleaan in enumerate(boolList):
            if booleaan==True:
                newQuat[index] = 0
        return self.normalizeList(newQuat)

    def closestQuaternion(self,q,qTarget,expScales):
        # method did not work for orientation problem
        qr = self.relativeRotationQuat(q,qTarget)
        print("qr1: {}".format(qr))
        maxIndex = expScales.index(max(expScales))
        zeroList = [True,True,True,False] # -> [0,0,0,w]
        zeroList[maxIndex] = False # -> [0,0,z,w]
        print(zeroList)
        qr = self.zeroQuaternion(qr,zeroList) # -> [0,0,z,w]
        print("qr2: {}".format(qr))
        # corrected = quaternion_multiply(correction,q)
        qexp = quaternion_multiply(q,qr)
        # qexp = quaternion_multiply(q,self.inverseQuat(qr))
        return qexp

    def normOfDifferences(self,q1,q2):
        # from http://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf (phi2)
        # dimensionless metric for distance between two rotations
        distance = min(np.linalg.norm(q1-q2),np.linalg.norm(q1+q2))
        return distance

    def transformAngle(self,radian):
        # not used anymore replaced by other functuibn
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

if __name__ == "__main__":  


    ########### fix rotatoin #############
    # newUserQuat = [0.056808103171724085, -0.09129308613382892, 0.9696100483698741, 0.2197607015331963]
    # expQuat = [0,0,0,1]



    EM = EllipsoidMessage()

    # Complete fucking process Make function for axisSwap part
    # exp
    qExpBaseRot = EM.normalizeList([ 0.5, 0.5, 0.5, -0.5 ])
    qExp = EM.rotateQuatOverOneAxis(quaternion_multiply(qExpBaseRot,[ 0.0868241, 0.0868241, 0.0075961, 0.9924039 ]),[0,0,1],0*(np.pi/4)) # in base
    # # qExp =  [ 0.5773503, 0, -0.5773503, 0.5773503 ]# in base
    expEigVec = EM.matrixFromQuat(qExp)
    expScales = [0.4,0.4,0.09]         # shortest is z-axis
    # user
    userQuat = [0,0,0,1]     # 
    userScales = [0.29,0.095,0.31]         # longest axis is x-axis
    userEigVec = EM.matrixFromQuat(userQuat)
    userEigVal = [(userScales[0]/(2.0*sqrt(2.0)))**2.0, (userScales[1]/(2.0*sqrt(2.0)))**2.0, (userScales[2]/(2.0*sqrt(2.0)))**2.0] # dont really care about this much
    print("userScales: {}, expScales: {}".format(userScales,expScales))
    # print("userEigVec: {}\nexpEigVec: {}".format(userEigVec,expEigVec))
    print(userEigVal)
    
    axis = 'short'

    swappedAxisQuat, newScales, _, _ = EM.axisSwap(expScales,userEigVec,userEigVal,0.03,0.20,axis)

    print("newScales {}".format(newScales))

    # flip axis, project and rotate user Quat
    qExpNew, qAlignedAndCorrected = EM.closestQuaternionProjection(swappedAxisQuat,qExp,newScales,axis)
    angle = EM.absoluteAngleBetweenEllipsoids(qExp,qExpNew,'deg')
    # x = array([1,2,3])
    # xx = array([[1,2,3],[4,5,6],[7,8,9]])
    # se = [2,0,1]
    
    # EM.shuffleList(x,se)
    # EM.shuffle2DList(xx,se,'col')
    # EM.shuffleMatrix(xx,se,'col')

    # print(type(EM.shuffleEig(x,xx,se)[1]))
    # print(EM.shuffleEig(x,xx,se)[1])
    



    frame_id='base_footprint' #wrist_ft_tool_link
    marker_ns='test'
    marker_id=1 
    positions=[0,0,0]
    positions1=[0.6,0,0]
    positions2=[1.2,0,0]
    positions3=[0.9,0,0.3]
    positions4=[0.9,0,0.6]
    positions5=[1.8,0,0]
    positions6=[2.4,0,0]
    positions7=[3,0,0]
    rgba=[0.95,0.95,0.5,0]

    ##############################################
    ##############################################
    init_node("test_ellipses",anonymous=False)
    # q1Aligned = [ 0.5, 0.5, 0.5, 0.5 ] # in base
    # # qExp = EM.rotateQuatOverOneAxis([ 0, -0.5773503, 0.5773503, 0.5773503 ],[0,0,1],8*(np.pi/4)) # in base
    # qExp = EM.rotateQuatOverOneAxis(quaternion_multiply(q1Aligned,[ 0.9645804, -0.2564888, 0.0533718, 0.0308142 ]),[0,0,1],8*(np.pi/4)) # in base
    # # qExp =  [ 0.5773503, 0, -0.5773503, 0.5773503 ]# in base
    # qExpNew, qAlignedAndCorrected = EM.closestQuaternionProjection(q1Aligned,qExp,expScales)
    ##############################################
    ##############################################

    

    while not is_shutdown(): 


        # q2 = [0.7071067811865475,0,0,0.7071067811865475]
        # # q2 = [ 0, 0, -0.4871745, -0.8733046 ]
        # # q1 = [0,0,0.7071067811865475,-0.7071067811865475]
        # q1 = [0.5,0.5,0.5,0.5]
        # qr = [0,0,0.7071067811865475,0.7071067811865475]


        # q3 = quaternion_multiply(qr,q2)
        EM.broadcastEllipsoidAxis(positions1,userQuat,frame_id,"userQuat")
        EM.broadcastEllipsoidAxis(positions2,swappedAxisQuat,frame_id,"q1Aligned")
        # EM.broadcastEllipsoidAxis(qAlignedAndCorrected,qRel,frame_id,"qRel")
        # EM.broadcastEllipsoidAxis(positions4,q0Rel,frame_id,"q0Rel")
        EM.broadcastEllipsoidAxis(positions5,qAlignedAndCorrected,frame_id,"qAlignedAndCorrected")
        EM.broadcastEllipsoidAxis(positions6,qExpNew,frame_id,"qExpNew")
        EM.broadcastEllipsoidAxis(positions7,qExp,frame_id,"qExp")
    # axis = [0,0,1] #-> z-axis
    # angle = np.pi/2 # rad

    # EM.rotateQuatOverOneAxis(q1,axis,angle)
    # print(q2)
    # qr = EM.relativeRotationQuat(q1,q2)
    # print("qr1: {}".format(qr))
    # qr = EM.zeroQuaternion(qr,[True,True,False,False]) #-> only rotation around Z axis!
    # print("qr2: {}".format(qr))
    # qNew = quaternion_multiply(qr,q1)
    # print(qNew)

    # correctedQuat = EM.closestQuaternion(q1,q2,expScales)

    # print("initial user Quat: {}".format(q1))
    # print("Experiment Quat: {}".format(q2))
    # print("New user quat: {}".format(correctedQuat))

    # print("check new user quat: {}".format(quaternion_multiply(qr,q1)))

    

