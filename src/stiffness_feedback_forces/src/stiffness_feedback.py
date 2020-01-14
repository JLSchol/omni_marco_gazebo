#!/usr/bin/env python
from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, wait_for_message
from rospy import ROSInterruptException, Time, Duration, sleep, get_param, Header, has_param, logfatal
from tf2_ros import TransformListener, Buffer, LookupException
from tf2_ros import ConnectivityException, ExtrapolationException
#import messages
# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Vector3, TransformStamped, Quaternion, Vector3Stamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
from phantom_omni.msg import OmniFeedback
from phantom_omni.msg import LockState

import numpy as np


class CalcHDFeedbackForce(object):
    def __init__(self,impedanceStiffnessLimits,maxForceLimitsHD
                    ,maxWorkrangeHD):
        # This class assumes max force/workrange to be equal allong the x,y,z axis
        # only assumes max stiffness allong the diagonal terms
        self._robotStiffnessLimits = impedanceStiffnessLimits #[min, max]
        self._maxForceLimitsHD = maxForceLimitsHD #[-N, +N]
        self._maxWorkRangeHD = maxWorkrangeHD #[-m, +m]

        # all with respect to the end effector frame\
        self._stiffnessRobot = np.array([])
        self._markerPosition = np.array([])
        self._forcesRobot = np.array([])

        self._currentPositionHD = np.array([])
        self._lockPositionHD = np.array([])
        self._stiffnessHD = np.array([])
        self._forcesHD = np.array([])

    def calcRobotForce(self):
        if self._isEmpty([self._stiffnessRobot,self._markerPosition]):
            self._forcesRobot = np.array([])
            return
        self._forcesRobot = self._stiffnessRobot.dot(self._markerPosition)
        
        
    def calcHDForce(self):
        if self._isEmpty([self._stiffnessRobot,self._currentPositionHD,
                                self._lockPositionHD]):
            self._forcesHD = np.array([]) # set forces to be empty
            return
        self._stiffnessHD = self._calcHDStiffness(self._stiffnessRobot)
        self._forcesHD = self._stiffnessHD.dot(self._currentPositionHD) 
        # limit force when above commanding range

    def setCurrentRobotStiffness(self,stiffnessRobot):
        self._stiffnessRobot = np.array(stiffnessRobot)

    def setVirtualMarkerPosition(self,virtualMarker):
        self._markerPosition = np.array(virtualMarker)

    def setCurrentHDPosition(self,currentPositionHD):
        self._currentPositionHD = np.array(currentPositionHD)

    def setLockPositionHD(self,lockPositionHD):
        self._lockPositionHD = np.array(lockPositionHD)

    def getHDStiffness(self):
        return (np.ndarray.tolist(self._stiffnessHD))

    def getHDForces(self):
        return (self._forcesHD[0],self._forcesHD[1],self._forcesHD[2])

    def getRobotForces(self):
        return (self._forcesRobot[0],self._forcesRobot[1],self._forcesRobot[2])

    def _calcHDStiffness(self,stiffnessRobot):
        # stiffnessMaxHD assumes now, that the lockstate is in the center(0,0,0) position of the HD
        stiffnessMaxHD = self._maxForceLimitsHD[1]/self._maxWorkRangeHD[1] 
        scaling = stiffnessMaxHD/self._robotStiffnessLimits[1]
        stiffnessHD = scaling*stiffnessRobot

        return np.array(stiffnessHD)

    def _isEmpty(self, numpyArrayList):
        for numpyArray in numpyArrayList:
            if numpyArray.size == 0:
                return True
            else:
                pass
        return False

        


class OmniFeedbackROS(object):
    def __init__(self):
        init_node("omni_feedback_force", anonymous=False)
        self._getParameters()
        #pub
        self._forceRobotPub = Publisher("virtual_robot_force", Vector3Stamped , queue_size=100)
        self._stiffnessHDPub = Publisher("omni_stiffness", Float32MultiArray , queue_size=100)
        self._forceHDPub = Publisher(self._outputTopic, OmniFeedback , queue_size=100)
        #sub
        # wait_for_message(self._stiffnessInput, Float32MultiArray, timeout=5)
        # wait_for_message(self._omniPositionInput, LockState, timeout=5)
        self._stiffnessSub = Subscriber(self._stiffnessInput, Float32MultiArray, self._stiffnessCallback)
        self._positionSub = Subscriber(self._omniPositionInput, LockState, self._omniPositionCallback)
        self._stiffnessMessage = [] 
        self._omniPositionMessage = []
        #tf
        self._tfBuffer = Buffer()
        listener = TransformListener(self._tfBuffer)

    def _getParameters(self):
        self._stiffnessInput = get_param("~RobotStiffness")
        self._omniPositionInput = get_param("~OmniPositionInput")
        self._outputTopic = get_param("~OutputTopic")
        self._HDFrame = get_param("~HDFrameName")
        self._EEFrame = get_param("~EEFrameName")
        self._virtualMarkerFrame = get_param("~virtualMarkerFrame")
        self._HDWorkRange = get_param("~HDWorkRange")
        self._HDMaxForce = get_param("~HDMaxForce")

        lambdaminPar = 'stiffness_learning/lambda_min'
        lambdamaxPar = 'stiffness_learning/lambda_max'
        if not has_param(lambdaminPar):
            logfatal("Could not retrive %s from the parameter server", lambdaminPar)
        if not has_param(lambdamaxPar):
            logfatal("Could not retrive %s from the parameter server", lambdamaxPar)

        self._stiffnessMin = get_param("/stiffness_learning/stiffness_min") 
        self._stiffnessMax = get_param("/stiffness_learning/stiffness_max")

    def _stiffnessCallback(self, message):
        self._stiffnessMessage = message

    def _omniPositionCallback(self, message):
        self._omniPositionMessage = message    


    def run(self):
        publishRate = 30 # Hz
        rosRate = Rate(publishRate)

        # initialize stiffness feedback            class([stiffness][force][workrange])
        CalcOmniFeedbackForce = CalcHDFeedbackForce([self._stiffnessMin,self._stiffnessMax],
                                                        [-self._HDMaxForce,self._HDMaxForce],
                                                        [-self._HDWorkRange,self._HDWorkRange]) 
        # 1) get info and check
        # 2) rotate omni vector to ee frame 
        # 3) find force from robot stiffness matrix in ee frame
        # 4) transform force in ee frame back to omni frame
        # 5) publish messages
        while not is_shutdown():
            # 1) get info and check
            if not (self._stiffnessMessage and self._omniPositionMessage):  
                # loginfo(self._stiffnessMessage)
                # loginfo(self._omniPositionMessage)
                continue
            # Get/Set stiffness matrix
            stiffnessMatrix = self._convertArrayToMatrix(self._stiffnessMessage)
            CalcOmniFeedbackForce.setCurrentRobotStiffness(stiffnessMatrix)
            # find transform (rotation) from wrist to omni and transform the forces 
            EndEffectorinHDFrame = self.listenToTransform(self._EEFrame,self._HDFrame) 
            # find vector of virtual_marker in endeffector frame and set in class
            markerInEEFrame = self.listenToTransform("virtual_marker",self._EEFrame) 
            trans = markerInEEFrame.transform.translation
            CalcOmniFeedbackForce.setVirtualMarkerPosition([trans.x, trans.y, trans.z])

            # 2) rotate omni vector to ee frame 
            currentPositionInOmni = self.rotateVector(EndEffectorinHDFrame.transform.rotation,
                                                    self._omniPositionMessage.current_position)
            lockPositionInOmni = self.rotateVector(EndEffectorinHDFrame.transform.rotation,
                                                        self._omniPositionMessage.lock_position)

            # 3) find force from stiffness matrix in ee frame (scaled down to HD capabilities)
            # Get/Set omni lock and current position
            CalcOmniFeedbackForce.setCurrentHDPosition([currentPositionInOmni.x, 
                                                        currentPositionInOmni.y,
                                                        currentPositionInOmni.z])
            CalcOmniFeedbackForce.setLockPositionHD([lockPositionInOmni.x, 
                                                        lockPositionInOmni.y, 
                                                        lockPositionInOmni.z])
           
            # get the virtual robot forces
            CalcOmniFeedbackForce.calcRobotForce()
            robotForces = CalcOmniFeedbackForce.getRobotForces()

            # get the omni stiffness matrix and feedbackForces
            CalcOmniFeedbackForce.calcHDForce()
            HDStiffness= CalcOmniFeedbackForce.getHDStiffness()
            forceInEEFrame= CalcOmniFeedbackForce.getHDForces()

            # 4) transform force in ee frame back to omni frame
            q = EndEffectorinHDFrame.transform.rotation
            q.w = -q.w #inverse the rotation
            forceInEEFrame = Vector3(forceInEEFrame[0],forceInEEFrame[1],forceInEEFrame[2])
            forceInOmniFrame = self.rotateVector(q,forceInEEFrame)

            # 5) publish messages
            forceRobotMsg = self._setRobotForceMessage(robotForces)
            stiffnessHDMsg = self._set2DMultiArray(HDStiffness, 3, 3)
            forceHDMsg = self._setOmniFeedbackMessage(forceInOmniFrame,
                                                    self._omniPositionMessage.lock_position)
                                                    
            # self._forceRobotPub.publish(forceRobotMsg) # incorrect Fix!!
            self._stiffnessHDPub.publish(stiffnessHDMsg)
            self._forceHDPub.publish(forceHDMsg)

            # loginfo(10*"---")
            rosRate.sleep()
    


    def rotateVector(self, q, v):
        vr = Vector3()
        vr.x =  (v.x*q.w*q.w + 2*v.z*q.w*q.y - 2*v.y*q.w*q.z + v.x*q.x*q.x + 
        2*v.y*q.x*q.y + 2*v.z*q.x*q.z - v.x*q.y*q.y - v.x*q.z*q.z)

        vr.y =  (v.y*q.w*q.w - 2*v.z*q.w*q.x + 2*v.x*q.w*q.z - v.y*q.x*q.x + 
        2*v.x*q.x*q.y + v.y*q.y*q.y + 2*v.z*q.y*q.z - v.y*q.z*q.z)

        vr.z =  (v.z*q.w*q.w + 2*v.y*q.w*q.x - 2*v.x*q.w*q.y - v.z*q.x*q.x + 
        2*v.x*q.x*q.z - v.z*q.y*q.y + 2*v.y*q.y*q.z + v.z*q.z*q.z)
        return vr;   

    def listenToTransform(self, sourceFrame, targetFrame):      
        try:        # lookup_transform(from this frame, to this frame)
            trans = self._tfBuffer.lookup_transform(sourceFrame, targetFrame, 
                                                            Time(0), Duration(30))
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return trans
        

    def _convertArrayToMatrix(self,multiArray):
        if not multiArray:  
            return
        
        dstride1 = multiArray.layout.dim[1].stride
        h = multiArray.layout.dim[0].size
        w = multiArray.layout.dim[1].size

        dataVector = multiArray.data
        matrix = []
        matrix = [[dataVector[i*dstride1 + j] for i in range(h)] for j in range(w)]
        return matrix
    

    def _setOmniFeedbackMessage(self,feedbackForce, lockPosition):
        head = Header()
        head.stamp = Time.now()
        head.frame_id = self._HDFrame

        f = Vector3()
        f.x = -feedbackForce.x # why the minus??
        f.y = -feedbackForce.y
        f.z = -feedbackForce.z

        p = lockPosition
        return OmniFeedback(    header = head,
                                force = f,
                                position = p )

    def _set2DMultiArray(self,matrix, height, width):
        message = Float32MultiArray()
        message.layout.dim.append(MultiArrayDimension())
        message.layout.dim.append(MultiArrayDimension())
        message.layout.dim[0].label = "height"
        message.layout.dim[1].label = "width"
        message.layout.dim[0].size = height
        message.layout.dim[1].size = width
        message.layout.dim[0].stride = height*width
        message.layout.dim[1].stride = width
        message.layout.data_offset = 0

        valueList = [value for row in matrix for value in row]
        message.data = valueList
        return message

    def _setRobotForceMessage(self,force):
        message = Vector3Stamped()
        # loginfo(message)
        # loginfo(type(message))
        message.header.stamp = Time.now()
        message.header.frame_id = "virtual_marker" # form some reason given in this frame and not correct!
        message.vector.x = force[0]
        message.vector.y = force[1]
        message.vector.z = force[2]
        # loginfo(message)

        return message




if __name__ == "__main__":

    try:

        node = OmniFeedbackROS()
        node.run()

    except ROSInterruptException:
        pass