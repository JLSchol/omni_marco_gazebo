#!/usr/bin/env python
from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, sleep, get_param, Header
from rospy import ROSInterruptException, Time, Duration 
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

#import messages
# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Vector3, TransformStamped, Quaternion
from std_msgs.msg import Float32MultiArray
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
        self._maxWorkRangeHD = maxWorkrangeHD #[-x, +x]

        # all with respect to the end effector frame
        self._stiffnessRobot = np.array([])
        self._currentPositionHD = np.array([])
        self._lockPositionHD = np.array([])
        self._forcesHD = np.array([])


    def _isEmpty(self, numpyArrayList):
        for numpyArray in numpyArrayList:
            if numpyArray.size == 0:
                return True
            else:
                pass
        return False

    def calcForce(self):
        if self._isEmpty([self._stiffnessRobot,self._currentPositionHD,
                                self._lockPositionHD]):
            self._forcesHD = np.array([]) # set forces to be empty
            return

        stiffnessHD = self._calcHDStiffness(self._stiffnessRobot)

        self._forcesHD = stiffnessHD.dot(self._currentPositionHD) 

        #  self._forcesHD = transformForces(self._forcesHD)
        # limit force when above commanding range

        # loginfo(self._forcesHD)


    def setCurrentRobotStiffness(self,stiffnessRobot):
        self._stiffnessRobot = np.array(stiffnessRobot)

    def setCurrentHDPosition(self,currentPositionHD):
        self._currentPositionHD = np.array(currentPositionHD)

    def setLockPositionHD(self,lockPositionHD):
        self._lockPositionHD = np.array(lockPositionHD)

    def getHDForces(self):
        return (self._forcesHD[0],self._forcesHD[1],self._forcesHD[2])

    def _calcHDStiffness(self,stiffnessRobot):
        # stiffnessMaxHD assumes now, that the lockstate is in the center(0,0,0) position of the HD
        stiffnessMaxHD = self._maxForceLimitsHD[1]/self._maxWorkRangeHD[1] 
        scaling = stiffnessMaxHD/self._robotStiffnessLimits[1]
        stiffnessHD = scaling*stiffnessRobot

        return np.array(stiffnessHD)

        


class OmniFeedbackROS(object):
    def __init__(self):
        init_node("omni_feedback_force", anonymous=False)
        self._getParameters()
        self._publisher = Publisher(self._outputTopic, OmniFeedback , queue_size=100)
        self._stiffnessSub = Subscriber(self._stiffnessInput, Float32MultiArray, self._stiffnessCallback)
        self._positionSub = Subscriber(self._omniPositionInput, LockState, self._omniPositionCallback)

        self._tfBuffer = Buffer()
        listener = TransformListener(self._tfBuffer)

        self._stiffnessMessage = []
        self._omniPositionMessage = []

    def _getParameters(self):
        self._stiffnessInput = get_param("~RobotStiffness")
        self._omniPositionInput = get_param("~OmniPositionInput")
        self._outputTopic = get_param("~OutputTopic")
        self._HDFrame = get_param("~HDFrameName")
        self._EEFrame = get_param("~EEFrameName")
        self._virtualMarkerFrame = get_param("~virtualMarkerFrame")
        self._HDWorkRange = get_param("~HDWorkRange")
        self._HDMaxForce = get_param("~HDMaxForce")
        self._stiffnessMin = get_param("/stiffness_learning/stiffness_min")
        self._stiffnessMax = get_param("/stiffness_learning/stiffness_max")

    def _stiffnessCallback(self, message):
        self._stiffnessMessage = message

    def _omniPositionCallback(self, message):
        self._omniPositionMessage = message        

    def run(self):

        publishRate = 30 # Hz
        rosRate = Rate(publishRate)

        # initialize stiffness feedback class 
        CalcOmniFeedbackForce = CalcHDFeedbackForce([self._stiffnessMin,self._stiffnessMax],
                                                        [-self._HDMaxForce,self._HDMaxForce],
                                                        [-self._HDWorkRange,self._HDWorkRange]) # [stiffness][force][workrange]
                                                # [0.5, 0.5, 0.5], [5, 5, 5], 
                                                # [-3.3,-3.3,-3.3], [3.3,3.3,3.3], 
                                                # [-60,-60,-60],[60,60,60])

        # 1) get info and check
        # 2) rotate omni vector to ee frame 
        # 3) find force from stiffness matrix in ee frame
        # 4) transform force in ee frame back to omni frame
        # 5) publish message
        while not is_shutdown():
            # 1) get info and check
            if not (self._stiffnessMessage and self._omniPositionMessage):  
                continue
            # Get/Set stiffness matrix
            # loginfo(self._omniPositionMessage)
            stiffnessMatrix = self._convertArrayToMatrix(self._stiffnessMessage)
            CalcOmniFeedbackForce.setCurrentRobotStiffness(stiffnessMatrix)
            # find transform (rotation) from wrist to omni and transform the forces (msg: TransformStamped)
            EndEffectorinHDFrame = self.listenToTransform(self._EEFrame,self._HDFrame) # source to target


            # 2) rotate omni vector to ee frame 
            currentPositionInOmni = self.rotateVector(EndEffectorinHDFrame.transform.rotation,
                                                                    self._omniPositionMessage.current_position)
            lockPositionInOmni = self.rotateVector(EndEffectorinHDFrame.transform.rotation,
                                                        self._omniPositionMessage.lock_position)


            # 3) find force from stiffness matrix in ee frame (scaled down to HD capabilities)
            # Get/Set omni lock and current position
            CalcOmniFeedbackForce.setCurrentHDPosition([currentPositionInOmni.x, currentPositionInOmni.y,
                                                                                     currentPositionInOmni.z])
            CalcOmniFeedbackForce.setLockPositionHD([lockPositionInOmni.x, lockPositionInOmni.y, lockPositionInOmni.z])
            # Do the calculations
            CalcOmniFeedbackForce.calcForce()
            # get the feedbackForces
            forceInEEFrame= CalcOmniFeedbackForce.getHDForces()


            # 4) transform force in ee frame back to omni frame
            q = EndEffectorinHDFrame.transform.rotation
            q.w = -q.w #inverse the rotation
            forceInEEFrame = Vector3(forceInEEFrame[0],forceInEEFrame[1],forceInEEFrame[2])
            forceInOmniFrame = self.rotateVector(q,forceInEEFrame)

            # 5) publish message
            message = self._setOmniFeedbackMessage(forceInOmniFrame,self._omniPositionMessage.lock_position)
            self._publisher.publish(message)

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

        try:                                       # lookup_transform(from this frame, to this frame)
            transform = self._tfBuffer.lookup_transform(sourceFrame, targetFrame, Time(0), Duration(3.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        return transform


    def _convertArrayToMatrix(self,multiArray):
        if not multiArray:  
            return
        
        dstride1 = multiArray.layout.dim[1].stride

        h = multiArray.layout.dim[0].size
        w = multiArray.layout.dim[1].size
        dataVector = multiArray.data
        # loginfo(dataVector)
        matrix = []
        matrix = [[dataVector[i*dstride1 + j] for i in range(h)] for j in range(w)]
        # loginfo(matrix)
        return matrix
    

    def _setOmniFeedbackMessage(self,feedbackForce, lockPosition):

        # why the minus??
        head = Header()
        head.stamp = Time.now()
        head.frame_id = self._HDFrame

        f = Vector3()
        f.x = -feedbackForce.x
        f.y = -feedbackForce.y
        f.z = -feedbackForce.z

        p = lockPosition

        return OmniFeedback(    header = head,
                                force = f,
                                position = p )

if __name__ == "__main__":

    try:

        node = OmniFeedbackROS()
        node.run()

    except ROSInterruptException:
        pass