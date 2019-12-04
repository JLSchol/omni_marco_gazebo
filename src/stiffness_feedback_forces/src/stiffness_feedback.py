#!/usr/bin/env python
from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, sleep, get_param
from rospy import ROSInterruptException

#import messages
# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Vector3
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

        self._stiffnessRobot = np.array([])
        self._currentPositionHD = np.array([])
        self._lockPositionHD = np.array([])
        self._forcesHD = np.array([])

        # print( "hallo" + str(4))

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
            return

        stiffnessHD = self._calcHDStiffness(self._stiffnessRobot)
        self._forcesHD = stiffnessHD.dot(self._currentPositionHD) 

        # limit force when above commanding range
        # 

        loginfo(self._forcesHD)


    def setCurrentRobotStiffness(self,stiffnessRobot):
        self._stiffnessRobot = np.array(stiffnessRobot)

    def setCurrentHDPosition(self,currentPositionHD):
        self._currentPositionHD = np.array(currentPositionHD)

    def setLockPositionHD(self,lockPositionHD):
        self._lockPositionHD = np.array(lockPositionHD)

    def getHDForces(self):
        return (self._forcesHD[0],self._forcesHD[1],self._forcesHD[2])

    def _calcHDStiffness(self,stiffnessRobot):
        # stiffnessMaxHD assumes that the lockstate is in the center(0,0,0) position of the HD
        stiffnessMaxHD = self._maxForceLimitsHD[1]/self._maxWorkRangeHD[1] 
        scaling = stiffnessMaxHD/self._robotStiffnessLimits[1]
        stiffnessHD = scaling*stiffnessRobot

        return np.array(stiffnessHD)

    # def _calcMaxHDStiffness(self,maxForce,maxDeviation,lockPosition):

        

    #     return maxStiffnessHD

        


class OmniFeedbackROS(object):
    def __init__(self):
        init_node("omni_feedback_force", anonymous=False)
        self._getParameters()
        self._publisher = Publisher(self._outputTopic, OmniFeedback , queue_size=100)
        self._stiffnessSub = Subscriber(self._stiffnessInput, Float32MultiArray, self._stiffnessCallback)
        self._positionSub = Subscriber(self._omniPositionInput, LockState, self._omniPositionCallback)

        self._stiffnessMessage = []
        self._omniPositionMessage = []

    def _getParameters(self):
        self._stiffnessInput = get_param("~RobotStiffness")
        self._omniPositionInput = get_param("~OmniPositionInput")
        self._outputTopic = get_param("~OutputTopic")

    def _stiffnessCallback(self, message):
        self._stiffnessMessage = message

    def _omniPositionCallback(self, message):
        self._omniPositionMessage = message        

    def run(self):

        publishRate = 30 # Hz
        rosRate = Rate(publishRate)

        # initialize stiffness feedback class
        CalcOmniFeedbackForce = CalcHDFeedbackForce([0,1000],[-3.3,3.3],[-60,60])
                                                # [0.5, 0.5, 0.5], [5, 5, 5], 
                                                # [-3.3,-3.3,-3.3], [3.3,3.3,3.3], 
                                                # [-60,-60,-60],[60,60,60])

        while not is_shutdown():
            if not (self._stiffnessMessage and self._omniPositionMessage):  
                continue

            # Get/Set stiffness matrix
            loginfo(self._omniPositionMessage)
            stiffnessMatrix = self._convertArrayToMatrix(self._stiffnessMessage)
            CalcOmniFeedbackForce.setCurrentRobotStiffness(stiffnessMatrix)

            # Get/Set omni current_position
            currentPosition = self._omniPositionMessage.current_position
            CalcOmniFeedbackForce.setCurrentHDPosition([currentPosition.x, currentPosition.y, currentPosition.z])

            # Get/Set omni lock_position position
            lockPosition = self._omniPositionMessage.lock_position
            CalcOmniFeedbackForce.setLockPositionHD([lockPosition.x, lockPosition.y, lockPosition.z])

            # Do the calculations
            CalcOmniFeedbackForce.calcForce()

            # get the feedbackForces
            feedBackForce = CalcOmniFeedbackForce.getHDForces()

            # Set feedback force message
            message = self._setOmniFeedbackMessage(feedBackForce)

            # publish feedback force message
            self._publisher.publish(message)

            loginfo(10*"---")
            rosRate.sleep()
        


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
    
    def _setOmniFeedbackMessage(self,feedbackForce):

        

        return OmniFeedback(    force = Vector3(-feedbackForce[0],
                                                -feedbackForce[1],
                                                -feedbackForce[2]),
                                position = Vector3(0,0,0)  )

if __name__ == "__main__":

    try:

        node = OmniFeedbackROS()
        node.run()

    except ROSInterruptException:
        pass