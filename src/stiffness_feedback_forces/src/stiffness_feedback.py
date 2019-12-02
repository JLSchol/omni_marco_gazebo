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
    def __init__(self,minRobotImpedanceStiffness,maxRobotImpedanceStiffness,
                    minForceHD, maxForceHD,maxWorkrangeHD):
        self._minRobotStiffness = minRobotImpedanceStiffness
        self._maxRobotStiffness = maxRobotImpedanceStiffness
        self._minForceHD = minForceHD
        self._maxForceHD = maxForceHD
        # self._maxWorkrangeHD = minStiffnessHD
        self._maxWorkrangeHD = maxWorkrangeHD

        self._stiffnessRobot = np.zeros(3,3)
        self._currentPositionHD = np.zeros(3)
        self._lockPositionHD = np.zeros(3)
        self._forcesHD = np.zeros(3)

        # print( "hallo" + str(4))

        # self._robotStiffness = np.zeros(3,3)

    def calcForce(self):
        stiffnessHD = self._calcHDStiffness()
        self._forcesHD = stiffnessHD * self._currentPositionHD


    def setCurrentRobotStiffness(self,stiffnessRobot):
        self._stiffnessRobot = np.array(stiffnessRobot)

    def setCurrentHDPosition(self,currentPositionHD):
        self._currentPositionHD = np.array(currentPositionHD)

    def setLockPositionHD(self,lockPositionHD):
        self._lockPositionHD = np.array(lockPositionHD)

    def getHDForces(self):
        return self._forcesHD

    def _calcHDStiffness(self):
        stiffnessHD = np.zeros(3,3)
        return stiffnessHD


        


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
        self._stiffnessInput = get_param("~StiffnessInput")
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
        CalcHDFeedbackForce CalcOmniFeedbackForce(100,1000,[0.5, 0.5, 0.5], [5, 5, 5],)

        while not is_shutdown():
            if not self._stiffnessMessage:  

                continue

            # Get/Set stiffness matrix
            stiffnessMatrix = self._convertArrayToMatrix(self._stiffnessMessage)
            CalcOmniFeedbackForce.setCurrentRobotStiffness(stiffnessMatrix)

            # Get/Set omni current_position
            currentPosition = self._omniPositionMessage.current_position
            CalcOmniFeedbackForce.setCurrentHDPosition(currentPosition)

            # Get/Set omni lock_position position
            lockPosition = self._omniPositionMessage.lock_position
            CalcOmniFeedbackForce.setLockPositionHD(lockPosition)

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
        loginfo(dataVector)
        matrix = []
        matrix = [[dataVector[i*dstride1 + j] for i in range(h)] for j in range(w)]
        loginfo(matrix)
        return matrix
    
    def _setOmniFeedbackMessage(self,feedbackForce):

        return OmniFeedback(    force = Vector3(feedbackForce[0],
                                                feedbackForce[1],
                                                feedbackForce[2]),
                                position = Vector3(0,0,0)  )

if __name__ == "__main__":

    try:

        node = OmniFeedbackROS()
        node.run()

    except ROSInterruptException:
        pass