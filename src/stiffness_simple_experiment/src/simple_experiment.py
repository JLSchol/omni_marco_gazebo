#!/usr/bin/env python
# rospy
from rospy import init_node, Publisher, Subscriber, is_shutdown, signal_shutdown, Rate, loginfo
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal, spin
# python 
# from keyboard import is_pressed
# custom class
from stiffness_visualization.ellipsoid_message import EllipsoidMessage
# from stiffness_visualization.draw_ellipsoid import DrawEllipsoid
#import ellipsoid sequence
from experiment_ellipsoids import ExperimentInfo
#import messages
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,String
from visualization_msgs.msg import Marker
#import custom messages
from phantom_omni.msg import LockState
from stiffness_commanding.msg import EigenPairs
from stiffness_simple_experiment.msg import gui_command

import numpy as np
# class ExperimentVariables():
#     def __init__(self,):



class SimpleExperiment(object):
    def __init__(self):
        init_node("simple_experiment",anonymous=True)
        # self.prevGuiMsg = gui_command()
        self._initVars()
        
        self._getParameters()

        self._guiSub = Subscriber("gui_commands", gui_command, self._guiCallBack)
        self._omniSub = Subscriber("/omni1_lock_state", LockState, self._phantomCallBack)
        self._eigenSub = Subscriber("/eigen_pair", EigenPairs, self._eigenCallBack)
        self._ellipsPub = Publisher("experiment_ellipsoid", Marker, queue_size=2)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

    def _getParameters(self):
        lambdaminPar = '/stiffness_commanding/lambda_min'
        lambdamaxPar = '/stiffness_commanding/lambda_max'
        for param in [lambdaminPar, lambdamaxPar]:
            if not has_param(param):
                logfatal("Could not retrive %s from the parameter server", param)
        self._lambda_min = get_param(lambdaminPar)
        self._lambda_max = get_param(lambdamaxPar)

    def _initVars(self):
        self.newGuiCommand = False
        # self.newExperiment = []
        # self.prevGuiMsg = gui_command()
        self.guiMsg = gui_command()
        self.lockStateMsg = LockState()
        self.prevLockStateMsg = LockState()
        self.eigenPairMsg = EigenPairs()

        self.guiMsg.start_experiment = []
        self.userTrialPass=False
        self.prevTrialNr = []
        self.trialNr = 0
        self.trialTime = []
        self.startTimeTrial = []
        self.stopTimeTrial =[]
        self.qAllignedQuats = [0,0,0,1]
        self.newQuats = [0,0,0,1]
        self.originalQuat = [0,0,0,1]

    def _resetTrials(self):
        self.prevTrialNr = []
        self.trialNr = 0
        self.trialTime = []
        self.startTimeTrial = []


    def run(self):
        # intiialize custom classes
        EI = []
        EM = []

        # initialize ellipsoid data
        frame_id='wrist_ft_tool_link' #wrist_ft_tool_link
        marker_ns='ellips_experiment'
        marker_id=1
        positions=[0,0,0]
        rgba=[0.95,0.95,0.05,0.2]
        
        rosRate = Rate(100)
        while not is_shutdown():
            if not self.guiMsg.start_experiment:
                print("node launched exp not started")
                # need to reset certain variables
                continue

            # if self.lockStateMsg.lock_white:
            #     print("ellipsoid is fixed, puaze tussen trial")
            #     continue
            # print("while: "str(self.lockStateMsg.lock_white))
            if ( (self.lockStateMsg.lock_white==True) and (self.lockStateMsg.header != self.prevLockStateMsg.header) ):
                print("lockstate = True")
                self.userTrialPass = True

            # print("while lock: "+ str(self.lockStateMsg.lock_white))
            # print("while :"+str(self.userTrialPass))
            if not isinstance(self.prevTrialNr,int):
                print('in first loop')
                # _checkGuiMsg and initialization and 
                self.startTimeTrial = Time.now()
                self.prevTrialNr = 0
                # need to check if start experiment has changed
                EI = ExperimentInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
                EM = EllipsoidMessage()
                self.userTrialPass=False
                print(10*"-----")
                # print("crach2")

            # other loops check for trial pass and update
            elif self._newTrial(self.guiMsg):
                print('in NEWTRIAL loop')
                # Do stuff to update trial properties
                # update trial completion times
                self.trialTime, self.startTimeTrial = self._getUpdatedTrialTimes(
                                                                            self.startTimeTrial)
                # update trial number
                self.trialNr,self.prevTrialNr = self._getUpdatedTrialNumbers(
                                                    self.trialNr,self.guiMsg.trial_change,self.userTrialPass)
                acc = self._getAccuracy(self.prevTrialNr,EI,EM)
                self._logPub.publish(self._createLogString(self.prevTrialNr,self.trialTime,acc))

                # update boolian 
                self.userTrialPass=False
                print(10*"-----")

            scales,quats = EI.getShape(self.trialNr,EI.data)
            ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                positions,quats,scales,rgba)
            EM.broadcastEllipsoidAxis(positions,self.originalQuat,frame_id,'original_usercommanded')
            EM.broadcastEllipsoidAxis(positions,self.qAllignedQuats,frame_id,'largest_axis_alligned')
            EM.broadcastEllipsoidAxis(positions,self.newQuats,frame_id,'corrected_ellips')
            EM.broadcastEllipsoidAxis(positions,quats,frame_id,marker_ns)


            self._ellipsPub.publish(ellipsoid)  
            
            self.userTrialPass=False
            # shutdown node from gui command
            # if self.guiMsg.start_experiment == False:
            #     signal_shutdown("User requisted shutdown")

            rosRate.sleep()
         
    def _firstLoop(self):
        # needs other check:
        # - if start experiment has changed
        # - if practice run has changed
        if self.prevTrialNr:
            # print(self.prevTrialNr)
            return True

    def _newTrial(self,guiMsg):
        if self.userTrialPass==True:
            print("new trial from user pass")
            return True
        elif self.newGuiCommand == True and guiMsg.trial_change!=0:
            print('new trial from gui')
            self.newGuiCommand = False
            return True
        else:
            # print("No new trial")
            return False
    

    def _getUpdatedTrialTimes(self,startTime):
        stopTime = Time.now()
        time = stopTime - startTime
        return time, stopTime

    def _getUpdatedTrialNumbers(self,trialNr,guiMsgTrial,userTrialPass):
        # update trial number
        prevTrialNr = trialNr
        if userTrialPass: # increase with one
            trialNr += 1
        else: #increase or decrease depending on command
            trialNr += guiMsgTrial
        return trialNr,prevTrialNr

    def _getAccuracy(self,trialNr,EI,EM):
        ############ EXPERIMENT ELLIPSOID ############
        # get scales and quats from spawned ellipsoid
        scalesExperiment, quatsExperiment = EI.getShape(trialNr,EI.data)
        ############ PARTICIPANT ELLIPSOID ############
        # Convert message to vector value pairs
        eigVectors, eigValues = EM.EigenPairMsgsToMatrixVector(self.eigenPairMsg)
        # get ellipsoid scales , quats from user 
        scales = EM.getEllipsoidScales(eigValues, self._lambda_min, self._lambda_max)       
        quats = EM.getQuatFromMatrix(eigVectors)

        # get new quaternion where the longest axis of user and experiment is alligned
        # By shuffeleng the eigenvectors
        shuffleSequence = EM.getShuffleSequence(scalesExperiment,scales,eigValues,eigVectors)
        newValues, newVectors = EM.shuffleEig(eigValues, eigVectors, shuffleSequence)
        if not EM.checkRightHandedNessMatrix(newVectors):
            logfatal("not a valid rotation, could NOT find a solution")
        newScales = EM.getEllipsoidScales(newValues, self._lambda_min, self._lambda_max)

        self.originalQuat = list(quats)
        qAlligned = list(EM.getQuatFromMatrix(newVectors))
        self.newQuats, self.qAllignedQuats = EM.closestQuaternionProjection(qAlligned,quatsExperiment,newScales)
        # self.oneAxisAllignedQuat = list(EM.getQuatFromMatrix(newVectors))
        # self.newQuats = list(EM.closestQuaternion(self.oneAxisAllignedQuat,quatsExperiment,scalesExperiment))

        # find the closest orientation by rotating around the longest axis



        print(10*"----")
        print("Experiment scales: {}  ;   quats: {}".format(scalesExperiment, quatsExperiment))
        print("initial User scales: {}  ;   quats: {}".format(scales, quats))
        print("One axis alligned: {}  ;   quats: {}".format(newScales, self.qAllignedQuats))
        print("New user scales: {}  ;   quats: {}".format(newScales, self.newQuats))
        print(10*"----")
        
        angle = EM.absoluteAngle(quatsExperiment,self.newQuats,'deg')
        print(angle)
        # rotationAccuracy = 100.0 - (angle/90.0)

        return angle

            
    def _createLogString(self,trial,time,accuracy):
        
        # time = round(float(time)*float(10**(-9)),3)
        logString = "trial number: {}, time: {} seconds, accuracy: {}%".format(
                                                                trial,time.to_sec(),accuracy)
        return logString



        
    def _guiCallBack(self, message):
        self.guiMsg = message
        self.newGuiCommand = True
        # if self.guiMsg.experiment_number != self.prevGuiMsg.experiment_number:
        #     self.newExperiment = True  
        #     print("new experiment_number")      
        print("Gui callback received")

    def _phantomCallBack(self, message):
        self.prevLockStateMsg = self.lockStateMsg
        self.lockStateMsg = message

    def _eigenCallBack(self, message):
        self.eigenPairMsg = message







if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        # spin()

    except ROSInterruptException:
        pass