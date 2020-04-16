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
        self._logPub = Publisher("simple_experiment_logger", String, queue_size=1)
        self._textVisPUb = Publisher("accuracy_ellipsoid_text", Marker, queue_size=2)

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
        # self.qAllignedQuats = [0,0,0,1]
        self.userQuat = [0,0,0,1]
        self.userScales = []
        self.originalUserQuat = [0,0,0,1]

    def _resetTrials(self):
        self.prevTrialNr = []
        self.trialNr = 0
        self.trialTime = []
        self.startTimeTrial = []


    def run(self):
        # intiialize custom classes
        EI = []
        EM = EllipsoidMessage()

        # initialize ellipsoid data
        frame_id='wrist_ft_tool_link' #wrist_ft_tool_link
        marker_ns='ellips_experiment'
        marker_id=1
        positions=[0,0,0]
        rgba=[0.95,0.95,0.05,0.2]
        
        rosRate = Rate(100)
        counter = 0
        while not is_shutdown():
            # If experiment is not yet started, stay in this loop
            if not self.guiMsg.start_experiment:
            	EM.deleteMarker(marker_ns,marker_id)
                if counter%200 == 0: # every 200th loop
                    print("node launched exp not started or pauzed...")
                counter+=1
                rosRate.sleep()
                # need to reset certain variables
                continue

            # if trial number exceeds total trials, the experiment is finished 
            # Stay in this loop
            # EI = ExperimentInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
            if EI != []:
                # print(EI)
                if (len(EI.data['trialNr']) < self.trialNr + 1):        # self.trialNr start at 0 therefore +1
                    EM.deleteMarker(marker_ns,marker_id)
                    finishText = EM.createMarkerText(frame_id,"text",3,"Experiment Finished\n\nGood Job!",[0,0,0],0.1,[1,1,1,1],3)
                    self._textVisPUb.publish(finishText)
                    counter+=1
                    rosRate.sleep()
                    continue

            # If buttonpress and not start of node, user clicked button to advance to next trial
            if (self.lockStateMsg.lock_white==True) and (self.lockStateMsg.header != self.prevLockStateMsg.header):
                # print("User advanced to next trial")
                self.userTrialPass = True

            # if prevtrialnumber is not yet a number, experiment has not yet started
            # Create starting point for time 
            if not isinstance(self.prevTrialNr,int):
                print(10*"-----")
                print('Experiment started, first trial')
                # _checkGuiMsg and initialization and 
                self.startTimeTrial = Time.now()
                self.prevTrialNr = 0
                # need to check if start experiment has changed
                EI = ExperimentInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
                
                self.userTrialPass=False
                print(10*"-----")
                # print("crach2")

            # other loops check for trial pass and update
            elif self._newTrial(self.guiMsg):
                print('in new trial loop')
                # Do stuff to update trial properties
                # get trial time, refresh starting time of trial
                self.trialTime, self.startTimeTrial = self._getUpdatedTrialTimes(
                                                                            self.startTimeTrial)
                # update trial number
                self.trialNr,self.prevTrialNr = self._getUpdatedTrialNumbers(
                                                    self.trialNr,self.guiMsg.trial_change,self.userTrialPass)
                shapeAcc, rotationAcc = self._getAccuracy(self.prevTrialNr,EI,EM)
                self._logPub.publish(self._createLogString(self.prevTrialNr,self.trialTime,shapeAcc,rotationAcc))

                shapeText, orientationText = self._getMarkerTexts(shapeAcc, rotationAcc, EM, frame_id)

                # self._textVisPUb.publish(markerText)
                self._textVisPUb.publish(shapeText)
                self._textVisPUb.publish(orientationText)

                # briefly delete marker such that it is clear that a new trial starts ### does not work...
                EM.deleteMarker(marker_ns,marker_id)

                # update boolian 
                self.userTrialPass=False
                print(10*"-----")


            # if trialnr is smaller than total trials, the experiment publish the experiment ellipsoids
            if len(EI.data['trialNr']) >= self.trialNr+1: # self.trialNr start at 0 therefore +1
                scales,quats = EI.getShape(self.trialNr,EI.data)
                ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                    positions,quats,scales,rgba)

                EM.broadcastEllipsoidAxis(positions,quats,frame_id,marker_ns)
                self._ellipsPub.publish(ellipsoid)  
            # elif len(EI.data['trialNr']) <= self.trialNr + 1: # self.trialNr start at 0 therefore +1
            #     EM.deleteMarker(marker_ns,marker_id)
            # else:
            #     print("something went wrong")

            # always puvblish user ellipsoid
            EM.broadcastEllipsoidAxis(positions,self.originalUserQuat,frame_id,'original_user_ellips')
            # EM.broadcastEllipsoidAxis(positions,self.qAllignedQuats,frame_id,'minmax_axis_alligned')
            EM.broadcastEllipsoidAxis(positions,self.userQuat,frame_id,'corrected_user_ellips')
            
            self.userTrialPass=False
            # shutdown node from gui command
            # if self.guiMsg.start_experiment == False:
            #     signal_shutdown("User requisted shutdown")
            counter += 1
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
            print("new trial from user")
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
        self.originalUserQuat = list(quats)

        # get new quaternion where the longest axis of user and experiment is alligned 
        # by swapping axes e.g. [x=y,y=z,z=x]
        # Then check direction of the new alligned axis ans rotate 180 when pionting in opposite directino
        # finally, rotate around that axis to the closest orientation of the exp ellipsoid
        # find the closest orientation by rotating around the longest axis
        
        axis = EM.getCharacteristicAxis(scalesExperiment,scales)

        swappedAxisQuat, self.userScales, _, _ = EM.axisSwap(scalesExperiment,eigVectors,eigValues, 
        														self._lambda_min, self._lambda_max,axis)
        self.userQuat,_ = EM.closestQuaternionProjection(swappedAxisQuat,quatsExperiment,self.userScales,axis)
    

        # print(10*"----")
        # print("Experiment scales: {}  ;   quats: {}".format(scalesExperiment, quatsExperiment))
        # print("initial User scales: {}  ;   quats: {}".format(scales, quats))
        # # print("One axis alligned: {}  ;   quats: {}".format(newScales, self.qAllignedQuats))
        # print("New user scales: {}  ;   quats: {}".format(self.userScales, self.userQuat))
        # print(10*"----")
        
    
        # userVolume = EM.volumeEllipsoid(scales)
        # experimentVolume = EM.volumeEllipsoid(scalesExperiment)
        # shapeAccuracy = round(percentage(abs(userVolume-experimentVolume),experimentVolume),2)
        # print("userVolume = {} [m3]; experimentVolume = {} [m3]".format(userVolume,experimentVolume))

        angle = EM.absoluteAngleBetweenEllipsoids(quatsExperiment,self.userQuat,'deg')

        userShape = EM.distanceEllipsoid(scales)
        experimentShape = EM.distanceEllipsoid(scalesExperiment)

        percentage = lambda nominator, denominator: 100 * (1 - float(nominator)/float(denominator))
        rotationAccuracy = round(percentage(angle,90.0),2)
        shapeAccuracy = round(percentage(abs(userShape-experimentShape),experimentShape),2)

        # print("Absolute angle = {} [degrees]; max angle = {} [degrees]".format(angle,90.0))
        # print("userShape = {} [m]; experimentShape = {} [m]".format(userShape,experimentShape))

        return shapeAccuracy, rotationAccuracy

            
    def _createLogString(self,trial,time,volAcc,rotAcc):
        t = round(time.to_sec(),2)
        logString = "trial number: {}, time: {} [sec], shape: {} [%], orientation: {} [%]".format(trial,t,volAcc,rotAcc)
        return logString

    # def _createFeedbackTextString(self,volAcc,rotAcc):
    # 	textString = "Shape: {} %\nOrientation: {} %".format(volAcc,rotAcc)
    # 	return textString
    def _feedBackText(self, identifier, value, newLine=True):
        if newLine:
            text = "\n\n{}: {} %".format(identifier,value)
        elif not newLine:
            text = "{}: {} %".format(identifier,value)
        else:
            print("{} is not a valid 3th input argument. Try 'True' or 'False'".format(newLine))
        return text

    def _getMarkerTexts(self,shapeAcc,rotationAcc,EM,frame_id):
        marker_ns = "text"

        textPosSh = [-0.3,-0.25,-0.6]
        # textPosOr = [-0.3,-0.25,-0.6]
        textPosOr = [textPosSh[0]+0.01, textPosSh[1]+0.01, textPosSh[2]]

        # textPosOr = [-0.3,0,-0.6]
        textHeightSH = 0.1
        idSh = 1
        rgbSh = self._getColorText(shapeAcc)
        shapeStr = self._feedBackText("Shape",shapeAcc,False)

        coeff = 0.88

        # textPosOr = [textPosSh[0]+1.5*textHeightSH, textPosSh[1]+1.5*textHeightSH, textPosSh[2]-0.05]
        textHeightOr = textHeightSH*coeff # because this text is closer to the camera make smaller
        idOr = 2
        rgbOr = self._getColorText(rotationAcc)
        orientationsStr = self._feedBackText("Orientation",rotationAcc,True)

        lifeTime = 3 # sec

        shapeText = EM.createMarkerText(frame_id,marker_ns,idSh,
                                        shapeStr,
                                        textPosSh,textHeightSH,rgbSh,lifeTime)

        orientationText = EM.createMarkerText(frame_id,marker_ns,idOr,
                                        orientationsStr,
                                        textPosOr,textHeightSH,rgbOr,lifeTime)
        return shapeText, orientationText

    def _getColorText(self,acc):
        color = []
        if acc >= 90.0:
            color = [0.25,0.74,0.25,1] # 64, 191, 64 green
        elif acc >= 80.0 and acc < 90:
        # elif 90.0 < acc <= 80:
            color = [1,0.457,0.102,1] # 255, 117, 26 orange
        else:
            color = [1,0.3,0.3,1]  # 255, 77, 77 red
        return color



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