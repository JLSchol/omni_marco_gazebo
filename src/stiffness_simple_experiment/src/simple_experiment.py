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
from stiffness_commanding.msg import EigenPairs, VectorValue
# custom msgs
from stiffness_simple_experiment.msg import gui_command


# class ExperimentVariables():
#     def __init__(self,):



class SimpleExperiment(object):
    def __init__(self):
        init_node("simple_experiment",anonymous=True)
        # self.prevGuiMsg = gui_command()
        self._initVars()
        
        # self._getParameters()

        self._guiSub = Subscriber("gui_commands", gui_command, self._guiCallBack)
        self._omniSub = Subscriber("/omni1_lock_state", LockState, self._phantomCallBack)
        self._ellipsPub = Publisher("experiment_ellipsoid", Marker, queue_size=2)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

    def _initVars(self):
        self.newGuiCommand = False
        # self.newExperiment = []
        # self.prevGuiMsg = gui_command()
        self.guiMsg = gui_command()
        self.lockStateMsg = LockState()
        self.prevLockStateMsg = LockState()

        self.guiMsg.start_experiment = []
        self.userTrialPass=False
        self.prevTrialNr = []
        self.trialNr = 0
        self.trialTime = []
        self.startTimeTrial = []
        self.stopTimeTrial =[]

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
        frame_id='base_footprint' #wrist_ft_tool_link
        marker_ns='ellips_experiment'
        marker_id=1
        positions=[0.5,0.5,0.5]
        rgba=[0.9,0.9,30.15,0.2]
        
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

            print("while lock: "+ str(self.lockStateMsg.lock_white))
            print("while :"+str(self.userTrialPass))
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
                acc = self._getAccuracy()
                self._logPub.publish(self._createLogString(self.prevTrialNr,self.trialTime,acc))

                # update boolian 
                self.userTrialPass=False
                print(10*"-----")

            data = EI.data
            scales,quats = EI.getShape(self.trialNr,data)
            ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                positions,quats,scales,rgba)
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

    def _getAccuracy(self):
        return 89.9

            
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
        print("Callback received")

    def _phantomCallBack(self, message):
        self.prevLockStateMsg = self.lockStateMsg
        self.lockStateMsg = message
        
        # print("cb: " + str(self.lockStateMsg.lock_white))






if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        # spin()

    except ROSInterruptException:
        pass