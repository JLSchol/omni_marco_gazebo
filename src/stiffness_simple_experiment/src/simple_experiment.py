#!/usr/bin/env python

from rospy import init_node, Publisher, Subscriber, is_shutdown, signal_shutdown, Rate, loginfo
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal, spin
# custom class
from stiffness_visualization.ellipsoid_message import EllipsoidMessage
# from stiffness_visualization.draw_ellipsoid import DrawEllipsoid

#import messages
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,String
from visualization_msgs.msg import Marker
#import custom messages
from stiffness_commanding.msg import EigenPairs, VectorValue
# custom msgs
from stiffness_simple_experiment.msg import gui_command

class ExperimentInfo(object):
    def __init__(self,experimentNr=1,PracticeRun=False):
        self.data = self.getInfo(experimentNr,PracticeRun)     

    def getInfo(self,experimentNr,PracticeRun):
        experiment = str(experimentNr) + str(PracticeRun)
        switcher={
            '1False':self.experiment1Real,
            '1True':self.experiment1Practice,
            '2False':self.experiment2Real,

        }
        if experiment in switcher:
            func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
                                                                    .format(experiment))
            self.data = func()
            return func()
        else:
            print("invalid experiment name: {}".format(experiment))

    def experiment1Real(self):
        infoSequence={  'experiment': '1False',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
    def experiment1Practice(self):
        infoSequence={  'experiment': '1True',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
    def experiment2Real(self):
        infoSequence={  'experiment': '2False',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
        
    def getShape(self,trialNr,data):
        scales = data['scale'][trialNr]
        orientation = data['orientation'][trialNr]
        return scales, orientation


class SimpleExperiment(object):
    def __init__(self):
        init_node("simple_experiment",anonymous=True)
        # self.prevGuiMsg = gui_command()
        self._initVars()
        
        # self._getParameters()

        self._guiSub = Subscriber("gui_commands", gui_command, self._guiCallBack)
        self._ellipsPub = Publisher("experiment_ellipsoid", Marker, queue_size=2)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

    def _initVars(self):
        self.newGuiCommand = False
        self.newExperiment = []
        self.prevGuiMsg = gui_command()
        self.guiMsg = gui_command()
        self.guiMsg.start_experiment = []
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
        
        # rosRate = Rate(5)
        while not is_shutdown():
            if not self.guiMsg.start_experiment:
                print("node launched exp not started")
                continue

            # if self.newExperiment==True:
            #     print("new experiment started")
            #     self._resetTrials()
            #     self.newExperiment = False

            # print(self.prevTrialNr)
            # if self._firstLoop():
            if not isinstance(self.prevTrialNr,int):
                print('in first loop')
                # _checkGuiMsg and initialization and 
                self.startTimeTrial = Time.now()
                self.prevTrialNr = 0
                # need to check if start experiment has changed
                EI = ExperimentInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
                EM = EllipsoidMessage()
                print(10*"-----")
                # print("crach2")

            elif self._newTrial(self.guiMsg):
                print('in NEWTRIAL loop')
                # Do stuff to update trial properties
                # update trial completion times
                self.trialTime, self.startTimeTrial = self._getUpdatedTrialTimes(
                                                                            self.startTimeTrial)
                # update trial number
                self.trialNr,self.prevTrialNr = self._getUpdatedTrialNumbers(
                                                    self.trialNr,self.guiMsg.trial_change,False)
                acc = self._getAccuracy()
                self._logPub.publish(self._createLogString(self.prevTrialNr,self.trialTime,acc))
                print(10*"-----")
                # print("crach3")
            # publish ellipsoid
            # print('normal')
            # print("crach4")
            data = EI.data
            # print(self.trialNr)
            scales,quats = EI.getShape(self.trialNr,data)
            ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                positions,quats,scales,rgba)
            self._ellipsPub.publish(ellipsoid)  

            
            # shutdown node from gui command
            if self.guiMsg.start_experiment == False:
                signal_shutdown("User requisted shutdown")

            # rosRate.sleep()
         
    def _firstLoop(self):
        # needs other check:
        # - if start experiment has changed
        # - if practice run has changed
        if self.prevTrialNr:
            # print(self.prevTrialNr)
            return True

    def _newTrial(self,guiMsg):
        userTrialPass = False
        # print(newGuiCommand)
        # print(guiMsg.trial_change)
        if userTrialPass==True:
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

    def _getUpdatedTrialNumbers(self,trialNr,guiMsgTrial,userTrialPass=True):
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
        logString = "trial number: {}, time: {} seconds, accuracy: {}%".format(
                                                                trial,time,accuracy)
        return logString



        
    def _guiCallBack(self, message):
        self.guiMsg = message
        self.newGuiCommand = True
        # if self.guiMsg.experiment_number != self.prevGuiMsg.experiment_number:
        #     self.newExperiment = True  
        #     print("new experiment_number")      
        print("Callback received")



if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        spin()

    except ROSInterruptException:
        pass