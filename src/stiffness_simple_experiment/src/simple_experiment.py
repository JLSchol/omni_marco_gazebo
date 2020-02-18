#!/usr/bin/env python

from rospy import init_node, Publisher, Subscriber, is_shutdown, signal_shutdown, Rate, loginfo, spin
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal
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
            # '11':self.experiment1Practice,
            # '20':self.experiment2Real,
            # '21':self.experiment2Practice,
            # '30':self.experiment2Real,
            # '31':self.experiment2Practice
        }
        if experiment in switcher:
            func = switcher.get(experiment, lambda: "invalid experiment definition: {}".format(experiment))
            self.data = func()
            return func()
        else:
            print("invalid experiment name: {}".format(experiment))

    def experiment1Real(self):
        infoSequence={  'experiment': '1False',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.0,0.0,0.0],
                                    [0.0,0.0,0.0]],
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
        self.prevGuiMsg = gui_command()
        self.guiMsg = gui_command()
        self.guiMsg.start_experiment = []
        self.prevTrialNr = []
        self.trialNr = 0
        self.startTimeTrial = []
        self.stopTimeTrial =[]
        
        # self._getParameters()

        self._guiSub = Subscriber("gui_commands", gui_command, self._guiCallBack)
        self._ellipsPub = Publisher("experiment_ellipsoid", Marker, queue_size=10)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

        

    def run(self):
        # self.guiMsg.experiment_number = 1
        # Learning = False
        # trialNr = 0
        # initialize classes
        EI = ExperimentInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
        EM = EllipsoidMessage()
        # initialize ellipsoid data
        frame_id='base_footprint' #wrist_ft_tool_link
        marker_ns='ellips_experiment'
        marker_id=1
        positions=[0.5,0.5,0.5]
        rgba=[222,235,38,0.5]
        
        # rosRate = Rate(5)
        while not is_shutdown():
            if not self.guiMsg.start_experiment:
                continue

            # start time
            self.startTimeTrial = Time.now()
            # Check for next trial and update trialNr 
            self.trialNr = self._newTrial(self.trialNr)    

            # loginfo(self.prevGuiMsg)
            loginfo(self.guiMsg)
            # get the data which spawns the ellipsoids
            # keep in while loop such that the node does not have to be killed for 
            # every experiment
            data = EI.getInfo(self.guiMsg.experiment_number,self.guiMsg.learning)
            scales,quats = EI.getShape(self.trialNr,data)
            ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                positions,quats,scales,rgba)

            # publish ellipsoid
            self._ellipsPub.publish(ellipsoid)  

            # Only publish logger when new trial has started
            # With the data from the previous trial
            if self.prevTrialNr != self.trialNr:
                time = self._getTrialTime()
                acc = self._getAccuracy()
                self._logPub.publish(self._createLogString(self.prevTrial,time,acc))
            
            # update for new loop
            self.prevTrialNr = self.trialNr
            self.prevGuiMsg = self.guiMsg

            # shutdown node from gui command
            if self.guiMsg.start_experiment == False:
                signal_shutdown("User requisted shutdown")

            # rosRate.sleep()
            
    def _newTrial(self,trialNr,):
        # newTrial = False
        # if gui command +1--> new trial
        # if user command --> new trial
        userCommand = True
        if self.guiMsg.trial == 1 or userCommand == True:
            trialNr = trialNr+1
        # if gui command -1--> prev trial
        elif self.guiMsg.trial == -1:
            trialNr = trialNr-1

        return trialNr        

    def _getTrialTime(self,):

        return 11.11
    def _getAccuracy(self):
        return 89.9

            
    def _createLogString(self,trial,time,accuracy):
        logString = "trial number: {}, time: {} seconds, accuracy: {}%".format(trial,time,accuracy)
        return logString



    # def _getParameters(self):
    #     pass

        
    def _guiCallBack(self, message):
        self.guiMsg = message



if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        spin()

    except ROSInterruptException:
        pass