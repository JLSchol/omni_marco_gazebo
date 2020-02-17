#!/usr/bin/env python

from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, spin
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
                        'scale': [  [0.15,0.0,0.0],[0.0,0.0,0.0],
                                    [0.0,0.0,0.0]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
        
    def getShape(self,trialNr,data):
        scales = data['scale'][trialNr]
        orientation = data['orientation'][trialNr]
        return scales, orientation


    # <arg name="lambda_min" value="0.03"/>

class SimpleExperiment(object):
    def __init__(self):
        init_node("simple_experiment",anonymous=True)
        # self._getParameters()

        # self._guiSub = Subscriber("gui_commands", gui_command, self._guiCallBack)
        self._ellipsPub = Publisher("Experiment_ellipsoid", Marker, queue_size=10)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

        

    def run(self):
        
        # if self.guiMsg.start_experiment:
        expNr = 1
        Learning = False
        trialNr = 0
        EI = ExperimentInfo(expNr,Learning)
        EM = EllipsoidMessage()
        frame_id='base_footprint' #wrist_ft_tool_link
        marker_ns='ellips_experiment'
        marker_id=1
        positions=[0.5,0.5,0.5]
        rgba=[222,235,38,0.3]
        
        rosRate = Rate(30)
        print("inrun")
        while not is_shutdown():
            # loginfo("In while")
            data = EI.getInfo(expNr,Learning)
            
            scales,quats = EI.getShape(trialNr,data)
            ellipsoid = EM.getEllipsoidMsg(frame_id,marker_ns,marker_id,
                                                positions,quats,scales,rgba)
            loginfo(ellipsoid)
            self._ellipsPub(ellipsoid)
            rosRate.sleep()
            
            




    # def _getParameters(self):
    #     pass

        
    # def _guiCallBack(self, message):
    #     self.guiMsg = message



if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        spin()

    except ROSInterruptException:
        pass