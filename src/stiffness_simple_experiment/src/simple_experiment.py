#!/usr/bin/env python

from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, spin
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal
# custom class
from stiffness_visualization.ellipsoid_message import EllipsoidMessage
# 
from std_msgs.msg import String

class SimpleExperiment(object):
    def __init__(self):
        init_node("simple_experiment",anonymous=True)
        self._getParameters()

        self._guiSub = Subscriber("gui_command", String, self._guiCallBack)
        self._logPub = Publisher("simple_experiment_logger", String,queue_size=1)

        

    def run(self):
        while not is_shutdown:
            print("hoi")




    def _getParameters(self):
        pass

    def _guiCallBack(self, message):
        self.guiMsg = message



if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        spin()

    except ROSInterruptException:
        pass