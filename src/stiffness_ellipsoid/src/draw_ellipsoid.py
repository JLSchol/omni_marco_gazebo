#!/usr/bin/env python


from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, sleep
from rospy import ROSInterruptException

#import messages
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from visualization_msgs.msg import Marker

import PyKDL


import numpy as np


class DrawEllipsoid(object):
    def __init__(self):
        init_node("draw_ellipsoid", anonymous=True)
        self._publisher = Publisher("/ellipsoid_visualization", Marker, queue_size=50)
        self._subscribers = Subscriber("/covariance_matrix", Float32MultiArray, self._callback)
        self._getParameters()
        self._multiArray = []

    def _getParameters(self):
        pass

    def _callback(self, message):
        self._multiArray = message
        

    def run(self):
        # if len(self._multiArray.data) ==0
        #     return
        
        publishRate = 30 # Hz
        rosRate = Rate(publishRate)
        while not is_shutdown():

            loginfo("running")
            loginfo(self._multiArray)

            rosRate.sleep()





if __name__ == "__main__":

    init_node("draw_ellipsoid", anonymous=True)

    try:

        node = DrawEllipsoid()
        node.run()

    except ROSInterruptException:
        pass