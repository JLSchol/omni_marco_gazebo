#!/usr/bin/env python

from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, spin
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal
# custom class
from ellipsoid_message import EllipsoidMessage

class SimpleExperiment(object):
    def __init__(self):
        pass

    def run(self):
        while not is_shutdown:
            print("hoi")





if __name__ == "__main__":
    try:
        node = SimpleExperiment()
        node.run()
        spin()

    except ROSInterruptException:
        pass