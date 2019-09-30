#!/usr/bin/env python


#import rospy
from rospy import init_node, is_shutdown, get_param, Rate, sleep, ROSInterruptException, Publisher, loginfo
from geometry_msgs.msg import Vector3

# from tf2_ros import TransformBroadcaster, TransformStamped
from phantom_omni.msg import OmniFeedback



class sendMockForces():
    def __init__(self):
        init_node("mock_forces", anonymous=True)
        self._getParameters()
        self._publisher = Publisher(self._outputTopic, OmniFeedback , queue_size=100)


    def _getParameters(self):
        self._outputTopic = get_param("~OutputTopic")


    def run(self):
        rosRate = Rate(30)
        while not is_shutdown():
            
            message = self._setForces()
            loginfo(message)
            self._publisher.publish(message)
            rosRate.sleep()


    def _setForces(self):

        return OmniFeedback(    force=Vector3(1,1,1), 
                                position=Vector3(0,0,0)   )                               



if __name__ == "__main__":
	
	try:
		node = sendMockForces()
		node.run()
	except ROSInterruptException:
		pass
