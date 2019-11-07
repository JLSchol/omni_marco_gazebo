#!/usr/bin/env python


#import rospy
from rospy import init_node, is_shutdown, get_param, Rate, sleep, ROSInterruptException, Publisher, loginfo
from geometry_msgs.msg import Point


# from phantom_omni.msg import OmniFeedback
from phantom_omni.msg import LockState
from phantom_omni.msg import PhantomButtonEvent


class sendMockOmniPosition():
    def __init__(self):
        init_node("mock_omni_position", anonymous=True)
        self._getParameters()
        self._pubPosition= Publisher(self._outputTopicPosition, LockState , queue_size=1)
        self._pubButton = Publisher(self._outputTopicButton, PhantomButtonEvent , queue_size=1)


    def _getParameters(self):
        self._outputTopicPosition = get_param("~OutputTopicPosition")
        self._outputTopicButton = get_param("~OutputTopicButton")


    def run(self):
        rosRate = Rate(30)
        start = True
        # sleep(2) # wait a bit
        while not is_shutdown():
            
            buttonMsg = self._setButton(start)
            positionMsg = self._setLockState()

            loginfo(buttonMsg)
            loginfo(positionMsg)

            self._pubPosition.publish(positionMsg)
            self._pubButton.publish(buttonMsg)

            rosRate.sleep()
            start = False


    def _setButton(self,firstcall):
        if firstcall:
            greyButton = 1
            whiteButton = 1
        else:
            greyButton = 1
            whiteButton = 1

        return PhantomButtonEvent(  grey_button=greyButton, 
                                    white_button=whiteButton   )        

    def _setLockState(self):
       

        return LockState(       lock=True,
                                lock_position=Point(0,0,0), 
                                current_position=Point(0.2,0.2,0.2)   )  
                            



if __name__ == "__main__":
	
	try:
		node = sendMockOmniPosition()
		node.run()
	except ROSInterruptException:
		pass

