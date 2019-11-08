#!/usr/bin/env python


# rospy
from rospy import init_node, is_shutdown, get_param, Rate, sleep, ROSInterruptException, Publisher, loginfo, get_time, get_rostime
# standard messages
from geometry_msgs.msg import Point
# custom msgs
from phantom_omni.msg import LockState, PhantomButtonEvent
# python packages
from numpy import sin, cos, pi



class sendMockOmniPosition():
    def __init__(self):
        init_node("mock_omni_position", anonymous=True)
        self._getParameters()
        self._cycle_i = 1  # Initialize as 1 for first cycle
        self._startTime = get_rostime() # get start time ros
        self._passedTime = 0 # initialized passed time

        self._pubPosition= Publisher(self._outputTopicPosition, LockState , queue_size=1)
        self._pubButton = Publisher(self._outputTopicButton, PhantomButtonEvent , queue_size=1)


    def _getParameters(self):
        self._outputTopicPosition = get_param("~OutputTopicPosition")
        self._outputTopicButton = get_param("~OutputTopicButton")

        self._amplitudes = get_param("~amplitudes")
        self._frequencies = get_param("~frequencies")
        self._periods = get_param("~periods")

        # self._cycleTime = get_param("~cycleTime")
        self._angle_xy = get_param("~angle_xy")


    def run(self):
        rosRate = Rate(30)

        while not is_shutdown():
            buttonMsg = self._setButton()
            positionMsg = self._setLockState()

            # loginfo(buttonMsg)
            # loginfo(positionMsg)

            self._pubPosition.publish(positionMsg)
            self._pubButton.publish(buttonMsg)

            rosRate.sleep()


    def _setButton(self):

        return PhantomButtonEvent(  grey_button=1, 
                                    white_button=1   )        


    def _setLockState(self):
        output_point = self._makeCyclingSignal(self._amplitudes, self._frequencies)

        
        if self._angle_xy != 0:     # make ofdiagonal wiggle in x-y plane
            x = output_point*cos(self._angle_xy*(pi/180))
            y = output_point*sin(self._angle_xy*(pi/180))
            z = 0
        else:                       # make circel in x-y plane
            x = output_point
            y = output_point
            z = 0

        return LockState(       lock=True,
                                lock_position=Point(0,0,0), 
                                current_position=Point(x,y,z)   )  
                            

    def _makeCyclingSignal(self, amplitudes, frequencies):
        current_time = get_time()
        seconds = current_time - self._startTime.secs
        cycles = len(amplitudes)

        # get amplitude, frequency, 
        index_arr = (self._cycle_i % cycles)-1
        amplitude = amplitudes[index_arr]
        frequency = frequencies[index_arr]

        # get time for sinus_i
        onePeriodTime = 1/frequencies[index_arr]
        multiplePeriodTime = onePeriodTime*self._periods[index_arr]

        # make signal
        output = self._makeSinus(seconds,amplitude,frequency)

        if seconds>(multiplePeriodTime + self._passedTime):          # update position in array
            self._cycle_i = self._cycle_i + 1 
            self._passedTime = seconds

        return output


    def _makeSinus(self,time,amplitude,freq):
        sinus = amplitude*sin(freq*time)

        return sinus



if __name__ == "__main__":
	
	try:
		node = sendMockOmniPosition()
		node.run()
	except ROSInterruptException:
		pass

