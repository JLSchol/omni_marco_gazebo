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

        self._angle_xy = get_param("~angle_xy")
        self._angle_xy = float(self._angle_xy)

    def run(self):
        rosRate = Rate(60)

        while not is_shutdown():
            buttonMsg = self._setButton()
            positionMsg = self._setLockState()

            self._pubPosition.publish(positionMsg)
            self._pubButton.publish(buttonMsg)

            rosRate.sleep()


    def _setButton(self):

        return PhantomButtonEvent(  grey_button=1, 
                                    white_button=1   )        


    def _setLockState(self):
        # get data from sinus_i
        output_point = self._makeCyclingSignal(self._amplitudes, self._frequencies)

        # make ofdiagonal wiggle in x-y plane
        # if self._angle_xy != 0:     
        x = 0#output_point*cos(self._angle_xy*(pi/180))#output_point*cos(self._angle_xy*(pi/180))#output_point*cos(self._angle_xy*(pi/180))#output_point*cos(self._angle_xy*(pi/180)) # output_point*cos(self._angle_xy*(pi/180))
        y = output_point*cos(self._angle_xy*(pi/180))#output_point*cos(self._angle_xy*(pi/180))#output_point*sin(self._angle_xy*(pi/180)) #output_point*sin(self._angle_xy*(pi/180)) 
        z = output_point*sin(self._angle_xy*(pi/180))#output_point*sin(self._angle_xy*(pi/180))#0 #output_point*sin(self._angle_xy*(pi/180))
            # loginfo("x =" + str(x))
            # loginfo("y =" + str(y))
            # loginfo("z =" + str(z))
        # make circel in x-y plane    
        # else:                       
        #     x = output_point
        #     y = output_point
        #     z = 0

        return LockState(       lock=True,
                                lock_position=Point(0,0,0), 
                                current_position=Point(x,y,z)   )  
                            

    def _makeCyclingSignal(self, amplitudes, frequencies):
        # initialize time
        current_time = get_time()
        seconds = current_time - self._startTime.secs

        # get amount of sinusoides
        n_sinusoids = len(amplitudes)

        # get index of sinus_i
        sinus_i = ((self._cycle_i-1) % n_sinusoids)

        # get amplitude, frequency, 
        amplitude = float(amplitudes[sinus_i])
        frequency = float(frequencies[sinus_i])

        # get duration of time for sinus_i to run
        periodTime = 1/frequency
        runTimeSinus_i = periodTime*float(self._periods[sinus_i]) 

        # make signal
        output = self._makeSinus(seconds,amplitude,frequency)

        # update to sin_(i+1) when runtime of sin_i has passed
        if seconds>(runTimeSinus_i + self._passedTime):          
            self._cycle_i = self._cycle_i + 1 
            self._passedTime = self._passedTime + runTimeSinus_i

        return output


    def _makeSinus(self,time,amplitude,freq):
        angular_frequency = 2*pi*freq
        sinus = amplitude*sin( angular_frequency * time )

        return sinus



if __name__ == "__main__":
	
	try:
		node = sendMockOmniPosition()
		node.run()
	except ROSInterruptException:
		pass

