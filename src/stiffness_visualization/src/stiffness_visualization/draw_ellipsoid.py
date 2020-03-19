#!/usr/bin/env python

# ros classes
from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, spin
from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal
from tf2_ros import TransformBroadcaster
# custom class
from ellipsoid_message import EllipsoidMessage
#import messages
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
#custom messages
from stiffness_commanding.msg import EigenPairs

from std_msgs.msg import Float32MultiArray



class DrawEllipsoid(object):
    def __init__(self):
        init_node("draw_ellipsoid", anonymous=True)

        self._getParameters()

        self._publisher = Publisher(str(self._nodeName+self._outputTopicName), Marker, queue_size=50)
        self._subscriber = Subscriber(self._inputTopicName, EigenPairs, self._callback)
        
        self._eigenPair = []


    def _getParameters(self):
        # input output topic
        self._inputTopicName = get_param("~input_topic_name")
        self._outputTopicName = get_param("~output_topic_name")
        self._nodeName = get_param("~node_name")
        # get wiggle max and min from parameter server
        # Check if available from parameter server!
        lambdaminPar = 'stiffness_commanding/lambda_min'
        lambdamaxPar = 'stiffness_commanding/lambda_max'
        if not has_param(lambdaminPar):
            logfatal("Could not retrive %s from the parameter server", lambdaminPar)
        if not has_param(lambdamaxPar):
            logfatal("Could not retrive %s from the parameter server", lambdamaxPar)
        
        self._lambda_min = get_param("/stiffness_commanding/lambda_min")
        self._lambda_max = get_param("/stiffness_commanding/lambda_max")
            
     




    def _callback(self, message):
        self._eigenPair = message


    def run(self):

        ellipsoidMsgClass = EllipsoidMessage()
        
        # publishRate = 30 # Hz
        # rosRate = Rate(publishRate)
        while not is_shutdown():
            if not self._eigenPair:  
                continue

            (eigVectors,eigValues) = ellipsoidMsgClass.EigenPairMsgsToMatrixVector(self._eigenPair)

            rightHanded = ellipsoidMsgClass.checkRightHandedNessMatrix(eigVectors)
            if(rightHanded==False):
                loginfo("not a right handed frame!!!!!!")

            quaternion = ellipsoidMsgClass.getQuatFromMatrix(eigVectors)

            scales = ellipsoidMsgClass.getEllipsoidScales(eigValues,self._lambda_min,self._lambda_max)

            ellipsoid = ellipsoidMsgClass.getEllipsoidMsg(self._eigenPair.header.frame_id
                                                            ,"ellipsoid",0,[0,0,0],
                                                            quaternion,scales,[1,0.3,1,0.3])

            self._publisher.publish(ellipsoid)
            # publish ellipsoid orientation
            ellipsoidMsgClass.broadcastEllipsoidAxis([0,0,0],quaternion,self._eigenPair.header.frame_id,"Raw_user_ellips")
            # loginfo(10*"---")


    # def _broadcastEllipsoidAxis(self,position,quaternion,parentID,childID):
    #     br = TransformBroadcaster()
    #     t = TransformStamped()

    #     t.header.stamp = Time.now()
    #     t.header.frame_id = parentID
    #     t.child_frame_id = childID
    #     t.transform.translation.x = position[0]
    #     t.transform.translation.y = position[1]
    #     t.transform.translation.z = position[2]

    #     t.transform.rotation.x = quaternion[0]
    #     t.transform.rotation.y = quaternion[1]
    #     t.transform.rotation.z = quaternion[2]
    #     t.transform.rotation.w = quaternion[3]

    #     br.sendTransform(t)



if __name__ == "__main__":
    try:
        node = DrawEllipsoid()
        node.run()
        spin()

    except ROSInterruptException:
        pass