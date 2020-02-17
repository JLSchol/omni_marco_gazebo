# #!/usr/bin/env python

# # rospy
# from rospy import init_node, Publisher, Subscriber, is_shutdown, loginfo, sleep, spin
# from rospy import ROSInterruptException, Time, get_param, has_param, logwarn, logfatal


# # custom class
# from ellipsoid_message import EllipsoidMessage
# #import messages
# from visualization_msgs.msg import Marker
# #custom messages
# from stiffness_commanding.msg import EigenPairs

# class MarkerColour(object):
#     def __init__(self):
#         # init node
#         init_node("colour_marker", anonymous=True)
#         # get params
#         lambdaminPar = 'stiffness_commanding/lambda_min' #need to fix this hardcoded part!
#         lambdamaxPar = 'stiffness_commanding/lambda_max'
#         if not has_param(lambdaminPar):
#             logfatal("Could not retrive %s from the parameter server", lambdaminPar)
#         if not has_param(lambdamaxPar):
#             logfatal("Could not retrive %s from the parameter server", lambdamaxPar)
#         self._lambda_min = get_param("/stiffness_commanding/lambda_min")
#         self._lambda_max = get_param("/stiffness_commanding/lambda_max")
#         # subs
#         self._markerSub = Subscriber("/marker_visualization", Marker, self._markerCB)
#         self._eigenSub = Subscriber("/eigen_pair", EigenPairs, self._eigenCB)
#         # pub
#         self._markerPub = Publisher("/marker_visualization_coloured", Marker, queue_size=50)
#         # init class attributes
#         self._marker = []
#         self._eigenPair = []



#     def _markerCB(self,message):
#         self._marker = message
        
    
#     def _eigenCB(self,message):
#         self._eigenPair = message


#     def run(self):
#         if( (not self._eigenPair == True) or (not self._marker == True) ): 
#             pass
#         else:
#             helperClass = EllipsoidMessage()
#             (eigVectors,eigValues) = helperClass.EigenPairMsgsToMatrixVector(self._eigenPair)
#             scales = ellipsoidMsgClass.getEllipsoidScales(eigValues,self._lambda_min,self._lambda_max)


# if __name__ == "__main__":
#     try:
#         node = MarkerColour()

#         while not is_shutdown():
#             node.run()
#             spin()

#     except ROSInterruptException:
#         pass