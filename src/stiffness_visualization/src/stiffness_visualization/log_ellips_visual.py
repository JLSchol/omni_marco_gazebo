#!/usr/bin/env python

# ros classes
from rospy import init_node, Publisher, Subscriber, is_shutdown, Rate, loginfo, spin, Duration
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

from tf2_ros import TransformListener, Buffer, LookupException
from tf2_ros import ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_multiply

from stiffness_visualization.msg import LogEllips


class DrawEllipsoid(object):
    def __init__(self):
        init_node("draw_ellipsoid", anonymous=True)

        self._getParameters()

        self._pub_log = Publisher(str(self._nodeName+self._log_topic), LogEllips, queue_size=50)
        # self._pub_ee = Publisher(str(self._nodeName+self._ellips_ee_topic), Marker, queue_size=50)
        # self._pub_other = Publisher(str(self._nodeName+self._ellips_other_topic), Marker, queue_size=50)

        self._sub_cov = Subscriber(self._cov_eigen_pair_name, EigenPairs, self._cov_cb)
        self._sub_stiff = Subscriber(self._stiff_eigen_pair_name, EigenPairs, self._stiff_cb)
        
        self._cov_pair = []
        self._stiff_pair = []

        #tf
        self._tfBuffer = Buffer()
        listener = TransformListener(self._tfBuffer)

    def _getParameters(self):
        self._nodeName = get_param("~node_name")
        # input topics
        self._cov_eigen_pair_name = get_param("~cov_eigen_pair")
        self._stiff_eigen_pair_name = get_param("~stiff_eigen_pair")
        # output topics
        self._log_topic = get_param("~draw_ellips_log")
        self._ellips_ee_topic = get_param("~ellips_ee")
        self._ellips_other_topic = get_param("~ellips_other")

        self._base_frame = get_param("~source_frame")
        self._ee_frame = get_param("~target_frame")
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
            
     
    def _cov_cb(self, message):
        self._cov_pair = message

    def _stiff_cb(self, message):
        self._stiff_pair = message


    def run(self):

        ellipsoidMsgClass = EllipsoidMessage()
        
        # publishRate = 30 # Hz
        # rosRate = Rate(publishRate)
        while not is_shutdown():
            if not self._cov_pair:  
                continue
            if not self._stiff_pair:  
                continue

            # convert to workable numpy arrays
            (cov_vec,cov_val) = ellipsoidMsgClass.EigenPairMsgsToMatrixVector(self._cov_pair)
            (stiff_vec,stiff_val) = ellipsoidMsgClass.EigenPairMsgsToMatrixVector(self._stiff_pair)

            rightHanded1 = ellipsoidMsgClass.checkRightHandedNessMatrix(cov_vec)
            rightHanded2 = ellipsoidMsgClass.checkRightHandedNessMatrix(stiff_vec)
            if(rightHanded1==False) or (rightHanded2==False):
                loginfo("not a right handed frame!!!!!!")


            quaternion = ellipsoidMsgClass.getQuatFromMatrix(cov_vec)
            scales = ellipsoidMsgClass.getEllipsoidScales(cov_val,self._lambda_min,self._lambda_max)

            # # ispect msgs in rviz to see if correct
            # z_offset = 0.5 # use offset to plot above the ee frame for comparison
            # ellipsoid_ee = ellipsoidMsgClass.getEllipsoidMsg(self._cov_pair.header.frame_id
            #                                                 ,"ellipsoid_wrt_ee",0,[-z_offset,0,0],
            #                                                 quaternion,scales,[1,0.3,1,0.3])


            # express quaterionion (ellips in ee Frame) in other frame
            # find the pose of the ee expressed in base
            ee_in_base_frame = self.listenToTransform(self._base_frame,self._ee_frame) 
            # print(ee_in_base_frame)
            t = ee_in_base_frame.transform.translation
            r = ee_in_base_frame.transform.rotation
            pee_in_base = [t.x, t.y, t.z]
            qee_in_base = [r.x, r.y, r.z, r.w]
            # also find marker in base because fuck it
            marker_in_base = self.listenToTransform(self._base_frame,'virtual_marker') 
            marker_in_ee = self.listenToTransform(self._ee_frame,'virtual_marker')

            # rotate ellipsoid quaterion with ee quaterion
            q_ellips_in_base = quaternion_multiply(qee_in_base,quaternion) # pre because 
            other_q = quaternion_multiply(quaternion,qee_in_base) # for checking

            # # create msgs for testing
            # ellipsoid_base = ellipsoidMsgClass.getEllipsoidMsg(self._base_frame
            #                                                 ,"ellipsoid_wrt_base",0,
            #                                                 [pee_in_base[0],pee_in_base[1],pee_in_base[2]],# + 2*z_offset],
            #                                                 q_ellips_in_base,scales,[1,0.3,1,0.3])


            logmsg = self._setLogMsg(self._base_frame,pee_in_base,q_ellips_in_base,scales,
                                    pee_in_base, qee_in_base, pee_in_base, q_ellips_in_base,
                                    marker_in_base.transform.translation, marker_in_ee.transform.translation,
                                    self._cov_pair, self._stiff_pair,
                                    other_q)

            # # publish ellipsod in endeffector frame
            # # inspection
            # self._pub_ee.publish(ellipsoid_ee)
            # self._pub_other.publish(ellipsoid_base)
            # actual logger
            self._pub_log.publish(logmsg)

            # # # check with broadcaster
            # # ee
            # self._broadcastEllipsoidAxis([0,0,0],quaternion,'wrist_ft_tool_link','ellips_wrt_ee')
            # # # base
            # self._broadcastEllipsoidAxis([ pee_in_base[0] ,pee_in_base[1],pee_in_base[2] ], 
            # q_ellips_in_base,'base_footprint','ellips_wrt_base' ) #, + 2*z_offset],



    def listenToTransform(self, targetFrame, sourceFrame):      
        try:        # lookup_transform(from this frame, to this frame)
            trans = self._tfBuffer.lookup_transform(targetFrame, sourceFrame, 
                                                            Time(), Duration(30))
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return trans


    def _setLogMsg(self,frame_id,center,quaternions,scales, 
                    ee_pos, ee_quat, ellips_pos, ellips_quat,
                    marker_in_base, marker_in_ee,
                    cov_pair,stiff_pair, 
                    other_q):
        cov_values = []
        cov_vecs = []
        for pair in cov_pair.pairs:
            cov_values.append(pair.eigen_value)
            cov_vecs.append(pair.eigen_vector)


        msg = LogEllips()
        msg.header.frame_id = frame_id
        msg.header.stamp = Time.now()

        # used fro plotting pyplot
        msg.center.x = center[0]
        msg.center.y = center[1]
        msg.center.z = center[2]
        msg.quaternion.x = quaternions[0]
        msg.quaternion.y = quaternions[1]
        msg.quaternion.z = quaternions[2]
        msg.quaternion.w = quaternions[3]
        msg.scales.x = scales[0] 
        msg.scales.y = scales[1]
        msg.scales.z = scales[2] 

        # ee pose
        msg.ee_position.x = ee_pos[0]
        msg.ee_position.y = ee_pos[1]
        msg.ee_position.z = ee_pos[2]
        msg.ee_orientation.x = ee_quat[0]
        msg.ee_orientation.y = ee_quat[1]
        msg.ee_orientation.z = ee_quat[2]
        msg.ee_orientation.w = ee_quat[3]
   
        # ellipsoid pose
        msg.ellipsoid_position.x = ellips_pos[0]
        msg.ellipsoid_position.y = ellips_pos[1]
        msg.ellipsoid_position.z = ellips_pos[2]
        msg.ellipsoid_orientation.x = ellips_quat[0]
        msg.ellipsoid_orientation.y = ellips_quat[1]
        msg.ellipsoid_orientation.z = ellips_quat[2]
        msg.ellipsoid_orientation.w = ellips_quat[3]
    
        # virtual marker position in base frame
        msg.marker_in_base.x = marker_in_base.x
        msg.marker_in_base.y = marker_in_base.y
        msg.marker_in_base.z = marker_in_base.z

        # virtual marker position in ee frame
        msg.marker_in_ee.x = marker_in_ee.x
        msg.marker_in_ee.y = marker_in_ee.y
        msg.marker_in_ee.z = marker_in_ee.z

        msg.covariance_eig = cov_values
        msg.stiffness_eig = [pair.eigen_value for pair in  stiff_pair.pairs]

        msg.v1 = cov_vecs[0]
        msg.v2 = cov_vecs[1]
        msg.v3 = cov_vecs[2]

        # another quaternions
        msg.other_quat.x = other_q[0]
        msg.other_quat.y = other_q[1]
        msg.other_quat.z = other_q[2]

        return msg


    def _broadcastEllipsoidAxis(self,position,quaternion,parentID,childID):
        br = TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = Time.now()
        t.header.frame_id = parentID
        t.child_frame_id = childID
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        br.sendTransform(t)



if __name__ == "__main__":
    try:
        node = DrawEllipsoid()
        node.run()
        spin()

    except ROSInterruptException:
        pass