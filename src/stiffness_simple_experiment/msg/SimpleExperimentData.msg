# every trial of the experiment should generate 1 experiment data msg
# This makes the analysis easier because all the fundamental info is in this msg

std_msgs/Header header

int16 trial_nr 

float32 trial_time 
float32 shape_acc
float32 orientation_acc

float32 shape                       # average error between principle axis (radius)
float32 absolute_angle

geometry_msgs/Point user_scales 
geometry_msgs/Quaternion user_orientation

geometry_msgs/Point experiment_scales
geometry_msgs/Quaternion experiment_orientation

geometry_msgs/Point original_user_scales 
geometry_msgs/Quaternion original_user_orientation



