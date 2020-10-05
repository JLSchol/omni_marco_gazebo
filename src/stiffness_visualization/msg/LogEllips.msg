# ellipsoid msgs

std_msgs/Header header

# used for plotting ellips 
geometry_msgs/Point center 
geometry_msgs/Quaternion quaternion
geometry_msgs/Vector3 scales

# log ee pose wrt base
geometry_msgs/Point ee_position
geometry_msgs/Quaternion ee_orientation

# log ellipsoid pose wrt base
geometry_msgs/Point ellipsoid_position
geometry_msgs/Quaternion ellipsoid_orientation

# log marker position wrt base
geometry_msgs/Point marker_in_base

# log marker position wrt ee
geometry_msgs/Point marker_in_ee

# log covarianc eigen values
float32[] covariance_eig

# log stiffness eigen values
float32[] stiffness_eig

# log stiffness and covarioant eigen vectors (in ee_frame)
float32[] v1 
float32[] v2
float32[] v3

# other quaternion
geometry_msgs/Quaternion other_quat
