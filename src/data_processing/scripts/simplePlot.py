import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# paths
path2folder = "/home/jasper/omni_marco_gazebo/src/data_processing/data/202001101040_D10_W100_L0.01_0.45_S100_1000"

stiffnessFile = "stiffness_command.csv"
covarianceFile = "covariance_matrix.csv"
tfFile = "tf.csv"
tf_static = "tf_static.csv"
forceFile = "omni1_force_feedback.csv"

# /home/jasper/omni_marco_gazebo/src/data_processing/data/202001101040_D10_W100_L0.01_0.45_S100_1000


stiffnessPD = pd.read_csv(path2folder+"/"+stiffnessFile)
covariancePD = pd.read_csv(path2folder+"/"+covarianceFile)
tfPD = pd.read_csv(path2folder+"/"+tfFile)
tf_staticPD = pd.read_csv(path2folder+"/"+tf_static)
forcePD = pd.read_csv(path2folder+"/"+forceFile)


print(stiffnessPD)
firstTime = stiffnessPD['%time'][0]
lastTime = stiffnessPD['%time'][203]
timeVec = map(lambda x: round((x-firstTime)*float(10**(-9)),2),  stiffnessPD['%time'])
Kxx = stiffnessPD['field.data0']
Kyy = stiffnessPD['field.data4']
Kzz = stiffnessPD['field.data8']


print(tfPD)
virtual_marker = tfPD[tfPD['field.transforms0.child_frame_id'] == 'virtual_marker']

markerTimeVec = virtual_marker['%time']

x = map(lambda x: x-1,  virtual_marker['field.transforms0.transform.translation.x'])
y = map(lambda x: x-1,  virtual_marker['field.transforms0.transform.translation.y'])
z = map(lambda x: x-1,  virtual_marker['field.transforms0.transform.translation.z']); 



print(x)


# start stiffness: 1578649205 614104281	>	start tf: 1578649205 398765016
# end stiffness:   1578649215 124248465	<	end tf:   1578649215 153630004
	
# print(tfPD['field.transforms0.header.frame_id'])