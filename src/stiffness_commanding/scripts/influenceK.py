import numpy as np
import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as Rot
import sys




ref_angle = 0 	# reference/start orientation

eigenValues = [1, 0, 0] # if list has length 3, assume 3D rotation and specify rotation axis
rotation_axis = 'z' # if 2D specify None
dX = [1,0,0] # pertubation vector on stiffness matrix len(dX) needs to match len(eigenValues)

# eigenValues = [1, 0] # if list has length 3, assume 3D rotation and specify rotation axis
# rotation_axis = None # if 2D specify None
# dX = [1,0] # pertubation vector on stiffness matrix len(dX) needs to match len(eigenValues)

# set plots on or of
plot_angle = True
plot_shape = False

# print K,F and angle for given angles
print_angles = [0,5,15,30,45]
# print_angles = False

toDeg = lambda x:  x* np.pi/180.0
toRad = lambda x:  x* 180.0/np.pi

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def rotationMatrixFromAngle(angle, axis=None):
	c = np.cos(toDeg(angle))
	s = np.sin(toDeg(angle))
	if not axis: # Rotation in 2D rotation always rotates around 'z-axis'
		return np.array([[c,-s],[s,c]])
	if axis =='x':
		return np.array([ [1,0,0], [0,c,-s], [0,s,c] ])
	elif axis =='y':
		return np.array([ [c,0,s], [0,1,0], [-s,0,c] ])
	elif axis =='z':
		return np.array([ [c,-s,0], [s,c,0], [0,0,1] ])
	else:
		print("{} is not a valid axis argument. try as second argument None, 'x','y' or 'z'")


def rotationMatrixFromAxisAngle(axis, angle):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by angle radians.
    """
    angle = toRad(angle)
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(angle / 2.0)
    b, c, d = -axis * np.sin(angle / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def stiffnessFromEig(V,E):
	return V.dot(E.dot(V.T))

def diagonalMatrixFromValues(eigenList):
	return np.diag([lambda_i for lambda_i in eigenList])

def getDiagonalsMatrix(K):
	diagonals = np.diag(K).tolist()
	if len(diagonals) == 2:
		diagonals = diagonals + [None]
	return np.array(diagonals)
	# return np.diag(K)

def calcForce(K,dX):
	return K.dot(dX)

def printInstances(display_angles, angles, Ks, Fs):
	if not display_angles:
		return 0
	for angle in display_angles:
		
		nearest_angle, index = find_nearest(angles, angle)
		print('requested angle: {}\nnearest angle: {}'.format(angle,nearest_angle))
		# i = angles.tolist().index(angle)
		print('angle: {} \nstiffness:\n {} \nForce: {}\n\n'.format(nearest_angle,Ks[index],Fs[index]))

# generate list of angles and errors

# Get reference eigenValues, stiffness matrix and Force

#check
# print(rotationMatrixFromAngle(15,'x'))
# print(rotationMatrixFromAxisAngle([1,0,0],15))
# print(Rot.from_euler('x', 15, degrees=True).as_dcm())
# print(Rot.from('x', 15, degrees=True).as_dcm())
# sys.exit()


E = diagonalMatrixFromValues(eigenValues)
# ref K
K_ref = stiffnessFromEig(rotationMatrixFromAngle(ref_angle,rotation_axis), E)
F_ref = calcForce(K_ref, dX)

############## get arrays and plots by rotating the angle ###############
# other Ks
angles = np.linspace(0, 90, 1000) # varied angles
Ks, Fs, kxs, kys, kzs = [], [], [], [], []
for angle in angles:
	K = stiffnessFromEig(rotationMatrixFromAngle(angle,rotation_axis),E)
	[kx, ky, kz] = getDiagonalsMatrix(K)
	kxs.append(kx)
	kys.append(ky)
	kzs.append(kz)
	Ks.append(K)
	Fs.append(calcForce(K,dX))
printInstances(print_angles, angles, Ks, Fs)
# print(kzs)

if plot_angle:
	fig_K, ax_K = plt.subplots()
	ax_K.set_title('Stiffness diagonal (x,y,z) along ellips orientation')
	ax_K.set_xlabel('angle [deg] around ' + rotation_axis + ' axis')
	ax_K.set_ylabel('stiffness diagonal [N/m]')
	ax_K.plot(angles,kxs, label='kx')
	ax_K.plot(angles,kys, label='ky')
	ax_K.plot(angles,kzs, label='ky')
	ax_K.axhline(y=kxs[0],xmin=0,xmax=1,label='kx_0deg',linestyle='--',color='C0')
	ax_K.axhline(y=kys[0],xmin=0,xmax=1,label='ky_0deg',linestyle='--',color='C1')
	ax_K.axhline(y=kzs[0],xmin=0,xmax=1,label='kz_0deg',linestyle='--',color='C2')
	ax_K.legend()

	fig_F, ax_F = plt.subplots()
	ax_F.set_title('Force respons from perturbation in x-direction along ellips orientation')
	ax_F.set_xlabel('angle [deg] around ' + rotation_axis + ' axis')
	ax_F.set_ylabel('Force [N]')
	ax_F.plot(angles,Fs)
	ax_F.axhline(y=Fs[0][0],xmin=0,xmax=1,linestyle='--',color='C0')
	ax_F.axhline(y=Fs[0][1],xmin=0,xmax=1,linestyle='--',color='C1')
	ax_F.axhline(y=Fs[0][2],xmin=0,xmax=1,linestyle='--',color='C2')
	ax_F.legend(['Fx','Fy','Fz','Fx_0deg','Fy_0deg','Fz_0deg'])
	plt.show()


############ get arrays by increasing error ############

# shape_errors = np.linspace(0, 1, 100+1) # varies error of eigen value_1
# Ks, Fs, kxs, kys = [], [], [], []
# for error in shape_errors:

# 	Es = diagonalMatrixFromValues(L1-error,L2)
# 	R = rotationMatrixFromAngle(toDeg(0),rotation_axis)
# 	K = stiffnessFromEig(R, Es)
# 	[kx, ky] = getDiagonalsMatrix(K)
# 	kxs.append(kx)
# 	kys.append(ky)
# 	Ks.append(K)
# 	Fs.append(calcForce(K,dX))

# if plot_shape:
# 	fig_K, ax_K = plt.subplots()
# 	ax_K.set_title('Stiffness diagonal (x,y) along shape error (lambda_1)')
# 	ax_K.set_xlabel('error [-]')
# 	ax_K.set_ylabel('stiffness diagonal [N/m]')
# 	ax_K.plot(shape_errors,kxs, label='kx')
# 	ax_K.plot(shape_errors,kys, label='ky')
# 	ax_K.axhline(y=kxs[0],xmin=0,xmax=1,label='kx_0err',linestyle='--',color='C0')
# 	ax_K.axhline(y=kys[0],xmin=0,xmax=1,label='ky_0err',linestyle='--',color='C1')
# 	ax_K.legend()

# 	fig_F, ax_F = plt.subplots()
# 	ax_F.set_title('Force respons from perturbation in x-direction along shape error (lambda_1)')
# 	ax_F.set_xlabel('error [-]')
# 	ax_F.set_ylabel('Force [N]')
# 	ax_F.plot(shape_errors,Fs)
# 	ax_F.axhline(y=Fs[0][0],xmin=0,xmax=1,linestyle='--',color='C0')
# 	ax_F.axhline(y=Fs[0][1],xmin=0,xmax=1,linestyle='--',color='C1')
# 	ax_F.legend(['Fx','Fy','Fx_0err','Fy_0err'])
# 	plt.show()


