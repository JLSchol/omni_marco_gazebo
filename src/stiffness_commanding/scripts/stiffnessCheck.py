#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

######## input perturbation signal ########
dx = [0.1,0,0] # x axis
dx_signal = np.linspace(0,dx[0],1000)

######## create stiffness matrix ########
# robot stiffness limits
K_min = 100
K_max = 1000
# rotation component
angle = 0
rot_ax = 'z'
angle_signal = np.linspace(0,90,1000)
# size component
k_e1_signal = np.linspace(K_min, K_max)
# k_e1 = (K_min+K_max)/2.0 # varied axis corresponding with x
# k_e1 = K_min 	# varied axis corresponding with x
k_e1 = K_max 	# varied axis corresponding with x
k_e2 = 100.0
k_e3 = 100.01

######### tune parameters ########
fmax = 3.3
fmin = 0.33
xmax = 0.1
Khd_min = fmin/xmax
Khd_max = fmax/xmax
# #uncomment if manual setting
# Khd_min = 1.00
# Khd_max = 10





toDeg = lambda x:  x* np.pi/180.0
toRad = lambda x:  x* 180.0/np.pi

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

def stiffnessFromEig(V,E):
	return V.dot(E.dot(V.T))

def diagonalMatrixFromValues(eigenList):
	return np.diag([lambda_i for lambda_i in eigenList])

def createStiffnessMatrixFromAngle(angle, rot_ax, k_e1,k_e2,k_e3 ):
	# find stiffness matrix
	# define rotation
	R_in = rotationMatrixFromAngle(angle,rot_ax)
	# define diagonals
	Ke_in = diagonalMatrixFromValues([k_e1,k_e2,k_e3])
	# create stiffess matrix
	K = stiffnessFromEig(R_in, Ke_in)
	return K

def scaleStiffness(K,Khd_min,Khd_max,K_min,K_max):
	# decompose stiffness matrix
	gamma, R = np.linalg.eig(K)
	gamma = np.diag(gamma)
	# scale stiffness eigenvalues
	Kse = np.diag([Khd_min,Khd_min,Khd_min]) + (Khd_max - Khd_min)/(K_max-K_min) * (gamma - np.diag([K_min,K_min,K_min]))
	# rotate back
	Ks = R.dot(Kse.dot(R.T))
	return Ks

def plotStiffness(angles, rot_ax, k_e1_signal,k_e2,k_e3):
	for angle in angles:
		kxx, kyy, kzz = [], [], []
		ksxx, ksyy, kszz = [], [], []
		for k_e1 in k_e1_signal:
			# find stiffness matrix
			K = createStiffnessMatrixFromAngle(angle, rot_ax, k_e1,k_e2,k_e3 )
			# scale stiffness matrix
			Ks = scaleStiffness(K,Khd_min,Khd_max,K_min,K_max)
			#append
			kxx.append(K[0,0])
			kyy.append(K[1,1])
			kzz.append(K[2,2])
			ksxx.append(Ks[0,0])
			ksyy.append(Ks[1,1])
			kszz.append(Ks[2,2])


		fig_K, ax_K = plt.subplots(2)
		ax_K[0].set_title('Stiffness diagonal comparison for {} deg rotation'.format(angle))
		ax_K[0].set_ylabel('stiffness diagonal [N/m]')



		ax_K[1].set_xlabel("Stiffness eigenvalue " + r'$\gamma_{1}$' +" [N/m]")
		ax_K[1].set_ylabel('scaled stiffness diagonal [N/m]')

		ax_K[0].plot(k_e1_signal,kxx, label='kxx', lw=3, ls=':')
		ax_K[0].plot(k_e1_signal,kyy, label='kyy', lw=3, ls='--')
		ax_K[0].plot(k_e1_signal,kzz, label='kzz', lw=3)
		ax_K[0].set_ylim(0,1100)

		ax_K[1].plot(k_e1_signal,ksxx, label='kxx', lw=3, ls=':')
		ax_K[1].plot(k_e1_signal,ksyy, label='kyy', lw=3, ls='--')
		ax_K[1].plot(k_e1_signal,kszz, label='kzz', lw=3)
		ax_K[1].set_ylim(0,40)

		ax_K[0].legend(loc='best')
		ax_K[1].legend(loc='best')
	

def plotForce(dx_signal, angles, rot_ax, k_e1,k_e2,k_e3, Khd_min,Khd_max,K_min,K_max):
	fig_K, ax = plt.subplots(len(angles))
	for i,angle in enumerate(angles):
		fx, fy, fz = [], [], []
		for dx_i in dx_signal:
			# find stiffness matrix
			K = createStiffnessMatrixFromAngle(angle, rot_ax, k_e1,k_e2,k_e3 )
			# scale stiffness matrix
			Ks = scaleStiffness(K,Khd_min,Khd_max,K_min,K_max)

			# calculate force
			f = Ks.dot([dx_i,0,0])
			fx.append(f[0])
			fy.append(f[1])
			fz.append(f[2])

		# if i == 0:
		ax[0].set_title('Force respons from perturbation for 0,45,90 degree rotated ellipsoid')
		ax[len(angles)-1].set_xlabel('perturbation in x-direction [m]')
		ax[i].set_ylabel('force [N]')
		print(len(fx))
		print(len(dx_signal))
		# print(dx_signal)
		ax[i].plot(dx_signal,fx, label='fx', lw=3, ls=':')
		ax[i].plot(dx_signal,fy, label='fy', lw=3, ls='--')
		ax[i].plot(dx_signal,fz, label='fz', lw=3)
		ax[i].set_ylim(-0.1,3.5)
		ax[i].legend()



# def scaleStiffnessMatrix(K)


# find stiffness matrix
K = createStiffnessMatrixFromAngle(angle, rot_ax, k_e1,k_e2,k_e3 )

# scale stiffness matrix
Ks = scaleStiffness(K,Khd_min,Khd_max,K_min,K_max)

# calculate force
f = Ks.dot(dx)

print('Stiffness scaling check')
K = np.round(K,2)
Ks = np.round(Ks,2)
print(K)
print(Ks)
print(' ')

print('force')
f = np.round(f,2)
print(dx)
print(f)

######### PLOTS###########3
plotForce(dx_signal, [0,45,90], rot_ax, k_e1,k_e2,k_e3, Khd_min,Khd_max,K_min,K_max)
plotStiffness([0,45,90], rot_ax, k_e1_signal,k_e2,k_e3)
plt.show()


# ax_K.plot(angles,kxs, label='fy')
# ax_K.plot(angles,kxs, label='fz')

# fig_K, ax_K = plt.subplots()
# ax_K.set_title('Stiffness diagonal (x,y,z) along ellips orientation')
# ax_K.set_xlabel('angle [deg] around ' + rotation_axis + ' axis')
# ax_K.set_ylabel('stiffness diagonal [N/m]')
# ax_K.plot(angles,kxs, label='kx')
# ax_K.plot(angles,kys, label='ky')
# ax_K.plot(angles,kzs, label='ky')
# ax_K.axhline(y=kxs[0],xmin=0,xmax=1,label='kx_0deg',linestyle='--',color='C0')
# ax_K.axhline(y=kys[0],xmin=0,xmax=1,label='ky_0deg',linestyle='--',color='C1')
# ax_K.axhline(y=kzs[0],xmin=0,xmax=1,label='kz_0deg',linestyle='--',color='C2')
# ax_K.legend()

# fig_F, ax_F = plt.subplots()
# ax_F.set_title('Force respons from perturbation in x-direction along ellips orientation')
# ax_F.set_xlabel('angle [deg] around ' + rotation_axis + ' axis')
# ax_F.set_ylabel('Force [N]')
# ax_F.plot(angles,Fs)
# ax_F.axhline(y=Fs[0][0],xmin=0,xmax=1,linestyle='--',color='C0')
# ax_F.axhline(y=Fs[0][1],xmin=0,xmax=1,linestyle='--',color='C1')
# ax_F.axhline(y=Fs[0][2],xmin=0,xmax=1,linestyle='--',color='C2')
# ax_F.legend(['Fx','Fy','Fz','Fx_0deg','Fy_0deg','Fz_0deg'])

# print
# print('Rotation mat check')
# R_in = np.round(R_in,2)
# R = np.round(R,2)
# print(R_in)
# print(R)

# print('Stiffness eig check')
# Ke_in = np.round(Ke_in,2)
# gamma = np.round(gamma,2)
# print(Ke_in)
# print(gamma)
# print(' ')

# print('Stiffness eig scaling check')
# Kse = np.round(Kse,2)
# print(gamma)
# print(Kse)
# print(' ')

# print('Stiffness scaling check')
# K = np.round(K,2)
# Ks = np.round(Ks,2)
# print(K)
# print(Ks)
# print(' ')

# print('force')
# f = np.round(f,2)
# print(dx)
# print(f)





