#!/usr/bin/env python

import numpy as np

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



fmax = 3.3
fmin = 0.0
xmax = 0.1
dx = 0.1

# Khd_max = fmax/xmax
# Khd_min = fmin/xmax

Khd_min = 1.001
Khd_max = 10

K_min = 100
K_max = 1000

k = (K_min+K_max)/2.0
# k = K_max
# k = K_min

# Km = diagonalMatrixFromValues([100,200,500])
Km = diagonalMatrixFromValues([200,450,700])
R = rotationMatrixFromAngle(30,'x')
Kmr = stiffnessFromEig(R, Km)

print(Km)
# print(R)
print(Kmr)
print('')

val, vec = np.linalg.eig(Kmr)
# vec=vec.T
val = np.diag(val)



kmr_check = stiffnessFromEig(vec, val)
# print(vec)
# print(val)
# print(kmr_check)
# print('')


k1 = k


# np.linalg(Km)

# Khdm = Khd_min + (Khd_max - Khd_min)/(K_max-K_min) * (Km - K_min)
Khdm = np.diag([Khd_min,Khd_min,Khd_min]) + (Khd_max - Khd_min)/(K_max-K_min) * (Kmr - np.diag([K_min,K_min,K_min]))
Khdme = np.diag([Khd_min,Khd_min,Khd_min]) + (Khd_max - Khd_min)/(K_max-K_min) * (Km - np.diag([K_min,K_min,K_min]))
khdmer = stiffnessFromEig(R, Khdme)
Khd1 = Khd_min + (Khd_max - Khd_min)/(K_max-K_min) * (k1 - K_min)


fm = Khdm*dx
f1 = Khd1*dx

# print(Khd_min)
# print(Khd_max)
# print()
# print(Khdm)
print(Khdme)
print(khdmer)
# print(Khd1)
print()
# print(fm)
# print(f1)