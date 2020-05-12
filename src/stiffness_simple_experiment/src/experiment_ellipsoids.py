#!/usr/bin/env python
from random import randrange, shuffle, Random
import numpy as np


class ExperimentInfo(object):
	def __init__(self,experimentNr=1,PracticeRun=False):
		self.data = self.getInfo(experimentNr,PracticeRun)     

	def getInfo(self,experimentNr,PracticeRun):
		experiment = str(experimentNr) + str(PracticeRun)
		switcher={
		'1False':self.experiment1DofFrontReal,
		'2False':self.experiment2DofFrontReal,
		'3False':self.experiment1DofTopReal,
		'4False':self.experiment2DofTopReal,

		'1True':self.experiment1DofFrontPractice,
		'2True':self.experiment2DofFrontPractice,
		'3True':self.experiment1DofTopPractice,
		'4True':self.experiment2DofTopPractice,

		'0False': self.freeForAll,

		}
		if experiment in switcher:
			func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
						.format(experiment))
			self.data = func()
			return func()
		else:
			print("invalid experiment name: {}".format(experiment))

	def experiment1DofFrontReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [4, 5]  
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 1

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofTopReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [4, 5]  
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 2

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofFrontReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 5]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [4,1,1]
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [1,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofTopReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 5]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [4,1,1]
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,1,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 4

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence


	def experiment1DofFrontPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 4]  
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 5

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofTopPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 4]  
		# create size list ordered
		minSize, maxSize = [0.088, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 6

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofFrontPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [7, 3]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [5,1,1]
		# create size list ordered
		minSize, maxSize = [0.095, 0.52] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [1,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofTopPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [7, 3]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [5,1,1]
		# create size list ordered
		minSize, maxSize = [0.095, 0.52] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,1,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def freeForAll(self):
		pass


	def getShape(self,trialNr,data):
		scales = data['scale'][trialNr]
		orientation = data['orientation'][trialNr]
		return scales, orientation

	def generateRandList(self,minValue, maxValue, step, amount):
		coef = 10000.0
		minValue = int(minValue*coef)
		maxValue = int((maxValue+step)*coef)
		step = int(step*coef)

		randList = []
		stop = False
		while len(randList) < amount and stop==False:
			randomNr = randrange(minValue,maxValue,step)/coef
			# print("random nr", format(randomNr))            
			if not randList:
				randList.append(randomNr)
				continue

			if randomNr not in randList:
				randList.append(randomNr)

			if len(randList) == ((maxValue-minValue)/step):
				stop=True

		return randList


	def generateScales(self,minSize, maxSize):
		pass

	def shuffleList(self,lijst,sequence):
		lijst = np.array(lijst)
		newList = []
		if lijst.ndim == 2:
			newList = lijst[sequence,:]
		elif lijst.ndim == 1:
			newList = lijst[sequence]
		else:
			print("This function does not handle a {}dimensional lsit".format(lijst.ndim))

		return newList.tolist()

		# def create1DofSequence(self,smallMedium,mediumLarge,
		# 					minAngle, maxAngle ,step, amountInRandList, 
		# 													repetitions):
			
		# 	sizes1DOF = [smallMedium, mediumLarge, smallMedium, mediumLarge]

		# 	# Angle1 = EI.generateRandList(minAngle, maxAngle, step, amount) #0,15
		# 	# Angle2 = EI.generateRandList(minAngle+20, maxAngle+15, step, amount) # 20,30
		# 	# Angle3 = EI.generateRandList(minAngle+20+15, maxAngle+15+15, step, amount) # 35, 45
		# 	# angles1DOF = [Angle1[0], Angle1[1], Angle2[0], Angle2[1], Angle3[0], Angle3[1]]

		# 	angles1DOF = EI.generateRandList(minAngle, maxAngle, step, amountInRandList)

		# 	# shuffle zize and angles in some way
		# 	# shuffleSeq = EI.generateRandList(0, 5, 1, 6)
		# 	# shuffleSeq = [int(x) for x in shuffleSeq]
		# 	# sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
		# 	# angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


		# 	sizes1DOF = repetitions*sizes1DOF 
		# 	angles1DOF = repetitions*angles1DOF

		# 	shuffleSeq = self.generateRandList(0, 29, 1, 30)
		# 	shuffleSeq = [int(x) for x in shuffleSeq]
		# 	# sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
		# 	# angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


		# 	# # add starting angle and size to list
		# 	sizes1DOF = [mediumLarge] + sizes1DOF
		# 	angles1DOF = [0] + angles1DOF
		# 	print(sizes1DOF)
		# 	print(angles1DOF)
		# 	return sizes1DOF, angles1DOF

	def makeAngles(self,start,stop,amount):
		interval = (float(stop)-start)/amount
		angleList = [interval*i for i in range(amount)]
		return list(angleList)

	def makeSizes(self,listWithSizes, sizeOfList):
		lengthListWithSizes = len(listWithSizes)
		modCheck = sizeOfList % lengthListWithSizes
		if modCheck !=0:
			print("sizeOfList={} is not a multiple of len(listWithSizes) = {}".format(sizeOfList,lengthListWithSizes))
			return
		if not isinstance(listWithSizes, list):
			listWithSizes = [listWithSizes]

		multiplier = sizeOfList/lengthListWithSizes
		sizeList = multiplier*listWithSizes

		return sizeList

	def repeatLists(self, sizeList, angleList, repititions):

		repeatedSize = repititions*sizeList
		repeatedAngles = repititions*angleList
		return repeatedSize, repeatedAngles

	def normalizeList(self,lijst):
		norm = np.sqrt(sum([q**2 for q in lijst]))
		normalizedList = [np.array(q)/norm for q in lijst]
		return normalizedList

	def quatFromAxisAngle(self, axis, angle):
		axis = self.normalizeList(axis)
		angleRad = angle/57.2957795
		quat = np.zeros(4)
		s = np.sin(angleRad/2)
		quat[0] = axis[0] * s
		quat[1] = axis[1] * s
		quat[2] = axis[2] * s
		quat[3] = np.cos(angleRad/2)
		# print(type(quat))
		return quat.tolist()

	def quatMultiply(self, q1, q2):
		mult = np.zeros(4)
		mult[0] =  q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
		mult[1] = -q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
		mult[2] =  q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3] + q1[3] * q2[2]
		mult[3] = -q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] + q1[3] * q2[3]
		return mult.tolist()

	def createScalesAndOrientations(self, sizes, sizeAxis, otherSize, angles, rotAxis, ellipsType='1DOF'):
		# initialize the scales
		scaleList = []
		quatList = []
		baseScaleList = [otherSize, otherSize, otherSize]
		qBase = [0,0,0,1] # no rotation

		
		# get index value or list depending on DOF
		variedSizesIndex = [] 
		if ellipsType == '1DOF':
			variedSizesIndex = sizeAxis.index(1)
		elif ellipsType == '2DOF':
			variedSizesIndex = [i for i,x in enumerate(sizeAxis) if x == 1]

		# loop over the trials
		for trialIndex, angle in enumerate(angles):

			# first the angles
			rot = self.quatFromAxisAngle(rotAxis,angle)
			qBaseRotated = self.quatMultiply(qBase,rot)
			quatList.append(qBaseRotated)

			# then the scales
			if ellipsType == '1DOF':
				baseScaleList[variedSizesIndex] = sizes[trialIndex]
				scaleList.append(baseScaleList[:]) # need to do [:] to make a NEW list which needs to be appended

			elif ellipsType == '2DOF':
				# loop over the index values at wich position the baseScaleList should be altered for size[trialNr][1axis/2ndaxis]
				for i,variedV in enumerate(variedSizesIndex): 
					baseScaleList[variedV] = sizes[trialIndex][i]
				scaleList.append(baseScaleList[:])

		return scaleList, quatList

	def experiment1Dof(self,amountDistinctEllips,repetitionsEllips,	# 4, 5
						minSize, maxSize, minAngle, maxAngle,		# 0.08, 0.4, 0, 180
						sizeAxis, rotationAxis, seed):				# [0,0,1], [0,1,0], 5
		# [0,0,1] (vary size around local z axis), [0,1,0](rotate around local y-axis)
		quarter = (maxSize-minSize)/4
		# make better function out of this next two lines
		smallMedium, mediumLarge = [minSize+quarter, minSize+3*quarter] # not dynamic
		sizeList = self.makeSizes([smallMedium,mediumLarge],amountDistinctEllips)

		# create angle list ordered
		# minAngle, maxAngle = [0, 180]
		angleList = self.makeAngles(minAngle,maxAngle,amountDistinctEllips)

		assert len(sizeList) == len(angleList)

		# repeat list
		repeatedSize, repeatedAngles = self.repeatLists(sizeList, angleList, repetitionsEllips)

		# shuffle size angle pairs
		totalEllipses = amountDistinctEllips*repetitionsEllips
		shuffleSeq = range(totalEllipses)
		Random(seed).shuffle(shuffleSeq) # Set fixed seed!
		shuffledSizeList = self.shuffleList(repeatedSize, shuffleSeq)
		shuffledAngleList = self.shuffleList(repeatedAngles, shuffleSeq)    
		# add fake first trial to angles and sizes
		# add starting angle and size to list
		totalSizeList = [mediumLarge] + shuffledSizeList
		totalAngleList = [0] + shuffledAngleList

		# create quaternions ans scale list
		scales, orientations = self.createScalesAndOrientations(totalSizeList, sizeAxis, minSize, 
																	totalAngleList, rotationAxis)
		return scales, orientations


	def make2DofShapeList(self,xLength,yLength, amount):
		# print(range(amount))
		# print(xLength)
		# print(yLength)
		# print(10*'--')
		return [ [xLength,yLength] for i in range(amount) ]
		# return [self._make2DofShape(xLength, yLength) for i in range(amount)] 



	def experiment2Dof(self,amountOvals,amountSmallPan,amountLargePan,repetitionsEllips,	
						minSize, maxSize, minAngle, maxAngle,		
						sizeAxis, rotationAxis, seed):
		# create function that makes sizes based on min/max
		quarter = (maxSize-minSize)/4
		small, large = [minSize+quarter, minSize+3*quarter] # not dynamic
		shapes = [[small,small],[small,large],[large,large]]

		# since min,min and max,max are pancakes, the orientation does not matter
		# therefore, the angles should only be applied to the "ovals" and not the "pancackes"

		######## OVALS  ########
		# size list 
		ovalsSizeList = self.make2DofShapeList(small, large, amountOvals)
		# angle list
		ovalAngleList = self.makeAngles(minAngle, maxAngle, amountOvals)
		assert len(ovalsSizeList) == len(ovalAngleList)

		######## PANCCKACES  ########
		# size list 
		pancakeSizelist = self.make2DofShapeList(small, small, amountSmallPan)
		pancakeSizelist.extend(self.make2DofShapeList(large, large, amountLargePan))
		# angle list
		pancakeAngleList = len(pancakeSizelist)*[0] #angel = 0 degrees

		######## ADD  ########
		sizeList = []
		angleList = []
		sizeList.extend(ovalsSizeList)
		sizeList.extend(pancakeSizelist)
		angleList.extend(ovalAngleList)
		angleList.extend(pancakeAngleList)
		assert len(sizeList) == len(angleList)
		
		######## REPEAT LIST  ########
		repeatedSizes, repeatedAngles = self.repeatLists(sizeList, angleList, repetitionsEllips)
		assert len(repeatedSizes) == len(repeatedAngles)

		######## SHUFFLE  ########
		totalEllipses = repetitionsEllips*(len(sizeList))
		# print("totalEllipses= {}".format(totalEllipses))
		shuffleSeq = range(totalEllipses)
		Random(seed).shuffle(shuffleSeq) # Set fixed seed!
		shuffledSizeList = self.shuffleList(repeatedSizes, shuffleSeq)
		shuffledAngleList = self.shuffleList(repeatedAngles, shuffleSeq) 

		######## ADD START TRIAL  ########
		paarts = [small,small]
		totalSizeList = []
		totalSizeList.append(paarts)
		totalSizeList.extend(shuffledSizeList)

		totalAngleList = [0] + shuffledAngleList

		scales, orientations = self.createScalesAndOrientations(totalSizeList, sizeAxis, minSize, 
																	totalAngleList, rotationAxis, '2DOF')
		return scales, orientations
		




if __name__ == "__main__":  
	EI = ExperimentInfo(1,False)
	# minAngle, maxAngle, step, amount = [-90, 90, 180/4, 4]
	# amount of different ellipses
	amountDistinctEllips = 6

	# repetitions of each ellipsoid
	repetitionsEllips = 5

	# create size list ordered
	minSize, maxSize = [0.088, 0.56557] 
	minAngle, maxAngle = [0, 180]

	# varied size axis
	sizeAxis = [1,0,1]
	# rotation axis
	rotationAxis = [0,1,0]
	# seed
	seed = 5

	# print(list3)

	################################# testing #################################
	# scales, quats = EI.experiment2Dof(4,1,1, repetitionsEllips, 
	# 									minSize, maxSize, minAngle, maxAngle, 
	# 									sizeAxis, rotationAxis, seed)
	################################# WORKS #################################
	# scales, orientations = EI.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
	# 									minSize, maxSize, minAngle, maxAngle, 
	# 									sizeAxis, rotationAxis, seed)

