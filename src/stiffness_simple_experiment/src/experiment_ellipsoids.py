#!/usr/bin/env python
from random import randrange, shuffle, Random
import copy
import numpy as np


class ExperimentInfo(object):
	def __init__(self,experimentNr=1,PracticeRun=False):
		# generates 2 sizes based on min/max values and coefficent
		self.minSize, self.maxSize = 0.0848528137424, 0.5656854249492381 #= max =  lambdamax(=0.2)* 2.0*sqrt(2.0)= 0.03,0.2 =0.5656854249492381
		self.smallCoeff, self.largeCoeff = 0.4, 0.9
		#  small = minSize + smallCoeff*(maxSize-minSize)
		#  large = minSize + largeCoeff*(maxSize-minSize)
		
		self.minAngle, self.maxAngle = 0, 180 #	 generates 4 equally distrubuted angles [0,45,90,135]

		self.data = self.getInfo(experimentNr,PracticeRun)
		# print(self.data['scale'])     
		# print(self.data['orientation'])     

	def getInfo(self,experimentNr,PracticeRun):
		experiment = str(experimentNr) + str(PracticeRun)
		switcher={
		# '1False':self.experiment1DofFrontReal,
		'1False':self.experiment1DofFrontReal_new,

		# '2False':self.experiment2DofFrontReal,
		'2False':self.experiment2DofFrontReal_2,

		# '3False':self.experiment1DofTopReal,
		'3False':self.experiment1DofTopReal_new,

		# '4False':self.experiment2DofTopReal,
		'4False':self.experiment2DofTopReal_2,

		# '1True':self.experiment1DofFrontPractice,
		'1True':self.experiment1DofFrontPractice_new,

		# '2True':self.experiment2DofFrontPractice,
		'2True':self.experiment2DofFrontPractice_2,

		# '3True':self.experiment1DofTopPractice,
		'3True':self.experiment1DofTopPractice_new,

		# '4True':self.experiment2DofTopPractice,
		'4True':self.experiment2DofTopPractice_2,

		'0False': self.freeForAll,
		'0True': self.freeForAll,
		}
		if experiment in switcher:
			func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
						.format(experiment))

			# IF EXPERIMENT FAILS MIGHT WATN TO UNCOMMENT LINE BELOW 
			# self.data = func() 
			return func()
		else:
			print("invalid experiment name: {}".format(experiment))

	def freeForAll(self):
		scales1, quats1 = [0.0848528137424, 0.0848528137424, 0.4453907034356], [0.0, 0.0, 0.0, 1.0]
		scales2, quats2 = [0.2226953517178, 0.0848528137424, 0.4453907034356], [0.0, 0.0, 0.0, 1.0]
		scales3, quats3 = [0.0848528137424, 0.0848528137424, 0.4453907034356], [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]
		scales4, quats4 = [0.0848528137424, 0.2226953517178, 0.4453907034356], [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]

		scales = 3*[scales1] + 3*[scales2] + 3*[scales3] + 3*[scales4] 
		orientations = 3*[quats1] + 3*[quats2] + 3*[quats3] + 3*[quats4] 

		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}

		return infoSequence



	def experiment1DofFrontReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [4, 5]  
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 1

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff, 
												minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofFrontReal_new(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 4]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 1

		scales, orientations = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
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
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 2

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff, 
												minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofTopReal_new(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 4]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 2

		scales, orientations = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		infoSequence['orientation'][0] = [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]
		return infoSequence

	def experiment2DofFrontReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 5]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [4,1,1]
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [1,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff,
												minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '2DofFront',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofFrontReal_2(self):
		######### experiment1DofFrontReal #########
		######### 1DOF settings #########
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		# amountDistinctEllips, repetitionsEllips = [4, 5]  
		amountOfOrientations, repetitionsEllips = [4, 4]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 1

		scales1DoF, orientations1DoF = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		######### 2DOF #########
		######## USE THE SIGAR FROM 1DOF CASE AND TURN INTO 2DOF OVALS  ########
		######## ADD 10 PANCACKES TO IT  ########
		amountSmallPan, amountLargePan = [4,4] 
		sclaes2Dof, orientaitons2Dof = self.experiment2DoF_2(scales1DoF, orientations1DoF, rotationAxis, 
															self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff,
																amountSmallPan, amountLargePan)
		infoSequence={  'experiment': '2DofFront',
						'practice': False,
						'trialNr': range(len(sclaes2Dof)),
						# tussen 0.0849 - 0.56557
						'scale': sclaes2Dof,
						'orientation': orientaitons2Dof
						}
		return infoSequence

	def experiment2DofTopReal(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 5]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [4,1,1]
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75]
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,1,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 4

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff,
												minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': False,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofTopReal_2(self):
		######### experiment1DofTopReal #########
		######### 1DOF settings #########
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 4]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 2

		scales1DoF, orientations1DoF = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		######### 2DOF #########
		######## USE THE SIGAR FROM 1DOF CASE AND TURN INTO 2DOF OVALS  ########
		######## ADD 10 PANCACKES TO IT  ########
		amountSmallPan, amountLargePan = [4,4] 
		sclaes2Dof, orientaitons2Dof = self.experiment2DoF_2(scales1DoF, orientations1DoF, rotationAxis, 
															self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff,
																amountSmallPan, amountLargePan)
		infoSequence={  'experiment': '2DofTop',
						'practice': False,
						'trialNr': range(len(sclaes2Dof)),
						# tussen 0.0849 - 0.56557
						'scale': sclaes2Dof,
						'orientation': orientaitons2Dof
						}
		infoSequence['orientation'][0] = [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]
		return infoSequence


	def experiment1DofFrontPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [6, 4]  
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 5

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff, 
												minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofFrontPractice_new(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 2]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales, orientations = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
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
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 6

		scales, orientations = self.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff, 
												minAngle, maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment1DofTopPractice_new(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 2]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 4

		scales, orientations = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		infoSequence={  'experiment': '1DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		infoSequence['orientation'][0] = [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]
		return infoSequence

	def experiment2DofFrontPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [7, 3]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [5,1,1]
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [1,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 7

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff,
												minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '2DofFront',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence

	def experiment2DofFrontPractice_2(self):
		######### experiment1DofFrontReal #########
		######### 1DOF settings #########
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		# amountDistinctEllips, repetitionsEllips = [4, 5]  
		amountOfOrientations, repetitionsEllips = [4, 2]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [0,1,0]
		# seed to create a pseudo random sequence
		seed = 3

		scales1DoF, orientations1DoF = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		######### 2DOF #########
		######## USE THE SIGAR FROM 1DOF CASE AND TURN INTO 2DOF OVALS  ########
		######## ADD 10 PANCACKES TO IT  ########
		amountSmallPan, amountLargePan = [2,2] 
		sclaes2Dof, orientaitons2Dof = self.experiment2DoF_2(scales1DoF, orientations1DoF, rotationAxis, 
															self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff,
																amountSmallPan, amountLargePan)
		infoSequence={  'experiment': '2DofFront',
						'practice': True,
						'trialNr': range(len(sclaes2Dof)),
						# tussen 0.0849 - 0.56557
						'scale': sclaes2Dof,
						'orientation': orientaitons2Dof
						}
		return infoSequence

	def experiment2DofTopPractice(self):
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountDistinctEllips, repetitionsEllips = [7, 3]  
		# verdeling van ellipses
		amountOvals,amountSmallPan,amountLargePan = [5,1,1]
		# create size list ordered
		minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
		smallCoeff, largeCoeff = [0.25, 0.75] 
		minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
		# varied size axis
		sizeAxis = [0,1,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 8

		scales, orientations = self.experiment2Dof(amountOvals,amountSmallPan,amountLargePan,repetitionsEllips, 
												minSize, maxSize, self.smallCoeff, self.largeCoeff,
												minAngle, maxAngle, 
												sizeAxis, rotationAxis,seed)
		infoSequence={  'experiment': '1DofTop',
						'practice': True,
						'trialNr': range(len(scales)),
						# tussen 0.0849 - 0.56557
						'scale': scales,
						'orientation': orientations
						}
		return infoSequence
	
	def experiment2DofTopPractice_2(self):
		######### experiment1DofTopReal #########
		######### 1DOF settings #########
		# total amount of ellipses = amountDistinctEllips * repetitionsEllips
		amountOfOrientations, repetitionsEllips = [4, 2]  
		# varied size axis
		sizeAxis = [0,0,1]
		# rotation axis
		rotationAxis = [1,0,0]
		# seed to create a pseudo random sequence
		seed = 4

		scales1DoF, orientations1DoF = self.experiment1Dof_new(amountOfOrientations, repetitionsEllips, 
												self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff, 
												self.minAngle, self.maxAngle, 
												sizeAxis, rotationAxis, seed)
		######### 2DOF #########
		######## USE THE SIGAR FROM 1DOF CASE AND TURN INTO 2DOF OVALS  ########
		######## ADD 10 PANCACKES TO IT  ########
		amountSmallPan, amountLargePan = [2,2] 
		sclaes2Dof, orientaitons2Dof = self.experiment2DoF_2(scales1DoF, orientations1DoF, rotationAxis, 
															self.minSize, self.maxSize, self.smallCoeff, self.largeCoeff,
																amountSmallPan, amountLargePan)
		infoSequence={  'experiment': '2DofTop',
						'practice': True,
						'trialNr': range(len(sclaes2Dof)),
						# tussen 0.0849 - 0.56557
						'scale': sclaes2Dof,
						'orientation': orientaitons2Dof
						}
		infoSequence['orientation'][0] = [0.7071067813133527, 0.0, 0.0, 0.7071067810597423]
		return infoSequence


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

	def make2DofShapeList(self,xLength,yLength, amount):
		return [ [xLength,yLength] for i in range(amount) ]


	def getSmallLargeValues(self, minSize, maxSize, smallCoeff, largeCoeff):
		# coeff ranges between 0-1 where 0 is min and 1 is max
		diff = maxSize - minSize
		smallSize = minSize + smallCoeff*diff
		largeSize = minSize + largeCoeff*diff

		return smallSize, largeSize


	def experiment1Dof_new(self, amountOfOrientations,repetitionsEllips,	# 4, 5
						minSize, maxSize, smallCoeff, largeCoeff, # 0.08, 0.4, 0.25, 0.75
						minAngle, maxAngle,		# 0, 180
						sizeAxis, rotationAxis, seed):				# [0,0,1], [0,1,0], 5
		# [0,0,1] (vary size around local z axis), [0,1,0](rotate around local y-axis)

		angleList = self.makeAngles(minAngle,maxAngle,amountOfOrientations)

		smallMedium, mediumLarge = self.getSmallLargeValues(minSize,maxSize,smallCoeff,largeCoeff)
		smallList = [smallMedium for element in angleList]
		largelist = [mediumLarge for element in angleList]

		sizeList = smallList+largelist
		angleList = 2*angleList
		assert len(sizeList) == len(angleList)
		
		# repeat list
		repeatedSize, repeatedAngles = self.repeatLists(sizeList, angleList, repetitionsEllips)
		assert len(repeatedSize) == len(repeatedAngles)

		# shuffle size angle pairs
		totalEllipses = len(repeatedAngles)
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
																	totalAngleList, rotationAxis, '1DOF')
		
		return scales, orientations



	def experiment1Dof(self,amountDistinctEllips,repetitionsEllips,	# 4, 5
						minSize, maxSize, smallCoeff, largeCoeff, # 0.08, 0.4, 0.25, 0.75
						minAngle, maxAngle,		# 0, 180
						sizeAxis, rotationAxis, seed):				# [0,0,1], [0,1,0], 5
		# [0,0,1] (vary size around local z axis), [0,1,0](rotate around local y-axis)

		smallMedium, mediumLarge = self.getSmallLargeValues(minSize,maxSize,smallCoeff,largeCoeff)
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




	def experiment2DoF_2(self, scales_1dof, orientation_1dof, rotationAxis, 
						minSize, maxSize, smallCoeff, largeCoeff,
						amountSmallPan, amountLargePan):
		########## GET THE VARIED AXIS OF THE 2DOF SHAPE BASED ON ROTATION AXIS ##########
		# if we are rotating around 1 axis we are varying the size of the other axes
		sizeAxis2Dof = []
		if rotationAxis == [0,1,0]:
			sizeAxis2Dof = [1,0,1]
		elif rotationAxis == [1,0,0]:
			sizeAxis2Dof = [0,1,1]
		elif rotationAxis == [0,0,1]:
			sizeAxis2Dof = [1,1,0]
		else:
			print("DIT IS ONMOGELIJK o.O (foute invoer van rotatie as in experiment2DoF_2 func)")


		######## Create the OVALS  based of 1Dof sigars ########
		# convert 1Dof scalesList to 2 Dof scales by keeping the large axis 
		# and setting the second to large axis to half the size of the large axis
		scales_2dof = copy.deepcopy(scales_1dof)
		for scales_xyz in scales_2dof: # e.g. scales_xyz = [0.0848528137424, 0.0848528137424, 0.4453907034356]
			max_value = max(scales_xyz) 			# 0.4453907034356
			half_value = max_value/2 			# 0.22369
			max_index = scales_xyz.index(max_value) 	# 2
			# set the other varied axis to # 0.22369 (replace one of the 0.0845 with 0.2236)
			for i,scale_i in enumerate(sizeAxis2Dof): # sizeAxis2Dof = [1,0,1] or [0,1,1] vert or hor respectevily
				# if (scale_i == 1) containts a 1 than it is a varied size axis
				# AND
				# if (max_index != i) iterator is not equal to the index of the max value
				if (scale_i == 1) and (max_index != i): 
					scales_xyz[i] = half_value
					# result: [0.0848528137424, 0.0848528137424, 0.4453907034356] replaced with [0.22369, 0.0848528137424, 0.4453907034356]
		

		######## Create the PANCCKACES  ########
		small, large = self.getSmallLargeValues(minSize,maxSize,smallCoeff,largeCoeff)
		# size list 
		pancakeSizelist = self.make2DofShapeList(small, small, amountSmallPan)
		pancakeSizelist.extend(self.make2DofShapeList(large, large, amountLargePan))
		# angle list
		pancakeAngleList = len(pancakeSizelist)*[0] #angel = 0 degrees -> [0,0,0,etc]
		panScale, panQuat = self.createScalesAndOrientations(pancakeSizelist, sizeAxis2Dof, minSize, 
																	pancakeAngleList, rotationAxis,"2DOF")


		######## EXTEND QUATS/SCALES 2DOF(ONLY CONTAINING OVALS NOW) WITH PANCAKES QUATS/SCALES  ########
		scales_2dof.extend(panScale)
		orientation_1dof.extend(panQuat) # orientations remain the same! in 2Dof and 1Dof


		######### REMOVE FIRST FAKE START TRIAL  ########
		first_trial_scales = scales_2dof.pop(0)
		first_trial_ori = orientation_1dof.pop(0)
		assert len(scales_2dof)==len(orientation_1dof)

		######## SHUFFLE  ########
		totalShapes = len(scales_2dof)
		shuffleSeq = range(totalShapes)
		Random(3).shuffle(shuffleSeq) # fixed seed
		scales = self.shuffleList(scales_2dof, shuffleSeq)
		orientations = self.shuffleList(orientation_1dof, shuffleSeq)

		# ######## ADD START TRIAL  ########
		scales.insert(0,first_trial_scales)
		orientations.insert(0,first_trial_ori)
		assert len(scales)==len(orientations)

		return scales, orientations



	def experiment2Dof(self,amountOvals,amountSmallPan,amountLargePan,repetitionsEllips,	
						minSize, maxSize, smallCoeff, largeCoeff, 
						minAngle, maxAngle,		
						sizeAxis, rotationAxis, seed):

		# create function that makes sizes based on min/max and small/large coeff
		small, large = self.getSmallLargeValues(minSize,maxSize,smallCoeff,largeCoeff)
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
	pass
	# print(20*'=1=')
	# EI = ExperimentInfo(2,False)
	# print(EI.data['scale'])

	# EI.experiment1Dof_new()
	# print(20*'--')
	# EI = ExperimentInfo(2,True)
	# print(20*'=2=')
	# EI = ExperimentInfo(3,True)
	# print(20*'--')
	# EI = ExperimentInfo(4,True)
	# print(20*'=3=')
	# EI = ExperimentInfo(1,False)
	# print(20*'--')
	# EI = ExperimentInfo(2,False)
	# print(20*'=4=')
	# EI = ExperimentInfo(3,False)
	# print(20*'--')
	# EI = ExperimentInfo(4,False)



	# # total amount of ellipses = amountDistinctEllips * repetitionsEllips
	# amountDistinctEllips, repetitionsEllips = [4, 5]  
	# # create size list ordered
	# minSize, maxSize = [0.0848528137424, 0.56557] # generates 2 sizes: [minSize+quarter, minSize+3*quarter] for 4 ellipses
	# minAngle, maxAngle = [0, 180] #generates 4 equally distrubuted angles [0,45,90,135]
	# # varied size axis
	# sizeAxis = [0,0,1]
	# # rotation axis
	# rotationAxis = [0,1,0]
	# # seed to create a pseudo random sequence
	# seed = 1

	# # scales, orientations = EI.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
	# # 										minSize, maxSize, minAngle, maxAngle, 
	# # 										sizeAxis, rotationAxis, seed)



	#################
	# EI = ExperimentInfo(1,False)
	# # minAngle, maxAngle, step, amount = [-90, 90, 180/4, 4]
	# # amount of different ellipses
	# amountDistinctEllips = 6

	# # repetitions of each ellipsoid
	# repetitionsEllips = 5

	# # create size list ordered
	# minSize, maxSize = [0.0848528137424, 0.56557] 
	# minAngle, maxAngle = [0, 180]

	# # varied size axis
	# sizeAxis = [1,0,1]
	# # rotation axis
	# rotationAxis = [0,1,0]
	# # seed
	# seed = 5

	# print(list3)

	################################# testing #################################
	# scales, quats = EI.experiment2Dof(4,1,1, repetitionsEllips, 
	# 									minSize, maxSize, minAngle, maxAngle, 
	# 									sizeAxis, rotationAxis, seed)
	################################# WORKS #################################
	# scales, orientations = EI.experiment1Dof(amountDistinctEllips, repetitionsEllips, 
	# 									minSize, maxSize, minAngle, maxAngle, 
	# 									sizeAxis, rotationAxis, seed)

