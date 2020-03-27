#!/usr/bin/env python
from random import randrange, shuffle
import numpy as np


class ExperimentInfo(object):
	def __init__(self,experimentNr=1,PracticeRun=False):
		self.data = self.getInfo(experimentNr,PracticeRun)     

	def getInfo(self,experimentNr,PracticeRun):
		experiment = str(experimentNr) + str(PracticeRun)
		switcher={
		'1False':self.experiment1Real,
		'1True':self.experiment1Practice,
		'2False':self.experiment2Real,
		'3False':self.experiment3Real,

		}
		if experiment in switcher:
			func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
			            .format(experiment))
			self.data = func()
			return func()
		else:
			print("invalid experiment name: {}".format(experiment))

	def experiment1Real(self):
		# testDict = {i: i*i for i in range(10)}
		infoSequence={  'experiment': '1DofLeftRight',
						'trialNr': range(0,31),
						# tussen 0.0849 - 0.56557
						'scale': [  [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],

									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
									[0.4,0.09,0.09]],
						'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],

											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
											[0.0,0.0,0.0,1.0]]
						}
		return infoSequence

	def experiment1Practice(self):
		infoSequence={  'experiment': '1True',
						'trialNr': range(0,2),
						# tussen 0.0849 - 0.56557
						'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
									[0.4,0.1,0.1]],
						'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
											[0.0, 0.5, 0.0, 0.8660254]]
						}
		return infoSequence

	def experiment2Real(self):
		infoSequence={  'experiment': '2False',
		'trialNr': range(0,2),
		# tussen 0.0849 - 0.56557
		'scale': [  [0.4,0.09,0.4],
					[0.4,0.4,0.09],
					[0.09,0.4,0.4],[0.09,0.4,0.4]],
					'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],                                    
					[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0]]
						}
		return infoSequence

	def experiment3Real(self):
		infoSequence={  'experiment': '3False',
						'trialNr': range(0,3),
						# tussen 0.0849 - 0.56557
						'scale': [  [0.09,0.09,0.4],[0.4,0.09,0.4],
						[0.2,0.09,0.4],[0.09,0.09,0.4]],
						'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],                                    
						[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0]]
						}
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




	def generateScales(self,minSize, maxSize):
		pass

	def shuffleList(self,lijst,sequence):
		lijst = np.array(lijst)
		newList = lijst[sequence]
		return newList.tolist()

	def create1DofSequence(self,smallMedium,mediumLarge,
						minAngle, maxAngle ,step, amountInRandList, 
														repetitions):
		sizes1DOF = [smallMedium, mediumLarge, smallMedium, mediumLarge]

		# Angle1 = EI.generateRandList(minAngle, maxAngle, step, amount) #0,15
		# Angle2 = EI.generateRandList(minAngle+20, maxAngle+15, step, amount) # 20,30
		# Angle3 = EI.generateRandList(minAngle+20+15, maxAngle+15+15, step, amount) # 35, 45
		# angles1DOF = [Angle1[0], Angle1[1], Angle2[0], Angle2[1], Angle3[0], Angle3[1]]

		angles1DOF = EI.generateRandList(minAngle, maxAngle, step, amountInRandList)

		# shuffle zize and angles in some way
		# shuffleSeq = EI.generateRandList(0, 5, 1, 6)
		# shuffleSeq = [int(x) for x in shuffleSeq]
		# sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
		# angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


		sizes1DOF = repetitions*sizes1DOF 
		angles1DOF = repetitions*angles1DOF

		shuffleSeq = self.generateRandList(0, 29, 1, 30)
		shuffleSeq = [int(x) for x in shuffleSeq]
		# sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
		# angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


		# # add starting angle and size to list
		sizes1DOF = [mediumLarge] + sizes1DOF
		angles1DOF = [0] + angles1DOF
		print(sizes1DOF)
		print(angles1DOF)
		return sizes1DOF, angles1DOF

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



if __name__ == "__main__":  
	EI = ExperimentInfo(1,False)
	# minAngle, maxAngle, step, amount = [-90, 90, 180/4, 4]

	# amount of different ellipses
	amountDistinctEllips = 4

	# repetitions of each ellipsoid
	repetitionsEllips = 5

	# create size list ordered
	smallMedium, mediumLarge = [0.12, 0.4]
	sizeList = EI.makeSizes([smallMedium,mediumLarge],amountDistinctEllips)

	# create angle list ordered
	minAngle, maxAngle = [0, 180]
	angleList = EI.makeAngles(minAngle,maxAngle,amountDistinctEllips)

	assert len(sizeList) == len(angleList)

	# repeat list
	repeatedSize, repeatedAngles = EI.repeatLists(sizeList, angleList, repetitionsEllips)

	# shuffle size angle pairs
	totalEllipses = amountDistinctEllips*repetitionsEllips
	shuffleSeq = range(totalEllipses)
	shuffle(shuffleSeq)
	shuffledSizeList = EI.shuffleList(repeatedSize, shuffleSeq)
	shuffledAngleList = EI.shuffleList(repeatedAngles, shuffleSeq)    
	# add fake first trial to angles and sizes
	# add starting angle and size to list
	totalSizeList = [mediumLarge] + shuffledSizeList
	totalAngleList = [0] + shuffledAngleList

	# create quaternions List 
	# for experiment 1 rotate over y axis [0,1,0]
	minSize = 0.088
	maxSize = 0.56557
	axis = [0,1,0]
	q1 = [0,0,0,1] # no rotation
	quatList = []
	sizeList = []
	for index, angle in enumerate(totalAngleList):
		qrot = EI.quatFromAxisAngle(axis,angle)
		q1Rotated = EI.quatMultiply(q1,qrot)
		quatList.append(q1Rotated)

		# need to allocate totalSizeList[index] automatically to y-axis
		sizes = [minSize, totalSizeList[index], minSize]
		sizeList.append(sizes)

	# this makes np.array somehow
	print(quatList)

