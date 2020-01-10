#!/usr/bin/env python

import pandas as pd
from os import listdir


class ImportFiles(object):
	def __init__(self,basePath):
		self.basePath = basePath #/home/jasper/omni_marco_gazebo/src/data_processing/data
		self.allBagNames = listdir(basePath)
		self.allBagDirs = [self.basePath +"/"+ str(bagName) for bagName in self.allBagNames]
		self.bagNames = []
		self.bagDirs = [] 

		self.allFileNames = []

		

	def setBags(self,bagNameList):
		bagNames = []
		if isinstance(bagNameList, str):
			bagNames = [bagNameList]

		self.bagNames = bagNames
		self.bagDirs = [self.basePath +"/"+ str(bagName) for bagName in bagNames]

	def findFiles(self,bagDirs,fileType="all"):

		if fileType == "all":
			listOfFiles = [listdir(bagDir) for bagDir in bagDirs]
			return listOfFiles
		


	def importCSV(self,topics="all"):
		if topic == "all":
			#import all
			print("if")
		else:
			print("else")
			#[importCommand(topic) for topic in topics]

		return "pandas structure(s) from CSV(s)"

	def importYaml(self):
		return "pandas structure from single Yaml"

	def importAll(self):
		return (self.importCSV("all"), self.importYaml())

	def getCSVHeaderLists(self,pandasCSVList):

		headers = [getHeaderCommand(file) for file in pandasCSVList]
		return headers

	def getParamList(self):
		return "parameters"




if __name__== "__main__":
	# dir_path = os.path.dirname(os.path.realpath(__file__))
	path = "/home/jasper/omni_marco_gazebo/src/data_processing/data/202001101040_D10_W100_L0.01_0.45_S100_1000"

	#constructor
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data")

	#methods
	names = IF.allBagNames
	IF.setBags(IF.allBagNames[0])
	# IF.setBags([20,21])

	print(IF.findFiles(IF.bagDirs))



	#Attributes
	# print(IF.basePath)
	# print(type(IF.basePath))
	# print(10*"----")

	# print(IF.allBagNames)
	# print(type(IF.allBagNames))
	# print(10*"----")

	# print(IF.allBagDirs)
	# print(type(IF.allBagDirs))
	# print(10*"----")

	# print(IF.bagNames)
	# print(type(IF.bagNames))
	# print(10*"----")

	# print(IF.bagDirs)
	# print(type(IF.bagDirs))




	