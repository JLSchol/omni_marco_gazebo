#!/usr/bin/env python
import yaml
import pandas as pd
from os import listdir


class ImportFiles(object):
	def __init__(self,basePath):
		self.basePath = basePath #/home/jasper/omni_marco_gazebo/src/data_processing/data
		self.allBagNames = listdir(basePath)
		self.allBagDirs = [self.basePath +"/"+ str(bagName) for bagName in self.allBagNames]
		self.bagNames = []
		self.bagDirs = [] 
		

	def setBags(self,bagNameList):
		self.bagNames = bagNameList
		if isinstance(bagNameList, str):
			self.bagNames = [bagNameList]
		self.bagDirs = [self.basePath +"/"+ str(bagName) for bagName in self.bagNames]


	def findFiles(self,bagDirs,fileExtension="all"):
		listOfFiles =[]
		files = []
		if len(bagDirs) == 1:
			listOfFiles = listdir(bagDirs[0])
		else:
			print("Only a single bagDir can be provided")
			return
		
		if fileExtension == "all":
			files = listOfFiles

		elif fileExtension == "csv":
			for file in listOfFiles:
				filestr = str(file)
				if filestr.endswith("csv"):
					files.append(file)
		elif fileExtension == "yaml":
			for file in listOfFiles:
				filestr = str(file)
				if filestr.endswith("yaml"):
					files.append(file)
		elif fileExtension == "bag":
			for file in listOfFiles:
				filestr = str(file)
				if filestr.endswith("bag"):
					files.append(file)
		else:
			print("unkown fileExtension type \n try: all, csv, bag or yaml")

		return files
				# if fileExtension == "all":
		# 	return listOfFiles
		# elif fileExtension == "csv":
		# 	csvFiles = [str(file) for file in listOfFiles if str(file).endswith('csv')]
		# 	return csvFiles
		# elif fileExtension == "yaml":
		# 	yamlFiles = [file for file in listOfFiles if file.endswith('yaml')]
		# 	return yamlFiles
		# elif fileExtension == "bag":
		# 	bagFiles = [file for file in listOfFiles if file.endswith('bag')]
		# 	return bagFiles
		# else:
		# 	print("unkown fileExtension type \n try: all, csv, bag or yaml")



	def importCSV(self,topic="all"):
		if not self.bagDirs:
			print("no directory set use: setBags('bagDirectory')")
			return

		pandaaas = []
		if topic == "all":
			csvFiles = self.findFiles(self.bagDirs,"csv")
			print(csvFiles)
			if isinstance(csvFiles, str):
				csvFiles = [csvFiles]

			pandaaas = [pd.read_csv(self.bagDirs[0] +"/"+ str(csvfile)) for csvfile in csvFiles]
	
		elif isinstance(topic,str):
			pandaaas = pd.read_csv(self.bagDirs[0] +"/"+topic)

		elif isinstance(topic,list): 
			pandaaas = [pd.read_csv(self.bagDirs[0] +"/"+ str(topic_i)) for topic_i in topic]

		return pandaaas


	def importYaml(self,fileDir):
		# pd.io.json.json_normalize(yaml.load(fileDir))# , 'reviews', 'doc'
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
	# dirs = IF.bagDirs; print(dirs)
	print(IF.findFiles(IF.bagDirs))
	print(IF.importYaml("/home/jasper/omni_marco_gazebo/src/data_processing/data/202001101040_D10_W100_L0.01_0.45_S100_1000/202001101040_D10_W100_L0.01_0.45_S100_1000_par.yaml"))


	# files = IF.importCSV('all')
	# files1 = IF.importCSV('omni1_joint_states.csv')
	# files2 = IF.importCSV(['omni1_joint_states.csv', 'omni1_force_feedback.csv'])


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




	