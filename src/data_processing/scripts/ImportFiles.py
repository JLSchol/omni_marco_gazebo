#!/usr/bin/env python
import yaml
import pandas as pd
from os import listdir


class ImportFiles(object):
	def __init__(self,basePath,bagName="empty"):
		self.basePath = basePath #/home/jasper/omni_marco_gazebo/src/data_processing/data

		allBagNames = listdir(basePath)
		allBagDirs = [self.basePath +"/"+ str(Name) for Name in allBagNames]

		# these parameters are tuples of (nameList,completePathList) pairs
		# indexing: bag[0=name or 1=path] [0 till end of list]
		self.allBags = (allBagNames,allBagDirs)
		self.bag = []
		self.allFiles = []
		self.csvFiles = []
		self.yamlFiles = []
		self.rosBagFiles = []

		self.topics = []


		if bagName is not "empty":
			self.setBag(bagName)
			
	def _openYaml(self,fileDir):
		with open(fileDir, 'r') as stream:
		    try:
		        yamltje = yaml.safe_load(stream)
		        # print(pd.io.json.json_normalize(yaml.safe_load(yaml.safe_load(stream))))
		    except yaml.YAMLError as exc:
		        print(exc)
    		return yamltje

	
	def _getTopic(self,fileName):
		sep = "."
		if isinstance(fileName,str):
			head = fileName.split(sep, 1)[0]
			return head
		elif isinstance(fileName,list) == True and len(fileName) ==1:
			head = fileName[0].split(sep, 1)[0]
			return head
		elif isinstance(fileName,list) == True and len(fileName) != 1:
			heads = [ singleFile.split(sep, 1)[0] for singleFile in fileName]
			return heads
		else:
			print("no string or list of strings provided")


	def _addBasePath(self,basePath,paths):

		if isinstance(paths,str):
			return str(basePath)+"/"+ str(paths)

		elif isinstance(paths,list) == True and len(paths) ==1:
			return str(basePath)+"/"+str(paths[0])

		elif isinstance(paths,list) == True and len(paths) != 1:
			completePaths = [basePath +"/"+ str(path) for path in paths]
			return completePaths
		else:
			print("no string or list of strings provided")

	
	def _findFiles(self,bagPath,fileExtension="all"): 
		listOfFiles =[]
		files = []
		if isinstance(bagPath, str):
			listOfFiles = listdir(bagPath)
			# print(listOfFiles)
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

		if len(files) == 1:
			files = str(files[0])

		return files

	def setBag(self,bagName="empty"):

		if bagName == "empty" and len(self.bag) != 0:
			print("The bag class has already been intitialized with bag directory: {}".format(self.bag[0]))
			

		elif bagName == "empty" and len(self.bag) == 0:
			print("Provide a bag directory as input for 'setBag(dir)'or use the second argument of the class initializer")
			

		elif isinstance(bagName, str):

			if len(self.bag) == 0:
				print("The bag class will be intitialized with bag directory: {}".format(bagName))

			else: #len(self.bag) !=0:
				print("The bag class has already been intitialized with bag directory: \
				{} \n and will be re-initialized with bag directory: {}".format(self.bag[0],bagName))
				return

			bagPath = self._addBasePath(self.basePath,bagName)
			self.bag = (bagName, bagPath)

			allFileNames = self._findFiles(bagPath) # hier gaat het mis
			# print("all file names are {}".format(allFileNames))
			self.allFiles = (allFileNames, self._addBasePath(bagPath,allFileNames))

			csvFileNames = self._findFiles(bagPath,"csv")
			self.csvFiles = (csvFileNames, self._addBasePath(bagPath,csvFileNames))

			yamlFileNames = self._findFiles(bagPath,"yaml")
			self.yamlFiles = (yamlFileNames, self._addBasePath(bagPath,yamlFileNames))

			rosBagFileNames = self._findFiles(bagPath,"bag")
			self.rosBagFiles = (rosBagFileNames, self._addBasePath(bagPath,rosBagFileNames))

			self.topics = self._getTopic(csvFileNames)
		else:
			print("Something went wrong in setbag()")





	def importCSV(self,topic="all"):
		if not self.bag[1]:
			print("no directory set use: setBags('bagDirectory')")
			return

		pandaaas = []
		if topic == "all":
			csvFiles = self._findFiles(self.bag[1],"csv")

			if isinstance(csvFiles, str):
				csvFiles = [csvFiles]
			# print(self.bag[1] +"/"+ str(csvFiles[2]))
			pandaaas = [pd.read_csv(self.bag[1] +"/"+ str(csvfile)) for csvfile in csvFiles]
			# print(csvFiles[11])
			# pandaaas = pd.read_csv(self.bag[1]+"/"+str(csvFiles[11]))
	
		elif isinstance(topic,str):
			pandaaas = pd.read_csv(self.bag[1] +"/"+topic)

		elif isinstance(topic,list): 
			pandaaas = [pd.read_csv(self.bag[1] +"/"+ str(topic_i)) for topic_i in topic]

		return pandaaas, csvFiles


	def importYaml(self,fileDir="all"):
		if fileDir=="all":
			fileDirs = self.yamlFiles[1]

			yamltjes = [self._openYaml(file) for file in fileDirs]
			return yamltjes, self.yamlFiles[0]
		else:
			yaml = self._openYaml(fileDir)
			return yaml



	def importAll(self):
		panda, pandanameList = self.importCSV()
		yam, yamNamesList = self.importYaml()
		print("The panda files: \n {} \n ----".format(pandanameList))
		print("The yaml files: \n {} \n ----".format(yamNamesList))
		return panda, pandanameList, yam, yamNamesList




		# self.allBags = (allBagNames,allBagDirs)
		# self.bag = []
		# self.allFiles = []
		# self.csvFiles = []
		# self.yamlFiles = []
		# self.rosBagFiles = []

		# self.topics = []