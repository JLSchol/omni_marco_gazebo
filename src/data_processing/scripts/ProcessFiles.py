#!/usr/bin/env python

from ImportFiles import ImportFiles



class ProcessFiles(object):
	def __init__(self,yamltje=0,csvtjes=0):
		self._csvFlag = False
		self._yamlFlag = False
		if yamltje != 0:
			csvInput = True
		if csvtjes != 0:
			yamlInput = True
		self.originalParam = yamltje
		self.originalCsvs = csvtjes

		self.paramFile = []
		self.csvFiles =[]

		self.nodeList = []
		self.paramValue = []
		self.topicList = []

	def trimDict(self,dictionairtje,removeKeysList):
		if isinstance(removeKeysList,str):
			removeKeysList = [removeKeysList]

		for key in removeKeysList:
			if key in dictionairtje:
				del dictionairtje[key]
			else:
				print("key: {} is not in the dictionair".format(key))

		return dictionairtje


	def setParams(self,trimmedCsvPandas,csvList,trimmedDictionair):
		self.paramFile = trimmedDictionair
		self.csvFiles = trimmedCsvPandas
		self.setYamlParams(trimmedDictionair)
		self.setTopicList(csvList)


	def setYamlParams(self,dictionair):
		self.nodeList = self._getNodes(dictionair)
		self.paramValue = self._getParamValuePairs(dictionair)

	def setTopicList(self,csvList):
		if isinstance(csvList,str):
			csvList = [csvList]
		self.topicList = [self._getTopic(csvName) for csvName in csvList]


	def _getNodes(self,dictionair):
		if not isinstance(dictionair,dict):
			print("instance is not a dictionair")
			return

		ActiveNodes = [node for node in dictionair] 
		return ActiveNodes

	def _getParamValuePairs(self,dictionair):
		if not isinstance(dictionair,dict):
			print("instance is not a dictionair")
			return

		nodes = self._getNodes(dictionair)
		if isinstance(nodes,str):
			print("string found")
			nodes = [nodes]  #convert single node of type string to list for proper iteration

		pairDict = {}
		for node in nodes:
			for key,value in dictionair[node].items():
				pairDict[key]=value
		return pairDict

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



if __name__== "__main__":
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202001140841_D30_W100_L0.01_0.45_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	csvsPandas,csvList,yamlDict = IF.importAll()
	# csvsPandas, csvList = IF.importCSV()
	# yamlDict = IF.importYaml()
	print(csvList)
	# print(csvsPandas)
	# print(yamlDict)
	
	PF = ProcessFiles(yamlDict,csvsPandas)
	paramDict = PF.trimDict(yamlDict,['rosdistro','roslaunch','run_id','rosversion'])

	
	
	PF.setParams(csvsPandas,csvList,paramDict)
	print(10*"----------")



		# self.originalParam = yamltje
		# self.originalCsvs = csvtjes

		# self.paramFile = []
		# self.csvFiles =[]

		# self.nodeList = []
		# self.paramValue = []
		# self.topicList = []