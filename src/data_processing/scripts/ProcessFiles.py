#!/usr/bin/env python

from ImportFiles import ImportFiles
import pandas as pd



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

	# def setyamlInfo(self,dictionair):


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

	def unixTimeToSec(self,time):
		return(time*float(10**(-9)))

	def _convertDfTime(self,df):
		f = lambda x: x*float(10**(-9))
		timeCol = df['%time'].apply(f)
		df['%time'] = timeCol
		return df

	def convertDfTimes(self,dfList):
		newDfList = []
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList] # make list such that iterable
			listBool = False

		for df in dfList:
			df = self._convertDfTime(df)
			
			df['dateTime'] = pd.to_datetime(df['%time'],unit='s')
			df.set_index('dateTime',inplace=True)
			# print(df.head(5))
			
			newDfList.append(df)

		if listBool == False:
			return newDfList[0] # return pandas
		else:
			return newDfList # return pandas list


	def getTimeVec(self,dfList,roundDecimal):
		timeVecList =[]
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList]
			listBool = False

		for df in dfList:
			firstTime = df['%time'].iloc[0]
			timeVec = map(lambda x: round( (x-firstTime),roundDecimal),df['%time'])
			timeVecList.append(timeVec)

		if listBool == False:
			return timeVecList[0] # return pandas
		else:
			return timeVecList # return pandas list

	# def allignTime(self,dfList):
	# 	if not isinstance(dfList,list):
	# 		dfList = [dfList]
	# 	slicedDfList = []
	# 	latestFirstTime = []
	# 	earliestLastTime = []

	# 	for df in dfList:
	# 		firstTime = df['%time'].iloc[0]
	# 		lastTime = df['%time'].iloc[-1]
	# 		print(firstTime)
	# 		print(lastTime)
	# 		print('---------')
	# 		if not latestFirstTime:
	# 			latestFirstTime = firstTime
	# 			earliestLastTime = lastTime
	# 			continue
	# 		if firstTime > latestFirstTime:
	# 			latestFirstTime = firstTime
	# 		if lastTime < earliestLastTime:
	# 			earliestLastTime = lastTime

	# 	for df in dfList:
	# 		# print(df.head(5))
	# 		df = df[df['%time']>=latestFirstTime]
	# 		df = df[df['%time']<=earliestLastTime]
	# 		slicedDfList.append(df)

	# 	return slicedDfList

	def resample(self,dfList):
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList]
			listBool = False

		sampledList = []
		for df in dfList:
			sampled = df.resample('10ms', label='left', closed='left', axis=0).first()
			sampledList.append(sampled)


		if listBool == False:
			return sampledList[0] # return pandas
		else:
			return sampledList # return pandas list


	def alignSamples(self,dfList):
		if not isinstance(dfList,list):
			dfList = [dfList]
		slicedDfList = []
		latestFirstTime = []
		earliestLastTime = []

		for df in dfList:
			firstTime = df.index.values[0]
			lastTime = df.index.values[-1]

			if not latestFirstTime:
				latestFirstTime = firstTime
				earliestLastTime = lastTime
			else:
				if firstTime > latestFirstTime:
					latestFirstTime = firstTime
				if lastTime < earliestLastTime:
					earliestLastTime = lastTime

		for df in dfList:
			df = df[df.index>=latestFirstTime]
			df = df[df.index<=earliestLastTime]
			slicedDfList.append(df)

		return slicedDfList





if __name__== "__main__":
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/test","202001301532_D5_W200_L0.03_0.2_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	csvsPandas,csvList,yamlDict,yamlList = IF.importAll()

	
	PF = ProcessFiles(yamlDict[1])
	print(len(csvsPandas))

	csvsPandas.pop(11)
	csvsPandas.pop(6)
	csvList.pop(11)
	csvList.pop(6)

	print(csvList)
	csvsPandas = PF.convertDfTimes(csvsPandas)
	csvsPandas = PF.resample(csvsPandas)
	csvsPandas = PF.alignSamples(csvsPandas)


	for i,csvPanda in enumerate(csvsPandas):
		csvPanda.to_csv(str("bewerkt_"+ csvList[i]))
		# print(csvPanda.index.values)
		# print(csvPanda.columns.values)
		# print(csvPanda.head(1))
		# print(csvPanda.tail(1))
		# print(csvPanda.index.values[0])
		# print(csvPanda.index.values[-1])
		# print(csvPanda.shape)
		# print(csvPanda.index.values[0])
		# print(10*"-----")

	# [stiffness_command,omni_stiffness] = PF.convertDfTimes([stiffness_command,omni_stiffness])
	# [stiffness_command,omni_stiffness] = PF.allignTime([stiffness_command,omni_stiffness])
	# [stiffness_command,omni_stiffness] = PF.resample([stiffness_command,omni_stiffness])

	# print(stiffness_command.index.values)
	# print(stiffness_command.columns.values)
	# print(stiffness_command.head(5))
	# print(stiffness_command)

	# print(omni_stiffness.index.values)
	# print(omni_stiffness.columns.values)
	# print(omni_stiffness.head(5))
	# print(omni_stiffness)


	# [sVec,oVec] = PF.getTimeVec([stiffness_command,omni_stiffness],2)


	# print(sVec)
	# print(10*"-----")
	# print(oVec)


	# get time vector
	# timeVec = PF.getTimeVec(stiffness_command)
	# print(timeVec)


	# PF = ProcessFiles(yamlDict,csvsPandas)
	# paramDict = PF.trimDict(yamlDict,['rosdistro','roslaunch','run_id','rosversion'])

	# print(force.columns.values)
	# print(force.index.values)

	# print(10*"----------")



