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


	def convertDfTimes(self,dfList):
		newDfList = []
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList] # make list such that iterable
			listBool = False

		for df in dfList:
			df = self._convertDfTime(df)
			df.loc[:,'dateTime'] = pd.to_datetime(df['%time'],unit='s')
			df.set_index('dateTime',inplace=True)
			newDfList.append(df)

		if listBool == False:
			return newDfList[0] # return pandas
		else:
			return newDfList # return pandas list

	def addTimeVecCol(self,dfList,columnName='timeVec',roundoff=2):
		newDfList =[]
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList]
			listBool = False

		for df in dfList:
			firstTime = df['%time'].iloc[0]
			print(firstTime)
			timeVec = map(lambda x: round( (x-firstTime),roundoff),df['%time'])
			df[columnName] = timeVec
			newDfList.append(df)

		if listBool == False:
			return newDfList[0] # return pandas
		else:
			return newDfList # return pandas list


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

	def alignTimeIndex(self,dfList):
		if not isinstance(dfList,list):
			dfList = [dfList]
		slicedDfList = []
		latestFirstTime = []
		earliestLastTime = []
		# print(dfList)
		for i,df in enumerate(dfList):
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

	def unixTimeToSec(self,time):
		return(time*float(10**(-9)))




	def _convertDfTime(self,df):
		f = lambda x: x*float(10**(-9))
		# timeCol = df['%time'].apply(f)
		# df['%time'] = df['%time'].apply(f)
		df.loc[:,'%time'] = df['%time'].apply(f)
		# df['%time'] = timeCol
		return df

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



	def splitTfFrames(self,tfPanda):
		# initialize
		tfPandaList = []
		tfSplitted = []
		parentFieldName = 'field.transforms0.header.frame_id'
		childFieldName = 'field.transforms0.child_frame_id'

		# get unique parent and child frame list(s)
		parentFrame = tfPanda[parentFieldName].unique().tolist()
		nr_parents = len(parentFrame)
		if nr_parents > 1:
			print("There are {} parent frames! \n Function only works with 1 parent!".format(nr_parents))
			return
		childFrameList = tfPanda[childFieldName].unique().tolist()
		
		# Split (on child frame names only!)
		for childName in childFrameList:
			tfSplitted = tfPanda[tfPanda[childFieldName] == childName]
			tfPandaList.append(tfSplitted)

		# add ".csv" for consistency in names
		childFrameList = [str(name +"_tf.csv") for name in childFrameList]

		return tfPandaList, childFrameList

	def trimCsvs(self, csvsPandas, csvNameList, trimList):
		removedCsvList = []
		removedNameList = []
		toRemoveIndices = []

		# print(any(i in csvNameList for i in trimList))
		both = set(csvNameList).intersection(trimList)
		indices = [csvNameList.index(x) for x in both]

		for index in indices:
			poppedCsv = csvsPandas.pop(index)
			poppedName = csvNameList.pop(index)
			removedCsvList.append(poppedCsv)
			removedNameList.append(poppedName)

		return csvsPandas, csvNameList, removedCsvList, removedNameList


if __name__== "__main__":
	# set correct directories
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/test","202001301532_D5_W200_L0.03_0.2_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()

	# load files in the process class
	PF = ProcessFiles(yamlDict[1])
	
	# remove tf topics as they need specific processing
	removeList = ['tf.csv','tf_static.csv'] 
	csvsPandas,csvNames,tfPandas,tfNames = PF.trimCsvs(csvsPandas, csvNames, removeList) # might shuffels imput list!

	# split Tf child frames into seperate pandas data frame objects
	tfList, tfNameList = PF.splitTfFrames(tfPandas[1])  
	tf_staticList, tf_staticNameList = PF.splitTfFrames(tfPandas[0])

	# add to list of csvs and names
	csvsPandas.extend(tfList)
	csvNames.extend(tfNameList)
	csvsPandas.extend(tf_staticList)
	csvNames.extend(tf_staticNameList)

	# add columns,convert time, add dateTime as index, sample and square data frames
	
	csvsPandas = PF.convertDfTimes(csvsPandas)
	csvsPandas = PF.resample(csvsPandas)
	csvsPandas = PF.alignTimeIndex(csvsPandas)
	csvsPandas = PF.addTimeVecCol(csvsPandas,"timeVec",2)

	print(csvNames)

	for i,csvPanda in enumerate(csvsPandas):
		# csvPanda.to_csv(str("new_"+ csvNames[i]))
		# print(csvPanda.index.values)
		# print(csvPanda.columns.values)
		# print(csvPanda.head(1))
		# print(csvPanda.tail(1))
		# print(csvPanda.index.values[0])
		# print(csvPanda.index.values[-1])
		# print(csvPanda.shape)
		# print(csvPanda.index.values[0])
		print(10*"-----")


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



