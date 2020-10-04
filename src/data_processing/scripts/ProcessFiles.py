#!/usr/bin/env python

from ImportFiles import ImportFiles
import pandas as pd

def checkForNan(dfs, column='where'):
	if not isinstance(dfs, list):
		dfs = [dfs]
	for df in dfs:
		if column == 'any':
			isnan = df.isnull().values.any()
			amount = df.isnull().sum().sum()
			# print('NaN in data frame? {}'.format(isnan))
			print('there are {} nan values total'.format(amount))
		elif column == 'where':
			print(df.isna().any())
		else:
			isnan = df[column].isnull().values.any()
			amount = df[column].isnull().sum()
			print('NaN in data frame? {}'.format(isnan))
			print('there are {} nan values total'.format(amount))



class ProcessFiles(object):
	def __init__(self,csvtjes=0,csvNaampies=0,yamltje=0):
		# self._csvFlag = False
		# self._yamlFlag = False
		# if yamltje != 0:
		# 	csvInput = True
		# if csvtjes != 0:
		# 	yamlInput = True
		# self.originalParam = yamltje
		# self.originalCsvs = csvtjes
		# self.originalCsvNames = csvNaampies # *1 to create new object

		self.processedParam = []
		self.processedCsvs =[]
		self.processedCsvNames = []

		# self.tfFiles = []
		# self.topicFiles = []

		self.nodeList = []
		self.paramValue = []
		# self.topicList = []

	def setParams(self,trimmedCsvPandas,csvList,trimmedDictionair):
		self.processedParam = trimmedDictionair
		self.processedCsvs = trimmedCsvPandas
		self.processedCsvNames = csvList
		self.setYamlParams(trimmedDictionair)
		# self.setTopicList(csvList)

	def processCsvs(self,csvsPandas, csvNames1, resample=False ,roundOff=2):
		tfPandas = []
		tfNames = []
		# remove tf/tf_static as they need specific processing
		tfId = "tf"
		removeList = [] 
		for csvName in csvNames1:
			if tfId in csvName:
				removeList.append(csvName)

		# why doe sthis happend
		# print(len(removeList))
		if not len(removeList) == 0 and len(removeList) <= 2: # 1 = static other is normal 2 or less
			csvsPandas,csvNames2,tfPandas,_ = self.trimCsvs(csvsPandas, csvNames1, removeList) # might shuffels imput list!
		elif len(removeList) == 0: # tf topics where not recorded
			pass
		else:
			print("tf topic list is of length {}, FAILED AND RETURND".format(len(removeList)))
			return

		if len(tfPandas) != 0: # extend csv with the trimmed tf data frames
			tfList = []
			tfNameList = []
			for tfPanda in tfPandas:
				# split Tf child frames into seperate pandas data frame objects
				tfs, tfCsvNames = self.splitTfFrames(tfPanda)  
				tfList.extend(tfs)
				tfNameList.extend(tfCsvNames)

			# add to list of csvs and names
			csvsPandas.extend(tfList)
			csvNames2.extend(tfNameList)
		
		# add columns,convert time, add dateTime as index, sample and square data frames
		# print('initial')
		# checkForNan(csvsPandas)
		csvsPandas = self.convertDfTimes(csvsPandas)
		# print('\nconvertDfTimes')
		# checkForNan(csvsPandas)
		if resample:
			# print(10*'RESAMPLING')
			csvsPandas = self.resample(csvsPandas)
		# print('\nresample')
		# checkForNan(csvsPandas)
		csvsPandas = self.alignTimeIndex(csvsPandas)
		# print('\nalignTimeIndex')
		# checkForNan(csvsPandas)
		csvsPandas = self.addTimeVecCol(csvsPandas,"timeVec",roundOff)	
		# print('\naddTimeVecCol')
		# checkForNan(csvsPandas)

		if len(tfPandas) !=0: # tf is processed
			return csvsPandas, csvNames2
		else:
			return csvsPandas, csvNames1

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

	def convertDfTimes(self,dfList):
		newDfList = []
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList] # make list such that iterable
			listBool = False

		for df in dfList:
			df = self._convertDfTime(df)
			pd.options.mode.chained_assignment = None
			df['dateTime'] = pd.to_datetime(df['%time'],unit='s').tolist()
			df.set_index('dateTime',inplace=True)
			newDfList.append(df)

			# break
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
			# print(len(df.index))
			# print(df)
			sampled = df.resample('10ms', label='left', closed='left', axis=0).first()
			# print(len(sampled.index))
			sampledList.append(sampled)
			break

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

	def addTimeVecCol(self,dfList,columnName='timeVec',roundoff=2):
		newDfList =[]
		listBool = True
		if not isinstance(dfList,list):
			dfList = [dfList]
			listBool = False

		for df in dfList:
			firstTime = df['%time'].iloc[0]
			timeVec = map(lambda x: round( (x-firstTime),roundoff),df['%time'])
			df[columnName] = timeVec
			newDfList.append(df)

		if listBool == False:
			return newDfList[0] # return pandas
		else:
			return newDfList # return pandas list

	def trimParamDict(self,dictionairtje,removeKeysList):
		if isinstance(removeKeysList,str):
			removeKeysList = [removeKeysList]

		for key in removeKeysList:
			if key in dictionairtje:
				del dictionairtje[key]
			else:
				print("key: {} is not in the dictionair".format(key))

		return dictionairtje

	def setYamlParams(self,dictionair):
		self.nodeList = self._getNodes(dictionair)
		self.paramValue = self._getParamValuePairs(dictionair)

	# def setTopicList(self,csvList):
	# 	if isinstance(csvList,str):
	# 		csvList = [csvList]
	# 	self.topicList = [self._getTopic(csvName) for csvName in csvList]

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

	# def _getTopic(self,fileName):
	# 	sep = "."
	# 	if isinstance(fileName,str):
	# 		head = fileName.split(sep, 1)[0]
	# 		return head
	# 	elif isinstance(fileName,list) == True and len(fileName) ==1:
	# 		head = fileName[0].split(sep, 1)[0]
	# 		return head
	# 	elif isinstance(fileName,list) == True and len(fileName) != 1:
	# 		heads = [ singleFile.split(sep, 1)[0] for singleFile in fileName]
	# 		return heads
	# 	else:
	# 		print("no string or list of strings provided")

	def _unixTimeToSec(self,time):
		return(time*float(10**(-9)))

	def _convertDfTime(self,df):
		f = lambda x: x*float(10**(-9))
		df.loc[:,'%time'] = df['%time'].apply(f)
		return df



if __name__== "__main__":
	# set correct directories
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/test","202001301532_D5_W200_L0.03_0.2_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()

	# load files in the process class
	PF = ProcessFiles(csvsPandas,csvNames,yamlDict[1]) # only sets original

	# process dict and csvs
	paramDict = PF.trimParamDict(yamlDict[1],['rosdistro','roslaunch','run_id','rosversion'])
	csvsPandas,csvAndTfNames = PF.processCsvs(csvsPandas,csvNames)

	# set class variables
	PF.setParams(csvsPandas,csvAndTfNames,paramDict)




	# # data checking
	for i,csvPanda in enumerate(csvsPandas):
		csvPanda.to_csv(str("new_"+ csvNames[i]))
	# 	print(csvPanda.index.values)
	# 	print(csvPanda.columns.values)
	# 	print(csvPanda.head(1))
	# 	print(csvPanda.tail(1))
	# 	print(csvPanda.index.values[0])
	# 	print(csvPanda.index.values[-1])
	# 	print(csvPanda.shape)
	# 	print(csvPanda.index.values[0])
		print(10*"-----")

	# # class variable
	# print(PF.processedParam)
	# print(PF.processedCsvs)
	# print(PF.processedCsvNames)

	# print(PF.nodeList) # error on omni1
	# print(PF.paramValue) # works