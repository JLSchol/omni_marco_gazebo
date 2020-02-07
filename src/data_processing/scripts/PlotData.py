#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from ImportFiles import ImportFiles
from ProcessFiles import ProcessFiles
from PlotTopicInfo import PlotTopicInfo

class PlotData(object):
	def __init__(self):
		pass

	# def basicPlot(df, dfXLabels, dfYLabels, xLabel, xLabels, title):
	def basicPlot(self,df, dfXColumn, dfYColumns):
		if isinstance(dfYColumns,str):
			dfYColumns = [dfYColumns]

		xValues = map(lambda x: x,  df[dfXColumn]); 
		
		fList = []
		axList = []
		plt.plot()
		for dfYColumn in dfYColumns:
			yValues = map(lambda x: x,  df[dfYColumn]); 
			plt.plot(xValues,yValues)
		return plt



	def simpleFigure(self,df,figureInfo):

		# if plt == 'empty':
		# 	plt.plot()
			# print(type(plt))

		title = figureInfo.get('title')
		# print(type(title))
		legend = figureInfo.get('legend')

		X = figureInfo.get('xAxis')
		Ys = figureInfo.get('yAxis')
		# print(figureInfo)
		xlabel = figureInfo.get('xLabel')
		ylabel = figureInfo.get('yLabel')
		# print(dfYColumns)
		# print(ylabel)

		plt.plot()
		plt.title(title)
		print(Ys)
		for i, Y in enumerate(Ys):
			# map
			# print(df[X])
			x = map(lambda x: x,  df[X])
			y = map(lambda x: x,  df[Y])
			plt.plot(x,y,label=legend[i])
		plt.xlabel(xlabel)
		plt.ylabel(ylabel)
		plt.legend(loc='best')
		plt.grid()

		return plt





if __name__== "__main__":
	# /home/jasper/omni_marco_gazebo/src/data_processing/test/202001301532_D5_W200_L0.03_0.2_S100_1000
	# dfList = [csvsPandas[3],csvsPandas[6],csvsPandas[9]]
	# nameList = [csvAndTfNames[3],csvAndTfNames[6],csvAndTfNames[9]]

	# /home/jasper/omni_marco_gazebo/src/data_processing/test/202002061052_D5_W200_L0.03_0.2_S100_1000
	# dfList = [csvsPandas[3],csvsPandas[6],csvsPandas[9]]
	# nameList = [csvAndTfNames[3],csvAndTfNames[6],csvAndTfNames[9]]

	# /home/jasper/omni_marco_gazebo/src/data_processing/test/202002061043_D30_W200_L0.03_0.2_S100_1000
	# dfList = [csvsPandas[3],csvsPandas[6],csvsPandas[10],csvsPandas[13],csvsPandas[14]]
	# nameList = [csvAndTfNames[3],csvAndTfNames[6],csvAndTfNames[10],csvAndTfNames[13],csvAndTfNames[14]]


	# "/home/jasper/omni_marco_gazebo/src/data_processing/data/202001140849_D30_W100_L0.01_0.45_S100_1000" not working
	# set correct directories
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/test","202002061043_D30_W200_L0.03_0.2_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()

	# load files in the process class
	PF = ProcessFiles(csvsPandas,csvNames,yamlDict[1]) # only sets original
	# process dict and csvs
	paramDict = PF.trimParamDict(yamlDict[1],['rosdistro','roslaunch','run_id','rosversion'])
	csvsPandas,csvAndTfNames = PF.processCsvs(csvsPandas,csvNames)
	print(csvAndTfNames)

	# set class variables not necessary
	PF.setParams(csvsPandas,csvAndTfNames,paramDict)


	# get list of things I want to plot
	# print(len(csvAndTfNames))
	dfList = [csvsPandas[3],csvsPandas[6],csvsPandas[10],csvsPandas[13],csvsPandas[14]]
	nameList = [csvAndTfNames[3],csvAndTfNames[6],csvAndTfNames[10],csvAndTfNames[13],csvAndTfNames[14]]
	# print(nameList)

	
	# init classes
	TI = PlotTopicInfo()
	PD = PlotData()

	for i,name in enumerate(nameList):
		info = TI.getInfo(name)
		figure = PD.simpleFigure(dfList[i],info)
		figure.show()




	# dfXLable = 'timeVec'
	# dfYLables = ['field.F32MA.data0','field.F32MA.data4','field.F32MA.data8']


	# Kfig = PD.basicPlot(stiffness_command,dfXLable,dfYLables)
	# Kfig.title("Human commanded Stiffness diagonals")
	# Kfig.show()