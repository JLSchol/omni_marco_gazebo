#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from ImportFiles import ImportFiles
from ProcessFiles import ProcessFiles

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

	# print(csvAndTfNames)

	stiffness_command = csvsPandas[3]
	# print(stiffness_command.columns.values)

	PD = PlotData()
	dfXLable = 'timeVec'
	dfYLables = ['field.F32MA.data0','field.F32MA.data4','field.F32MA.data8']
	Kfig = PD.basicPlot(stiffness_command,dfXLable,dfYLables)
	Kfig.title("Human commanded Stiffness diagonals")
	Kfig.show()