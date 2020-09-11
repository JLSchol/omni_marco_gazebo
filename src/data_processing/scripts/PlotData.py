#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
import numpy as np
from ImportFiles import ImportFiles
from ProcessFiles import ProcessFiles
from PlotTopicInfo import PlotTopicInfo

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



	def simpleFigure(self,df,figureInfo,fig=False,ax=False):

		if fig==False and ax==False:
			fig,ax = plt.subplots()

		# get the info
		title = figureInfo.get('title')
		legend = figureInfo.get('legend')
		X = figureInfo.get('xAxis')
		Ys = figureInfo.get('yAxis')
		xlabel = figureInfo.get('xLabel')
		ylabel = figureInfo.get('yLabel')
	
		# set in figure
		ax.set_title(title)
		for i, Y in enumerate(Ys):
			x = map(lambda x: x,  df[X])
			y = map(lambda x: x,  df[Y])
			ax.plot(x,y,label=legend[i])
		ax.set_xlabel(xlabel)
		ax.set_ylabel(ylabel)
		ax.legend(loc='best')
		ax.grid()

		return fig,ax

	def addSubPlot(self,df,figureInfo,fig,ax,position='row'):
		geometry=ax.get_geometry()
		geometry1 = []
		geometry2 = []
		nrows = []
		ncols = []

		tupleAdd = lambda t1,t2: tuple(np.array(t1)+np.array(t2))
		if position == 'row':
			geometry1 = tupleAdd((1,0,0),geometry)
			nrows =geometry1[0]
			ncols =geometry1[1]
		elif position == 'col':
			geometry1 = tupleAdd((0,1,0),geometry)
			nrows =geometry1[0]
			ncols =geometry1[1]
		else:
			print("{} is not a valid input. \n Use 'row' or 'col'".format(position))

		geometry2 = tupleAdd((0,0,1),geometry1)

		ax.change_geometry(*geometry1)

		ax = fig.add_subplot(*geometry2)
		# set on gridspec
		fig,ax = self.simpleFigure(df,figureInfo,fig,ax) 


		return fig,ax

	def getWiggleStiffnessFig(self, df1,df2):
		f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
		
		

		# get the info
		# title = figureInfo.get('title')
		# legend = figureInfo.get('legend')
		# X = figureInfo.get('xAxis')
		# Ys = figureInfo.get('yAxis')
		# xlabel = figureInfo.get('xLabel')
		# ylabel = figureInfo.get('yLabel')


		# set in figure
		# ax.set_title(title)
		# for i, Y in enumerate(Ys):
		# 	x = map(lambda x: x,  df[X])
		# 	y = map(lambda x: x,  df[Y])
		# 	ax.plot(x,y,label=legend[i])
		# ax.set_xlabel(xlabel)
		# ax.set_ylabel(ylabel)
		# ax.legend(loc='best')
		# ax.grid()
		# x = range(0,len(df1['field.F32MA.data0']))/1000
		# x = [i/1000 for i in range(0,len(df1['field.F32MA.data0']))]
		ax1.set_title('Stiffness Variations',fontsize=18)
		ax1.plot(df1['timeVec'],df1['field.F32MA.data0'],linewidth=2,color='black')
		ax1.legend([r'$K_{xx}$'],loc='best',fontsize=14)
		ax1.set_xlim(2.61,10.61)
		ax1.set_ylim(0,1100)
		# ax1.set_xlabel('time [s]')
		ax1.set_ylabel('stiffness [N/m]',fontsize=14)
		
		# print(x)
		ax1.grid()
		# print(df2.columns.values)
		# ax2.set_title('Perturbation Signal',fontsize=18)
		ax2.plot(df2['timeVec'],df2['field.current_position.y'],linewidth=2,color='black')
		ax2.set_xlim(2.61,10.61)
		ax2.axhline(y=0.05,linestyle='--',color='red')
		ax2.axhline(y=0.3,linestyle='-.',color='blue')
		ax2.axhline(y=-0.05,linestyle='--',color='red')
		ax2.axhline(y=-0.3,linestyle='-.',color='blue')
		ax2.set_xlabel('time [s]',fontsize=14)
		ax2.set_ylabel('deviation [m]',fontsize=14)
		ax2.legend([r'$x_{hd}$',r'$w_{min}$',r'$w_{max}$'],loc='best',fontsize=14)
		ax2.grid()
		


		return f,ax1,ax2







if __name__== "__main__":

	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202008280234_D20_W100_L0.034_0.204_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
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
	dfList = [csvsPandas[0],csvsPandas[1]]
	nameList = [csvAndTfNames[0],csvAndTfNames[1]]
	print(nameList)

	# print(dfList[0])
	# print(type(dfList[0]))
	# init classes
	TI = PlotTopicInfo()
	PD = PlotData()

	info = TI.getInfo(nameList[0])
	PD.getWiggleStiffnessFig(csvsPandas[0],csvsPandas[1])

	# fig,ax = PD.simpleFigure(dfList[0],info)
	# fig,ax = PD.addSubPlot(dfList[1],TI.getInfo(nameList[1]),fig,ax)





	# ax.get

	# fig,ax = PD.addSubPlot(dfList[2],TI.getInfo(nameList[2]),fig,ax)
	# xx =ax.get_lines()
	# x_data = xx[0].get_data()[0]
	# y_data = xx[0].get_data()[1]

	# fi2 = plt.figure
	# fi2.show()
	# fi2.plot(x_data,y_data)
	# plt.plot(xx)
	# plt.plot(3lines)
	# left  = 0.125  # the left side of the subplots of the figure
	# right = 0.9    # the right side of the subplots of the figure
	# bottom = 0.1   # the bottom of the subplots of the figure
	# top = 0.9      # the top of the subplots of the figure
	# wspace = 0.2   # the amount of width reserved for blank space between subplots
	# hspace = 0.5   # the amount of height reserved for white space between subplots
	# plt.subplots_adjust(left=None, bottom=bottom, right=None, top=top, wspace=None, hspace=hspace)

	# X = 10*np.random.rand(5,3)
	
	plt.show()
