#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
import pandas as pd
from matplotlib import cm
from matplotlib import colors as col
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from ImportFiles import ImportFiles
from ProcessFiles import ProcessFiles
from PlotTopicInfo import PlotTopicInfo
from tf.transformations import quaternion_matrix 
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass
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
def calcSurfacePointsOfEllipsoid(center, radii, rotatie_matrix,  resolutie):

	u = np.linspace(0.0, 2.0 * np.pi, resolutie)
	v = np.linspace(0.0, np.pi, resolutie)
	x = radii[0] * np.outer(np.cos(u), np.sin(v))
	y = radii[1] * np.outer(np.sin(u), np.sin(v))
	z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
	for i in range(len(x)):
		for j in range(len(x)):
			[x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotatie_matrix) + center
	return x,y,z



def findNearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx

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

	def getWiggleStiffnessFig(self, df1,df2,xlim1):
		# the following data is used for this
		# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202008280234_D20_W100_L0.034_0.204_S100_1000")



		print(df1.loc[:,['timeVec','field.F32MA.data0']])
		print(df2.loc[:,['timeVec','field.current_position.y']])
		# print(df2['timeVec','field.current_position.y'])
		# check settings and 
		f, (ax2, ax1) = plt.subplots(2, 1, sharex=True)
		ax1.set_title('Stiffness Commands',fontsize=18)
		ax1.plot(timevec1,df1['field.F32MA.data0'],linewidth=2,color='black')
		ax1.legend([r'$K_{xx}$'],loc='best',fontsize=14)
		ax1.set_xlabel('time [s]',fontsize=14)
		ax1.set_xlim(xlim1)
		ax1.set_ylim(0,1100)
		# ax1.set_xlabel('time [s]')
		ax1.set_ylabel('stiffness [N/m]',fontsize=14)
		
		# print(x)
		ax1.grid()
		# print(df2.columns.values)
		ax2.set_title('Perturbation Signal',fontsize=18)
		ax2.plot(timevec2,df2['field.current_position.y'],linewidth=2,color='black')
		ax2.set_xlim(xlim1)
		ax2.axhline(y=0.05,linestyle='--',color='red')
		ax2.axhline(y=0.3,linestyle='-.',color='blue')
		ax2.axhline(y=-0.05,linestyle='--',color='red')
		ax2.axhline(y=-0.3,linestyle='-.',color='blue')
		
		ax2.set_ylabel('deviation [m]',fontsize=14)
		ax2.legend([r'$x_{hd}$',r'$w_{min}$',r'$w_{max}$'],loc='best',fontsize=14)
		ax2.grid()

		# plt.subplots_adjust(hspace = 0.3)
		

		return f,ax1,ax2

	def taskDemoEigValueStiffnessPlot(self, eigen_df, stiffness_df, eigen_info, stiffness_info):
		fig, axs = plt.subplots(2)
		# fig.suptitle('Vertically stacked subplots')
		_, axs[0] = self.simpleFigure(eigen_df, eigen_info, fig, axs[0])
		_, axs[1] = self.simpleFigure(stiffness_df, stiffness_info, fig, axs[1])

		for ax in axs:

			# fontsizes
			s_t = ax.get_title()
			s_x = ax.get_xlabel()
			s_y = ax.get_ylabel()
			s_l = ax.get_legend()
			ax.set_title(s_t, fontsize=18)
			ax.set_xlabel(s_x, fontsize=14)
			ax.set_ylabel(s_y, fontsize=14)

			# linewidths
			ax.legend(fontsize=14, loc='best')
			for line in ax.get_legend().get_lines():
				line.set_linewidth(2)

			lines = ax.get_lines()
			for line in lines:
				line.set_linewidth(2)

		# tune axis
		plt.subplots_adjust(hspace = 0.3)
		axs[0].set_ylim(0, 1200)
		axs[0].set_xlabel('')
		
		return fig, axs

	def taskDemoPlot3D(self, timeVec, centers, scales, quats):

		# timeSamples = [0.9, 2, 3.6, 4.7 ,8.5, 12]
		timeSamples = [0.9, 2, 3.6, 4.7 ,7, 9, 12]
		# timeSamples = [1, 2, 4, 5.16 ,9, 12]

		# timeSamples = [0.5*i for i in range(30)]
		ellips_selection_indices = [findNearest(timeVec,t)[1] for t in timeSamples]

		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		#set colour map so each ellipsoid has a unique colour
		norm = col.Normalize(vmin=0, vmax=len(scales))
		cmap = cm.viridis
		m = cm.ScalarMappable(norm=norm, cmap=cmap)

		scale_ellips = 0.3

		# plot ellips 1 by one
		# probably need to rotate the scales also before????
		xc, yc, zc, = [], [], []
		for i, (center,scale,quaternion) in enumerate(zip(centers,scales,quats)):
			if i in ellips_selection_indices:


				T_matrix = quaternion_matrix(quaternion)
				RT_matrix = T_matrix[0:3,0:3]
				R_matrix = RT_matrix.T # swap rows with colums to find correct rotatino matrix

				radii = [scale_ellips*scale_i for scale_i in scale]
				x_surf,y_surf,z_surf = calcSurfacePointsOfEllipsoid(center, radii, R_matrix, 30)
				
				ax.plot_surface(x_surf, y_surf, z_surf,
						rstride=4, cstride=4,  color=m.to_rgba(i), linewidth=0.1, alpha=0.25, shade=True)
				xc.append(center[0])
				yc.append(center[1])
				zc.append(center[2])


		center_T = np.array(centers).T.tolist()
		ax.plot3D(center_T[0],center_T[1],center_T[2], 'black')

		square_size = 0.8
		xmin = 0.3
		ymin = -0.4
		zmin = 0.4
		ax.set_title('Robot Stiffness Trajectory', fontsize=18)
		ax.set_xlim3d(xmin, xmin+square_size)
		ax.set_ylim3d(ymin, ymin+square_size)
		ax.set_zlim3d(zmin, zmin+square_size)
		# ax.set_ylim3d(-0.5, 0.5)
		# ax.set_zlim3d(-1, 1)
		ax.set_xlabel('x [m]' ,fontsize=14)
		ax.set_ylabel('y [m]' ,fontsize=14)
		ax.set_zlabel('z [m]' ,fontsize=14)

		offsets_x = [-0.1,0,0,0,0,0,0]
		offsets_y = [0.08,0,-0.05,0.05,0.1,0.1,0.1]
		offsets_z = [-0.11,-0.11,0,0.18,0.18,0.18,0.18]
		sumItemsOfTwoList = lambda x,y: [x+y for x,y in zip(x,y)]
		xs = sumItemsOfTwoList(xc, offsets_x)
		ys = sumItemsOfTwoList(yc, offsets_y)
		zs = sumItemsOfTwoList(zc, offsets_z)
		# zdirs = ['y','y','y','y','y','y','y']
		zdirs = 7*[None]
		texts_3d = [str(round(t,2))+ ' [s]' for t in timeSamples]

		self.annotateEllips3D(ax,xs,ys,zs,zdirs,texts_3d,'black')
		return fig, ax


	def annotateEllips3D(self,axis, xs, ys, zs, zdirs, texts, color):
		for zdir, x, y, z, text in zip(zdirs, xs, ys, zs, texts):
			axis.text(x, y, z, text, zdir, color=color)



def wiggleReportFigure(show=True):
	IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202008280234_D20_W100_L0.034_0.204_S100_1000") # paper figure 
	xlim = (2.61,10.61)
	stiff_i, pert_i = 0,1

	# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202011031840_D20_W100_L0.034_0.204_S100_1000") #,"Noise on signal"
	# xlim = (0.,8)
	# stiff_i, pert_i = 0,1

	# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202011031903_D20_W200_L0.034_0.204_S100_1000") #,"doubled sliding window"
	# xlim = (0.3,8.3)
	# stiff_i, pert_i = 0,1

	# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202011031855_D20_W100_L0.034_0.204_S100_1000") #,"doubled frequency window"
	# xlim = (1.05,9.05)
	# stiff_i, pert_i = 0,1



	# load files in the process class
	csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()
	PF = ProcessFiles(csvsPandas,csvNames,yamlDict[1]) # only sets original
	# process dict and csvs
	paramDict = PF.trimParamDict(yamlDict[1],['rosdistro','roslaunch','run_id','rosversion'])
	csvsPandas,csvAndTfNames = PF.processCsvs(csvsPandas,csvNames)
	print(csvAndTfNames)

	# set class variables not necessary
	# PF.setParams(csvsPandas,csvAndTfNames,paramDict)

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

	# info = TI.getInfo(nameList[0])
	stiffness = csvsPandas[0]
	perturbation = csvsPandas[1]

	PD.getWiggleStiffnessFig(csvsPandas[stiff_i],csvsPandas[pert_i],xlim)

	if show==True:
		plt.show()



def taskDemoReportFigure(show=True):
	# data
		# VIDEO
		# 202010021049_D60_W100_L0.03_0.2_S10_1000
		# 1 sec CONTINOUS
		# 202010021105_D60_W100_L0.03_0.2_S10_1000
		# 202010021133_D60_W100_L0.03_0.2_S10_1000
		# 2 sec CONTINOUS
		# 202010021142_D60_W200_L0.03_0.2_S10_1000
		# 202010021150_D60_W200_L0.03_0.2_S10_1000
		# 1 sec FIXED
		# 202010021105_D60_W100_L0.03_0.2_S10_1000
		# 202010021123_D60_W100_L0.03_0.2_S10_1000
		# 2 sec FIXED
		# 202010021159_D60_W200_L0.03_0.2_S10_1000
		# 202010021209_D60_W200_L0.03_0.2_S10_1000

		# new
		# continious 1 sec continious
		# 202010050928_D60_W100_L0.03_0.2_S100_1000
		# 202010051310_D60_W100_L0.03_0.2_S100_1000


	IF = ImportFiles(
		"/home/jasper/omni_marco_gazebo/src/data_processing/data","202010051310_D60_W100_L0.03_0.2_S100_1000") 
	csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()

	# load files in the process class and proces
	PF = ProcessFiles(csvsPandas,csvNames) # only sets original
	# processCsvs does: 
		# converting of "%time" to readable 'dateTime'
		# resamples if set to True when all the data frames need equal points
		# Allign timevecotr of all the data frames
		# Add a time vector colomn 'timeVec'
	resample = False

	csvsPandas,csvNames = PF.processCsvs(csvsPandas,csvNames,resample)

	# initialize plotclasses
	TI = PlotTopicInfo() # used to get info of the plot stored in dictionairs (titles, marker sieze etc)
	PD = PlotData()


	ellips_log = csvsPandas[0]
	stiffness = csvsPandas[3]
	ellips_log = addDiffCol(ellips_log, False)
	eigen_values = fixEigenalues(ellips_log)


	ellipoid_info = TI.getInfo(csvNames[3])
	timeVec = ellips_log['timeVec'] 
	centers = ellips_log.loc[:,['field.center.x', 'field.center.y', 'field.center.z']].values.tolist()
	scales = ellips_log.loc[:,['field.scales.x', 'field.scales.y', 'field.scales.z']].values.tolist()
	quats = ellips_log.loc[:,['field.quaternion.x', 'field.quaternion.y', 'field.quaternion.z', 'field.quaternion.w']].values.tolist()
	stiffness_eig = ellips_log.loc[:,['field.stiffness_eig0', 'field.stiffness_eig1', 'field.stiffness_eig2']].values.tolist()

	fig, ax = PD.taskDemoEigValueStiffnessPlot(eigen_values, stiffness, TI.getInfo('eigen_after_shuffle'), TI.getInfo('stiffness_command.csv'))

	fig3, ax3 = PD.taskDemoPlot3D(timeVec, centers, scales, quats)

	if show==True:
		plt.show()



def addDiffCol(ellips_log, save=False):
	x0 = ellips_log['field.stiffness_eig0'].diff().abs().astype(float)
	x0.iloc[0] = 0.0
	x1 = ellips_log['field.stiffness_eig1'].diff().abs().astype(float)
	x1.iloc[0] = 0.0
	x2 = ellips_log['field.stiffness_eig2'].diff().abs().astype(float)
	x2.iloc[0] = 0.0


	ellips_log['d0'] = x0
	ellips_log['d1'] = x1
	ellips_log['d2'] = x2

	if save:
		ellips_log.to_csv("/home/jasper/omni_marco_gazebo/src/data_processing/eig.csv")
	return ellips_log

def sliceColumn(df,lower,upper,column_names,order):

	sliced_list = df[column_names].iloc[lower:upper]

	# dfnew = pd.DataFrame()
	sliced_list['e0'] = sliced_list.iloc[:,1+order[0]]
	sliced_list['e1'] = sliced_list.iloc[:,1+order[1]]
	sliced_list['e2'] = sliced_list.iloc[:,1+order[2]]

	return sliced_list


def fixEigenalues(df):

	df = df.reset_index()

	threshold = 45
	index =  df[(df['d0'] > threshold) | (df['d1'] > threshold) | (df['d2'] > threshold)]

	names = ['timeVec','field.stiffness_eig0','field.stiffness_eig1','field.stiffness_eig2']
	names2 = ['timeVec','e0','e1','e2']
	e1 = []
	print(index.index[15]+1100)

	d1 =  sliceColumn(df, 0, index.index[0], names, [0,1,2] )
	d2 =  sliceColumn(df, index.index[0], index.index[1], names, [0,2,1] )
	d3 =  sliceColumn(df, index.index[1], index.index[2], names, [0,1,2] )
	d4 =  sliceColumn(df, index.index[2], index.index[3], names, [0,2,1] )
	d5 =  sliceColumn(df, index.index[3], index.index[4], names, [0,1,2] )
	d6 =  sliceColumn(df, index.index[4], index.index[5], names, [0,2,1] )
	d7 =  sliceColumn(df, index.index[5], index.index[6], names, [0,1,2] )
	d8 =  sliceColumn(df, index.index[6], index.index[7], names, [0,2,1] )
	d9 =  sliceColumn(df, index.index[7], index.index[8], names, [0,1,2] )
	d10 =  sliceColumn(df, index.index[8], index.index[9], names, [0,2,1] )
	d11 =  sliceColumn(df, index.index[9], index.index[10], names, [0,1,2] )
	d12 =  sliceColumn(df, index.index[10], index.index[11], names, [0,2,1] )
	d13 =  sliceColumn(df, index.index[11], index.index[12], names, [0,1,2] )
	d14 =  sliceColumn(df, index.index[12], index.index[13], names, [0,2,1] )
	d15 =  sliceColumn(df, index.index[13], index.index[14], names, [2,1,0] ) # switch!!

	d161 =  sliceColumn(df, index.index[14], index.index[14]+200, names, [1,2,0] )
	d162 =  sliceColumn(df, index.index[14]+200, index.index[15]-1800, names, [2,1,0] )
	d163 =  sliceColumn(df, index.index[15]-1800, index.index[15], names, [1,2,0] )

	d17 =  sliceColumn(df, index.index[15], index.index[16], names, [2,1,0] )
	d18 =  sliceColumn(df, index.index[16], index.index[17], names, [1,2,0] )
	d19 =  sliceColumn(df, index.index[17], index.index[18], names, [2,1,0] )
	d20 =  sliceColumn(df, index.index[18], df.index.values[-1], names, [1,2,0] )


	# d16 =  sliceColumn(df, index.index[14], index.index[15], names, [1,2,0] )
	# d17 =  sliceColumn(df, index.index[15], index.index[16], names, [2,1,0] )
	# d18 =  sliceColumn(df, index.index[16], index.index[17], names, [1,2,0] )
	# d19 =  sliceColumn(df, index.index[17], index.index[18], names, [2,1,0] )
	# d20 =  sliceColumn(df, index.index[18], df.index.values[-1], names, [1,2,0] )

	dfnew = pd.concat([d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d161,d162,d163, d17, d18, d19, d20])
		# check
		# print(d1[names2].head(3))
		# print()
		# print(d1[names2].tail(3))
		# print(d2[names2].head(3))
		# print()
		# print(d2[names2].tail(3))
		# print(d3[names2].head(3))
		# print()
		# print(d3[names2].tail(3))
		# print(d4[names2].head(3))
		# print()
		# print(d4[names2].tail(3))
		# print(d5[names2].head(3))
		# print()
		# print(d5[names2].tail(3))
		# print(d6[names2].head(3))
		# print()
		# print(d6[names2].tail(3))
		# print(d7[names2].head(3))
		# print()
		# print(d7[names2].tail(3))
		# print(d8[names2].head(3))
		# print()
		# print(d8[names2].tail(3))
		# print(d9[names2].head(3))
		# print()
		# print(d9[names2].tail(3))
		# print(d10[names2].head(3))
		# print()
		# print(d10[names2].tail(3))
		# print(d11[names2].head(3))
		# print()
		# print(d11[names2].tail(3))
		# print(d12[names2].head(3))
		# print()
		# print('d12-13')
		# print(d12[names2].tail(3))
		# print(d13[names2].head(3))
		# print('d13-14')
		# print(d13[names2].tail(3))
		# print(d14[names2].head(3))
		# print('d14-15')
		# print(d14[names2].tail(3))
		# print(d15[names2].head(3))
		# print('d15-16')
		# print(d15[names2].tail(3))
		# print(d16[names2].head(3))
		# print('d16-17')
		# print(d16[names2].tail(3))
		# print(d17[names2].head(3))
		# print('d17-18')
		# print(d17[names2].tail(3))
		# print(d18[names2].head(3))
		# print('d18-19')
		# print(d18[names2].tail(3))
		# print(d19[names2].head(3))
		# print()
		# dfnew.plot(x='timeVec',y=['e2','e1','e0'], ylim=(0,1200))
		# plt.show()

	return dfnew








if __name__== "__main__":

	# call generic functions to plot
	# taskDemoReportFigure(True)
	wiggleReportFigure(True)

	# close everything
	plt.close()


############ general process for plotting  ############
	# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202008280234_D20_W100_L0.034_0.204_S100_1000") #,"202001131800_D30_W100_L0.01_0.45_S100_1000"
	# csvsPandas,csvNames,yamlDict,yamlNames = IF.importAll()

	# # load files in the process class
	# PF = ProcessFiles(csvsPandas,csvNames,yamlDict[1]) # only sets original
	# # process dict and csvs
	# paramDict = PF.trimParamDict(yamlDict[1],['rosdistro','roslaunch','run_id','rosversion'])
	# csvsPandas,csvAndTfNames = PF.processCsvs(csvsPandas,csvNames)
	# print(csvAndTfNames)

	# # set class variables not necessary
	# PF.setParams(csvsPandas,csvAndTfNames,paramDict)

	# # get list of things I want to plot
	# # print(len(csvAndTfNames))
	# dfList = [csvsPandas[0],csvsPandas[1]]
	# nameList = [csvAndTfNames[0],csvAndTfNames[1]]
	# print(nameList)

	# # print(dfList[0])
	# # print(type(dfList[0]))
	# # init classes
	# TI = PlotTopicInfo()
	# PD = PlotData()

	# info = TI.getInfo(nameList[0])
	# fig,ax = PD.simpleFigure(dfList[0],info)
	# fig,ax = PD.addSubPlot(dfList[1],TI.getInfo(nameList[1]),fig,ax)

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
	
	# plt.show()
