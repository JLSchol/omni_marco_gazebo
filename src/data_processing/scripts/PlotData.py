#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
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

# def rotateVector(q, v):
# 	vr = [0,0,0]
# 	vr[0] = (v[0]*q[3]*q[3] + 2*v[2]*q[3]*q[1] - 2*v[1]*q[3]*q[2] + v[0]*q[0]*q[0] + 
# 			2*v[1]*q[0]*q[1] + 2*v[2]*q[0]*q[2] - v[0]*q[1]*q[1] - v[0]*q[2]*q[2])

# 	vr[1] = (v[1]*q[3]*q[3] - 2*v[2]*q[3]*q[0] + 2*v[0]*q[3]*q[2] - v[1]*q[0]*q[0] + 
# 			2*v[0]*q[0]*q[1] + v[1]*q[1]*q[1] + 2*v[2]*q[1]*q[2] - v[1]*q[2]*q[2])

# 	vr[2] =	(v[2]*q[3]*q[3] + 2*v[1]*q[3]*q[0] - 2*v[0]*q[3]*q[1] - v[2]*q[0]*q[0] + 
# 			2*v[0]*q[0]*q[2] - v[2]*q[1]*q[1] + 2*v[1]*q[1]*q[2] + v[2]*q[2]*q[2])
# 	return vr

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

	def getWiggleStiffnessFig(self, df1,df2):
		# the following data is used for this
		# IF = ImportFiles("/home/jasper/omni_marco_gazebo/src/data_processing/data","202008280234_D20_W100_L0.034_0.204_S100_1000")

		print(df1.loc[:,['timeVec','field.F32MA.data0']])
		print(df2.loc[:,['timeVec','field.current_position.y']])
		# print(df2['timeVec','field.current_position.y'])
		# check settings and 
		f, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
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
# stiffness, ellipoid_info, timeVec, centers, scales, quats
	def taskDenomstrationPlot(self, stiffness_df, stiffness_info, timeVec, centers, scales, quats, stiffness_eig):

		# fig2, ax2 = self.simpleFigure(stiffness_df, stiffness_info)
		# fig3, ax3 = plt.subplots()
		# ax3.plot(timeVec,stiffness_eig)
		# ax3.set_xlim(0,100)
		# ax3.set_ylim(0,1200)

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
		ax.set_title('Robot Stiffness Trajecory')
		ax.set_xlim3d(xmin, xmin+square_size)
		ax.set_ylim3d(ymin, ymin+square_size)
		ax.set_zlim3d(zmin, zmin+square_size)
		# ax.set_ylim3d(-0.5, 0.5)
		# ax.set_zlim3d(-1, 1)
		ax.set_xlabel('x [m]')
		ax.set_ylabel('y [m]')
		ax.set_zlabel('z [m]')

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

		# plot figure 
		fig2, ax2 = self.simpleFigure(stiffness_df, stiffness_info)
		ax2.vlines(timeSamples,0,1200,linestyles=':')
		ax2.grid(False)


		# plt.show()

	def annotateEllips3D(self,axis, xs, ys, zs, zdirs, texts, color):
		for zdir, x, y, z, text in zip(zdirs, xs, ys, zs, texts):
			axis.text(x, y, z, text, zdir, color=color)


	def annotate2D(self,position, offsets, texts):
		pass


def wiggleReportFigure(show=True):
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
	print(ellips_log.columns.values)

	stiffness = csvsPandas[3]
	ellipoid_info = TI.getInfo(csvNames[3])
	timeVec = ellips_log['timeVec'] 
	centers = ellips_log.loc[:,['field.center.x', 'field.center.y', 'field.center.z']].values.tolist()
	scales = ellips_log.loc[:,['field.scales.x', 'field.scales.y', 'field.scales.z']].values.tolist()
	quats = ellips_log.loc[:,['field.quaternion.x', 'field.quaternion.y', 'field.quaternion.z', 'field.quaternion.w']].values.tolist()
	stiffness_eig = ellips_log.loc[:,['field.stiffness_eig0', 'field.stiffness_eig1', 'field.stiffness_eig2']].values.tolist()

	
	# centers_adjusted = []
	# for i,[cx,cy,cz] in enumerate(centers):
	# 	cx = i/float(len(timeVec))*3
	# 	centers_adjusted.append([cx,cy,cz])
	# centers = centers_adjusted

	PD.taskDenomstrationPlot(stiffness, ellipoid_info, timeVec, centers, scales, quats, stiffness_eig)

	if show==True:
		plt.show()



# oud
	# # get plotting information and data frame
	# # transformation
	# base_ee_transform = csvsPandas[0]
	# # 'eigen_pair_stiffness.csv'
	# stiffness_eigen = csvsPandas[1]
	# info_stiffness_eigen = TI.getInfo(csvNames[1])
	# # 'draw_ellipsoidellipsoid_visualization.csv'
	# ellipsoid = csvsPandas[7]
	# ellipoid_info = TI.getInfo(csvNames[7])
	# # stiffness_command.csv
	# stiffness = csvsPandas[3]
	# ellipoid_info = TI.getInfo(csvNames[3])


	# # sample quats and scales based on df base_ee_transform	
	# # print(len(base_ee_transform.index))
	# # print(len(stiffness_eigen.index))
	# # print(len(ellipsoid.index))

	# # centers = base_ee_transform.loc[:,['field.pose.position.x', 'field.pose.position.y', 
	# # 													'field.pose.position.z']].values.tolist()
	# timeVec = ellipsoid['timeVec'] 
	# scales = ellipsoid.loc[:,['field.scale.x', 'field.scale.y', 'field.scale.z']].values.tolist() 	#[ [x,y,z], [],[ etc]]
	# quats = ellipsoid.loc[:,['field.pose.orientation.x', 'field.pose.orientation.y', 				#[ [x,y,z], [],[ etc]]
	# 						'field.pose.orientation.z', 'field.pose.orientation.w']].values.tolist()
	# centers = [np.linspace(0,3,len(ellipsoid['field.pose.position.x'])).tolist(), 
	# 		[0]*len(ellipsoid['field.pose.position.x']) , [0]*len(ellipsoid['field.pose.position.x'])] #[ [......], [......], [......] ]

	# PD.taskDenomstrationPlot(stiffness, ellipoid_info, timeVec, centers, scales, quats)






if __name__== "__main__":

	# call generic functions to plot
	taskDemoReportFigure(True)
	# wiggleReportFigure(True)

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
