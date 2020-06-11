#!/usr/bin/env python
from ManageDataDirectories import ManageDataDirectories
from ParticipantData import ParticipantData
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
from matplotlib.lines import Line2D  
from matplotlib.patches import Patch
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass


class ProcessSimpleExperiment():
	def __init__(self):
		self.topic = 'simple_experiment'
		self.exp_IDs = ['1R', '2R', '3R', '4R', '4L', '3L', '2L', '1L']
		

	def removeFirstTrials(self,dataDict,exp_IDs=['1R', '2R', '3R', '4R', '4L', '3L', '2L', '1L'], 
																topics=['simple_experiment_data']):
		for exp_x in exp_IDs:
			for topic in topics:
				df = dataDict[exp_x][topic]
				df.drop(df.head(1).index, inplace=True)
		# note that df is removed in place thus dataDict is changed in place!

	def getSizeID1DoF(self, df):
		values = df["field.experiment_scales.z"].unique()
		values.sort() # ascending

		fn = lambda x: 'small_cigar' if x["field.experiment_scales.z"]==values[0] else 'large_cigar'

		df2 = df.apply(fn, axis=1)
		return df2.tolist()

	def getSizeID2DoF(self, df):
		# one of these 2 columns is not varied and the smallest size of the ellipsoid
		valuesx = df["field.experiment_scales.x"].unique() 
		valuesy = df["field.experiment_scales.y"].unique()
		
		small = df['field.experiment_scales.z'].min() # z axis should always have 2 sizes as defined in experiment_ellipsoids.py
		direction_id =[]

		scales_1 = []
		scales_2 = []
		if len(valuesx) == 1: # if only one value in len(valuesx) is unique -> x is smallest axis of ellipsoid
			direction_id = 'yz'
			scales_1 = df["field.experiment_scales.y"].values.tolist()
			scales_2 = df["field.experiment_scales.z"].values.tolist()
		elif len(valuesy) == 1: # if only one value in len(valuesy) is unique -> y is smallest axis of ellipsoid
			direction_id = 'xz'
			scales_1 = df["field.experiment_scales.x"].values.tolist()
			scales_2 = df["field.experiment_scales.z"].values.tolist()
		else:
			# z is always varying for the defined experiments
			Print(" deze bestaat als het goed is niet")

		textList = []
		shape_id = []
		for x,y in zip(scales_1,scales_2):
			if x != y:
				shape_id = 'oval'
			elif x == y and x == small:
				shape_id = 'small_circle'
			else:
				shape_id = "large_circle"

			# combine shape id and direction id
			text = shape_id+"_"+direction_id
			textList.append(text)
		return textList
		


	def getRotIDText(self, df):
		deg_key = ['0_deg'	,'30_deg',	'36_deg',	'45_deg',	'60_deg',	'72_deg',	'90_deg',	'108_deg',	'120_deg',	'135_deg',	'144_deg',	'150_deg']
		qw_val =[1,			0.9659258,	0.9510565,	0.9238795,	0.8660254,	0.809017,	0.7071068,	0.5877853,	0.5,		0.3826834,	0.309017,	0.258819]
		deg_2_qw = dict(zip(deg_key,qw_val))

		qx_list = df["field.experiment_orientation.x"].values.tolist()
		qy_list = df["field.experiment_orientation.y"].values.tolist()
		qz_list = df["field.experiment_orientation.z"].values.tolist()
		qw_list = df["field.experiment_orientation.w"].values.tolist()

		get_direction = lambda x,y,z: 'x' if x!=0 else('y' if y!=0 else 'z')
		# print(get_direction(0,2,0))
		combine_text = lambda a,y: str(a)+'_'+str(y)

		textList = []
		for qx,qy,qz,qw in zip(qx_list,qy_list,qz_list,qw_list):
			if abs(round(float(qw),3)) == abs(round(float(deg_2_qw['0_deg']),3)):
				text = '0_deg'
				# 0 degrees thus no rotation axis...
				# direction_text = get_direction(qx, qy, qz)
				# text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['30_deg']),3)):
				angle_text = '30_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['36_deg']),3)):
				angle_text = '36_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['45_deg']),3)):
				angle_text = '45_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['60_deg']),3)):
				angle_text = '60_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['72_deg']),3)):
				angle_text = '72_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['90_deg']),3)):
				angle_text = '90_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['108_deg']),3)):
				angle_text = '108_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['120_deg']),3)):
				angle_text = '120_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['135_deg']),3)):
				angle_text = '135_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['144_deg']),3)):
				angle_text = '144_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['150_deg']),3)):
				angle_text = '150_deg'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)
			else:
				print("something went wrong")
				return 0

		return textList




	def removeIncorrectTrials(self, df, indices):
		for index in indices:
			df.loc[index, :] = None

	def addColumns(self, df, column_labels, data_lists, insert_list):
		for label, data, index in zip(column_labels,data_lists,insert_list):
			df.insert(index, label, data, False)
			df[label] = data

	
	def main():
		pass

class PlotSimpleExperiment():
	def __init__(self):
		self.trial_time = {'title': "Time per Trial",
					'xLabel': "Trial [-]",
					'yLabel': "Time [s]",
					'xAxis': "field.trial_nr",
					'yAxes': ["field.trial_time"],
					'legends': ["time"],
					'linestyles': ['-'],
					'markers': ['d'],
					'colors':['b'],
					'loc':	"best",
					'grid': True,
					'linewidth' : 2,
					'fs_label' : 14,
					'fs_title' : 16,
					'fs_legend': 12,
					'xLim'	: False,
					'yLim'	:(0,15)
				}
		self.accuracy = {'title': "Accuracy Feedback",
					'xLabel': "Trial [-]",
					'yLabel': "Accuracy [%]",
					'xAxis': "field.trial_nr",
					'yAxes': ["field.shape_acc","field.orientation_acc"],
					'legends': ["shape","orientaton"],
					'linestyles': ['-','-'],
					'markers': ['d','d'],
					'colors':['b','r'],
					'loc':	"best",
					'grid': True,
					'linewidth' : 2,
					'fs_label' : 14,
					'fs_title' : 16,
					'fs_legend': 12,
					'xLim'	: False,
					'yLim'	:(80,100)
				}

		self.shape_error = {'title': "Shape Error Axes (diameter)",
					'xLabel': "Trial [-]",
					'yLabel': "Size error [m]",
					'xAxis': "field.trial_nr",
					'yAxes': ["field.error_sorted_principle_axes.x","field.error_sorted_principle_axes.y",
																	"field.error_sorted_principle_axes.z"], # large -> small
					'legends': ["axis 1","axis 2","axis 3"],
					'linestyles': ['-','-','-'],
					'markers': ['d','d','d'],
					'colors':['b','r','g'],
					'loc':	"best",
					'grid': True,
					'linewidth' : 2,
					'fs_label' : 14,
					'fs_title' : 16,
					'fs_legend': 12,
					'xLim'	: False,
					'yLim'	: False
				}

		self.user_quats= {'title': "User Quaterion",
					'xLabel': "Trial [-]",
					'yLabel': "Unit quaternion [-]",
					'xAxis': "field.trial_nr",
					'yAxes': ["field.user_orientation.x","field.user_orientation.y",
							"field.user_orientation.z","field.user_orientation.w"], # large -> small
					'legends': ["q_x","q_y","q_z","q_w"],
					'linestyles': ['-','-','-','-'],
					'markers': ['d','d','d','d'],
					'colors':['b','r','g','k'],
					'loc':	"best",
					'grid': True,
					'linewidth' : 2,
					'fs_label' : 14,
					'fs_title' : 16,
					'fs_legend': 12,
					'xLim'	: False,
					'yLim'	: (-1,1)
				}

		self.avg_shape_error = dict(self.trial_time)
		keys = ['title','yLabel','yAxes','legends','yLim']
		values = ["Average Axes Shape Error (radius)", "Error [m]", ["field.shape"],["avg shape error"],False]
		for key,value in zip(keys,values):
			self.avg_shape_error[key] = value

		self.abs_angle = dict(self.trial_time)
		keys = [ 'title', 'yLabel', 'yAxes', 'legends', 'yLim' ]
		values = ["Absolute Angle Between Ellipsoids", "angle [deg]", ["field.absolute_angle"], ['angle'] ,(0,90)]
		for key,value in zip(keys,values):
			self.abs_angle[key] = value

		self.user_scales = dict(self.shape_error)
		keys = [ 'title', 'yLabel', 'yAxes', 'legends', 'yLim' ]
		values = ["User Scales", "size [m]", ["field.user_scales.x","field.user_scales.y","field.user_scales.z"], 
		["axis 1","axis 2","axis 3"], (0,0.5)]
		for key,value in zip(keys,values):
			self.user_scales[key] = value

		self.exp_scales = dict(self.user_scales)
		keys = [ 'title', 'yAxes']
		values = ["Experiment Scales", ["field.experiment_scales.x","field.experiment_scales.y","field.experiment_scales.z"]]
		for key,value in zip(keys,values):
			self.exp_scales[key] = value

		self.or_user_scales = dict(self.exp_scales)
		keys = [ 'title', 'yAxes']
		values = ["User Scales original", ["field.original_user_scales.x","field.original_user_scales.y","field.original_user_scales.z"]]
		for key,value in zip(keys,values):
			self.or_user_scales[key] = value

		self.exp_quats = dict(self.user_quats)
		self.exp_quats['title'] = 'Experiment Quaterion'
		self.exp_quats['yAxes'] = ['field.experiment_orientation.x','field.experiment_orientation.y',
								'field.experiment_orientation.z','field.experiment_orientation.w']

		self.or_user_quats = dict(self.exp_quats)
		self.or_user_quats['title'] = 'Original User Quaternion'
		self.or_user_quats['yAxes'] = ['field.original_user_orientation.x','field.original_user_orientation.y',
								'field.original_user_orientation.z','field.original_user_orientation.w']



	def singleDFPlot(self,df,plot_info):
		fig,ax = plt.subplots()

		ax.set_title(plot_info['title'],fontsize=plot_info['fs_title'])

		for i, yAxis in enumerate(plot_info['yAxes']):

			# set the plot in axis object for each specified y-value
			ax.plot(df[plot_info['xAxis']],df[yAxis],label=plot_info['legends'][i],
				linewidth=plot_info['linewidth'], linestyle=plot_info['linestyles'][i],
				marker=plot_info['markers'][i],color=plot_info['colors'][i])

		if plot_info['xLim'] != False:
			ax.set_ylim(plot_info['xLim'])

		if plot_info['yLim'] != False:
			ax.set_ylim(plot_info['yLim'])

		ax.set_xlabel(plot_info['xLabel'],fontsize=plot_info['fs_label'])
		ax.set_ylabel(plot_info['yLabel'],fontsize=plot_info['fs_label'])
		ax.legend(loc=plot_info['loc'],fontsize=plot_info['fs_legend'])
		if plot_info['grid']:
			ax.grid()
		
		return fig,ax

	def addLineFromDfToPlot(self,ax,df,plot_info):
		# get current legend ahndles and labels
		# needs specification in plot_info of:
		# xAxis, yAxes, legends, linewidth, linestyles, markers, colors, fs_legend, los

		handles, labels = ax.get_legend_handles_labels()

		# create new line/lines 
		# append to handles and labels list of legend
		# add plot to ax
		for i, yAxis in enumerate(plot_info['yAxes']):

			# create a new line from the plot info and the data frame
			newLine = Line2D(df[plot_info['xAxis']],df[yAxis],label=plot_info['legends'][i],
				linewidth=plot_info['linewidth'], linestyle=plot_info['linestyles'][i],
				marker=plot_info['markers'][i],color=plot_info['colors'][i])

			labels.append(plot_info['legends'][i])
			handles.append(newLine)



			ax.plot(df[plot_info['xAxis']],df[yAxis],label=plot_info['legends'][i],
					linewidth=plot_info['linewidth'], linestyle=plot_info['linestyles'][i],
					marker=plot_info['markers'][i],color=plot_info['colors'][i])

		if plot_info['xLim'] != False:
			ax.set_ylim(plot_info['xLim'])

		if plot_info['yLim'] != False:
			ax.set_ylim(plot_info['yLim'])

		# legend must be reacreated as it cannot be updated
		lgd = ax.get_legend()
		lgd._legend_box = None
		lgd._init_legend_box(handles, labels)
		lgd.set_title(lgd.get_title().get_text())


		ax.legend(loc=plot_info['loc'],fontsize=plot_info['fs_legend'])
		return ax


if __name__ == "__main__":
	########## THE GETTING DATA DICTIONAIR PART ##########
	# single participant
	# get relavant directories, experiment_name abreviations, to set the data in the ParticipantData class
	part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1'

	MD = ManageDataDirectories()
	exp_IDs = MD.experiment_IDs # ['1L','1R','2L','2R','3L','3R','4L','4R']
	topics = ["simple_experiment_data"]
	# get relavant directories containing the files
	partx_info_file_path, _, partx_csvdir_paths, partx_txt_paths, _ = MD.getAllPathsOfParticipant(part_dir, MD.csv_dir_list)
	csvfile_paths = MD.getFilesInDirList(partx_csvdir_paths,".csv")
	# print(csvfile_paths)

	# set the files in one dictionari
	part = ParticipantData(exp_IDs, partx_info_file_path, partx_txt_paths, partx_csvdir_paths, csvfile_paths, topics)
	data = part.data


	########## THE PROCESSING PART ##########
	proces = ProcessSimpleExperiment()

	# remove all the first trials
	proces.removeFirstTrials(data)

	# list data of 1 participant: use this to check with figures
	data_P1_1L = data['1L']['simple_experiment_data']
	data_P1_1R = data['1R']['simple_experiment_data']
	data_P1_2L = data['2L']['simple_experiment_data']
	data_P1_2R = data['2R']['simple_experiment_data']
	data_P1_3L = data['3L']['simple_experiment_data']
	data_P1_3R = data['3R']['simple_experiment_data']
	data_P1_4L = data['4L']['simple_experiment_data']
	data_P1_4R = data['4R']['simple_experiment_data']

	
	# add new columns that identify type of elipsoid 1dof case
	size_id_list = proces.getSizeID1DoF(data_P1_1L) #returns dataframe
	rot_id_list = proces.getRotIDText(data_P1_1L)
	proces.addColumns(data_P1_1L, ['size_type','rot_type'], [size_id_list,rot_id_list], [5,6])
	print(data['1L']['simple_experiment_data'].columns.values)
	print(data['1L']['simple_experiment_data'].index.values)
	print(data['1L']['simple_experiment_data'].loc[3,:])

	# add new column that identifes tupe of elliposdi in 2dof case
	size_id_list2 = proces.getSizeID2DoF(data_P1_4R) 
	rot_id_list2 = proces.getRotIDText(data_P1_4R)
	proces.addColumns(data_P1_4R, ['size_type','rot_type'], [size_id_list2,rot_id_list2], [5,6])
	print(data['4R']['simple_experiment_data'].columns.values)
	print(data['4R']['simple_experiment_data'].index.values)
	print(data['4R']['simple_experiment_data'].loc[3,:])


	# Set trials that are failed to None; specify manually the trial number e.g. 2 and 17
	# proces.removeIncorrectTrials(data_P1_3L, [3,17])
	# print(data_P1_3L.loc[3,:])
	# print(part.data['1L']['simple_experiment_data'].head(1))

	########## THE PLOTTING PART ##########
	# initialize plot class
	plot_exp = PlotSimpleExperiment()


	# (1) plot trial times
	# copy and set custom legends
	time1_info = dict(plot_exp.trial_time) 
	time1_info['legends'] = ['Part1_Exp1L']
	# plot using custom legend
	fig_time,ax_time = plot_exp.singleDFPlot(data_P1_3L, time1_info)

	# add trial times plot from other experiment
	# set new color for line
	# time2_info = dict(time1_info)
	# time2_info['colors'] = ['r']
	# time2_info['legends'] = ['Part1_Exp3L']
	# # add line to figure using other color
	# ax_time = plot_exp.addLineFromDfToPlot(ax_time, data_P1_3L, time2_info)

	# (2) plot accuracy
	acc_info = dict(plot_exp.accuracy)
	fig_acc, ax_acc = plot_exp.singleDFPlot(data_P1_3L, acc_info)

	# (3) plot shape error (diameter)
	shape_error_info = dict(plot_exp.shape_error)
	fig_shape, ax_shape = plot_exp.singleDFPlot(data_P1_3L, shape_error_info)

	# (4) plot average shape error (diameter)
	avg_shape_error_info = dict(plot_exp.avg_shape_error)
	fig_avgShape, ax_avgShape = plot_exp.singleDFPlot(data_P1_3L, avg_shape_error_info)

	# (5) plot angle
	angle_info = dict(plot_exp.abs_angle)
	fig_angle, ax_angle = plot_exp.singleDFPlot(data_P1_3L, angle_info)

	# (6) plot user scales
	user_scales_info = dict(plot_exp.user_scales)
	fig_us, ax_us = plot_exp.singleDFPlot(data_P1_3L, user_scales_info)

	# (7) plot experiment scales
	exp_scales_info = dict(plot_exp.exp_scales)
	fig_es, ax_es = plot_exp.singleDFPlot(data_P1_3L, exp_scales_info)

	# (8) plot originals experiment scales
	or_exp_scales_info = dict(plot_exp.or_user_scales)
	fig_oes, ax_oes = plot_exp.singleDFPlot(data_P1_3L, or_exp_scales_info)

	# (9) plot user quats
	user_quats_info = dict(plot_exp.user_quats)
	fig_us, ax_us = plot_exp.singleDFPlot(data_P1_3L, user_quats_info)

	# (10) plot experiment quats
	exp_quats_info = dict(plot_exp.exp_quats)
	fig_es, ax_es = plot_exp.singleDFPlot(data_P1_3L, exp_quats_info)

	# (11) plot originals experiment quats
	or_exp_quats_info = dict(plot_exp.or_user_quats)
	fig_oes, ax_oes = plot_exp.singleDFPlot(data_P1_3L, or_exp_quats_info)




	plt.show()
	# check what kind of set attributes the ax class has
	# ax_getters = [getter for getter in dir(ax) if 'get' in getter]
	# print(ax_getters)

	# # check what kind of set attributes the ax class has
	# ax_setters = [setter for setter in dir(ax) if 'set' in setter]
	# print(ax_setters)

	# # check what kind of set attributes the fig class has
	# fig_setter = [setter for setter in dir(fig) if 'set' in setter]
	# print(fig_setter)