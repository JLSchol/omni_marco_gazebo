#!/usr/bin/env python
from os import path
from ManageDataDirectories import ManageDataDirectories
from ParticipantData import ParticipantData
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
from matplotlib.lines import Line2D  
from matplotlib.patches import Patch
import pandas as pd
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
		
	def filterOnUniqueColumnValues(self, df, column_name):
		newDfList = []
		uniques = df[column_name].unique()
		for unique_x in uniques:
			# for each_unique in index_df:
			is_value = df[column_name] == unique_x
			newDf = df[is_value]
			newDfList.append(newDf)
		return newDfList, uniques

	def createNewDFs(self,newColumns,dfs,dfs_column_ids):
		# get data
		dfList = []
		for column_id in dfs_column_ids: # each column_id get his own df
			data = {}
			for newName,df in zip(newColumns,dfs):
				data[newName] = df[column_id]

			newDf = pd.DataFrame.from_dict(data)
			newDf = newDf.transpose()
			# newDf.sort_index(axis=0)
			dfList.append(newDf)

		return dfList


	def getRotIDText(self, df):
		deg_key = ['0'	,'30',	'36',	'45',	'60',	'72',	'90',	'108',	'120',	'135',	'144',	'150']
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
			if abs(round(float(qw),3)) == abs(round(float(deg_2_qw['0']),3)):
				text = '0'
				# 0 degrees thus no rotation axis...
				# direction_text = get_direction(qx, qy, qz)
				# text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['30']),3)):
				angle_text = '30'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['36']),3)):
				angle_text = '36'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['45']),3)):
				angle_text = '45'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['60']),3)):
				angle_text = '60'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['72']),3)):
				angle_text = '72'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['90']),3)):
				angle_text = '90'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['108']),3)):
				angle_text = '108'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['120']),3)):
				angle_text = '120'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['135']),3)):
				angle_text = '135'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['144']),3)):
				angle_text = '144'
				direction_text = get_direction(qx, qy, qz)
				text = combine_text(angle_text, direction_text)
				textList.append(text)

			elif abs(round(float(qw),3)) == abs(round(float(deg_2_qw['150']),3)):
				angle_text = '150'
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
					'yLim'	:(0,100)
				}

		self.shape_error = {'title': "Shape Error Axes (diameter)",
					'xLabel': "Trial [-]",
					'yLabel': "Error axes [m]",
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

		self.type_info= {'title': "Title",
					'xLabel': "Size [-]",
					'yLabel': "Performance [-]",

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



	def singlePlotDf(self,df,plot_info):
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

	def singleScatterPlotDf(self, df, plot_info):
		fig,ax = plt.subplots()

		ax.set_title(plot_info['title'],fontsize=plot_info['fs_title'])

		for i, yAxis in enumerate(plot_info['yAxes']):

			# set the plot in axis object for each specified y-value
			ax.scatter(df[plot_info['xAxis']],df[yAxis],label=plot_info['legends'][i],
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



	def plotTypes(self,df,title,title_fs,xLabel,yLabel,fs_label,legend,fs_legend,marker='*',color='b',grid=True):		
		# xl,yl = df.size()
		fig,ax = plt.subplots()
		
		ax.set_title(title,fontsize=title_fs)

		xticks = df.index.values
		x = range(len(xticks))
		for y in df.columns.values:
			ax.scatter(x,df[y],
				# label=plot_info['legends'][i],
				# linewidth=plot_info['linewidth'], linestyle=plot_info['linestyles'][i],
				marker=marker,color=color)

		ax.set_xticks(x)
		ax.set_xticklabels(xticks)
		ax.set_xlabel(xLabel,fontsize=fs_label)
		ax.set_ylabel(yLabel,fontsize=fs_label)
		ax.legend(legend, loc='best',fontsize=fs_legend)
		if grid:
			ax.grid()

		return fig,ax

	def saveFigs(self,figs,fig_names,out_dir,extension='.png'):
		for fig,fig_name in zip(figs,fig_names):
			fig.savefig(path.join(out_dir,(fig_name+extension)))


if __name__ == "__main__":
	########## THE GETTING DATA DICTIONAIR PART ##########
	# single participant
	# get relavant directories, experiment_name abreviations, to set the data in the ParticipantData class
	participant_nr = 2
	part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_'+str(participant_nr)

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

	# REMOVE ALL FIRST TRIALS
	proces.removeFirstTrials(data)

	# list data of 1 participant: use this to make switching with plots esier
	exp_data = {
					1: data['1L']['simple_experiment_data'],
					2: data['1R']['simple_experiment_data'],
					3: data['2L']['simple_experiment_data'],
					4: data['2R']['simple_experiment_data'],
					5: data['3L']['simple_experiment_data'],
					6: data['3R']['simple_experiment_data'],
					7: data['4L']['simple_experiment_data'],
					8: data['4R']['simple_experiment_data'],
	}
	exp_id = {
					1: '1_DoF_Front_Practice',
					2: '1_DoF_Front_Real',
					3: '2_DoF_Front_Practice',
					4: '2_DoF_Front_Real',
					5: '1_DoF_Horizon_Practice',
					6: '1_DoF_Horizon_Real',
					7: '2_DoF_Horizon_Practice',
					8: '2_DoF_Horizon_Real'
	}
	
	nr = 8
	dof = 2

	if dof == 1:
		# ADD COLLOMNS THAT IDENTIFY THE TYPE OF THE ELLIPSOID 1DOF
		size_id_list = proces.getSizeID1DoF(exp_data[nr]) #returns dataframe
		rot_id_list = proces.getRotIDText(exp_data[nr])
		proces.addColumns(exp_data[nr], ['size_type','rot_type'], [size_id_list,rot_id_list], [5,6])
	elif dof ==  2:
		# ADD COLLOMNS THAT IDENTIFY THE TYPE OF THE ELLIPSOID 2DOF
		size_id_list2 = proces.getSizeID2DoF(exp_data[nr]) 
		rot_id_list2 = proces.getRotIDText(exp_data[nr])
		proces.addColumns(exp_data[nr], ['size_type','rot_type'], [size_id_list2,rot_id_list2], [5,6])



	# FIX NEW DF SUCH THAT IT CAN BE PLOTTED (CAREFULL WEIRD PROCESS BELOW)
	size_dfs, uniques_sizes = proces.filterOnUniqueColumnValues(exp_data[nr],'size_type')
	rot_dfs, uniques_rot = proces.filterOnUniqueColumnValues(exp_data[nr],'rot_type')
	# creates dfs for every field in fieldList with index being the uniques and columns being the index of the 
	# createNewDFs(newColumns,dfs,dfs_column_ids):

	[size_time_df,size_accShape_df,size_accRot_df] = proces.createNewDFs(uniques_sizes,size_dfs,[
										'field.trial_time',"field.shape_acc","field.orientation_acc"]) #returns a list
	[rot_time_df,rot_accShape_df,rot_accRot_df] = proces.createNewDFs(uniques_rot,rot_dfs,[
										'field.trial_time',"field.shape_acc","field.orientation_acc"]) #returns a list





	# REMOVE INCORRECT TRIALS
	# Set trials that are failed to None; specify manually the trial number e.g. 2 and 17
	if nr == 7 and participant_nr == 2:
		proces.removeIncorrectTrials(exp_data[nr], [1,9])
	# print(exp_data[nr].loc[3,:])


	########## THE PLOTTING PART ##########
	# set output directory for figures
	out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_2" +"/"+exp_id[nr]
	# initialize plot class

	# plot types
	plot_exp = PlotSimpleExperiment()
	# shapes
	fig_s_t,ax_s_t = plot_exp.plotTypes(size_time_df,'Trial time vs Shape',16,'Size Type [-]','Time [s]',14,[exp_id[nr]],12)
	fig_s_as,ax_s_ac = plot_exp.plotTypes(size_accShape_df,'Shape Accuracy vs Shape',16,'Size Type [-]','Shape Accuracy [%]',14,[exp_id[nr]],12)
	fig_s_ao,ax_s_ao = plot_exp.plotTypes(size_accRot_df,'Orientation Accuracy vs Shape',16,'Size Type [-]','Orientation Accuracy [%]',14,[exp_id[nr]],12)
	# rotations
	fig_r_t,ax_st = plot_exp.plotTypes(rot_time_df,'Trial time vs Orientation',16,'Orientation Type [deg]','Time [s]',14,[exp_id[nr]],12)
	fig_r_as,ax_st = plot_exp.plotTypes(rot_accShape_df,'Shape Accuracy vs Orientation',16,'Orientation Type [deg]','Shape Accuracy [%]',14,[exp_id[nr]],12)
	fig_r_ao,ax_st = plot_exp.plotTypes(rot_accRot_df,'Orientation Accuracy vs Orientation',16,'Orientation Type [deg]','Orientation Accuracy [%]',14,[exp_id[nr]],12)

	type_figs = [fig_s_t,fig_s_as,fig_s_ao,fig_r_t,fig_r_as,fig_r_ao]
	type_names = ['typeSize_vs_time','typeSize_vs_accShape','typeSize_vs_accRot','typeOrientation_vs_time',
	'typeOrientation_vs_accShape','typeOrientation_vs_accRot']
	plot_exp.saveFigs(type_figs,type_names,out_dir,'.png')

	# (1) plot trial times
	# copy and set custom legends
	time1_info = dict(plot_exp.trial_time) 
	time1_info['legends'] = [exp_id[nr]]
	# plot using custom legend
	fig_time,ax_time = plot_exp.singlePlotDf(exp_data[nr], time1_info)

	# # add trial times plot from other experiment
	# # set new color for line
	# time2_info = dict(time1_info)
	# time2_info['colors'] = ['r']
	# time2_info['legends'] = [exp_id[nr]]
	# # add line to figure using other color
	# ax_time = plot_exp.addLineFromDfToPlot(ax_time, exp_data[nr], time2_info)

	# (2) plot accuracy
	acc_info = dict(plot_exp.accuracy)
	acc_info['title'] = exp_id[nr]
	fig_acc, ax_acc = plot_exp.singlePlotDf(exp_data[nr], acc_info)

	# (3) plot shape error (diameter)
	shape_error_info = dict(plot_exp.shape_error)
	shape_error_info['title'] = exp_id[nr]
	fig_shape, ax_shape = plot_exp.singlePlotDf(exp_data[nr], shape_error_info)

	# (4) plot average shape error (diameter)
	avg_shape_error_info = dict(plot_exp.avg_shape_error)
	fig_avgShape, ax_avgShape = plot_exp.singlePlotDf(exp_data[nr], avg_shape_error_info)

	# (5) plot angle
	angle_info = dict(plot_exp.abs_angle)
	fig_angle, ax_angle = plot_exp.singlePlotDf(exp_data[nr], angle_info)

	# (6) plot user scales
	user_scales_info = dict(plot_exp.user_scales)
	fig_us, ax_us = plot_exp.singlePlotDf(exp_data[nr], user_scales_info)

	# (7) plot experiment scales
	exp_scales_info = dict(plot_exp.exp_scales)
	fig_es, ax_es = plot_exp.singlePlotDf(exp_data[nr], exp_scales_info)

	# (8) plot originals experiment scales
	or_exp_scales_info = dict(plot_exp.or_user_scales)
	fig_oes, ax_oes = plot_exp.singlePlotDf(exp_data[nr], or_exp_scales_info)

	# (9) plot user quats
	user_quats_info = dict(plot_exp.user_quats)
	fig_uq, ax_uq = plot_exp.singlePlotDf(exp_data[nr], user_quats_info)

	# (10) plot experiment quats
	exp_quats_info = dict(plot_exp.exp_quats)
	fig_eq, ax_eq = plot_exp.singlePlotDf(exp_data[nr], exp_quats_info)

	# (11) plot originals experiment quats
	or_exp_quats_info = dict(plot_exp.or_user_quats)
	fig_oeq, ax_oeq = plot_exp.singlePlotDf(exp_data[nr], or_exp_quats_info)

	basic_figs = [fig_time,fig_acc,fig_shape,fig_avgShape,fig_angle,fig_us,fig_es,fig_oes,fig_uq,fig_eq,fig_oeq]
	basic_names = ["1_trial_time","2_accuracy","3_scale_error","4_average_scales","5_angle","6_user_scales",
	"7_exp_scales","8_exp_scales_original","9_user_quats","10_exp_quats","11_user_quats_original"]
	plot_exp.saveFigs(basic_figs,basic_names,out_dir,'.png')


	# more advanced plots


	# plt.draw()
	# check what kind of set attributes the ax class has
	# ax_getters = [getter for getter in dir(ax) if 'get' in getter]
	# print(ax_getters)

	# # check what kind of set attributes the ax class has
	# ax_setters = [setter for setter in dir(ax) if 'set' in setter]
	# print(ax_setters)

	# # check what kind of set attributes the fig class has
	# fig_setter = [setter for setter in dir(fig) if 'set' in setter]
	# print(fig_setter)