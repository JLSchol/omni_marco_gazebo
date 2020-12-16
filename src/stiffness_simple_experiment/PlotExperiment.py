#!/usr/bin/env python
from SimpleExperimentTopic import ProcessSimpleExperiment
from os import path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
from matplotlib import rcParams
from matplotlib import cm
from matplotlib import colors as col
from mpl_toolkits.mplot3d import Axes3D

from matplotlib.lines import Line2D  
from matplotlib.text import Text
from matplotlib.patches import Patch
from matplotlib import tight_layout
from matplotlib.font_manager import FontProperties
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass
import seaborn as sns
import itertools
import pandas as pd
import numpy as np
import scipy as sc
from tf.transformations import quaternion_matrix
# from scipy.spatial.transform import Rotation as R


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

		self.projected_scales = dict(self.user_scales)
		keys = [ 'title', 'yAxes']
		values = ["Projected Scales", ['projected_scales.x','projected_scales.y','projected_scales.z']]
		for key,value in zip(keys,values):
			self.projected_scales[key] = value

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

	def saveFigs(self,figs,fig_names,out_dir,extension='.eps'):
		for fig,fig_name in zip(figs,fig_names):
			fig.savefig(path.join(out_dir,(fig_name+extension)))

	def saveFigs2(self,figs,fig_names,out_dirs,extension='.eps'):
		for fig,fig_name,out_dir in zip(figs,fig_names,out_dirs):
			print(fig)
			print(fig_name)
			print(out_dir)
			fig.savefig(path.join(out_dir,(fig_name+extension)))

	def setColor(self,bp_dict,colorList=['#d15a5a', '#4d67bf','#74cf70']):
		for element in bp_dict.keys():

			plt.setp(bp_dict[element], color='k')
			if element == 'boxes':
				for i,line2D in enumerate(bp_dict[element]):
					line2D.set_facecolor(colorList[i])

	def BoxPlotGroup2axisTypes(self,dfList,column_names,tick_labels,part='All'):
		# TO DO
		# add title
		# get legend on nice position
		# add sample size on top?
		getData = []
		clean = lambda series: series.dropna().tolist()
		if part == 'All':
			getData = lambda df, part, fields: [clean(df.loc[:,field]) for field in fields]
		else:
			getData = lambda df, part, fields: [clean(df.loc[part,field]) for field in fields]


		colors = ['#d15a5a', '#4d67bf','#74cf70']

		fig,ax1 = plt.subplots()
		ax1.set_title('Performance measures per trial type',fontsize=18)
		ax1.set_ylabel('accuracy [%]')
		ax1.set_ylim(40,100)
		ax2 = ax1.twinx() # second axisObj that has x-axis in common
		ax2.set_ylabel('time [s]')
		ax2.set_ylim(0,35)

		p1=1 # skip zero tick of box plot
		tick_values=[]
		for df in dfList:
			# print(part)
			ys = getData(df,part,column_names)
			# ys = [df.loc[:,name].dropna().tolist() for name in column_names]

			p2=p1+1 # move p2 1 tick to the right of p1
			p3=p2+1 # move p3 1 tick to the right of p2
			# print(ys[2])

			bp = ax1.boxplot([ys[0],ys[1]], positions=[p1, p2], widths=0.8, patch_artist=True)	
			bp2 = ax2.boxplot(ys[2], positions=[p3], widths=0.8, patch_artist=True)	
			# add bp2 to bp
			for key in bp.keys():
				bp[key].extend(bp2[key])

			# set color
			self.setColor(bp,colors)
			# updat position 1	
			p1 += 4 # skip 4 ticks: p1,p2,p3,emptyspace
			tick_values.append(p2)
			# label_list.append(label)
		# ax1.axhline(85, color=colors[0], lw=2)
		# ax1.axhline(90, color='black',ls='--',lw=2)

		# ax1.axhline(70, color=colors[1],lw=2)
		# ax1.axhline(71.28, color='black',ls='--',lw=2)

		x_upper_lim = 4*len(dfList)

		ax2.set_xticks(tick_values)
		# ax2.set_xticklabels(tick_labels,rotation=45)
		ax2.set_xlim(0,x_upper_lim)

		ax2.legend([bp["boxes"][0],bp["boxes"][1],bp["boxes"][2]],['shape [%]','orientation [%]','trial time [s]'],
						loc='center left',fontsize='10')

		ax1.set_xticklabels(tick_labels,rotation=70)
		# ax1.set_xmargin()
		fig.tight_layout()
		return fig

	def BoxPlotGroup2axis(self,exp1,exp2,exp3,exp4):
		# add sample size+significance on top?

		colors = ['#d15a5a', '#4d67bf','#74cf70']

		fig,ax1 = plt.subplots()
		ax1.set_title('Performance measures per condition',fontsize=18)
		ax1.set_ylabel('accuracy [%]')
		ax1.set_ylim(40,100)
		ax2 = ax1.twinx() # second axisObj that has x-axis in common
		ax2.set_ylabel('time [s]')
		ax2.set_ylim(0,35)

		p1=1 # skip zero tick of box plot
		for exp_i, exp_x in enumerate([exp1,exp2,exp3,exp4]):
			# update position 2,3
			p2=p1+1 # move p2 1 tick to the right of p1
			p3=p2+1 # move p3 1 tick to the right of p2

			bp = ax1.boxplot([exp_x[0],exp_x[1]], positions=[p1, p2], widths=0.4, patch_artist=True)	
			bp2 = ax2.boxplot(exp_x[2], positions=[p3], widths=0.4, patch_artist=True)	
			# add bp2 to bp
			for key in bp.keys():
				bp[key].extend(bp2[key])


			# set color
			self.setColor(bp,colors)
			# updat position 1	
			p1 += 4 # skip 4 ticks: p1,p2,p3,emptyspace

		# ax1.axhline(85, color=colors[0], lw=2)
		# ax1.axhline(90, color='black',ls='--',lw=2)

		# ax1.axhline(70, color=colors[1],lw=2)
		# ax1.axhline(71.28, color='black',ls='--',lw=2)

		tick_values = [2, 6, 10, 14]
		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']
		ax2.set_xticks(tick_values)
		ax2.set_xticklabels(tick_labels)
		ax2.set_xlim(0,16)

		ax2.legend([bp["boxes"][0],bp["boxes"][1],bp["boxes"][2]],['shape','orientation','trial time'],
						loc='center left',fontsize='10')

		return fig

		

	def boxPlotExpMetric(self,exp_data):
		fig,ax = plt.subplots()
		
		
		bp = ax.boxplot(exp_data[0],exp_data[1],exp_data[2],exp_data[3])

		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']
		# ax.set_xticks(tick_values)
		ax.set_xticklabels(tick_labels)

		return fig

	def generateRawDataPlots(self, all_dfs, base_path, plot_part='all'):
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
		part_nr = 0
		for part_dfs in all_dfs: # for loop over all participants participants
			part_nr +=1
			for plot_part_nr in plot_part:
				if plot_part == 'all' or plot_part_nr == part_nr: 
					for exp_nr,df in part_dfs.items(): # loop over 8 conditions
						# (1) plot trial times
						# copy and set custom legends
						time1_info = dict(self.trial_time) 
						time1_info['legends'] = [exp_id[exp_nr]]
						# plot using custom legend
						fig_time,ax_time = self.singlePlotDf(df, time1_info)

						# add trial times plot from other experiment (1-2)
						# set new color for line
						# time2_info = dict(time1_info)
						# time2_info['colors'] = ['r']
						# time2_info['legends'] = [exp_id[exp_nr]]
						# # add line to figure using other color
						# ax_time = self.addLineFromDfToPlot(ax_time, df, time2_info)

						# (2) plot accuracy
						acc_info = dict(self.accuracy)
						acc_info['title'] = exp_id[exp_nr]
						fig_acc, ax_acc = self.singlePlotDf(df, acc_info)

						# (3) plot shape error (diameter)
						shape_error_info = dict(self.shape_error)
						shape_error_info['title'] = exp_id[exp_nr]
						fig_shape, ax_shape = self.singlePlotDf(df, shape_error_info)

						# (4) plot average shape error (diameter)
						avg_shape_error_info = dict(self.avg_shape_error)
						fig_avgShape, ax_avgShape = self.singlePlotDf(df, avg_shape_error_info)

						# (5) plot angle
						angle_info = dict(self.abs_angle)
						fig_angle, ax_angle = self.singlePlotDf(df, angle_info)

						# (6) plot user scales
						user_scales_info = dict(self.user_scales)
						fig_us, ax_us = self.singlePlotDf(df, user_scales_info)

						# (7) plot projected scales
						proj_scales_info = dict(self.projected_scales)
						fig_ps, ax_ps = self.singlePlotDf(df, proj_scales_info)
						
						# (8) plot experiment scales
						exp_scales_info = dict(self.exp_scales)
						fig_es, ax_es = self.singlePlotDf(df, exp_scales_info)


						# (9) plot originals experiment scales
						or_exp_scales_info = dict(self.or_user_scales)
						fig_oes, ax_oes = self.singlePlotDf(df, or_exp_scales_info)

						# (10) plot user quats
						user_quats_info = dict(self.user_quats)
						fig_uq, ax_uq = self.singlePlotDf(df, user_quats_info)

						# (11) plot experiment quats
						exp_quats_info = dict(self.exp_quats)
						fig_eq, ax_eq = self.singlePlotDf(df, exp_quats_info)

						# (12) plot originals experiment quats
						or_exp_quats_info = dict(self.or_user_quats)
						fig_oeq, ax_oeq = self.singlePlotDf(df, or_exp_quats_info)

						basic_figs = [fig_time,fig_acc,fig_shape,fig_avgShape,fig_angle,fig_us,fig_ps,fig_es,fig_oes,fig_uq,fig_eq,fig_oeq]
						basic_names = ["1_trial_time","2_accuracy","3_scale_error","4_average_scales","5_angle","6_user_scales",
						"7_proj_scales","8_exp_scales","9_exp_scales_original","10_user_quats","11_exp_quats","12_user_quats_original"]
						
						out_dir = base_path + str(part_nr)+"/"+exp_id[exp_nr]

						self.saveFigs(basic_figs,basic_names,out_dir,'.eps')
						plt.close('all')
						print('RUSTAAAGH!!!, plotjes worden gemaakt')
				else:
					continue


		plt.close('all')


	def generateBoxExp(self, exp_dfs, part_list ,base_path, real='real'):
		# some helpfull functions
		clean = lambda series: series.dropna().tolist()
		getPartData = lambda df, part, fields: [clean(df.loc[part,field]) for field in fields]
		getAllPartData = lambda df, fields: [clean(df.loc[:,field]) for field in fields]
		get_fig_name = lambda ID, ID2: 'box_time_accuracy_' + str(ID) + '_' + str(ID2)
		get_out_dir = lambda base_path, ID, : base_path + str(ID)

		# get plot data for the individual participnats and add to plot_data
		plot_data = []
		box_names = []
		out_dirs = []
		for part in part_list: # over participant names 'part_1, part_2 etc'
			plot_data_part_x = []

			for cond_i in exp_dfs: # over experiment conditions 1-4
				metrics_i = getPartData(cond_i, part, ['field.shape_acc','field.orientation_acc','field.trial_time'])
				plot_data_part_x.append(metrics_i)
			
			plot_data.append(plot_data_part_x)
			box_names.append(get_fig_name(real,part))
			out_dirs.append(get_out_dir(base_path,part))

		# get plot data for the participants combined and add this to the end of plot data
		plot_data_all_part = []
		for cond_i in exp_dfs: # over experiment conditions 1-4
			metrics_i = getAllPartData(cond_i, ['field.shape_acc','field.orientation_acc','field.trial_time'])
			plot_data_all_part.append(metrics_i)
		plot_data.append(plot_data_all_part)
		box_names.append(get_fig_name(real,'All'))
		out_dirs.append(get_out_dir(base_path,'All'))

		# plot all figures with the combined data of particpant ad the end
		box_figs = []
		for fig_x_data in plot_data:
			fig_x_time_acc = self.BoxPlotGroup2axis(fig_x_data[0], fig_x_data[1], fig_x_data[2], fig_x_data[3])
			box_figs.append(fig_x_time_acc)
		# plt.show()

		self.saveFigs2(box_figs,box_names,out_dirs,'.eps')
		plt.close('all')

	def generateBoxTypes(self, types_dfs, id_strings, part_list, base_path):
		fields = ['field.shape_acc','field.orientation_acc','field.trial_time']

		get_fig_name = lambda ID: 'box_time_acc_type_' + str(ID) 
		get_out_dir = lambda base_path, ID, : base_path + str(ID)

		box_figs = []
		partsPlusAll = part_list + ['All']
		for part in partsPlusAll:
			box_figs_part_x =[]
			box_names = []
			out_dirs = []
			for [lower_i,upper_i,ID] in [[0,8,'exp1'],[8,18,'exp2'],[18,26,'exp3'],[26,36,'exp4'],[0,36,'all']]: 
				figures_part_x_cond_i = self.BoxPlotGroup2axisTypes(
								types_dfs[lower_i:upper_i],fields,id_strings[lower_i:upper_i],part)

				if ID == 'all':
					figures_part_x_cond_i.set_size_inches(20,8, forward=True)

				box_figs_part_x.append(figures_part_x_cond_i)
				name = get_fig_name(ID)
				out_dir = get_out_dir(base_path, part)
			
				box_figs.append(box_figs_part_x)
				box_names.append(name)
				out_dirs.append(out_dir)

			self.saveFigs2(box_figs_part_x,box_names,out_dirs,'.eps')

		plt.close('all')

	def plotVanDerLaan(self, usefulness, satisfying):
		fig,ax = plt.subplots()

		sc = ax.scatter(usefulness[0],satisfying[0], label='participants (n=8)')
		ax.axhline(0,-2,2)
		ax.axvline(0,-2,2)

		title = "Van der Laan Questionnaire Scores"
		# ax.set_title(title,fontsize=18)
		ax.set_title(title,fontsize=20)

		# ax.legend(loc='upper left')
		ax.legend(loc='upper left',fontsize=16)


		ax.set_xlabel("usefulness ($M$={}, $SD$={}) ".format(round(usefulness[1],2),round(usefulness[2],2),fontsize=14))
		ax.set_ylabel("satisfying ($M$={}, $SD$={}) ".format(round(satisfying[1],2),round(satisfying[2],2),fontsize=14))
		ax.xaxis.label.set_size(16)
		ax.yaxis.label.set_size(16)
		# ax.set_xlabel("usefulness" ,fontsize=14)
		# ax.set_ylabel('satisfying',fontsize=14)

		ax.set_xlim(-2,2)
		ax.set_ylim(-2,2)

		# plt.show()
		return fig

	def barPlot2Axis(self, df_means, df_stds, part='all'):

		colors = ['#d15a5a', '#4d67bf']
		fig,ax1 = plt.subplots()
		ax1.set_title('Accuracy metrics per condition ',fontsize=18)
		ax1.set_ylabel('error [-]')
		ax1.set_ylim(0,1)
		ax2 = ax1.twinx() # second axisObj that has x-axis in common
		ax2.set_ylabel('angle [deg]')
		ax2.set_ylim(0,90)
		ax2.set_xlim(0,13)

		width = 0.9

		tick_values = [1, 4, 7, 10]
		for exp_i_mean, exp_i_std, idx in zip(df_means, df_stds, tick_values):

			mean1, std1 = exp_i_mean.loc[part,'normalizedErr'],exp_i_std.loc[part,'normalizedErr']
			mean2, std2 = exp_i_mean.loc[part,'field.absolute_angle'],exp_i_std.loc[part,'field.absolute_angle']

			bar1 = ax1.bar(idx, mean1, width=0.9, color=colors[0], lw=2)
			ax1.errorbar(idx+width/2, mean1, yerr=std1, ecolor='black', elinewidth=2)

			bar2 = ax2.bar(idx+1, mean2, width=0.9, color=colors[1], lw=2)
			ax2.errorbar(idx+1+width/2, mean2, yerr=std2, ecolor='black', elinewidth=2)

		ax1.axhline(0.1, color=colors[0], lw=2)
		ax1.axhline(0.1, color='black',ls='--',lw=2)

		ax2.axhline(25.84, color=colors[1],lw=2)
		ax2.axhline(25.84, color='black',ls='--',lw=2)

		tick_values = [2, 5, 8, 11]
		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']

		ax2.set_xticks(tick_values)
		ax2.set_xticklabels(tick_labels)
		ax2.legend((bar1,bar2),('normalized error','angle'),loc='best')

		return fig

	def barPlot1Axis(self, df_means, df_stds, part='all'):
		colors = ['#d15a5a', '#4d67bf']
		fig,ax = plt.subplots()
		ax.set_title('Accuracy measure per condition ',fontsize=18)
		ax.set_ylabel('accuracy [%]')
		ax.set_ylim(0,100)
		ax.set_xlim(0,13)
		tick_values = [1, 4, 7, 10]
		width = 0.9
		for exp_i_mean, exp_i_std, idx in zip(df_means, df_stds, tick_values):
			# if idx ==1:
			# 	print(exp_i_mean.loc[:,'field.absolute_angle'])
			# 	print(exp_i_std.loc[:,'field.absolute_angle'])
			# 	print()
			mean1, std1 = exp_i_mean.loc[part,'field.shape_acc'],exp_i_std.loc[part,'field.shape_acc']
			mean2, std2 = exp_i_mean.loc[part,'field.orientation_acc'],exp_i_std.loc[part,'field.orientation_acc']

			bar1 = ax.bar(idx, mean1, width=width, color=colors[0], lw=2)
			ax.errorbar(idx+width/2, mean1, yerr=std1, ecolor='black', elinewidth=2)

			bar2 = ax.bar(idx+1, mean2, width=width, color=colors[1], lw=2)
			ax.errorbar(idx+1+width/2, mean2, yerr=std2, ecolor='black', elinewidth=2)

		ax.axhline(90, color=colors[0], lw=2)
		ax.axhline(90, color='black',ls='--',lw=2)

		ax.axhline(71.28, color=colors[1],lw=2)
		ax.axhline(71.28, color='black',ls='--',lw=2)

		tick_values = [2, 5, 8, 11]
		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']

		ax.set_xticks(tick_values)
		ax.set_xticklabels(tick_labels)
		ax.grid()
		ax.legend((bar1,bar2),('size accuracy','orientation accuracy'),loc='best')
		# plt.draw()
		return fig

	def generateVanDerLaan(self, usefulness, satisfying, out_dir):
		fig = self.plotVanDerLaan(usefulness, satisfying)
		self.saveFigs2([fig], ['van_der_laan'], [out_dir])
		

	def generateBarPlot1Axis(self, df_means, df_stds, part_list, base_path):
		get_fig_name = lambda ID: 'bar_accuracy_exp_' + str(ID) 
		get_out_dir = lambda base_path, ID, : base_path + str(ID)

		bar_figs = []
		bar_names = []
		our_dirs = []
		part_ids = part_list + ['all']
		sub_dirs = part_list + ['All']
		for part_id, part_sub_dir in zip(part_ids,sub_dirs):

			bar_fig = self.barPlot1Axis(df_means, df_stds, part=part_id)

			bar_figs.append(bar_fig)
			bar_names.append(get_fig_name(part_id))
			our_dirs.append(get_out_dir(base_path, part_sub_dir))

		self.saveFigs2(bar_figs,bar_names,our_dirs,'.eps')
		plt.close('all')


	def generateBarPlot2Axis(self, df_means, df_stds, part_list, base_path):
		get_fig_name = lambda ID: 'bar_error_angle_exp_' + str(ID) 
		get_out_dir = lambda base_path, ID, : base_path + str(ID)

		bar_figs = []
		bar_names = []
		our_dirs = []
		part_ids = part_list + ['all']
		sub_dirs = part_list + ['All']
		for part_id, part_sub_dir in zip(part_ids,sub_dirs):

			bar_fig = self.barPlot2Axis(df_means, df_stds, part=part_id)

			bar_figs.append(bar_fig)
			bar_names.append(get_fig_name(part_id))
			our_dirs.append(get_out_dir(base_path, part_sub_dir))

		self.saveFigs2(bar_figs,bar_names,our_dirs,'.eps')
		plt.close('all')

	
	def BoxPlotGroup2axisPerPart(self,exp1,exp2,exp3,exp4):
		# TO DO
		# add title
		# get legend on nice position
		# add sample size on top?

		colors = ['#d15a5a', '#4d67bf','#74cf70']

		fig,ax1 = plt.subplots()
		ax1.set_ylabel('accuracy [%]')
		ax1.set_ylim(50,100)
		ax2 = ax1.twinx() # second axisObj that has x-axis in common
		ax2.set_ylabel('time [s]')
		ax2.set_ylim(0,20)

		p1=1 # skip zero tick of box plot
		for exp_i, exp_x in enumerate([exp1,exp2,exp3,exp4]):
			# update position 2,3
			p2=p1+1 # move p2 1 tick to the right of p1
			p3=p2+1 # move p3 1 tick to the right of p2

			bp = ax1.boxplot([exp_x[0],exp_x[1]], positions=[p1, p2], widths=0.4, patch_artist=True)	
			bp2 = ax2.boxplot(exp_x[2], positions=[p3], widths=0.4, patch_artist=True)	
			# add bp2 to bp
			for key in bp.keys():
				bp[key].extend(bp2[key])


			# set color
			self.setColor(bp,colors)
			# updat position 1	
			p1 += 4 # skip 4 ticks: p1,p2,p3,emptyspace

		ax1.axhline(90, color=colors[0], lw=2)
		ax1.axhline(90, color='black',ls='--',lw=2)

		ax1.axhline(71.28, color=colors[1],lw=2)
		ax1.axhline(71.28, color='black',ls='--',lw=2)

		tick_values = [2, 6, 10, 14]
		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']
		ax2.set_xticks(tick_values)
		ax2.set_xticklabels(tick_labels)
		ax2.set_xlim(0,16)

		ax2.legend([bp["boxes"][0],bp["boxes"][1],bp["boxes"][2]],['shape','orientation','trial time'],
						loc='lower left',fontsize='10')

		return fig

	# use
	def singleMetricDofPlanes4box(self, df, p_dof, p_plane, fig_info, swarm, annotate=True):

		fig, ax = plt.subplots()

		palet = sns.color_palette('colorblind')

		# boxplot
		sns.boxplot(x='DoF', y=fig_info['field'], hue='plane', data=df, palette=palet, ax=ax, showfliers=False)

		# swarm plot
		if swarm['plot']:
			sns.swarmplot(ax=ax, x='DoF', y=fig_info['field'], hue='plane', 
				size=swarm['size'], marker=swarm['marker'],linewidth=swarm['lw'],edgecolors=swarm['ec'], 
																dodge=True, data=df, palette=palet)

		fig, ax, xfrac = self.formatLegendBox(ax, fig)

		if annotate:
			self.annotateboxplot(fig, ax, [p_dof,p_plane], fig_info['field'], xfrac)

		ax.set_title(fig_info['title'])
		ax.set_ylabel(fig_info['yLabel'])

		ax.set_title(fig_info['title'], fontsize=20)
		ax.set_ylabel(fig_info['yLabel'], fontsize=16)
		ax.set_xlabel('DoF', fontsize=16)
		ax.set_xticklabels(['1 DoF', '2 DoF'], fontsize=16)

		return fig

	def singleMetricDofPlanesSizes2box(self, df, x, p_value, info, swarm):

		fig, ax = plt.subplots()
		palet = sns.color_palette('colorblind')

		# set order of plot
		order = []
		x_label = []
		if x == 'DoF':
			order = ['1_DoF', '2_DoF']
			x_tick_labels = ['1 DoF', '2 DoF'] # matched order with pandas dataframe statistics
		elif x == 'plane':
			order = ['horizontal', 'vertical']
			x_tick_labels = ['horizontal', 'vertical']
		elif x == 'size':
			order = ['large','small']
			x_tick_labels = ['large','small']

		# plot boxplot
		# print(df[x].unique())


		sns.boxplot(data=df, x=x, y=info['field'], order=order,
					ax=ax, palette=palet,  showfliers=False)
		# plot swarm plot
		sns.swarmplot(	data=df, x=x, y=info['field'], order=order,
						size=swarm['size'], marker=swarm['marker'],linewidth=swarm['lw'],edgecolors=swarm['ec'], 
						dodge=True,  palette=palet, ax=ax)


		# annotate pvalue
		if p_value <= 0.05:
			self.topannotation(ax, p_value)

		# set other figure properties
		ax.set_title(info['title'], fontsize=20)
		ax.set_ylabel(info['yLabel'], fontsize=16)
		ax.set_xlabel(x, fontsize=16)
		ax.set_xticklabels(x_tick_labels, fontsize=16)

		return fig

	# def singleMetric(self, dof_and_planes, metric, yAxis, swarm):
	def singleMetric(self, dof_and_planes, fig_info, swarm, annotate=True):

		if not isinstance(dof_and_planes, list):
			dof_and_planes = [dof_and_planes]
			# yAxis = [yAxis]
		# df should contan one column with scores
		# and identifiers for the dat as dofs and planes in additional columns
		# df = pd.DataFrame((_ for _ in itertools.izip_longest(*[dof1v,dof2v,dof1h,dof2h])), columns=['dof1v', 'dof2v', 'dof1h', 'dof2h'])
		dfList=[]
		dofs = ['1_dof','2_dof','1_dof','2_dof']
		planes = ['vertical','vertical','horizontal','horizontal']
		for serie,dof,plane in zip(dof_and_planes,dofs,planes):
			df = pd.DataFrame()
			df[fig_info['yLabel']] = serie
			df['Dof'] = dof
			df['Plane'] = plane
			dfList.append(df)
		newdf = pd.concat(dfList)

		fig, ax = plt.subplots()

		# boxplot
		sns.boxplot(y=fig_info['yLabel'], x='Dof', data=newdf, palette='colorblind', hue='Plane', ax=ax, showfliers=False)

		# swarm plot
		if swarm['plot']:
			sns.swarmplot(ax=ax, x="Dof", y=fig_info['yLabel'], hue="Plane", size=swarm['size'], 
								marker=swarm['marker'],linewidth=swarm['lw'],edgecolors=swarm['ec'], 
									dodge=True, data=newdf, palette="colorblind")


		fig, ax, xfrac = self.formatLegendBox(ax, fig)

		if annotate:
			self.annotateboxplot(fig, ax, swarm['tstats'], fig_info['field'], xfrac)

		ax.set_title(fig_info['title'])
		ax.set_ylim(fig_info['yLim']) # only set lower limit		
		# fig.set_size_inches(20,8, forward=True)
		return fig

	# still in use?????? think not 
	def singleMetricSize(self, df, x_field, fig_info, swarm=False):

		fig, ax = plt.subplots()

		# boxplot

		# x = 'Dof' or 'plane'
		# y = 'field.shape_acc' (field of the metric)
		# hue = 'size'
		print(fig_info)
		sns.boxplot(data=df, x=x_field, y=fig_info['field'], hue='size', 
					ax=ax, palette='colorblind',  showfliers=True)

		# swarm plot
		if swarm['plot']:
			sns.swarmplot(	data=df, x=x_field, y=fig_info['field'], hue='size', 
							size=swarm['size'], marker=swarm['marker'],linewidth=swarm['lw'],edgecolors=swarm['ec'], 
							dodge=True,  palette="colorblind", ax=ax)


		fig, ax, xfrac = self.formatLegendBox(ax, fig)


		self.annotateboxplot(fig, ax, swarm['tstats'], fig_info['field'], xfrac)

		ax.set_title(fig_info['title'])
		ax.set_ylabel(fig_info['yLabel'])
		# ax.set_ylim(fig_info['yLim']) # only set lower limit		
		# fig.set_size_inches(20,8, forward=True)
		# plt.show()
		return fig

	# used in sizeFigure 2 boxplots
	def signelMetricSize2(self,df, stats, info, swarm):
		x = 'size'
		
		fig, ax = plt.subplots()

		sns.boxplot(data=df, x='size', y=info['field'], order=['small','large'],
					ax=ax, palette='colorblind',  showfliers=True)

		sns.swarmplot(	data=df, x=x, y=info['field'], order=['small','large'],
						size=swarm['size'], marker=swarm['marker'],linewidth=swarm['lw'],edgecolors=swarm['ec'], 
						dodge=True,  palette="colorblind", ax=ax)


		p_value = stats.loc[info['field'],'psl']
		if p_value <= 0.05:
			self.topannotation(ax, p_value)

		ax.set_title(info['title'])
		ax.set_ylabel(info['yLabel'])
		return fig
		# ax.legend()
		# handles, labels = ax.get_legend_handles_labels()
		# print(handles)
		# print(labels)
		# legend = ax.legend(handles[0:2],labels[0:2], loc="best")

	def formatLegendBox(self, ax, fig):
		# Set Correct Legend
		handles, labels = ax.get_legend_handles_labels()

		legend_ofset = 0.46
		x_anch = 1+legend_ofset
		y_anch = 0.5
		shrink_x = 1 - legend_ofset/x_anch

		legend = ax.legend(handles[0:2],labels[0:2], loc="center right", bbox_to_anchor=(x_anch, y_anch), borderaxespad=0., fontsize=14)  # Set custom legend

		box = ax.get_position()
		ax.set_position([box.x0, box.y0, box.width*shrink_x, box.height])

		legendBbox = legend.get_bbox_to_anchor()

		return fig, ax, shrink_x


	def annotateboxplot(self, fig, ax, p_values, metric, xfrac):

		p12, pvh = [], []
		if isinstance(p_values, list) :
			p12, pvh = p_values[0], p_values[1]
		else:

			p12 = p_values.loc[metric,'p12']
			pvh = p_values.loc[metric,'pvh']

		significant = 0.05

		if p12 <= significant:
			self.topannotation(ax, p12)
		if pvh <= significant:
			self.legendannotation(ax, pvh, xfrac)#-.25, .25, maxval, p2)


	def topannotation(self, ax, p, yrel1=0.9, yrel2=0.95):
		# get x positions
		[x1, x2] = ax.get_xticks()

		# get y positions
		[ymin, ymax] = ax.get_ybound()
		h = ymax - ymin
		[y1, y2] = [ymin + (yrel1*h), ymin + (yrel2*h)]

		# plot line
		x = [x1, x1, x2, x2]
		y = [y1, y2, y2, y1]
		line = Line2D(x,y, linewidth=1.5, color='black')
		ax.add_line(line)

		# plot significance text markers: ***
		dotString = self.sigDots(p)
		fontdict = {'size': 16, 'weight': 'bold'}
		ax.text((x1+x2)*0.5, y2, dotString, fontdict=fontdict, ha='center', va='baseline')

	def legendannotation(self, ax, p, xfrac):
		arrowfraction = 0.3

		arrowx  = xfrac + 0.01
		arrowy0 = 0.46
		arrowy1 = 0.54
		arrowheight = arrowy1-arrowy0
		arrowwidth  = arrowheight * arrowfraction

		ann	= ax.annotate('', xy=(arrowx, arrowy0), xycoords='figure fraction',
				xytext=(arrowx, arrowy1), textcoords='figure fraction',
				fontsize=18,
				fontweight='bold',
				annotation_clip=False,
				arrowprops=dict(arrowstyle="-",
				connectionstyle="bar, fraction={}".format(arrowfraction),
				lw=1.5,
				ec="k"))
		anndot = ax.annotate(self.sigDots(p), xy=(arrowx-arrowwidth, arrowy0+arrowheight*.5), xycoords='figure fraction',
					textcoords='figure fraction',
					# fontdict = {'size': 16, 'weight': 'bold'},
					ha="center",
					va="center",
					fontsize=16,
					fontweight='bold',
					rotation=90,
					annotation_clip=False)

	def sigDots(self,p):
		dotstr = ""
		ding = '*'
		# ding = u'\25CF'
		if p<0.05:
		    dotstr += ding
		if p<0.01:
		    dotstr += ding
		if p<0.001:
		    dotstr += ding

		return dotstr

	




if __name__ == "__main__":
	PSE = PlotSimpleExperiment()
	process = ProcessSimpleExperiment()
	process.main()


	plt.show()



	# SHOW BAR PLOTS
	# print(type(process.means_real_exp))
	# for exp_means, exp_stds in zip(process.means_real_exp, process.stds_real_exp)
	# 	angles_mean = exp_means.loc['all','field.absolute_angle']
	# 	angles_std = exp_stds.loc['all','field.absolute_angle']
	# print(angles)
	# process.stds_real_exp
	# for part in ['part_1','part_2','part_3','part_4','part_5','part_6','part_7','part_8','all']:
	# 	PSE.barPlot1Axis(process.means_real_exp, process.stds_real_exp, part)
	# 	PSE.barPlot2Axis(process.means_real_exp, process.stds_real_exp, part)
	# PSE.barPlot1Axis(process.means_real_exp, process.stds_real_exp, 'all')
	# PSE.barPlot2Axis(process.means_real_exp, process.stds_real_exp, 'all')
	# plt.show()

	# GENERATE VAN DER LAAN PLOTS
	# out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	# PSE.generateVanDerLaan(process.usefull, process.satisfying, out_dir) # generate for all participants
	# plt.close()

	## GENERATE RAW DATA PLOTS
	# base_path_raw = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_"
	# PSE.generateRawDataPlots(process.all_dfs,base_path_raw, [8]) # if want to add specific participant(s)
	# PSE.generateRawDataPlots(process.all_dfs,base_path_raw) # plot for all participants
	# plt.close()

	# # GENERATE BOXPLOTS PER CONDITION
	# base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	# PSE.generateBoxExp(process.real_exp_dfs, process.part_name_list, base_path) # plot for all participants
	# plt.close()

	# GENERATE BOXPLOTS PER TRIAL TYPE
	# PSE.generateBoxTypes(process.types_dfs, process.IDstrings, process.part_name_list, base_path) # plot for all participants
	# plt.close()

	# # GENERATE BAR PLOTS
	# PSE.generateBarPlot1Axis(process.means_real_exp, process.stds_real_exp, process.part_name_list, base_path) # plot for all participants
	# PSE.generateBarPlot2Axis(process.means_real_exp, process.stds_real_exp, process.part_name_list, base_path) # plot for all participants
	# plt.close()




	## OLD FOR WHEN ONLY ADDING ONE PARTICIPANT TO BOXPLOTS
	# base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	# # PSE.generateBoxTypes(process.types_dfs, process.IDstrings, process.part_name_list, base_path) # plot for all participants
	# PSE.generateBoxTypes(process.types_dfs, process.IDstrings, ['part_8'], base_path) # if want to add specific participant(s)
	# plt.close()