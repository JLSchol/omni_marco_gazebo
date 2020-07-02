#!/usr/bin/env python
from os import path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gs
from matplotlib.lines import Line2D  
from matplotlib.patches import Patch
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
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

	def setColor(self,bp_dict,colorList=['red','blue','green']):
		for element in bp_dict.keys():

			plt.setp(bp_dict[element], color='k')
			if element == 'boxes':
				for i,line2D in enumerate(bp_dict[element]):
					line2D.set_facecolor(colorList[i])

	def BoxPlotGroup2axisTypes(self,dfList,column_names,tick_labels):
		# TO DO
		# add title
		# get legend on nice position
		# add sample size on top?

		colors = ['red','blue','green']

		fig,ax1 = plt.subplots()
		ax1.set_ylabel('accuracy [%]')
		ax1.set_ylim(50,100)
		ax2 = ax1.twinx() # second axisObj that has x-axis in common
		ax2.set_ylabel('time [s]')
		ax2.set_ylim(0,20)

		p1=1 # skip zero tick of box plot
		tick_values=[]
		for df in dfList:
			ys = [df.loc[:,name].dropna().tolist() for name in column_names]

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

		x_upper_lim = 4*len(dfList)

		ax2.set_xticks(tick_values)
		# ax2.set_xticklabels(tick_labels,rotation=45)
		ax2.set_xlim(0,x_upper_lim)

		ax2.legend([bp["boxes"][0],bp["boxes"][1],bp["boxes"][2]],['shape [%]','orientation [%]','trial time [s]'],
						loc='lower left',fontsize='10')

		ax1.set_xticklabels(tick_labels,rotation=70)
		# ax1.set_xmargin()
		fig.tight_layout()
		return fig

	def BoxPlotGroup2axis(self,exp1,exp2,exp3,exp4):
		# TO DO
		# add title
		# get legend on nice position
		# add sample size on top?

		colors = ['red','blue','green']

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

		tick_values = [2, 6, 10, 14]
		tick_labels = ['1 DoF vertical', '2 DoF vertical', '1 DoF horizontal', '2 DoF horizontal']
		ax2.set_xticks(tick_values)
		ax2.set_xticklabels(tick_labels)
		ax2.set_xlim(0,16)

		ax2.legend([bp["boxes"][0],bp["boxes"][1],bp["boxes"][2]],['shape','orientation','trial time'],
						loc='lower left',fontsize='10')

		return fig

		