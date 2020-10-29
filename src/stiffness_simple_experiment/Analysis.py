#!/usr/bin/env python
from SimpleExperimentTopic import ProcessSimpleExperiment
from PlotExperiment import PlotSimpleExperiment
from scipy import stats
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy
import itertools
import sys

class AnalyseSimpleExperiment():
	def __init__(self):
		self.metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
		# yAxes = ['relative shape accuracy[%]','orientation accuracy [%]','time [s]','absolute shape error [-]','angle [deg]']

		shape_acc_fig = {
					'field': 'field.shape_acc',
					'name': 'shape_acc',
					'title': 'Relative Average Size Accuracy',
					'yLabel': 'accuracy [%]',
					'yLim': (None, None)
					}
		orientation_acc_fig = {
					'field': 'field.orientation_acc',
					'name': 'orientation_acc',
					'title': 'Orientation Accuracy',
					'yLabel': 'accuracy [%]',
					'yLim': (None, None)
					}
		time_fig = {
					'field': 'field.trial_time',
					'name': 'trial_time',
					'title': 'Trial Time',
					'yLabel': 'time [s]',
					'yLim': (0,None)
					}
		shape_fig = {
					'field': 'field.shape',
					'name': 'shape',
					'title': 'Absolute Average Size Error',
					'yLabel': 'error [-]',
					'yLim': (None,None)
					}
		angle_fig = {
					'field': 'field.absolute_angle',
					'name': 'angle',
					'title': 'Orientation Error',
					'yLabel': 'angle [deg]',
					'yLim': (-10,None)
					}

		size_shape_acc = {
					'title': 'Relative Average Size Accuracy',
					'field': 'field.shape_acc',
					'yLabel': 'accuracy [%]',
		}
		size_orientation_acc = {
					'title': 'Orientation Accuracy',
					'field': 'field.orientation_acc',
					'yLabel': 'accuracy [%]',
		}
		
		size_time = {
					'title': 'Trial Time',
					'field': 'field.trial_time',
					'yLabel': 'time [s]',
		}
		size_shape = {
					'title': 'Absolute Average Size Error',
					'field': 'field.shape',
					'yLabel': 'error [-]',
		}
		size_angle = {
					'title': 'Orientation Error',
					'field': 'field.absolute_angle',
					'yLabel': 'angle [deg]',
		}

		self.swarmDep = {'plot': True,
				 	'size': 6,
				 	'marker': 'D',
				 	'lw': 1,
				 	'ec': 'black'}
		self.swarmIndep = {'plot': True,
				 	'size': 2,
				 	'marker': 'x',
				 	'lw': 1,
				 	'ec': 'black'}

		self.swarmBig = {'plot': True,
					# 'tstats': depTStats,
				 	'size': 6,
				 	'marker': 'D',
				 	'lw': 1,
				 	'ec': 'black'}
		self.swarmSmall = {'plot': True,
					# 'tstats': inDepTStats,
				 	'size': 2,
				 	'marker': 'x',
				 	'lw': 1,
				 	'ec': 'black'}
		self.swarmDofPlanes = {'plot': True,
					'tstats': [],
				 	'size': 2,
				 	'marker': 'x',
				 	'lw': 1,
				 	'ec': 'black'}
	

		self.fig_info = [shape_acc_fig, orientation_acc_fig, time_fig, shape_fig, angle_fig]
		self.fig_info_size = [size_shape_acc, size_orientation_acc, size_time, size_shape, size_angle]

	def generateRawDataPlots(self, all_dfs, base_path, plot_part='all'):
		plot_exp = PlotSimpleExperiment()
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
						time1_info = dict(plot_exp.trial_time) 
						time1_info['legends'] = [exp_id[exp_nr]]
						# plot using custom legend
						fig_time,ax_time = plot_exp.singlePlotDf(df, time1_info)

						# add trial times plot from other experiment (1-2)
						# set new color for line
						# time2_info = dict(time1_info)
						# time2_info['colors'] = ['r']
						# time2_info['legends'] = [exp_id[exp_nr]]
						# # add line to figure using other color
						# ax_time = plot_exp.addLineFromDfToPlot(ax_time, df, time2_info)

						# (2) plot accuracy
						acc_info = dict(plot_exp.accuracy)
						acc_info['title'] = exp_id[exp_nr]
						fig_acc, ax_acc = plot_exp.singlePlotDf(df, acc_info)

						# (3) plot shape error (diameter)
						shape_error_info = dict(plot_exp.shape_error)
						shape_error_info['title'] = exp_id[exp_nr]
						fig_shape, ax_shape = plot_exp.singlePlotDf(df, shape_error_info)

						# (4) plot average shape error (diameter)
						avg_shape_error_info = dict(plot_exp.avg_shape_error)
						fig_avgShape, ax_avgShape = plot_exp.singlePlotDf(df, avg_shape_error_info)

						# (5) plot angle
						angle_info = dict(plot_exp.abs_angle)
						fig_angle, ax_angle = plot_exp.singlePlotDf(df, angle_info)

						# (6) plot user scales
						user_scales_info = dict(plot_exp.user_scales)
						fig_us, ax_us = plot_exp.singlePlotDf(df, user_scales_info)

						# (7) plot projected scales
						proj_scales_info = dict(plot_exp.projected_scales)
						fig_ps, ax_ps = plot_exp.singlePlotDf(df, proj_scales_info)
						
						# (8) plot experiment scales
						exp_scales_info = dict(plot_exp.exp_scales)
						fig_es, ax_es = plot_exp.singlePlotDf(df, exp_scales_info)


						# (9) plot originals experiment scales
						or_exp_scales_info = dict(plot_exp.or_user_scales)
						fig_oes, ax_oes = plot_exp.singlePlotDf(df, or_exp_scales_info)

						# (10) plot user quats
						user_quats_info = dict(plot_exp.user_quats)
						fig_uq, ax_uq = plot_exp.singlePlotDf(df, user_quats_info)

						# (11) plot experiment quats
						exp_quats_info = dict(plot_exp.exp_quats)
						fig_eq, ax_eq = plot_exp.singlePlotDf(df, exp_quats_info)

						# (12) plot originals experiment quats
						or_exp_quats_info = dict(plot_exp.or_user_quats)
						fig_oeq, ax_oeq = plot_exp.singlePlotDf(df, or_exp_quats_info)

						basic_figs = [fig_time,fig_acc,fig_shape,fig_avgShape,fig_angle,fig_us,fig_ps,fig_es,fig_oes,fig_uq,fig_eq,fig_oeq]
						basic_names = ["1_trial_time","2_accuracy","3_scale_error","4_average_scales","5_angle","6_user_scales",
						"7_proj_scales","8_exp_scales","9_exp_scales_original","10_user_quats","11_exp_quats","12_user_quats_original"]
						
						out_dir = base_path + str(part_nr)+"/"+exp_id[exp_nr]

						plot_exp.saveFigs(basic_figs,basic_names,out_dir,'.png')
						plt.close('all')
						print('RUSTAAAGH!!!, plotjes worden gemaakt')
				else:
					continue


		plt.close('all')


	def generateBoxExp(self, exp_dfs, part_list ,base_path, real='real'):
		plot_exp = PlotSimpleExperiment()

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
			fig_x_time_acc = plot_exp.BoxPlotGroup2axis(fig_x_data[0], fig_x_data[1], fig_x_data[2], fig_x_data[3])
			box_figs.append(fig_x_time_acc)
		# plt.show()

		plot_exp.saveFigs2(box_figs,box_names,out_dirs,'.png')
		plt.close('all')

		

	def generateBoxTypes(self, types_dfs, id_strings, part_list, base_path):
		plot_exp = PlotSimpleExperiment()
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
				figures_part_x_cond_i = plot_exp.BoxPlotGroup2axisTypes(
								types_dfs[lower_i:upper_i],fields,id_strings[lower_i:upper_i],part)

				if ID == 'all':
					figures_part_x_cond_i.set_size_inches(20,8, forward=True)

				box_figs_part_x.append(figures_part_x_cond_i)
				name = get_fig_name(ID)
				out_dir = get_out_dir(base_path, part)
			
				box_figs.append(box_figs_part_x)
				box_names.append(name)
				out_dirs.append(out_dir)

			plot_exp.saveFigs2(box_figs_part_x,box_names,out_dirs,'.png')

		plt.close('all')


	def generateBoxMetrics(self,dfs, fields, parts, base_path):
		
		plot_exp = PlotSimpleExperiment()

	def getAverageOfMetricPerConditionDf(self,dfs,exclude_index, metric):
		filtered = []
		for df in dfs:
			x = df.loc[df.index != exclude_index, metric]
			filtered.append(x)
		df = pd.DataFrame(index = filtered[0].index)
		df['1_dof_v'] = filtered[0]
		df['2_dof_v'] = filtered[1]
		df['1_dof_h'] = filtered[2]
		df['2_dof_h'] = filtered[3]

		return df
		
	def stackTwoConditions(self,df,pair1,pair2):
		s11 = df[pair1[0]]
		s12 = df[pair1[1]]
		s21 = df[pair2[0]]
		s22 = df[pair2[1]]

		p1 = pd.concat([s11,s12])
		p2 = pd.concat([s21,s22])

		df = pd.DataFrame(index = p1.index)
		df[pair1[2]] = p1
		df[pair2[2]] = p2

		return df

	def getAverageDfs(self,dfs,metric):
		fourConditions = self.getAverageOfMetricPerConditionDf(dfs, 'all', metric)

		dof1ID = ['1_dof_v','1_dof_h','1_dof']
		dof2ID = ['2_dof_v','2_dof_h','2_dof']
		verticalID = ['1_dof_v','2_dof_v','vertical']
		horizontalID = ['1_dof_h','2_dof_h','horizontal']

		dofConditions = self.stackTwoConditions(fourConditions,dof1ID,dof2ID)
		planesConditions = self.stackTwoConditions(fourConditions,verticalID,horizontalID)

		return fourConditions, dofConditions, planesConditions

	def depTTestOnaverages(self,dof_and_planes, dof, planes, metric):

		textString = lambda t,p: "Dependent T: {}, PValue: {}".format(t,p)


		print('\naverages:')
		print(dof_and_planes)

		print("\n1v with 2v:")
		t1v2v, p1v2v = stats.ttest_rel(dof_and_planes['1_dof_v'],dof_and_planes['2_dof_v'], nan_policy='omit')
		print(textString(t1v2v, p1v2v))
		# print(stats.ttest_rel(dof_and_planes['1_dof_v'],dof_and_planes['2_dof_v']))

		print("\n1v with 1h:")
		t1v1h, p1v1h = stats.ttest_rel(dof_and_planes['1_dof_v'],dof_and_planes['1_dof_h'], nan_policy='omit')
		print(textString(t1v1h, p1v1h))
		
		print("\n1h with 2h:")
		t1h2h, p1h2h = stats.ttest_rel(dof_and_planes['1_dof_h'],dof_and_planes['2_dof_h'], nan_policy='omit')
		print(textString(t1h2h, p1h2h))

		print("\n2v with 2h:")
		t2v2h, p2v2h = stats.ttest_rel(dof_and_planes['2_dof_v'],dof_and_planes['2_dof_h'], nan_policy='omit')
		print(textString(t2v2h, p2v2h))

		print("\n1dof with 2dof:")
		t12, p12 = stats.ttest_rel(dof['1_dof'],dof['2_dof'], nan_policy='omit')
		print(textString(t12, p12))

		print("\nvertical with horizontal:")
		tvh, pvh = stats.ttest_rel(planes['vertical'],planes['horizontal'], nan_policy='omit')
		print(textString(tvh, pvh))
		# print(stats.ttest_rel(planes['vertical'],planes['horizontal']))


		columns = ['p12','t12','pvh','tvh','p1v2v','t1v2v','p1v1h','t1v1h','p1h2h','t1h2h','p2v2h','t2v2h']
		data = [p12,t12,pvh,tvh,p1v2v,t1v2v,p1v1h,t1v1h,p1h2h,t1h2h,p2v2h,t2v2h]
		indeptstats = pd.DataFrame(index=[metric], columns=columns)
		indeptstats.loc[metric,:] = data

		return indeptstats

	def depTTestOnAverageSize(self, df):
		textString = lambda t,p: "Dependent T: {}, PValue: {}".format(t,p)

		df = df.set_index('size')
		dfSmall = df.loc['small',:].sort_values(by=['participant','DoF','plane'])
		dfLarge = df.loc['large',:].sort_values(by=['participant','DoF','plane'])

		print('\naverages:')
		print(dfSmall)
		print(dfLarge)

		field_names = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
		tsl_list = []
		psl_list = []
		columns = ['tsl','psl',]
		indeptstats = pd.DataFrame(index=field_names, columns=columns)
		for name in field_names:
			tsl, psl = stats.ttest_rel(dfSmall[name], dfLarge[name], nan_policy='omit')
			indeptstats.loc[name,columns] = [tsl, psl]
		# tsl, psl = stats.ttest_rel(dfSmall[name], dfSmall[name], nan_policy='omit')
		print("\nsmall vs large:")
		print(indeptstats)

		return indeptstats
	
	def getSamplesOfMetrics(self, dfs, metric):
		serie_list = []
		# create df 1Dof vs 2 df
		print()
		for df in dfs:
			x = df.loc[:,metric]
			serie_list.append(x)

		dof_1_v = serie_list[0]
		dof_2_v = serie_list[1]
		dof_1_h = serie_list[2]
		dof_2_h = serie_list[3]

		p1 = pd.concat([dof_1_v,dof_1_v])
		p2 = pd.concat([dof_2_v,dof_2_h])

		pv = pd.concat([dof_1_v,dof_2_v])
		ph = pd.concat([dof_1_h,dof_2_h])

		return serie_list, [p1,p2], [pv,ph]

	def indepTTestOntrials(self, dof_and_planes, dof, plane, metric):

		print('\ndependent TTest:')
		print(metric)
		for ID, x in zip(['1_dof_v','2_dof_v','1_dof_h','2_dof_h'],dof_and_planes):
			print('\nCondition: {}'.format(ID))
			print(len(x.values))
			print(x.describe())

		textString = lambda t,p: "Independent T: {}, PValue: {}".format(t,p)
		# column_name = lambda base,ID: str(base) +'_'+ str(ID+1) 

		print("\n1v with 2v:")
		t1v2v, p1v2v = stats.ttest_ind(dof_and_planes[0],dof_and_planes[1], equal_var='False', nan_policy='omit')
		print(textString(t1v2v, p1v2v))

		print("\n1v with 1h:")
		t1v1h, p1v1h = stats.ttest_ind(dof_and_planes[0],dof_and_planes[2], equal_var='False', nan_policy='omit')
		print(textString(t1v1h, p1v1h))

		print("\n1h with 2h:")
		t1h2h, p1h2h = stats.ttest_ind(dof_and_planes[2],dof_and_planes[3], equal_var='False', nan_policy='omit')
		print(textString(t1h2h, p1h2h))

		print("\n2v with 2h:")
		t2v2h, p2v2h = stats.ttest_ind(dof_and_planes[1],dof_and_planes[3], equal_var='False', nan_policy='omit')
		print(textString(t2v2h, p2v2h))

		print("\n1dof with 2dof:")
		t12, p12 = stats.ttest_ind(dof[0],dof[1], equal_var='False', nan_policy='omit')
		print(textString(t12, p12))

		print("\nvertical with horizontal:")
		tvh, pvh = stats.ttest_ind(plane[0],plane[1], equal_var='False', nan_policy='omit')
		print(textString(tvh, pvh))

		columns = ['p12','t12','pvh','tvh','p1v2v','t1v2v','p1v1h','t1v1h','p1h2h','t1h2h','p2v2h','t2v2h']
		data = [p12,t12,pvh,tvh,p1v2v,t1v2v,p1v1h,t1v1h,p1h2h,t1h2h,p2v2h,t2v2h]
		indeptstats = pd.DataFrame(index=[metric], columns=columns)
		indeptstats.loc[metric,:] = data

		return indeptstats


	def getFigures(self, test, fig_info, swarmSpecs):
		figList = []
		PSE = PlotSimpleExperiment()
		if not isinstance(fig_info, list):
			fig_info = [fig_info]
		# 	yAxesnames = [yAxesnames]

		for fig_x in fig_info:
			print(fig_x['field'])
			print(fig_x['title'])
			print(fig_x['yLabel'])
			print(fig_x['yLim'])

			if test == 'dep':
				dof_and_planes, dof, plane = AE.getAverageDfs(process.means_real_exp, fig_x['field'])
				dof_and_planes_list = [data for name, data in dof_and_planes.iteritems()]
			elif test == 'indep':
				dof_and_planes_list, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,fig_x['field'])
			else:
				print("First input argument 'test' = {} is invalid".format(test))

			# print(dof_and_planes_list)
			fig = PSE.singleMetric(dof_and_planes_list,fig_x,swarmSpecs)
			figList.append(fig)
		return figList

	def getSizeFigures(self, real_exp_dfs, statsDf):
		# get means of all metrics in 2 df where mean is calculated wrt(dof, plane)
		mean_size_dof, mean_size_plane, mean_size_plane_dof = self.getAverageOfMetricsPerSizeDfs(real_exp_dfs)
		allFigs = []
		dofFigs = []
		planeFigs = []
		sizeFigs = []
		for info in self.fig_info_size:
			# figDof = PSE.singleMetricSize(mean_size_dof,'DoF', info, AE.swarmBig)
			# figPlane = PSE.singleMetricSize(mean_size_plane,'plane', info, AE.swarmBig)	
			figSize = PSE.signelMetricSize2(mean_size_plane_dof, statsDf, info,  AE.swarmBig)
			# dofFigs.append(figDof)
			# planeFigs.append(figPlane)
			sizeFigs.append(figSize)

		# allFigs.extend(dofFigs)
		# allFigs.extend(planeFigs)
		allFigs.extend(sizeFigs)
		return allFigs
	
	def showDfInfo(self, df):
		print("\nEVERYTHING ABOUT INDEX:")
		print(df.index)
		print(df.index.values)
		print(len(df.index.values))

		print("\nEVERYTHING ABOUT COLUMNS:")
		print(df.columns)
		print(df.columns.values)
		print(len(df.columns.values))
		
		print("\nGENERAL INFO")
		df.info()


	def keepColumns(self, real_exp_dfs, keep_list=False):
		# concat in one df
		df = pd.concat(real_exp_dfs)
		# only use columns we want to keep
		if not keep_list: # if no input, set standard list
			keep_list = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle','normalizedErr',
				'DoF','plane','size','type','rotation_axis','rotation']
		df = df.loc[:,keep_list]		
		df = df.reset_index()
		return df

	def removeRowsBasedOnColumnValue(self, df, column, value):
		df = df[df[column] != value]
		return df 

	def splitDfOnColumnValues(self, df, column, values=False):
		unique_values = df[column].unique()
		unique_values = unique_values[ unique_values!=np.array(None) ]
		# print(unique_values)
		dfList = []
		if not values:
			# print('if')
			for value in unique_values:
				df_x = df[ df[column]==value ]
				dfList.append(df_x)
		elif isinstance(values,list):
			# print('elif')
			for values in values:
				df_x = df[ df[column]==value ]
				dfList.append(df_x)
		else:
			# print('else')
			dfList = df[df[column]==values]

		return dfList, unique_values

	def getSortedDfs(self, dfs, order):
		dfList = []
		if not isinstance(dfs, list):
			dfs = [dfs]

		for df in dfs:
			df_x = df.sort_values(by=order, na_position='first')
			dfList.append(df_x)

		return dfList


	def getAverageOfMetricsPerSizeDfs(self, real_exp_dfs):
		# concat in one df
		df = pd.concat(real_exp_dfs)
		# only use columns we want to keep
		keep = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle','normalizedErr',
				'DoF','plane','size','type','rotation_axis','rotation']
		df = df.loc[:,keep]
		df = df.reset_index()

		mean_size_dof = df.groupby(['participant','DoF','size'])[keep[0:6]].mean().reset_index()
		mean_size_plane = df.groupby(['participant','plane','size'])[keep[0:6]].mean().reset_index()
		mean_size_dof_plane = df.groupby(['participant','DoF','plane','size'])[keep[0:6]].mean().reset_index()

		return mean_size_dof, mean_size_plane, mean_size_dof_plane

	def depTest(self, df, condition, test='wilcoxon', show=False):
		# textString = lambda t,p: "Dependent T: {}, PValue: {}".format(t,p)

		# split df in 2 conditions
		[cond_1, cond_2], [cond_1_ID, cond_2_ID] = self.splitDfOnColumnValues(df, condition)

		# print(cond_1)
		# print(type(cond_1))
		# print(len(cond_1))

		# get descriptive statistic means and std in convienient data structure
		descStats = pd.DataFrame(columns = ['score','metric','type_stat','condition'])
		# calc means/stds
		means_1 = cond_1.mean().reset_index().rename(columns={'index':'metric',0:'score'})
		means_1['type_stat'], means_1['condition'] = 'mean', cond_1_ID
		std_1 = cond_1.std().reset_index().rename(columns={'index':'metric',0:'score'})
		std_1['type_stat'], std_1['condition'] = 'std', cond_1_ID
		first_quantile_1 = cond_1.quantile(0.25).reset_index().rename(columns={'index':'metric',0.25:'score'})
		first_quantile_1['type_stat'], first_quantile_1['condition'] = 'first quantile', cond_1_ID
		median_1 = cond_1.median().reset_index().rename(columns={'index':'metric',0:'score'})
		median_1['type_stat'], median_1['condition'] = 'median', cond_1_ID
		third_quantile_1 = cond_1.quantile(0.75).reset_index().rename(columns={'index':'metric',0.75:'score'})
		third_quantile_1['type_stat'], third_quantile_1['condition'] = 'third quantile', cond_1_ID

		# print(median_1)
		# print(first_quantile_1)
		# print(third_quantile_1)

		means_2 = cond_2.mean().reset_index().rename(columns={'index':'metric',0:'score'})
		means_2['type_stat'], means_2['condition'] = 'mean', cond_2_ID
		std_2 = cond_2.std().reset_index().rename(columns={'index':'metric',0:'score'})
		std_2['type_stat'], std_2['condition'] = 'std', cond_2_ID
		first_quantile_2 = cond_2.quantile(0.25).reset_index().rename(columns={'index':'metric',0.25:'score'})
		first_quantile_2['type_stat'], first_quantile_2['condition'] = 'first quantile', cond_2_ID
		median_2 = cond_2.median().reset_index().rename(columns={'index':'metric',0:'score'})
		median_2['type_stat'], median_2['condition'] = 'median', cond_2_ID
		third_quantile_2 = cond_2.quantile(0.75).reset_index().rename(columns={'index':'metric',0.75:'score'})
		third_quantile_2['type_stat'], third_quantile_2['condition'] = 'third quantile', cond_2_ID


		# fix data frame
		descStats = pd.concat([means_1,std_1,first_quantile_1,median_1,third_quantile_1, means_2,std_2,first_quantile_2,median_2,third_quantile_2]).sort_values(by=['metric','type_stat']).set_index('metric')
		descStats = descStats[descStats.index != 'row_id']


		# get t statistics
		field_names = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
		columns_stats = ["t_stat",'p_value','df','med1-med2']
		depTstats = pd.DataFrame(index=field_names, columns=columns_stats)
		for name in field_names:
			# remove nans from data since the ommit nanpolicy did not work
			c1,c2 = np.array(cond_1[name].tolist()), np.array(cond_2[name].tolist())
			i1,i2 = np.where(np.isnan(c1)), np.where(np.isnan(c2)) 
			i = np.append(i1,i2)
			c1,c2 = np.delete(c1,i), np.delete(c2,i)
			# print(len(c1))
			assert len(c1) == len(c2)
			df = len(c1)
			# print(len(c1))
			# ttest
			t,p = [],[]
			if test == 'wilcoxon':
				(t,p) = stats.wilcoxon(c1, c2, zero_method='wilcox', correction=False) 
			elif test == 'depTtest':
				(t,p) = stats.ttest_rel(c1, c2, nan_policy='raise') #, nan_policy='omit'


			# calculate difference of medians for every metric
			m1 = descStats.loc[(descStats['type_stat']=='median') & (descStats['condition']==cond_1_ID)].loc[name,'score']
			m2 = descStats.loc[(descStats['type_stat']=='median') & (descStats['condition']==cond_2_ID)].loc[name,'score']
			med_dif = m1-m2


			depTstats.loc[name,columns_stats] = [t, p, df, med_dif]
		depTstats['condition'] = condition

		if show:
			print(100 * '--')
			print("\n{} vs {}:".format(cond_1_ID, cond_2_ID))

			print(depTstats)
			print(descStats)

		# sys.exit()

		return depTstats, descStats

	def getDepTTestFiguresAndStatsOnTrials(self,real_exp_dfs,test='wilcoxon'):
		# only keep metrics
		df = self.keepColumns(real_exp_dfs)
		# sort
		sort = ['participant','DoF','plane','size','rotation']
		[df] = self.getSortedDfs(df, sort)
		#test  averages
		t_size, mean_size = self.depTest(df,'size',test,False)
		t_plane, mean_plane = self.depTest(df,'plane',test,False)
		# flter de circels eruit voor fair statistical comparison of 1 vs 2 dof
		df_noCircles = self.removeRowsBasedOnColumnValue(df, 'type', 'circle')
		t_dof, mean_dof = self.depTest(df_noCircles,'DoF',test,False)

		# get all the stats
		tstats = pd.concat([t_dof,t_plane,t_size])
		means_stds = pd.concat([mean_dof,mean_plane,mean_size])

		# plot all the figures
		figs = []
		figNames = []
		makeName = lambda metric, x_ax, box, test: str(metric) +'_'+ str(x_ax) +'_'+ str(box)+ '_'+ str(test)
		for info in self.fig_info:
			p_dof = tstats[ tstats['condition']=='DoF'].loc[info['field'],'p_value']
			p_plane = tstats[ tstats['condition']=='plane'].loc[info['field'],'p_value']
			p_size = tstats[ tstats['condition']=='size'].loc[info['field'],'p_value']
			figs4boxes = PSE.singleMetricDofPlanes4box(df, p_dof, p_plane, info, self.swarmSmall)
			# plt.show()
			# sys.exit()

			name = makeName(info['name'], 'dof', '4box', test)

			figs.append(figs4boxes)
			figNames.append(name)
			for dfx, x, p_x in zip([df_noCircles, df, df],['DoF','plane','size'] ,[p_dof, p_plane, p_size]):
				figs2boxes = PSE.singleMetricDofPlanesSizes2box(dfx, x, p_x, info, self.swarmSmall)
				name = makeName(info['name'], x, '2box', test)
				figs.append(figs2boxes)
				figNames.append(name)
		
		# plt.show()

		return figs, figNames, tstats, means_stds


		# plt.exit()

	def testOnNormalityAndOnWeibullDistribution(self, real_exp_dfs):
		# test for each metric(=5)_ for each hypothesis(=3) both conditions (=2) e.g.:(1Dof,2Dof) 
		metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
		hypotheses = ['DoF','plane','size']
		conditions = [['1_DoF','2_DoF'],['vertical','horizontal'],['small','large']]

		# 15 tests
		# metrics = []

		# only keep metrics
		df = self.keepColumns(real_exp_dfs)
		# sort
		sort = ['participant','DoF','plane','size','rotation']
		[df] = self.getSortedDfs(df, sort)

		# flter de circels eruit voor fair statistical comparison of 1 vs 2 dof
		df_noCircles = self.removeRowsBasedOnColumnValue(df, 'type', 'circle')

		pd.DataFrame()
		i = 1
		for metric in metrics:
			for hypothesis, condition in zip(hypotheses, conditions):
				for condition_x in condition:
					data = []
					if hypothesis == 'DoF':
						data = df_noCircles[df_noCircles[hypothesis]==condition_x][metric].dropna().tolist()
					else:
						data = df[df[hypothesis]==condition_x][metric].dropna().tolist()
					# shapiro-wilkonson test on normality
					# print(np.array(data))
					s, p_s = stats.shapiro(data)

					distribution = 'logistic'
					a, crit_values, sign_level = stats.anderson(data, dist=distribution)
					# a,p_a = stats.anderson(data, dist='gumbel')

					non_normality_threshold = 0.05
					if p_s <= non_normality_threshold:
						print('metric: {}, hypothesis: {}, condition: {}'.format(metric,hypothesis,condition_x))
						print('p_value of {} <= than {} indicating non normality!!!\n'.format(p,non_normality_threshold))

					# gumbal_threshold = 0.05
					# if a > gumbal_threshold:
					# if a > crit_values[2]:
					# 	print(i)
					# 	print('metric: {}, hypothesis: {}, condition: {}'.format(metric,hypothesis,condition_x))
					# 	print('a_value of {} >= {} indicating NOT a {} distribution\n'.format(a,crit_values[2],distribution))
					i = i + 1



if __name__ == "__main__":
	AE = AnalyseSimpleExperiment()
	process = ProcessSimpleExperiment()
	PSE = PlotSimpleExperiment()
	process.main()

	################# shapiro wilkinson test on normality #################
	# AE.testOnNormalityAndOnWeibullDistribution(process.real_exp_dfs)

	################# dependent/paired test on ordered trials (specify wilcox(nonparametric) or depTtest(parametric)) #################
	test = 'wilcoxon'
	# test = 'depTtest'
	figs, figNames, tstats, means_stds = AE.getDepTTestFiguresAndStatsOnTrials(process.real_exp_dfs)
	out_dir_fig = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	out_dir_stats = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/data"
	PSE.saveFigs(figs,figNames,out_dir_fig)
	# statFileName = 'allTrials_' +test+'.csv'
	# tstats.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/' +statFileName)
	# means_stds.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/AllTrials_means_stds.csv')
	# plt.show()


	#################### PLOT THE DOF VS PLANES FOR ALL TRIALS AND AVERAGES  ########################
	# metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
	# yAxes = ['relative shape accuracy[%]','orientation accuracy [%]','time [s]','absolute shape error [-]','angle [deg]']
	# depFigs = AE.getFigures('dep', AE.fig_info, AE.swarmDep)
	# inDepFigs = AE.getFigures('indep', AE.fig_info, AE.swarmIndep)
	# out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	# PSE.saveFigs(depFigs,['dep_shape_acc','dep_orientation_acc','dep_trial_time','dep_shape','dep_angle'],out_dir)
	# PSE.saveFigs(inDepFigs,['indep_shape_acc','indep_orientation_acc','indep_trial_time','indep_shape','indep_angle'],out_dir)
	# plt.show()



	#################### DO STATISTICS FOR AVERAGE SIZES AND PLOT GENERATIONS ########################
	# # # get means of all metrics in 2 df where mean is calculated wrt(dof, plane)
	# mean_size_dof, mean_size_plane, mean_size_plane_dof = AE.getAverageOfMetricsPerSizeDfs(process.real_exp_dfs)
	# # #dep tt on averages
	# statsDf = AE.depTTestOnAverageSize(mean_size_plane_dof)
	# statsDf.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/depTStatsAverageSizes.csv')
	# # #plot
	# metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
	# sizeFigs = AE.getSizeFigures(process.real_exp_dfs, statsDf)
	# out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	# PSE.saveFigs(sizeFigs,['size_shape_acc','size_orientation_acc','size_trial_time','size_shape','size_angle'],out_dir)
	# plt.show()


	#################### DO STATISTICS FOR AVERAGE DOF VS PLANES WITH DEPENDENT TTEST  ########################
	#### Do statistics for the averages of 1vs2 dof en hor vs vertical with dependent ttest########
	# depStatsList = []
	# dof_and_planes=[]
	# metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
	# for metric in metrics:	
	# 	dof_and_planes, dof, plane = AE.getAverageDfs(process.means_real_exp,metric)
	# 	stats_metricx = AE.depTTestOnaverages(dof_and_planes, dof, plane, metric)
	# 	depStatsList.append(stats_metricx)
	# depTStats = pd.concat(depStatsList)
	# AE.swarmDep.update({'tstats': depTStats}) 
	# depTStats.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/depTStatsAverage.csv')
	#################### DO STATISTICS FOR ALL TRIALS DOF VS PLANES WITH INDEPENDENT TTEST  ########################
	###### Do statistics for all trials with independent ttest ########
	# metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
	# inDepStatsList = []
	# dof_and_planes=[]
	# for metric in metrics:
	# 	dof_and_planes, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,metric)
	# 	stats_metricx = AE.indepTTestOntrials(dof_and_planes, dof, plane, metric)
	# 	inDepStatsList.append(stats_metricx)
	# inDepTStats = pd.concat(inDepStatsList)
	# AE.swarmIndep.update({'tstats': inDepTStats}) 
	# inDepTStats.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/indepTStatsSamples.csv')
	#################### PLOT THE DOF VS PLANES FOR ALL TRIALS AND AVERAGES  ########################
	# metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','field.shape','field.absolute_angle']
	# yAxes = ['relative shape accuracy[%]','orientation accuracy [%]','time [s]','absolute shape error [-]','angle [deg]']
	# depFigs = AE.getFigures('dep', AE.fig_info, AE.swarmDep)
	# inDepFigs = AE.getFigures('indep', AE.fig_info, AE.swarmIndep)
	# out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	# PSE.saveFigs(depFigs,['dep_shape_acc','dep_orientation_acc','dep_trial_time','dep_shape','dep_angle'],out_dir)
	# PSE.saveFigs(inDepFigs,['indep_shape_acc','indep_orientation_acc','indep_trial_time','indep_shape','indep_angle'],out_dir)
	# plt.show()


	####### CREATE DF THAT HOLDS MEANS AND STD FOR EACH PARTICIPANT ##########
		# df = pd.DataFrame(index = process.means_real_exp[0].index)
		# column_name = lambda base,ID: str(base) +'_'+ str(ID+1) 
		# for i in [0,1,2,3]:
		# 	df[column_name('time_mean',i)] = process.means_real_exp[i]['field.trial_time']
		# 	df[column_name('time_std',i)] = process.stds_real_exp[i]['field.trial_time']
		# 	df[column_name('shape_mean',i)] = process.means_real_exp[i]['field.shape_acc']
		# 	df[column_name('shape_std',i)] = process.stds_real_exp[i]['field.shape_acc']
		# 	df[column_name('orientation_mean',i)] = process.means_real_exp[i]['field.orientation_acc']
		# 	df[column_name('orientation_std',i)] = process.stds_real_exp[i]['field.orientation_acc']

		# df.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/means_std_conditions_metrics.csv')



	####### OLD PLOTTING, MOVED TO PlotExperiment.py #######
	# base_path_raw = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_"
	# PSE.generateRawDataPlots(process.all_dfs,base_path_raw, [8]) # if want to add specific participant(s)
	# # PSE.generateRawDataPlots(process.all_dfs,base_path_raw) # plot for all participants
	# plt.close()

	# base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	# # PSE.generateBoxExp(process.real_exp_dfs, process.part_name_list, base_path) # plot for all participants
	# # print(process.part_name_list)
	# # print(process.real_exp_dfs[0].index.values)
	# PSE.generateBoxExp(process.real_exp_dfs, ['part_8'], base_path) # if want to add specific participant(s)
	# plt.close()

	# base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	# # PSE.generateBoxTypes(process.types_dfs, process.IDstrings, process.part_name_list, base_path) # plot for all participants
	# PSE.generateBoxTypes(process.types_dfs, process.IDstrings, ['part_8'], base_path) # if want to add specific participant(s)
	# plt.close()

	# base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	# print(process.real_exp_dfs[0].columns.values)
	# plt.figure();
	# bp = process.real_exp_dfs[0].loc[:,['field.error_sorted_principle_axes.x', 'field.error_sorted_principle_axes.y', 'field.error_sorted_principle_axes.z']].plot.box()

	# for i in [0,1,2,3]:
	# i = 0
	# bp = process.real_exp_dfs[i].loc['part_1',['field.error_sorted_principle_axes.x', 'field.error_sorted_principle_axes.y', 'field.error_sorted_principle_axes.z']].plot.box()
	# bp = process.real_exp_dfs[i].loc['part_1',['projected_error.x', 'projected_error.y', 'projected_error.z']].plot.box()
	# bp = process.real_exp_dfs[i].loc['part_1',['field.shape']].plot.box()
	# bp = process.real_exp_dfs[i].loc['part_1',['field.absolute_angle']].plot.box()
	# bp = process.real_exp_dfs[i].loc['part_1',['']].plot.box()
	# plt.show()


	# fields = []
	# PSE.generateBoxMetrics(process.real_exp_dfs, fields, process.part_name_list, base_path)

	# df2 = pd.DataFrame(np.random.rand(10, 4), columns=['a', 'b', 'c', 'd'])
	# print(df2)