#!/usr/bin/env python
from SimpleExperimentTopic import ProcessSimpleExperiment
from PlotExperiment import PlotSimpleExperiment
from scipy import stats
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from copy import deepcopy
import itertools

class AnalyseSimpleExperiment():
	def __init__(self):
		pass

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

	def doDepTTestOnaverages(self,dof_and_planes, dof, planes):
		print('\naverages:')
		print(dof_and_planes)
		print("\n1v with 2v:")
		print(stats.ttest_rel(dof_and_planes['1_dof_v'],dof_and_planes['2_dof_v']))
		print("\n1v with 1h:")
		print(stats.ttest_rel(dof_and_planes['1_dof_v'],dof_and_planes['1_dof_h']))
		print("\n1h with 2h:")
		print(stats.ttest_rel(dof_and_planes['1_dof_h'],dof_and_planes['2_dof_h']))
		print("\n2v with 2h:")
		print(stats.ttest_rel(dof_and_planes['2_dof_v'],dof_and_planes['2_dof_h']))
		print("\n1dof with 2dof:")
		print(stats.ttest_rel(dof['1_dof'],dof['2_dof']))
		print("\nvertical with horizontal:")
		print(stats.ttest_rel(planes['vertical'],planes['horizontal']))

	def getSamplesOfMetrics(self, dfs, metric):
		serie_list = []
		# create df 1Dof vs 2 df
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

	def doIndepTTestOntrials(self, dof_and_planes, dof, plane, metric):

		print('\ndependent TTest:')
		print(metric)
		for ID, x in zip(['1_dof_v','2_dof_v','1_dof_h','2_dof_h'],dof_and_planes):
			print('\nCondition: {}'.format(ID))
			print(len(x.values))
			print(x.describe())

		print('hoi')
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


	def dfToCsv(self):
		pass

if __name__ == "__main__":
	AE = AnalyseSimpleExperiment()
	process = ProcessSimpleExperiment()
	PSE = PlotSimpleExperiment()
	process.main()


# Inspect data 
# print('inspect')
# df = process.means_real_exp[0]
# print(inspect.head(5))
# print(df.index.values)
# print(df.columns.values)



######## plot all metrics nice with seaborn #######
dof_and_planes, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,'normalizedErr')
PSE.singleMetric(dof_and_planes,'shape [-]')
dof_and_planes, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,'field.absolute_angle')
PSE.singleMetric(dof_and_planes,'angle [deg]')
dof_and_planes, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,'field.trial_time')
PSE.singleMetric(dof_and_planes,'time [s]')



######## Do statistics for all trials with independent ttest ########
# 1 vs 2 dof
metrics = ['field.shape_acc','field.orientation_acc','field.trial_time','normalizedErr','field.absolute_angle']
depStatsList = []
dof_and_planes=[]
for metric in metrics:
	dof_and_planes, dof, plane = AE.getSamplesOfMetrics(process.real_exp_dfs,metric)
	stats_metricx = AE.doIndepTTestOntrials(dof_and_planes, dof, plane, metric)
	depStatsList.append(stats_metricx)
depTStats = pd.concat(depStatsList)
depTStats.to_csv('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/depTStats.csv')











######## Do statistics for the averages of 1vs2 dof en hor vs vertical with dependent ttest########
# field = 'field.shape_acc'
# metrics = ['field.shape_acc','field.absolute_angle','field.trial_time','field.absolute_angle','normalizedErr']
# metric = 'field.absolute_angle'
# dof_and_planes, dof, planes = AE.getAverageDfs(process.means_real_exp,metric)
# AE.doDepTTestOnaverages(dof_and_planes, dof, planes)



######## CREATE DF THAT HOLDS MEANS AND STD FOR EACH PARTICIPANT ##########
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