#!/usr/bin/env python
from SimpleExperimentTopic import ProcessSimpleExperiment
from PlotExperiment import PlotSimpleExperiment
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class AnalyseSimpleExperiment():
	def __init__(self):
		pass

	def generateRawDataPlots(self, all_dfs, base_path):
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
		for part_dfs in all_dfs:
			part_nr +=1
			for exp_nr,df in part_dfs.items():
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
		plt.close('all')


	def generateBoxExp(self, exp_dfs, parts ,base_path, real='real'):
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
		for part in parts: # over participant names 'part_1, part_2 etc'
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

		

	def generateBoxTypes(self, types_dfs, id_strings, parts, base_path):
		plot_exp = PlotSimpleExperiment()
		fields = ['field.shape_acc','field.orientation_acc','field.trial_time']

		get_fig_name = lambda ID: 'box_time_acc_type_' + str(ID) 
		get_out_dir = lambda base_path, ID, : base_path + str(ID)

		box_figs = []
		partsPlusAll = parts + ['All']
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






if __name__ == "__main__":
	AE = AnalyseSimpleExperiment()
	process = ProcessSimpleExperiment()
	process.main()

	base_path_raw = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_"
	AE.generateRawDataPlots(process.all_dfs,base_path_raw)

	base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	AE.generateBoxExp(process.real_exp_dfs, process.part_name_list, base_path)

	base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/"
	AE.generateBoxTypes(process.types_dfs, process.IDstrings, process.part_name_list, base_path)

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
	# AE.generateBoxMetrics(process.real_exp_dfs, fields, process.part_name_list, base_path)

	# df2 = pd.DataFrame(np.random.rand(10, 4), columns=['a', 'b', 'c', 'd'])
	# print(df2)