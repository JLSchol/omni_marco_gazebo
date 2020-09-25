#!/usr/bin/env python
from os import path
from numpy import where
import numpy as np
import sys
from copy import deepcopy
from stiffness_visualization.ellipsoid_message import EllipsoidMessage
from ManageDataDirectories import ManageDataDirectories
# from PlotExperiment import PlotSimpleExperiment
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
		# important! These scores represent the indices of the ticked boxes 1 till 5
		# The scores still have to be calclulated from these indices!!!
		self.vanderLaanTickedBoxes = [
							[1,2,4,2,2,2,1,2,3], 			# Participant 1
							[1,2,5,1,1,5,1,5,2],			# Participant 2
							[2,2,3,4,4,4,2,4,1], 			# Participant 3
							[2,3,4,3,2,3,2,4,1],			# Participant 4
							[2,4,3,5,3,2,3,3,1],			# Participant 5
							[1,3,5,2,2,4,1,4,1],			# Participant 6
							[2,3,4,2,2,4,2,4,2],			# Participant 7
							[2,3,5,3,2,3,2,4,3]				# Participant 8
							]
		self.usefull, self.satisfying = self.getVanderLaan(self.vanderLaanTickedBoxes)		
					
		# self.topic = 'simple_experiment'
		# self.exp_IDs = ['1L', '1R', '2L', '2R', '3L', '3R', '4L', '4R']
		self.exp_id = {
						1: '1_DoF_Front_Practice',
						2: '1_DoF_Front_Real',
						3: '2_DoF_Front_Practice',
						4: '2_DoF_Front_Real',
						5: '1_DoF_Horizon_Practice',
						6: '1_DoF_Horizon_Real',
						7: '2_DoF_Horizon_Practice',
						8: '2_DoF_Horizon_Real'
		}
		self.part_list = [1,2,3,4,5,6,7,8]
		self.uniqueEllipsoids = []
		self.IDstrings = []			# string containing the ticklabels e.g. [type_rotation_axis_size, cigar_45_y_large]
		self.all_data = [] 			#  
		self.all_dfs = [] 			# 2d list with containing df;s [participants, conditions(8)]
		self.all_real_exp_dfs = [] 	# 2d list with containing df;s [participants, real_conditions(4)]
		self.all_prac_exp_dfs = [] 	# 2d list with containing df;s [participants, prac_conditions(4)]
		self.all_types_dfs = [] 	# 2d list with containing df;s [participants, types(36)]
		self.part_name_list = [] 	# [part_1, part_2 etc] (folder names)
		self.real_exp_dfs = []		# 1D list [exp1_df, exp2_df, exp3_df, exp4_df] containing all the realexpdf of all participatns
		self.prac_exp_dfs = []		# 1D list [exp1_df, exp2_df, exp3_df, exp4_df] containing all the pracexpdf of all participatns
		self.types_dfs = []			# 1D list [type1_df, type1_df, type1_df, .....] containing all the types of all participatns
		self.means_real_exp = []	# [exp1_df, exp2_df, exp3_df, exp4_df] for df_index = [part_1, part_2 .... alll]
		self.stds_real_exp = [] 	# 
		self.means_stds_types = []
		# need variables that store part names, data, other stuff in list for easy acces all over the code
		

	def removeFirstTrials(self,dataDict,exp_IDs=['1L', '1R', '2L', '2R', '3L', '3R', '4L', '4R'], 
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


	def add1Or2Dof(self, df):
		# excluding fake trial
		# scales = df["field.experiment_scales.x","field.experiment_scales.y","field.experiment_scales.z"].iloc[0]
		scales = df.loc[1,["field.experiment_scales.x","field.experiment_scales.y","field.experiment_scales.z"]].tolist()
		label = 'DoF'
		if len(scales) == len(set(scales)):
			df[label] = '2_DoF'
		else:
			df[label] = '1_DoF'

	def addVerticalOrHowizontal(self, df):
		plane=[]

		if (df['field.experiment_orientation.x']==0.0).all():
			plane = 'vertical'
		elif (df['field.experiment_orientation.y']==0.0).all():
			plane = 'horizontal'
		else:
			print('not 0 for all orientation (x and y) for all trials ?!?!')

		df['plane'] = plane





		# length_df = len(df)

		# if length_df == trial_amount_1Dof:
		# 	# add column with 1_DoF
		# 	df[label] = '1_DoF'
		# elif length_df == trial_amount_2Dof:
		# 	df[label] = '2_DoF'
		# else:

			# print("amount of trials 1/2: {}/{}, Does not match df length: {}".format(trial_amount_1Dof,trial_amount_2Dof,length_df))



	def addSmallOrLarge(self, df):
		values = df["field.experiment_scales.z"].unique()
		values.sort() # ascending
		df['size'] = where(df["field.experiment_scales.z"]==values[0], 'small', 'large')


	def addType(self, df):
		allUnique = lambda x,y,z: False if ( len([x,y,z])>len(set([x,y,z])) ) else True
		if df['DoF'].all() == '1_DoF':
			df['type'] = 'cigar'
		elif df['DoF'].all() == '2_DoF':
			fn = lambda df: 'oval' if ( allUnique(df['field.experiment_scales.x'],df['field.experiment_scales.y'],df['field.experiment_scales.z'])) else 'circle'
			df['type'] = df.apply(fn, axis=1)
		else:
			print(' df["DoF"].all() == only 1_DoF or only 2_DoF')

	
	def addRotationAxis(self, df):
		rot_axis=[]
		if (df['field.experiment_orientation.x']==0.0).all():
			if (df['field.experiment_orientation.y']==0.0).all():
				rot_axis = 'z-axis'
			elif (df['field.experiment_orientation.z']==0.0).all():
				rot_axis = 'y-axis'
			else:
				print('no rot_axisation for all trials ?!?!')
		elif (df['field.experiment_orientation.y']==0.0).all():
			if (df['field.experiment_orientation.z']==0.0).all():
				rot_axis = 'x-axis'
			elif (df['field.experiment_orientation.x']==0.0).all():
				rot_axis = 'z-axis'
			else:
				print('no rot_axisation for all trials ?!?!')

		df['rotation_axis'] = rot_axis


	def addRotation(self, df):
		
		deg_vals = ['0'	,'30',	'36',	'45',	'60',	'72',	'90',	'108',	'120',	'135',	'144',	'150']
		qw_keys =[1,			0.9659258,	0.9510565,	0.9238795,	0.8660254,	0.809017,	0.7071068,	0.5877853,	0.5,		0.3826834,	0.309017,	0.258819]
		qw_keys = [round(qw_key,3) for qw_key in qw_keys]
		qw_2_deg = dict(zip(qw_keys,deg_vals))

		fn = lambda qwList, qw_2_deg_dict: [qw_2_deg_dict[round(qw,3)] for qw in qwList]

		df['rotation'] = fn(df['field.experiment_orientation.w'].tolist(), qw_2_deg)

		# df['addRotation'] = df.apply(fn(),axis=1)


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

	def removeFailed001Trials(self,df):
		# print(df.index)
		# print(df.loc[1,'field.trial_time'])
		# print(type(df.loc[1,'field.trial_time']))
		indices = df.index[df['field.trial_time'] <= 0.02].tolist()
		# print(indices)
		# print(df.index[df['field.trial_time'] <= 0.02])
		columns = df.columns.values
		to_delete = ['participant','DoF','plane','size','rotation_axis','rotation','type']
		none_columns = [value for value in columns if not value in to_delete]

		if not len(indices) == 0:
			self.removeIncorrectTrials(df, indices, none_columns)

	def removeIncorrectTrials(self, df, indices, none_columns):
		columns = df.index.values.tolist()
		for index in indices:
			df.loc[index, none_columns] = None
			

	def addColumns(self, df, column_labels, data_lists, insert_list):
		for label, data, index in zip(column_labels,data_lists,insert_list):
			df.insert(index, label, data, False)
			df[label] = data


	def getUniqueEllipsCombinations(self, dfs, column_names=['DoF','size','type','rotation_axis','rotation']):
		new_dfs = []
		key_names = [str(i+1) for i in range(len(dfs))]
		for df_x in dfs:
			df_x = df_x[column_names].drop_duplicates().dropna()
			df_x['rotation'] = df_x['rotation'].astype(int)
			df_x.sort_values(by=['type','rotation','rotation_axis','size'],axis=0,ascending=True, inplace=True)
			df_x = df_x.reset_index()
			# print(type(df_x.loc[0,'rotation']))
			new_dfs.append(df_x)
		# add participants to exp x
		df_out = pd.concat(new_dfs,axis=0,keys=key_names).reset_index()
		del df_out['level_0'] # not sure if needed
		del df_out['level_1']
		del df_out['index']
		
		addUnd = lambda x,y: str(x)+'_'+str(y)
		# makeString = lambda x1,x2,x3,x4,x5: addUnd(addUnd(addUnd(addUnd(x1, x2),x3),x4),x5)
		makeString = lambda x1,x2,x3,x4: addUnd(addUnd(addUnd(x1, x2),x3),x4)
		stringIDList = []
		# swap index
		# print(df_out)
		for i in range(len(df_out.index)):
			# print(df_out.loc[i,:].tolist())
			x1,x2,x3,x4,x5 = df_out.loc[i,:].tolist()
			# stringID = makeString(x1, x3, x5, x4, x2)
			stringID = makeString(x3, x5, x4, x2)
			stringIDList.append(stringID)
		# stringIDList = array(stringIDList).sort().tolist()
		# print(df_out)
		# print(stringIDList)
		return df_out, stringIDList

	def _select_rows(self,df,search_strings):
	    unq,IDs = np.unique(df,return_inverse=True)
	    unqIDs = np.searchsorted(unq,search_strings)
	    return df[((IDs.reshape(df.shape) == unqIDs[:,None,None]).any(-1)).all(0)]

	def _select_rows_2(self,df,search_string,column_names):
		df2 = []
		search_string = [str(x) for x in search_string]
		for name,value in zip(column_names,search_string):
			df = df.loc[df[name]==value]
			df2 = df

		return df2

	def getExperimentDfsList(self, ):
		pass

	def getUniqueDfsList(self, df_uniques, df_exp_list):
		types2DList=[]
		column_names = df_uniques.columns.values
		i = 0
		for i_type, row in df_uniques.iterrows(): # loop over the unique types of ellipsoids
			search_string = row.tolist()

			type_x_expList = []
			for i_exp, exp_x in enumerate(df_exp_list): # loop over experiment 1, 2, 3, 4
				# df = self._select_rows(exp_x,search_string) # this function fails an only sorts numbers
				df = self._select_rows_2(exp_x,search_string,column_names) # this function fails an only sorts 
				type_x_expList.append(df)	# list containing 4 df of one type
			types2DList.append(type_x_expList) # list containing 20 types that contain 4 dfs per type

		types = []
		for type_xList in types2DList:
			type_x = pd.concat(type_xList) 
			types.append(type_x)

		return types

	def concatDfForMultiplePart(self, df2DList, type_index, part_names):
		# adds the dfs found in df2DList[part_x][type_index of types]
		# print('--')
		subList = []
		for dfPart_x in df2DList:
			df = dfPart_x[type_index] # ellips type i
			subList.append(df)
		# for i in range(len(df2DList)):
		# 	df = df2DList[i][type_index]
		# 	subList.append(df)
		newdf = pd.concat(subList, ignore_index=False, names=['participant','row_id'], keys=part_names)
		# print(newdf)
		return newdf

	def getRowList(self, df, column_names):
		return [row.tolist() for i,row in df[column_names].iterrows()]
		
	def addProjectedScales(self,df):
		quat_exp_fields = ['field.experiment_orientation.x','field.experiment_orientation.y','field.experiment_orientation.z','field.experiment_orientation.w']
		quat_user_fields = ['field.user_orientation.x','field.user_orientation.y','field.user_orientation.z','field.user_orientation.w']
		scale_exp_fields = ['field.user_scales.x','field.user_scales.y','field.user_scales.z']
		scale_user_fields = ['field.experiment_scales.x','field.experiment_scales.y','field.experiment_scales.z']

		quats_exp = self.getRowList(df, quat_exp_fields)
		scales_exp = self.getRowList(df, scale_exp_fields)
		quats_usr = self.getRowList(df, quat_user_fields)
		scales_usr = self.getRowList(df, scale_user_fields)
		EM = EllipsoidMessage()
		proj = [EM.projectScales(qe, se, qu, su)[0] for qe, se, qu, su in zip(quats_exp,scales_exp,quats_usr,scales_usr)]
		proj_error = [EM.projectScales(qe, se, qu, su)[1] for qe, se, qu, su in zip(quats_exp,scales_exp,quats_usr,scales_usr)]
		# proj = out[0]
		# proj_error = out[1]
		# proj_error = [EM.projectScales(qe, se, qu, su)[1] for qe, se, qu, su in zip(quats_exp,scales_exp,quats_usr,scales_usr)]
		# print(proj)
		self.addColumns(df, ['projected_scales.x','projected_scales.y','projected_scales.z'], np.array(proj).T.tolist(), [13,14,15])
		self.addColumns(df, ['projected_error.x','projected_error.y','projected_error.z'], np.array(proj_error).T.tolist(), [13,14,15])

		# print(df[['projected_scales.x','projected_scales.y','projected_scales.z']].values)

	def addNormalizedShapeError(self,df): # check the calculation of this function and compare with average error 'field.shape' !!!
		error_fields = ['field.error_sorted_principle_axes.x','field.error_sorted_principle_axes.y','field.error_sorted_principle_axes.z']
		ref_ellips_size_fields = ['field.experiment_scales.x','field.experiment_scales.y','field.experiment_scales.z']

		error = self.getRowList(df, error_fields)
		ref_ellips_size = self.getRowList(df, ref_ellips_size_fields)

		normalizeErr = lambda e, ref: np.array(e)/np.array(ref)

		normError = [np.mean(normalizeErr(e, ref)) for e, ref in zip(error, ref_ellips_size)]

		# # Test if normalizedErr found matches percetage
		# print('normError')
		# print(normError)
		# # check if corresponds with percentage
		# print('to_perc')
		# to_perc = lambda x,y: x/y*100
		# perc = [100 - np.mean(to_perc(e, ref)) for e, ref in zip(np.array(error), np.array(ref_ellips_size))]
		# print(perc)

		self.addColumns(df, ['normalizedErr'], [normError], [10])



	def checkPerc(self, normErrorValue, shapePerc):
		pass
		# convert normErrorValue to shape perc

		

	def convertVanderLaanScores(self, tick_box_list):
		usefulness_items = [1,3,5,7,9]
		satisfying_items = [2,4,6,8]

		scores_left_right = [2, 1, 0, -1, -2]
		scores_right_left = [-2, -1, 0, 1, 2]


		if len(tick_box_list)!= 9:
			print('tick_box_list {} is has not a length of 9 items'.format(tick_box_index))
			return

		converted_scores_list = []
		usefull_scores = []
		satisfying_scores = []
		for item,tick_box_index in enumerate(tick_box_list):
			item+=1
			if tick_box_index not in [1,2,3,4,5]:
				print('tick_box_index {} is not a int ranging in [1-5]'.format(tick_box_index))
				return

			if item in usefulness_items:
				if item in [1,5,7,9]:
					# print(scores_left_right[tick_box_index-1])
					usefull_scores.append(scores_left_right[tick_box_index-1])
					converted_scores_list.append(scores_left_right[tick_box_index-1])
				elif item in [3]:
					# print(scores_left_right[tick_box_index-1])
					usefull_scores.append(scores_right_left[tick_box_index-1])
					converted_scores_list.append(scores_right_left[tick_box_index-1])
			elif item in satisfying_items:
				if item in [2,4]:
					satisfying_scores.append(scores_left_right[tick_box_index-1])
					converted_scores_list.append(scores_left_right[tick_box_index-1])
				elif item in [6,8]:
					satisfying_scores.append(scores_right_left[tick_box_index-1])
					converted_scores_list.append(scores_right_left[tick_box_index-1])

			else:
				print('item {} is not supposed to be in tick_box_list'.format(item))


		usefull_score = sum(usefull_scores)/5.0
		satisfying_score = sum(satisfying_scores)/4.0

		return usefull_score, satisfying_score, converted_scores_list

	def getVanderLaan(self, scores):
		usefull_list = []
		satisfying_list = []
		for part_score in scores:
			usefull_score, satis_score, _ = self.convertVanderLaanScores(part_score)
			usefull_list.append(usefull_score)
			satisfying_list.append(satis_score)

		mean_usefull = np.mean(np.array(usefull_list))
		std_usefull = np.std(np.array(usefull_list))

		mean_satisfying = np.mean(np.array(satisfying_list))
		std_satisfying = np.std(np.array(satisfying_list))

		return (usefull_list, mean_usefull, std_usefull), (satisfying_list, mean_satisfying, std_satisfying)

	def getMeanStdsDfs(self, df):
		dfm = df.mean(level='participant')
		dfs = df.std(level='participant')
		# print(dfm)
		dfmm = dfm.mean()
		dfms = dfm.std()
		# dfm.loc['mean_of_means'] = dfmm
		# dfs.loc['std_of_means'] = dfms
		# print(dfms)


		dfm.loc['all',:] = df.mean()
		dfs.loc['all',:] = df.std()
		return (dfm, dfs)



	def main(self):
		########## THE GETTING DATA DICTIONAIR PART ##########
		# single participant
		# get relavant directories, experiment_name abreviations, to set the data in the ParticipantData class
		MD = ManageDataDirectories()

		self.all_data = []

		partName = lambda nr: 'part_' + str(nr)
		self.part_name_list = []
		for part_nr in self.part_list: # loop over participant folders
			part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_'+str(part_nr)

			exp_IDs = MD.experiment_IDs # ['1L','1R','2L','2R','3L','3R','4L','4R']
			topics = ["simple_experiment_data"]
			# get relavant directories containing the files
			partx_info_file_path, _, partx_csvdir_paths, partx_txt_paths, _ = MD.getAllPathsOfParticipant(part_dir, MD.csv_dir_list)
			csvfile_paths = MD.getFilesInDirList(partx_csvdir_paths,".csv")

			# set the files in one dictionari
			part_x = ParticipantData(exp_IDs, partx_info_file_path, partx_txt_paths, partx_csvdir_paths, csvfile_paths, topics)
			# add van der laan scores
			part_x.setVanDerLaan(self.vanderLaanTickedBoxes[part_nr-1])
			# data = part_x.data
			self.all_data.append(part_x)
			# self.all_data.append(deepcopy(part_x))
			self.part_name_list.append(partName(part_nr))
			# self.part_name_list.append(partName(2))

		# we now have a dictionair With all the participants information


		# GET LIST OF ALL THE DFS
		self.all_dfs = [] 
		for part_x in self.all_data: # loop over all participants containing the participant data class
			exp_data = {
							1: part_x.data['1L']['simple_experiment_data'],
							2: part_x.data['1R']['simple_experiment_data'],
							3: part_x.data['2L']['simple_experiment_data'],
							4: part_x.data['2R']['simple_experiment_data'],
							5: part_x.data['3L']['simple_experiment_data'],
							6: part_x.data['3R']['simple_experiment_data'],
							7: part_x.data['4L']['simple_experiment_data'],
							8: part_x.data['4R']['simple_experiment_data'],
						}
			self.all_dfs.append(exp_data)


		# REMOVE ALL THE FIRST TRIALS
		for part_x_data in self.all_data:
			self.removeFirstTrials(part_x_data.data)

		#  ADD COLUMNS AND REMOVE INCORRECT TRIALS THAT ARE SKIPPED
		for part_x_dfs_dict in self.all_dfs:

			for df,expnr in zip(part_x_dfs_dict.values(),part_x_dfs_dict.keys()):
				# print(expnr)
				self.add1Or2Dof(df)
				self.addVerticalOrHowizontal(df)
				self.addSmallOrLarge(df)
				self.addType(df)
				self.addRotationAxis(df)
				self.addRotation(df)
				self.addProjectedScales(df)
				self.addNormalizedShapeError(df) # check this function!!!
				self.removeFailed001Trials(df)
				# sys.exit()

		# IF NEED REMOVE TRIALS MANUALLY
		# self.removeIncorrectTrials(df, indices)

		
		# CREAT CONVINEANT LIST per experiment
		# all_real_exp_dfs[PARTICIPANT],[realExp] = df 		--> size([part_amount,4real])
		# all_real_exp_dfs[PARTICIPANT],[practiceExp] = df 	--> size([part_amount,4prac])
		for part_x_dfs_dict in self.all_dfs:
			real_dfs = [part_x_dfs_dict[2],part_x_dfs_dict[4],part_x_dfs_dict[6],part_x_dfs_dict[8]]
			prac_dfs = [part_x_dfs_dict[2],part_x_dfs_dict[4],part_x_dfs_dict[6],part_x_dfs_dict[8]]
			self.all_real_exp_dfs.append(real_dfs)
			self.all_prac_exp_dfs.append(prac_dfs)
		# print(self.all_real_exp_dfs)
		# print(len(self.all_real_exp_dfs)) # amount of participants
		# print(len(self.all_real_exp_dfs[0])) # amount of experiments

		# CREAT CONVINEANT LIST PER ELLIPSOID TYPE
		# sort types together and put in one big 
		self.uniqueEllipsoids, self.IDstrings = self.getUniqueEllipsCombinations(
						[self.all_dfs[0][2], self.all_dfs[0][4], self.all_dfs[0][6], self.all_dfs[0][8]] )
		# get all the types per
		self.all_types_dfs = []
		for part_x_dfs_dict in self.all_dfs:
			experiments = [ part_x_dfs_dict[2], part_x_dfs_dict[4], part_x_dfs_dict[6], part_x_dfs_dict[8] ]
			type_Dfs_part_x = self.getUniqueDfsList(self.uniqueEllipsoids, experiments)
			# print(len(typesDfs_part_x)) # length of nr of types with 4 dataframes
			self.all_types_dfs.append(type_Dfs_part_x) # length of nr participants
		# print(self.all_types_dfs)
		# print(len(self.all_types_dfs)) # amount of part
		# print(len(self.all_types_dfs[0])) # amount of types



		# CONCAT ALL THE PARTICIPANTS for the 4 experiments
		# self.all_real_exp_dfs = [self.all_real_exp_dfs,self.all_real_exp_dfs]
		for i in range(len(self.all_real_exp_dfs[0])):  # loop over index of the experiments conditions
			concat_real_i = self.concatDfForMultiplePart(self.all_real_exp_dfs, i, self.part_name_list) # concats ellipse type i=0,1,2,3 ... 19
			concat_prac_i = self.concatDfForMultiplePart(self.all_prac_exp_dfs, i, self.part_name_list) # concats ellipse type i=0,1,2,3 ... 19
			self.real_exp_dfs.append(concat_real_i)	
			self.prac_exp_dfs.append(concat_prac_i)	
		# print(len(self.real_exp_dfs)) # 4
		# print(len(self.real_exp_dfs[0])) # part*amount of trials of that condition
		# print(self.real_exp_dfs[0])
		# print(self.real_exp_dfs[0].index)
		# print(self.real_exp_dfs[0].loc[:,:])
		# print(self.real_exp_dfs[0].loc['part_1'])
		# print(self.real_exp_dfs[0].loc['part_2'])



		# CONCAT OVER ALL THE PARTICIPANTS for the types
		for i in range(len(self.all_types_dfs[0])):  # loop over index of the ellipsoid types
			concat_type_x = self.concatDfForMultiplePart(self.all_types_dfs, i, self.part_name_list) # concats ellipse type i=0,1,2,3 ... 19
			self.types_dfs.append(concat_type_x)	
		# print(self.types_dfs[0].index)
		# print(self.types_dfs[0].loc[:,:])
		# print(self.types_dfs[0].loc['part_1'])

		# concat all_exp, all_types over all participants
		# combined_type_x = self.concatDfForMultiplePart(typesDf, i, part_name_list)

		self.means_stds_real_exp = []
		for df in self.real_exp_dfs:
			means, stds = self.getMeanStdsDfs(df)
			self.means_real_exp.append(means)
			self.stds_real_exp.append(stds)
			# print(means)
			# print(self.means_real_exp)
			# means_of_means = df.mean(level='participant')
			# std_of_means = df.std(level='participant')
			




def combineData(dfList):
	pass



if __name__ == "__main__":
	# pass
	process = ProcessSimpleExperiment()
	process.main()

	# # usefull_list = []
	# # satisfying_list = []
	# # for part_vdl in process.vanderLaanTickedBoxes:
	# # 	useful, satis, converted = process.convertVanderLaanScores(part_vdl)
	# # 	print(converted)
	# # 	usefull_list.append(useful)
	# # 	satisfying_list.append(satis)

	# # meanuse = np.mean(np.array(usefull_list))
	# # stduse = np.std(np.array(usefull_list))

	# # meansat = np.mean(np.array(satisfying_list))
	# # stdsat = np.std(np.array(satisfying_list))
	# # print('satisfying_list')
	# # print(satisfying_list)
	# # print('usefull_list')
	# # print(usefull_list)

	# # print('use')
	# # print(meanuse)
	# # print(stduse)
	# # print('sat')
	# # print(meansat)
	# # print(stdsat)

	# sys.exit()
	# process.main()
	# ########## THE GETTING DATA DICTIONAIR PART ##########
	# # single participant
	# # get relavant directories, experiment_name abreviations, to set the data in the ParticipantData class
	# MD = ManageDataDirectories()
	# part_data_list = []
	# for part_nr in [1]:
	# 	part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_'+str(part_nr)

	# 	exp_IDs = MD.experiment_IDs # ['1L','1R','2L','2R','3L','3R','4L','4R']
	# 	topics = ["simple_experiment_data"]
	# 	# get relavant directories containing the files
	# 	partx_info_file_path, _, partx_csvdir_paths, partx_txt_paths, _ = MD.getAllPathsOfParticipant(part_dir, MD.csv_dir_list)
	# 	csvfile_paths = MD.getFilesInDirList(partx_csvdir_paths,".csv")

	# 	# set the files in one dictionari
	# 	part_x = ParticipantData(exp_IDs, partx_info_file_path, partx_txt_paths, partx_csvdir_paths, csvfile_paths, topics)
	# 	part_data_list.append(part_x)


	# ########## THE PROCESSING PART ##########
	# proces = ProcessSimpleExperiment()
	# # also initialize the plot class already
	# plot_exp = PlotSimpleExperiment()

	# # testData = part_data_list[0].data['2R']['simple_experiment_data']
	# # proces.addProjectedScales(testData)

	# # proces.add1Or2Dof(testData)
	# # proces.addSmallOrLarge(testData)
	# # proces.addType(testData)
	# # proces.addRotationAxis(testData)
	# # proces.addRotation(testData)
	# # proces.getUniqueEllipsCombinations([testData])
	# # print(testData.columns.values)
	# # print(testData.loc[:,['type','field.experiment_scales.x','field.experiment_scales.z']])
	# # print(testData.loc[:,['DoF','size','type','rotation_axis','rotation']])
	# # print(testData['field.experiment_scales.x'])

	# # sys.exit()
	
	# data = []
	# typesDf=[]
	# # print(part_data_list[2].data['1L']['simple_experiment_data'])
	# # loop over the participants
	# for part_nr,part_x in enumerate(part_data_list): 
	# 	part_nr += 1 # start at 1 ipv 0
	# 	# part_nr = 6 # start at 1 ipv 0
	# 	# part_nr = 3 # only for exploring part 3
	# 	data = part_x.data

	# 	exp_data = {
	# 					1: data['1L']['simple_experiment_data'],
	# 					2: data['1R']['simple_experiment_data'],
	# 					3: data['2L']['simple_experiment_data'],
	# 					4: data['2R']['simple_experiment_data'],
	# 					5: data['3L']['simple_experiment_data'],
	# 					6: data['3R']['simple_experiment_data'],
	# 					7: data['4L']['simple_experiment_data'],
	# 					8: data['4R']['simple_experiment_data'],
	# 	}
	# 	# REMOVE ALL FIRST TRIALS
	# 	proces.removeFirstTrials(part_x.data)

	# 	# loop over the experiments
	# 	for exp_nr in proces.exp_id.keys(): 
	# 		# this is a bit outdated process?
	# 		if exp_nr == 1 or exp_nr == 2 or exp_nr == 5 or exp_nr == 6: # 1 DOF
	# 			# ADD COLLOMNS THAT IDENTIFY THE TYPE OF THE ELLIPSOID 1DOF
	# 			size_id_list = proces.getSizeID1DoF(exp_data[exp_nr]) #returns dataframe
	# 			rot_id_list = proces.getRotIDText(exp_data[exp_nr])
	# 			proces.addColumns(exp_data[exp_nr], ['size_type','rot_type'], [size_id_list,rot_id_list], [5,6])
	# 		elif exp_nr == 3 or exp_nr == 4 or exp_nr == 7 or exp_nr == 8: # 2 DOF
	# 			# ADD COLLOMNS THAT IDENTIFY THE TYPE OF THE ELLIPSOID 2DOF
	# 			size_id_list2 = proces.getSizeID2DoF(exp_data[exp_nr]) 
	# 			rot_id_list2 = proces.getRotIDText(exp_data[exp_nr])
	# 			proces.addColumns(exp_data[exp_nr], ['size_type','rot_type'], [size_id_list2,rot_id_list2], [5,6])



	# 		# FIX NEW DF SUCH THAT IT CAN BE PLOTTED (CAREFULL WEIRD PROCESS BELOW)
	# 		size_dfs, uniques_sizes = proces.filterOnUniqueColumnValues(exp_data[exp_nr],'size_type')
	# 		rot_dfs, uniques_rot = proces.filterOnUniqueColumnValues(exp_data[exp_nr],'rot_type')
	# 		# creates dfs for every field in fieldList with index being the uniques and columns being the index of the 
	# 		# createNewDFs(newColumns,dfs,dfs_column_ids):

	# 		[size_time_df,size_accShape_df,size_accRot_df] = proces.createNewDFs(uniques_sizes,size_dfs,[
	# 											'field.trial_time',"field.shape_acc","field.orientation_acc"]) #returns a list
	# 		[rot_time_df,rot_accShape_df,rot_accRot_df] = proces.createNewDFs(uniques_rot,rot_dfs,[
	# 											'field.trial_time',"field.shape_acc","field.orientation_acc"]) #returns a list


	# 		print(exp_nr)
	# 		# if exp_nr%2==0: # only real experiments
	# 		# 	t1,t2 = 20,30
	# 		# 	if part_nr ==3 or part_nr ==4:
	# 		# 		t1,t2 = 32,40
	# 		proces.add1Or2Dof(exp_data[exp_nr])
	# 		proces.addSmallOrLarge(exp_data[exp_nr])
	# 		proces.addType(exp_data[exp_nr])
	# 		proces.addRotationAxis(exp_data[exp_nr])
	# 		proces.addRotation(exp_data[exp_nr])
	# 		proces.addProjectedScales(exp_data[exp_nr])
	# 			# print(exp_data[exp_nr].columns.values)
			

	# 		# REMOVE INCORRECT TRIALS
	# 		# Set trials that are failed to None; specify manually the trial number e.g. 2 and 17
	# 		# if exp_nr == 7 and part_nr == 2:
	# 		# 	proces.removeIncorrectTrials(exp_data[exp_nr], [1,9])
	# 		# if exp_nr == 2 and part_nr == 1:
	# 		# 	proces.removeIncorrectTrials(exp_data[exp_nr], [1,4])
	# 		# if 
	# 		# print(exp_data[nr].loc[3,:])


		


	# 		########## THE PLOTTING PART ##########
	# 		# set output directory for figures
	# 		base_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_"
	# 		out_dir = base_path + str(part_nr)+"/"+proces.exp_id[exp_nr]
	# 		# initialize plot class
			

	# 		# plot types (1-6) 
	# 		# shapes
	# 		fig_s_t,ax_s_t = plot_exp.plotTypes(size_time_df,'Trial time vs Shape',16,'Size Type [-]','Time [s]',14,[proces.exp_id[exp_nr]],12)
	# 		fig_s_as,ax_s_ac = plot_exp.plotTypes(size_accShape_df,'Shape Accuracy vs Shape',16,'Size Type [-]','Shape Accuracy [%]',14,[proces.exp_id[exp_nr]],12)
	# 		fig_s_ao,ax_s_ao = plot_exp.plotTypes(size_accRot_df,'Orientation Accuracy vs Shape',16,'Size Type [-]','Orientation Accuracy [%]',14,[proces.exp_id[exp_nr]],12)
	# 		# rotations
	# 		fig_r_t,ax_st = plot_exp.plotTypes(rot_time_df,'Trial time vs Orientation',16,'Orientation Type [deg]','Time [s]',14,[proces.exp_id[exp_nr]],12)
	# 		fig_r_as,ax_st = plot_exp.plotTypes(rot_accShape_df,'Shape Accuracy vs Orientation',16,'Orientation Type [deg]','Shape Accuracy [%]',14,[proces.exp_id[exp_nr]],12)
	# 		fig_r_ao,ax_st = plot_exp.plotTypes(rot_accRot_df,'Orientation Accuracy vs Orientation',16,'Orientation Type [deg]','Orientation Accuracy [%]',14,[proces.exp_id[exp_nr]],12)

	# 		type_figs = [fig_s_t,fig_s_as,fig_s_ao,fig_r_t,fig_r_as,fig_r_ao]
	# 		type_names = ['typeSize_vs_time','typeSize_vs_accShape','typeSize_vs_accRot','typeOrientation_vs_time',
	# 		'typeOrientation_vs_accShape','typeOrientation_vs_accRot']
	# 		plot_exp.saveFigs(type_figs,type_names,out_dir,'.png')
	# 		plt.close('all')

	# 		# (1) plot trial times
	# 		# copy and set custom legends
	# 		time1_info = dict(plot_exp.trial_time) 
	# 		time1_info['legends'] = [proces.exp_id[exp_nr]]
	# 		# plot using custom legend
	# 		fig_time,ax_time = plot_exp.singlePlotDf(exp_data[exp_nr], time1_info)

	# 		# add trial times plot from other experiment (1-2)
	# 		# set new color for line
	# 		# time2_info = dict(time1_info)
	# 		# time2_info['colors'] = ['r']
	# 		# time2_info['legends'] = [proces.exp_id[exp_nr]]
	# 		# # add line to figure using other color
	# 		# ax_time = plot_exp.addLineFromDfToPlot(ax_time, exp_data[exp_nr], time2_info)

	# 		# (2) plot accuracy
	# 		acc_info = dict(plot_exp.accuracy)
	# 		acc_info['title'] = proces.exp_id[exp_nr]
	# 		fig_acc, ax_acc = plot_exp.singlePlotDf(exp_data[exp_nr], acc_info)

	# 		# (3) plot shape error (diameter)
	# 		shape_error_info = dict(plot_exp.shape_error)
	# 		shape_error_info['title'] = proces.exp_id[exp_nr]
	# 		fig_shape, ax_shape = plot_exp.singlePlotDf(exp_data[exp_nr], shape_error_info)

	# 		# (4) plot average shape error (diameter)
	# 		avg_shape_error_info = dict(plot_exp.avg_shape_error)
	# 		fig_avgShape, ax_avgShape = plot_exp.singlePlotDf(exp_data[exp_nr], avg_shape_error_info)

	# 		# (5) plot angle
	# 		angle_info = dict(plot_exp.abs_angle)
	# 		fig_angle, ax_angle = plot_exp.singlePlotDf(exp_data[exp_nr], angle_info)

	# 		# (6) plot user scales
	# 		user_scales_info = dict(plot_exp.user_scales)
	# 		fig_us, ax_us = plot_exp.singlePlotDf(exp_data[exp_nr], user_scales_info)

	# 		# (7) plot projected scales
	# 		proj_scales_info = dict(plot_exp.projected_scales)
	# 		fig_ps, ax_ps = plot_exp.singlePlotDf(exp_data[exp_nr], proj_scales_info)
			
	# 		# (8) plot experiment scales
	# 		exp_scales_info = dict(plot_exp.exp_scales)
	# 		fig_es, ax_es = plot_exp.singlePlotDf(exp_data[exp_nr], exp_scales_info)


	# 		# (9) plot originals experiment scales
	# 		or_exp_scales_info = dict(plot_exp.or_user_scales)
	# 		fig_oes, ax_oes = plot_exp.singlePlotDf(exp_data[exp_nr], or_exp_scales_info)

	# 		# (10) plot user quats
	# 		user_quats_info = dict(plot_exp.user_quats)
	# 		fig_uq, ax_uq = plot_exp.singlePlotDf(exp_data[exp_nr], user_quats_info)

	# 		# (11) plot experiment quats
	# 		exp_quats_info = dict(plot_exp.exp_quats)
	# 		fig_eq, ax_eq = plot_exp.singlePlotDf(exp_data[exp_nr], exp_quats_info)

	# 		# (12) plot originals experiment quats
	# 		or_exp_quats_info = dict(plot_exp.or_user_quats)
	# 		fig_oeq, ax_oeq = plot_exp.singlePlotDf(exp_data[exp_nr], or_exp_quats_info)

	# 		basic_figs = [fig_time,fig_acc,fig_shape,fig_avgShape,fig_angle,fig_us,fig_ps,fig_es,fig_oes,fig_uq,fig_eq,fig_oeq]
	# 		basic_names = ["1_trial_time","2_accuracy","3_scale_error","4_average_scales","5_angle","6_user_scales",
	# 		"7_proj_scales","8_exp_scales","9_exp_scales_original","10_user_quats","11_exp_quats","12_user_quats_original"]
	# 		plot_exp.saveFigs(basic_figs,basic_names,out_dir,'.png')
	# 		plt.close('all')
	# 		print('RUSTAAAGH!!!, plotjes worden gemaakt')
	# 	# if part_nr == 1:

	# 	sys.exit()
	# 	# 	print('not stopped?')			

	# 	# Create list with uniqe ellipsoids
	# 	# only for the real trials
	# 	# only need to do this once

	# 	if part_nr == 1:
	# 		# proces.uniqueEllipsoids, proces.IDstrings = proces.getUniqueEllipsCombinations([exp_data[2],exp_data[4],exp_data[6],exp_data[8]])
	# 		proces.uniqueEllipsoids, proces.IDstrings = proces.getUniqueEllipsCombinations(
	# 			[part_data_list[0].data['1R']['simple_experiment_data'],
	# 			part_data_list[0].data['2R']['simple_experiment_data'],
	# 			part_data_list[0].data['3R']['simple_experiment_data'],
	# 			part_data_list[0].data['4R']['simple_experiment_data']])
	# 		# print(proces.IDstrings)
	# 		# print(proces.uniqueEllipsoids)

		
	# 	experiments = [ exp_data[2], exp_data[4], exp_data[6], exp_data[8] ]
	# 	typesDf_part_x = proces.getUniqueDfsList(proces.uniqueEllipsoids, experiments)
	# 	# print(typesDf_part_x)
	# 	typesDf.append(typesDf_part_x)
		
	# 	# print(self.IDstrings)
	# 	# print(self.uniqueEllipsoids)
	# 	# sys.exit()


	# # print(len(typesDf)) # 2
	# # print(len(typesDf[0])) # 20
	# # more advanced plots
	# # boxplots



	# ############## GET DATA OF EACH PARTICIPANT ##############
	# # initialize some names and properties
	# column_names = ['Exp1_Shape','Exp1_Orientation','Exp1_Trial_time',
	# 'Exp2_Shape','Exp2_Orientation','Exp2_Trial_time',
	# 'Exp3_Shape','Exp3_Orientation','Exp3_Trial_time',
	# 'Exp4_Shape','Exp4_Orientation','Exp4_Trial_time']

	# y_axes_names = ['Acc [%]','Time [s]']
	# x_tick_labels = ['experiment 1', 'experiment 2', 'experiment 3', 'experiment 4']
	# x_ticks = range(len(x_tick_labels))

	# # data that is interesting to keep
	# keep_list = ['field.trial_time','field.shape_acc','field.orientation_acc','field.shape','field.absolute_angle']

	# # get name and data per participant
	# part_name_list = []
	# # create name string
	# partName = lambda nr: 'part_' + str(nr)
	# means_std_per_part = [] # list with means and std per participant in pandas df
	# for part_nr,part_x in enumerate([part_data_list[0]]): # for loop over the participants
	# 	part_nr += 1 # start at part_1 (no patient zero this time)
	# 	# part_nr=6
	# 	part_name_list.append(partName(part_nr))
	# 	exp_data = {
	# 					1: part_x.data['1L']['simple_experiment_data'],
	# 					2: part_x.data['1R']['simple_experiment_data'],
	# 					3: part_x.data['2L']['simple_experiment_data'],
	# 					4: part_x.data['2R']['simple_experiment_data'],
	# 					5: part_x.data['3L']['simple_experiment_data'],
	# 					6: part_x.data['3R']['simple_experiment_data'],
	# 					7: part_x.data['4L']['simple_experiment_data'],
	# 					8: part_x.data['4R']['simple_experiment_data'],
	# 	}
	# 	experiments = [ exp_data[2], exp_data[4], exp_data[6], exp_data[8] ]
	# ################## GET MEANS AND STDS OF ONE PARTICIPANT FOR ALL PARTICIPANTS ################## 
	# 	# get from from data class
	# 	part_avgstds = part_x.getMeanAndStds(experiments)
	# 	# drop unimportant rows
	# 	part_avgstds = part_avgstds.transpose()[keep_list]
	# 	# add to list
	# 	means_std_per_part.append( part_avgstds )
	# # create, rename and shape one df from list of dfs
	# mean_stds_all_part = pd.concat(means_std_per_part, axis=0, keys=part_name_list)
	# mean_stds_all_part = mean_stds_all_part.reset_index()
	# mean_stds_all_part.rename(columns = {'level_0':'participants', 'level_1':'calc'},inplace=True)
	# mean_stds_all_part = mean_stds_all_part.pivot_table(index='participants', columns='calc', values=keep_list)

	# ################## GET MEANS OF THE MEANS AND STDS OF THE PARTICIPANTS ################## 
	# average_of_means_stds = mean_stds_all_part.mean(axis=0)
	# average_of_means_stds = average_of_means_stds.reset_index()
	# average_of_means_stds.rename(columns = {'level_0':'metrics', 0:'data'}, inplace=True)
	# average_of_means_stds = average_of_means_stds.pivot_table(index='metrics',columns='calc',values='data')
	# # print(average_of_means_stds.iloc[:,0:4])


	# ################## GET MEANS AND STDS OF THE TOTAL TRIALS ################## 
	# # get means/stds of all trials
	# # keep_list = ['size_type','rot_type','field.trial_nr','field.trial_time','field.shape_acc','field.orientation_acc','field.shape','field.absolute_angle']
	# experiments = ['1R', '2R', '3R', '4R']
	# topic = 'simple_experiment_data'
	# keep_list = ['field.trial_nr','field.trial_time','field.shape_acc','field.orientation_acc','field.shape','field.absolute_angle']
	

	# exp_data = [] # list of 4 experiments df
	# for i, experiment in enumerate(experiments):

	# 	# get data of experiment_x for all participants
	# 	dfList = []
	# 	for i, part_x in enumerate(part_data_list): 
	# 		dx = part_x.data[experiment][topic][keep_list]
	# 		dfList.append(dx)
	# 	# add participants to exp x
	# 	exp_x_data = pd.concat(dfList,axis=0,keys=part_name_list)
	# 	# print(exp_x_data)	
	# 	exp_data.append(exp_x_data)

	# # wie = 'jelle'
	# wie = 'jasper'
	# # wie = 'jasper2'
	# # wie = 'iedereen'
	# out_dir = []
	# if wie == 'jelle':
	# 	exp_data = [exp_x_data.loc[ part_name_list[1], :] for exp_x_data in exp_data]
	# 	out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_2"
	# elif wie == 'jasper':
	# 	exp_data = [exp_x_data.loc[ part_name_list[0], :] for exp_x_data in exp_data]
	# 	out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_1"
	# elif wie == 'jasper2':
	# 	exp_data = [exp_x_data.loc[ part_name_list[0], :] for exp_x_data in exp_data]
	# 	out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/part_6"
	# else:
	# 	out_dir = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/figures/All"
	# 	pass

	# # remove nans and convert to list for each experiment
	# getData = lambda df,name: df[name].dropna().tolist()
	# exp1 = [getData(exp_data[0],'field.shape_acc'), getData(exp_data[0],'field.orientation_acc'), getData(exp_data[0],'field.trial_time')]
	# exp2 = [getData(exp_data[1],'field.shape_acc'), getData(exp_data[1],'field.orientation_acc'), getData(exp_data[1],'field.trial_time')]
	# exp3 = [getData(exp_data[2],'field.shape_acc'), getData(exp_data[2],'field.orientation_acc'), getData(exp_data[2],'field.trial_time')]
	# exp4 = [getData(exp_data[3],'field.shape_acc'), getData(exp_data[3],'field.orientation_acc'), getData(exp_data[3],'field.trial_time')]


	# # Boxplottypes
	# # combine participants
	# ################## PLOT BOXPLOT types ##################
	# all_types_all_part = []
	# # all_types_part_0 = typesDf[0] 
	# # all_types_part_1 = typesDf[1] 
	# all_types_part_2 = typesDf[0] 

	# for i in range(len(typesDf[0])):  # loop over index of the ellipsoid types
	# 	combined_type_x = proces.concatDfForMultiplePart(typesDf, i, part_name_list) # concats ellipse type i=0,1,2,3 ... 19
	# 	all_types_all_part.append(combined_type_x)

	# # print(all_types_all_part[4:10])
	# # .loc[:,['DoF','size','type','rotation_axis','rotation']]
	# # fig_time_acc_types_1 = plot_exp.BoxPlotGroup2axisTypes(all_types_all_part[0:4],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[0:4])
	# # fig_time_acc_types_2 = plot_exp.BoxPlotGroup2axisTypes(all_types_all_part[4:10],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[4:10])
	# # fig_time_acc_types_3 = plot_exp.BoxPlotGroup2axisTypes(all_types_all_part[10:14],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[10:14])
	# # fig_time_acc_types_4 = plot_exp.BoxPlotGroup2axisTypes(all_types_all_part[14:20],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[14:20])
	# # fig_time_acc_types_5 = plot_exp.BoxPlotGroup2axisTypes(all_types_all_part,['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings)
	# # fig_time_acc_types_5.set_size_inches(20,8, forward=True)
	# # fig_time_acc_types_1 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_0[0:4],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[0:4])
	# # fig_time_acc_types_2 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_0[4:10],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[4:10])
	# # fig_time_acc_types_3 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_0[10:14],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[10:14])
	# # fig_time_acc_types_4 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_0[14:20],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[14:20])
	# # fig_time_acc_types_5 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_0,['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings)
	# # fig_time_acc_types_5.set_size_inches(20,8, forward=True)
	# # fig_time_acc_types_1 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_1[0:4],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[0:4])
	# # fig_time_acc_types_2 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_1[4:10],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[4:10])
	# # fig_time_acc_types_3 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_1[10:14],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[10:14])
	# # fig_time_acc_types_4 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_1[14:20],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[14:20])
	# # fig_time_acc_types_5 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_1,['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings)
	# # fig_time_acc_types_5.set_size_inches(20,8, forward=True)
	# fig_time_acc_types_1 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_2[0:8],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[0:8])
	# fig_time_acc_types_2 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_2[8:18],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[8:18])
	# fig_time_acc_types_3 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_2[18:26],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[18:26])
	# fig_time_acc_types_4 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_2[26:36],['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings[26:36])
	# fig_time_acc_types_5 = plot_exp.BoxPlotGroup2axisTypes(all_types_part_2,['field.shape_acc','field.orientation_acc','field.trial_time'],proces.IDstrings)
	# fig_time_acc_types_5.set_size_inches(20,8, forward=True)
	
	# box_figs = [fig_time_acc_types_1,fig_time_acc_types_2,fig_time_acc_types_3,fig_time_acc_types_4,fig_time_acc_types_5]
	# box_names = ["types_exp_1","types_exp_2","types_exp_3","types_exp_4","types_exp_all"]
	# plot_exp.saveFigs(box_figs,box_names,out_dir,'.png')
	# # plt.show()
	# plt.close('all')


	# # print(all_types_all_part[0].columns.values)
	# # print(all_types_all_part[0].index.values)
	# # print(all_types_all_part[0][all_types_all_part[0]=='part_1'].head(1))

	# ################## PLOT BOXPLOT ##################
	# fig_time_acc = plot_exp.BoxPlotGroup2axis(exp1,exp2,exp3,exp4)

	# box_figs = [fig_time_acc]
	# box_names = ["box_time_accuracy"]
	# plot_exp.saveFigs(box_figs,box_names,out_dir,'.png')
	# plt.close('all')





	# ################## OTHER ##################
	# # plt.draw()
	# # check what kind of set attributes the ax class has

	# # ax_getters = [getter for getter in dir(ax) if 'get' in getter]
	# # print(ax_getters)

	# # check what kind of set attributes the ax class has
	# # ax_setters = [setter for setter in dir(ax) if 'set' in setter]
	# # print(ax_setters)

	# # # check what kind of set attributes the fig class has
	# # fig_setter = [setter for setter in dir(fig) if 'set' in setter]
	# # print(fig_setter)