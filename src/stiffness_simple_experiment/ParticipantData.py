#!/usr/bin/env python
import yaml
import pandas as pd
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass
# import pandas.io.common

# custom imports
from ManageDataDirectories import ManageDataDirectories


class ParticipantData():
	# class that loades and stores data from the participant
	# class that stores data from the participant
	def __init__(self,exp_IDs, infofile_path, experiment_notes_paths ,csvdir_paths, csvfile_paths, topics):

		self.exp_IDs = exp_IDs 						# keys from dict identifying which experiment
		self.infofile_path = infofile_path				# full path to participant information file
		self.experiment_notes_paths = experiment_notes_paths		# full path to experiment notes for all tests
		self.csvdir_paths = csvdir_paths					# full path to directories that contain the csv files 
		self.csvfile_paths = csvfile_paths				# full path to csv files of the recorded topics
		self.topics = [str(topic) for topic in topics]		# topic names (without /) used for keys in dictionair



		###### INITIALIZE STRUCTURE OF DATA DICTIONAIR ######
		# set keys for sub dictionair: expteriment_dict
		self.experiment_keys = ['experiment_notes'] + self.topics
		# initialize sub dictionair
		experiment_dict = dict.fromkeys(self.experiment_keys)
		# experiment_dict = {
		# 	"experiment_notes"	: 	"list of strings with experiment notes from txt file",
		#	"topic1"			:	"pandas data frame of topic 1",
		# 	"topic2"			: 	"pandas data frame of topic 2"
		#					}

		# Set keys for dictionair: data
		self.data_keys = ['participant_info'] + self.exp_IDs + ['vanDerLaan']
		# initialize dictionair 
		self.data = dict.fromkeys(self.data_keys)
		for exp_ID in self.exp_IDs:
			self.data[exp_ID] = dict(experiment_dict)

		###### SET DATA IN THE DICTIONAIR ######
		# set participant info file
		self.setParticipantInfo(self.infofile_path)

		# set experiment notes
		self.setTestsNotes(self.exp_IDs, self.experiment_notes_paths)

		# set pandas data frame for all recorded topics
		self.setExperimentData(self.exp_IDs, self.csvdir_paths, self.csvfile_paths, self.topics)


			# self.data = {
			# "participant_info": 	"infoDictionair",

			# "1L": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "1R": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "2L": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "2R": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "3L": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "3R": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "4L": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# "4R": 					{"experiment_notes": 	"txtfile",
			# 						"topic1": 				"pandas data frame of topic 1",
			# 						"topic2": 				"pandas data frame of topic 2"},
			# 			}


	def _load_csv_in_df(self, file_path):
		df = []
		try:
			df = pd.read_csv(file_path)
		except pd.io.common.EmptyDataError:
	  		print("csv is empty and not loaded into df")
	  		pass
	  	return df
		
	def _load_txt(self, file_path):
		try:
		    with open(file_path) as f:
		        return f.readlines()
		except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
		    print 'oops'

	def _load_yaml(self, file_path):
		with open(file_path, 'r') as stream:
		    try:
		        yamltje = yaml.safe_load(stream)
		    except yaml.YAMLError as exc:
		        print(exc)
    		return yamltje


	def setParticipantInfo(self, file_path):
		self.data['participant_info'] = self._load_yaml(file_path)
		

	def _setSingleTestNotes(self,experiment,file_path):
		# one note from one of the eight tests
		test_notes = self._load_txt(file_path)
		self.data[experiment]['experiment_notes'] = test_notes[:]
	def setTestsNotes(self,experiment_names,file_paths):
		for experiment_name,path in zip(experiment_names,file_paths):
			# sets all the note of all tests
			# write a check that verifies if self.exp_IDs[i] matches the file_paths 
			self._setSingleTestNotes(experiment_name, path)
		
	def _setTopicsInExp(self,experiment,topics,file_paths):
		for topic,path in zip(topics,file_paths):
			if topic in path:
				df = self._load_csv_in_df(path)
				self.data[experiment][topic] = df
			else:
				print("topic {} and file path {} do not correspond and are skiped".format(topic,path))
				continue

			
	def setExperimentData(self,experiment_names, experiment_dirs, all_file_paths, topics):
		# set for all experiments ['1L','1R', etc]
		for exp_name, exp_dir in zip(experiment_names, experiment_dirs):
			paths_containing_exp_dir = []
			for path in all_file_paths:
				if exp_dir in path:
					paths_containing_exp_dir.append(path)
			self._setTopicsInExp(exp_name, topics, paths_containing_exp_dir)

	def getMeanAndStds(self,dfList):
		ms_list = []
		for i,df in enumerate(dfList):
			means = df.mean(axis = 0)
			stds = df.std(axis = 0)
			ms_df = pd.concat([means,stds], axis=1, keys=['mean_'+str(i+1),'std_'+str(i+1)])
			ms_list.append(ms_df)

		complete_df = pd.concat(ms_list,axis=1)
		return complete_df

	def setVanDerLaan(self,scores):
		keys = [1,2,3,4,5,6,7,8,9]
		self.data['vanDerLaan'] = dict(zip(keys, scores))





	def main(self):
	
		# test functions
		# text = self._load_txt('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/Part1_Exp1_Learn_05181708.txt')
		# df = self._load_csv_in_df('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/csvs_1_learn/simple_experiment_data.csv')
		# yaml = self._load_yaml('/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/part_1_info.yaml')
		# print(self.data)

		print("KEYS:")
		print("data.keys(): {}".format(self.data.keys()))
		print("data[expID].keys(): {}".format(self.data['1L'].keys()))
		print('--')


		print("info_dict: {}".format(self.data['participant_info']))
		print("experiment1L_notes: {}".format(self.data['1L']['experiment_notes'][1]))
		print("experiment4R_notes: {}".format(self.data['4R']['experiment_notes'][1]))
		print('--')

		print("experiment_data_trialtime: {}".format(self.data['1L']['simple_experiment_data']['field.trial_time'][0]))
		print("experiment_data_trialtime: {}".format(self.data['4R']['simple_experiment_data']['field.trial_time'][0]))
		# print()

		# print(self.data['1R']['simple_experiment_data'].head(1))
		# print(self.data['2L']['simple_experiment_data'].head(1))
		# print(self.data['2R']['simple_experiment_data'].head(1))
		# print(self.data['3L']['simple_experiment_data'].head(1))
		# print(self.data['3R']['simple_experiment_data'].head(1))
		# print(self.data['4L']['simple_experiment_data'].head(1))
		# print(self.data['4R']['simple_experiment_data'].head(1))




if __name__ == "__main__":
	MD = ManageDataDirectories()
	topics = MD.getTopics()
	exp_IDs = MD.experiment_IDs
	part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1'

	part_info_file_path, _, paths_csvdirs, partx_txt_paths, _ = MD.getAllPathsOfParticipant(part_dir, MD.csv_dir_list)
	csvfile_paths = MD.getFilesInDirList(paths_csvdirs)

	
	# print(MD.experiment_IDs) # ['1L','1R','2L','2R','3L','3R','4L','4R']


	PD = ParticipantData(exp_IDs, part_info_file_path, partx_txt_paths, paths_csvdirs, csvfile_paths, topics)
	PD.addVanDerLaan(scores)


	# PD.setParticipantInfo(part_info_file_path)
	# PD.setTestsNotes(MD.experiment_IDs,partx_txt_paths)

	# PD.setExperimentData(MD.experiment_IDs, paths_csvdirs, paths_csvfiles, topics)

	PD.main()