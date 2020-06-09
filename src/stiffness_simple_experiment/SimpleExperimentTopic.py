#!/usr/bin/env python
from ManageDataDirectories import ManageDataDirectories
from ParticipantData import ParticipantData

# single participant
# get relavant directories, experiment_name abreviations, to set the data in the ParticipantData class
part_dir = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1'

MD = ManageDataDirectories()
exp_IDs = MD.experiment_IDs # ['1L','1R','2L','2R','3L','3R','4L','4R']
topics = ["simple_experiment_data"]
# get relavant directories containing the files
partx_info_file_path, _, partx_csvdir_paths, partx_txt_paths, _ = MD.getAllPathsOfParticipant(part_dir, MD.csv_dir_list)
csvfile_paths = MD.getFilesInDirList(partx_csvdir_paths)




# set the files in one dictionari
PD = ParticipantData(exp_IDs, partx_info_file_path, partx_txt_paths, partx_csvdir_paths, csvfile_paths, topics)


partx_data = PD.data
# dictionair keys and subkeys:
# partx_data.keys(): 							['participant_info', '1R', '2R', '3R', '4R', '4L', '3L', '2L', '1L']
	# partx_data['participant_info'].keys(): 	['gender', 'age', 'number', 'experience', 'hand']
	# partx_data[1R].keys(): 					['experiment_notes', 'simple_experiment_data']


def removeFirstTrials(dataDict,exp_IDs):
	for exp_x in exp_IDs:
		df = dataDict[exp_x]['simple_experiment_data']
		df.drop(df.head(1).index, inplace=True)
	# note that df is removed in place thus dataDict is changed in place!
	return dataDict

partx_data = removeFirstTrials(partx_data,exp_IDs)
print(partx_data['1L']['simple_experiment_data'].head(1))