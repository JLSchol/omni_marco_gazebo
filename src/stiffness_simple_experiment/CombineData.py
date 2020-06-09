#!/usr/bin/env python
from ManageDataDirectories import ManageDataDirectories
from ParticipantData import ParticipantData
import os
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass
import json
import copy
import pandas as pd
# find some dirs,
# set some variables
# get some names

# werkt niet

class ManageRawData():
	def __init__(self):
		self.data_path = '/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data'
		self.part_dirs = os.listdir(self.data_path)
		part_paths = [self.data_path + '/' + part_dir for part_dir in self.part_dirs]

		MD = ManageDataDirectories()
		self.exp_IDs = MD.experiment_IDs # ['1L','1R','2L','2R','3L','3R','4L','4R']
		self.topics = ["simple_experiment_data"]

		self.df_index = [str(i) for i in range(1,31)]
		# self.df_index = [ 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30]

		# get data from each particpant and combine in part_data
		part_data = []
		for partx_path in part_paths:
			# get relavant directories containing the files for each participant x
			partx_info_file_path, _, partx_csvdir_paths, partx_txt_paths, _ = MD.getAllPathsOfParticipant(partx_path, MD.csv_dir_list)
			csvfile_paths = MD.getFilesInDirList(partx_csvdir_paths)

			# set data in participant class
			PD = ParticipantData(self.exp_IDs, partx_info_file_path, partx_txt_paths, partx_csvdir_paths, csvfile_paths, self.topics)

			# create list with all participant dicts 
			part_data.append(PD.data)


		# set the files in one dictionair
		self.raw_data = dict(zip(self.part_dirs,part_data))
		# self.raw_data_json = self.convertToJson(dict(self.raw_data))
	
	def convertToJson(self,data):
		i=0
		data2 = copy.deepcopy(data)
		# print(data['part_3']['4R']['simple_experiment_data'])
		# print(type(data['part_3']['4R']['simple_experiment_data']))
		# print()
		for part in self.part_dirs:
			for exp in self.exp_IDs:
				for topic in self.topics:
					try:
						data2[part][exp][topic] = data2[part][exp][topic].to_json()
					except AttributeError:
						print("Wat de schijt gebeurt er nou weer")
						continue
		return data2



	def writeToFile(self,jsonData, file_path='/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/raw_data.json'):
		raw_data_json = json.dumps(jsonData)
		with open(file_path, 'w') as f:
			f = open("raw_data.json","w")
			f.write(raw_data_json)


	def readFromFile(self, file_path='/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/raw_data.json'):
		f = open('raw_data.json')
		d = json.load(f)
		f.close()
		i=0
		for part in self.part_dirs:
			for exp in self.exp_IDs:
				for topic in self.topics:
					i +=1
					print(i)
					# print(data['part_3']['4R']['simple_experiment_data'])
					df = pd.read_json(d[part][exp][topic])
					# print(self.df_index)
					# df.reindex(self.df_index)
					# print(df.index.values)
					print( df.ix[23,'field.trial_time'] )
					# print(df.index.values)
					print(self.raw_data[part][exp][topic].ix[23,'field.trial_time'])
					print"--"
		print(self.raw_data['part_3']['4R']['simple_experiment_data'].index.values)
		# print(type(d['part_3']['4R']['simple_experiment_data']))

		# with open(file_path, 'r') as f:
		# 	print(type(f))
		# 	data = json.load(f)

	def main(self):
		dataJson = self.convertToJson(self.raw_data)
		self.writeToFile(dataJson)
		self.readFromFile()


if __name__ == "__main__":
	MRD = ManageRawData()
	MRD.main()