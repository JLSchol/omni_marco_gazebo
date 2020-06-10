#!/usr/bin/env python
import sys
import subprocess
from os import listdir, walk ,mkdir, path
import errno
# python 2.7
try:
	from itertools import izip as zip
except ImportError:
	pass

# custom classes
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '/home/jasper/omni_marco_gazebo/src/data_processing/scripts')
from ImportFiles import ImportFiles


class ManageDataDirectories():
	def __init__(self):
		# top data dir
		self.root_dir_with_data = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data"
		self.part_paths = self.getparticipantPaths(self.root_dir_with_data)
		#directories to be created
		self.csv_dir_list = ["csvs_1_learn","csvs_1_real","csvs_2_learn","csvs_2_real",
					"csvs_3_learn","csvs_3_real","csvs_4_learn","csvs_4_real"]

		self.experiment_IDs = ['1L','1R','2L','2R','3L','3R','4L','4R'] # this is a predefined and ordered list

		self.csv_dir_list.sort()	
		self.folder_bags = []



	def getparticipantPaths(self,root_dir_with_data):
		part_dirs = listdir(root_dir_with_data)
		part_dirs.sort()
		append_folder = lambda path,folder: str(path) + "/" + str(folder) 
		part_paths = [append_folder(self.root_dir_with_data, part_dir) for part_dir in part_dirs]
		return part_paths

	def createCsvDirsForEveryParticipant(self,part_paths,csv_dir_list):
		# Creat sub dirs for every participant		
		for part_path in part_paths:
			[self.createDir(part_path, sub_dir) for sub_dir in csv_dir_list]

	def createDir(self, path, dir_name):
		full_path = path + "/" + dir_name
		try:
			mkdir(full_path)
			print("created: {}".format(full_path))

		except OSError as e:
		    if e.errno == errno.EEXIST:
		        print("directiory: {} already exists in: {}".format(dir_name,path))
		    else:
		        raise


	def callBashFile(self,path,file_name,args):
		# fileName = "kill_nodes.sh"
		# path = "/home/usr/blablabla"
		# args = "$1 $2 $3" etc..
		full_path = path + "/" + file_name
		command = full_path +" "+ args
		subprocess.call(command,shell=True)

	def getAllPathsOfParticipant(self,part_path,csv_dir_list):
		# get all the pahts and sort
		# returns the full paths of all relavant files/ directories
		# part_info, rosbagpaths, csvDirectories, trialnotes(txt), roslaunchParameters

		file_folders = listdir(part_path)
		partx_bag_paths = self.getBagPaths(part_path,file_folders)
		partx_txt_paths = self.getTxtPaths(part_path,file_folders)
		append_folder = lambda path,folder: str(path) + "/" + str(folder)
		partx_csvdir_paths = [append_folder(part_path, csv_dir) for csv_dir in csv_dir_list] 
		partx_yaml_paths = self.getYamlPaths(part_path,file_folders)

		partx_bag_paths.sort()
		partx_txt_paths.sort()
		partx_csvdir_paths.sort()
		part_info_file = self.ifContains(partx_yaml_paths, "info.yaml")

		if part_info_file != None:
			partx_yaml_paths.remove(part_info_file)
		partx_yaml_paths.sort()

		return part_info_file, partx_bag_paths, partx_csvdir_paths, partx_txt_paths, partx_yaml_paths

	def getBagPaths(self,folder_path,file_folder_list):
		bag_paths = [folder_path+"/"+file_folder for file_folder in file_folder_list if file_folder.endswith(".bag")]
		return bag_paths

	def getTxtPaths(self,folder_path,file_folder_list):
		txt_paths = [folder_path+"/"+file_folder for file_folder in file_folder_list if file_folder.endswith(".txt")]
		return txt_paths
		
	def getYamlPaths(self,folder_path,file_folder_list):
		yaml_paths = [folder_path+"/"+file_folder for file_folder in file_folder_list if file_folder.endswith(".yaml")]
		return yaml_paths
		
	def ifContains(self, string_list, identifier):
		for string in string_list:
			if identifier in string:
				return string
			else:
				pass

	def exportBagsToCsvDirs(self, bash_file_path, bash_file_name, folder_bag_dict, topics):
		for output_dir, rosbag_path in folder_bag_dict.items():
			#check if output dir and rosbag_path match!
			part_nr_check = self.checkPartNr(output_dir, rosbag_path)
			practice_learn_check = self.checkPracticeLearn(output_dir, rosbag_path)
			exp_check = self.checkExpNr(output_dir, rosbag_path)
			if part_nr_check == True and practice_learn_check == True and exp_check == True:
				bash_args = rosbag_path + " " + output_dir + " " + topics
				print("Exporting: {}\nWith topic(s): {}\nto:\n{}".format(rosbag_path,topics,output_dir))
				self.callBashFile(bash_file_path, bash_file_name, bash_args)
			else:
				print("{} is not exported to \n{}\n Due to wrong bag/dir combination (check string)"
																			.format(rosbag_path,output_dir))
				continue

	def checkPartNr(self, output_dir, rosbag_path):
		partNr = output_dir.split("data/part_",1)[1][0] # find partNr from outputDir
		unique_string_dir = "data/part_"+partNr
		unique_string_bag = "Part"+partNr

		dir_check = unique_string_dir in output_dir
		bag_check = unique_string_bag in rosbag_path
		if dir_check == True and bag_check==True:
			return True
		else:
			print("partNr does not match in {} and \n{}".format(output_dir, rosbag_path))
			return False
		
	def checkPracticeLearn(self,output_dir, rosbag_path):
		# need to check real or learn
		# guess learn
		unique_string_dir = "_learn"
		unique_string_bag = "_Learn_"
		if "_real" in output_dir:
			# change to real
			unique_string_dir = "_real"
			unique_string_bag = "_Real_"
		dir_check = unique_string_dir in output_dir
		bag_check = unique_string_bag in rosbag_path
		if dir_check == True and bag_check==True:
			return True
		else:
			print("Practice/Learning does not match in {} and \n{}".format(output_dir, rosbag_path))
			return False
		
	def checkExpNr(self, output_dir, rosbag_path):
		expNr = output_dir.split("csvs_",1)[1][0] # find partNr in outputDir
		unique_string_dir = "csvs_"+expNr
		unique_string_bag = "_Exp"+expNr+"_"

		dir_check = unique_string_dir in output_dir
		bag_check = unique_string_bag in rosbag_path
		if dir_check == True and bag_check==True:
			return True
		else:
			print("exp number does not match in {} and \n{}".format(output_dir, rosbag_path))
			return False
		

	def getTopics(self,dir_path="/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/csvs_1_learn"):
		topics = []
		for (dirpath, dirnames, filenames) in walk(dir_path):
			for fileName in filenames:
				topic = fileName.split(".", 1)[0]
				topics.append(topic)
			return topics

	def getFilesInDir(self,dir_path):
		files = listdir(dir_path)
		return [dir_path + '/' + file for file in files]


	def getFilesInDirList(self,dir_paths,extension='.csv'):
		file_paths = []
		for path in dir_paths: #list with paths to different directories
			file_path_list = self.getFilesInDir(path) # add the files to directory path
			for path in file_path_list: # for each path is fileList
				if path.endswith(extension): # filter for correct extension
					file_paths.append(path)
		return file_paths

	def addParticipant(self,part_path,topics):
		bash_file_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/"
		bash_file_name = "rosbag_2_csvs.sh"
		if not isinstance(topics, list):
			if isinstance(topics, str):
				topics = [topics]
			else:
				print("in addParticipant(self,part_path,topics); topics is not of type string or list")

		# add folders
		[self.createDir(part_path, sub_dir) for sub_dir in self.csv_dir_list]

		# get paths
		_, partx_bag_paths, partx_csvdir_paths,_,_ = self.getAllPathsOfParticipant(part_path, 
																							self.csv_dir_list)
		partx_folder_bag_dict = dict(zip(partx_csvdir_paths, partx_bag_paths))

		# export topics from bag
		for topic in topics:
			self.exportBagsToCsvDirs(bash_file_path, bash_file_name, partx_folder_bag_dict, topic)





	def main(self):
		# folder structure
		# Create dirs to store the data
		# root_dir_with_data
		# part_paths (part_1, part_2 etc)
		# csv_expi_learn/real, part_i_info, parti_Expi_learn/real.(bag/txt/yaml,

		# create the csvs directories for every participant
		# extract rosbags in that directory

		# path to participants

		# create csv_data folders for each participant

		self.createCsvDirsForEveryParticipant(self.part_paths, self.csv_dir_list)

		# find folder and bag locations for each participant put in dictionair
		folder_bags = []
		for part_path in self.part_paths:
			# part_info_file, partx_bag_paths, partx_csvdir_paths, partx_txt_paths, partx_yaml_paths
			_, partx_bag_paths, partx_csvdir_paths,_,_ = self.getAllPathsOfParticipant(part_path, 
																							self.csv_dir_list)

			partx_folder_bag_dict = dict(zip(partx_csvdir_paths, partx_bag_paths))
			folder_bags.append(partx_folder_bag_dict)



		# use the dictionair to call a bash file that extracts the rosbags to a csv file
		# ./rosbag2csvs.sh 	path2file 	outputDir 
		# $0 				$1			$2 		
		bash_file_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/"
		bash_file_name = "rosbag_2_csvs.sh"
		for ipart,part_path in enumerate(self.part_paths):
			# check if size of dict is bot equal and 8
			assert len(folder_bags[ipart].keys()) == len(folder_bags[ipart].values()) 
			if len(folder_bags[ipart].keys()) != 8:
				print("amount of bags is: {} and should be 8 if all the experiments are done"
														.format(len(folder_bags[ipart].keys())))
				print("check bagfiles and csv folders for participant: {}".format(ipart))
			self.exportBagsToCsvDirs(bash_file_path, bash_file_name, folder_bags[ipart], "/simple_experiment_data")

		# export rosbags to correct folder
		# every experiment has an unique rosbag thus nr_bag = 4(part)*8(exp) = 32 bags 
		# self.exportBagsToCsvDirs()

		
		# data_dirs_dict = {part_dir: {} for part_dir in part_dirs}
		# print(data_dirs_dict)
		# part_keys = ["info","folders","bags","parameters","notes"]
		# exp_keys = ["1L","1R","2L","2R","3L","3R","4L","4R"]


if __name__ == "__main__":
	MD = ManageDataDirectories()

	print("uncomment addParticipant() or main() or callBashFile() part")
	
	# # add everything from all participants
	# MD.main()

	# # add single participant
	# partpath = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_3"
	# topics = ["/simple_experiment_data"]
	# MD.addParticipant(partpath,topics)

	# # extract specific topic(s) from bash file to specific folder
	# destination_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/csvs_4_real"
	# bag_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/data/part_1/Part_Exp4_Real_05141713.bag"

	# bash_file_name = "rosbag_2_csvs.sh"
	# folder_with_bash_file = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment"
	# topics = "/simple_experiment_data"
	# bash_args = bag_path + " " + destination_path + " " + topics
	# MD.callBashFile(folder_with_bash_file,bash_file_name,bash_args)



