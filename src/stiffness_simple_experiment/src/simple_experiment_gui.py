#!/usr/bin/env python

from rospy import Subscriber, Publisher, init_node, Time
import os
import errno
import subprocess
import yaml
import roslaunch
from time import strftime
import unicodedata
import inspect


from Tkinter import * 
from functools import partial
import ttk
from std_msgs.msg import Header, Bool, String
# custom msgs
from stiffness_simple_experiment.msg import gui_command

class GuiWindow(Frame):

	def __init__(self,master=None):
		Frame.__init__(self,master)
		self.master = master

		# GUI VARIABLES 
		# traced button variables from gui
		self.experimentNumber = IntVar(self)
		self.learning = IntVar(self)

		self.participantNumber = StringVar(self)
		self.gender = StringVar(self)
		self.participantAge = StringVar(self)
		self.experience = StringVar(self)
		self.hand = StringVar(self)

		self.pathToPkg, self.pathToPkgData, self.absPaths = self.getPaths()
		self.saveDir = StringVar(self)
		self.fileName = StringVar(self, value=self.generateFileName())

		# ROS VARIABLE AND INITIALIZATIONS
		self.guiMsg = []
		self.logString	= []
		self.loggerNodeName = "experiment_log"
		init_node('simple_experiment_gui')
		self._initializeGuiCommands()
		self._guiSub = Subscriber('simple_experiment_logger',String, self._guiLoggerCB)
		self._initializePublishers()

		# intialize layout and buttons
		self.initializeWindow()


	################################################################################################################
	############################################# LAYOUT, BUTTONS AND WIDGETS ######################################
	################################################################################################################
	def initializeWindow(self):
		# Frame
		w=900
		h=700
		self.master.title("Simple experiment")
		self.master["bg"]="black"
		self.master.geometry("900x700")
		frame = Frame(self.master, bg='#ff6666').place(relx=0.02,rely=0.02,relwidth=0.96,relheight=0.96)

		col1 = 0.05
		col2 = 0.425
		col3 = 0.80
		row1 = 0.05
		row2 = 0.3
		row3 = 0.55
		row4 = 0.9
		bw = 0.15
		bh = 0.05

		######################################################## ROW 1 ########################################################
		launchStiffness = Button(self.master, text="Launch stiffness pkg", command=partial(self.launchStiffnessCB, arg='hoi'))
		launchStiffness.place(relx=col1,rely=row1,relwidth=bw,relheight=bh)
		launchExp = Button(self.master, text="Launch experiment", command=self.launchExperimentCB)
		launchExp.place(relx=col1,rely=row1+1.5*bh,relwidth=bw,relheight=bh)
		launchRviz = Button(self.master, text="Launch rviz gui", command=self.launchRvizCB)
		launchRviz.place(relx=col1,rely=row1+3*bh,relwidth=bw,relheight=bh)

		killStiffness = Button(self.master, text="Kill stiffness pkg", command=self.killStiffnessCB)
		killStiffness.place(relx=col2,rely=row1,relwidth=bw,relheight=bh)
		killExp = Button(self.master, text="Kill experiment", command=self.killExperimentCB)
		killExp.place(relx=col2,rely=row1+1.5*bh,relwidth=bw,relheight=bh)
		killRviz = Button(self.master, text="Kill rviz gui", command=self.killRvizCB)
		killRviz.place(relx=col2,rely=row1+3*bh,relwidth=bw,relheight=bh)

		self.expInfoText = Text(self.master, yscrollcommand=True)
		self.expInfoText.place(relx=col2+bw+0.05,rely=row1,relwidth=0.325,relheight=0.45)




		######################################################## ROW 2 ########################################################
		exp1 = Button(self.master, text="Start experiment", command= lambda: self.startExperimentCB(self.experimentNumber.get()))
		exp1.place(relx=col1,rely=row2,relwidth=bw,relheight=bh)
		comboBoxExpNr = ttk.Combobox(self.master,values=[1,2,3,4] ,textvariable=self.experimentNumber)
		self.experimentNumber.trace('w',self.experimentNrCB)
		comboBoxExpNr.place(relx=(col1+bw),rely=row2,relwidth=0.1,relheight=bh)


		checkBoxLearning = Checkbutton(self.master, text="practice?", variable=self.learning)
		self.learning.trace('w', self.LearningPhaseCB)
		checkBoxLearning.place(relx=(col1+bw+0.1),rely=row2,relwidth=0.1,relheight=bh)
		stopExp = Button(self.master, text="Pause experiment", command=self.pauseExperimentCB)
		stopExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)


		prevTrial = Button(self.master, text="previous trial", command= lambda: self.trialCB(-1))
		prevTrial.place(relx=col1,rely=(row2+1.5*bh),relwidth=bw,relheight=bh)
		nextTrial = Button(self.master, text="next trial", command=lambda: self.trialCB(1))
		nextTrial.place(relx=col2,rely=(row2+1.5*bh),relwidth=bw,relheight=bh)


		######################################################## ROW 3 ########################################################
		labelUsrinfo = Label(self.master, text='Participant nr')
		labelUsrinfo.place(relx=col1,rely=row3,relwidth=0.15,relheight=bh)
		labelUsrGender = Label(self.master, text='Gender')
		labelUsrGender.place(relx=col1,rely=row3+bh,relwidth=0.15,relheight=bh)
		labelUsrAge = Label(self.master, text='Age')
		labelUsrAge.place(relx=col1,rely=row3+2*bh,relwidth=0.15,relheight=bh)
		labelUsrExp = Label(self.master, text='Experience')
		labelUsrExp.place(relx=col1,rely=row3+3*bh,relwidth=0.15,relheight=bh)
		labelUsrHand = Label(self.master, text='Handedness')
		labelUsrHand.place(relx=col1,rely=row3+4*bh,relwidth=0.15,relheight=bh)

		entryPartNr = Entry(self.master, bg='white', textvariable=self.participantNumber)
		self.participantNumber.trace('w',self.partNrCB)
		entryPartNr.place(relx=(bw +col1),rely=row3,relwidth=0.1,relheight=bh)
		comboOptionGender = ttk.Combobox(self.master,values=['male','female','other'] ,textvariable=self.gender)
		self.gender.trace('w',self.genderCB)
		comboOptionGender.place(relx=(col1+bw),rely=row3+bh,relwidth=0.1,relheight=bh)
		entryPartAge = Entry(self.master, bg='white', textvariable=self.participantAge)
		self.participantAge.trace('w',self.partAgeCB)
		entryPartAge.place(relx=(bw +col1),rely=row3+2*bh,relwidth=0.1,relheight=bh)
		comboOptionExp = ttk.Combobox(self.master,values=['none','1h','10h','1d','1w','10w','more'] ,textvariable=self.experience)
		self.experience.trace('w',self.experianceCB)
		comboOptionExp.place(relx=(col1+bw),rely=row3+3*bh,relwidth=0.1,relheight=bh)
		saveParticipantButton = Button(self.master, text="save info", command=self.saveParticipantInfoCB)
		saveParticipantButton.place(relx=col1,rely=row3+5*bh,relwidth=bw,relheight=bh)

		comboOptionHand = ttk.Combobox(self.master,values=['right','left'] ,textvariable=self.hand)
		self.hand.trace('w',self.handCB)
		comboOptionHand.place(relx=(col1+bw),rely=row3+4*bh,relwidth=0.1,relheight=bh)



		self.loggerWindow = Text(self.master, yscrollcommand=True)
		self.loggerWindow.place(relx=col2,rely=row3,relwidth=0.525,relheight=0.2)

		saveExpNotesButton = Button(self.master, text="save notes/logger", command=self.saveExperimentNotesAndLoggerCB)
		saveExpNotesButton.place(relx=col2,rely=row3+0.2,relwidth=bw,relheight=bh)


		######################################################## ROW 4 ########################################################
		startLog = Button(self.master, text="Start logger", command=self.startLoggerCB)
		startLog.place(relx=col1,rely=row4,relwidth=bw,relheight=bh)

		comboOptionPath = ttk.Combobox(self.master,values=self.absPaths ,textvariable=self.saveDir, 
										postcommand=lambda: comboOptionPath.configure(values=self.absPaths))
		self.saveDir.trace('w',self.pathCB)
		comboOptionPath.place(relx=(col1+bw),rely=row4,relwidth=0.4,relheight=bh)

		entryFileName = Entry(self.master, bg='white', textvariable=self.fileName)
		self.fileName.trace('w',self.fileNameCB)
		entryFileName.place(relx=(col1+bw+0.4),rely=row4,relwidth=0.2,relheight=bh)

		stopLog = Button(self.master, text="Stop logger", command=self.stopLoggerCB)
		stopLog.place(relx=col3,rely=row4,relwidth=bw,relheight=bh)


	################################################################################################################
	############################################  ROS PUB,INIT AND SUB CB   #####################################
	################################################################################################################
	def _initializeGuiCommands(self):
		self.guiMsg = gui_command()
		self.guiMsg.header.stamp = Time.now()
		# self.guiMsg.start_stiffness = False
		self.guiMsg.start_experiment = False
		self.guiMsg.trial_change = 0

	def _initializePublishers(self):
		# self.stiffnessPub = Publisher('start_stiffness',Bool, latch=False, queue_size=1) # to stiffness code
		self.guiPub = Publisher('gui_commands',gui_command, latch=False, queue_size=1) # to experiment
		# self.loggerPub= Publisher('rosbag_log',Bool, queue_size=1)


	def _guiLoggerCB(self,message):
		self.logString = message
		self.loggerWindow.insert(END, str(self.logString) + "\n")


	################################################################################################################
	############################ GUI BUTTON CALLBACKS AND OTHER EVENT FUNCTIONS#######################
	################################################################################################################

	######################################################## ROW 1 ########################################################
	def _startRosLaunch(self, filePath):
		try:
			uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
			roslaunch.configure_logging(uuid)
			launch = roslaunch.parent.ROSLaunchParent(uuid, [filePath])
			launch.start()
			print("{} is launched".format(filePath))
		except:
			print("Er launched iets NIET wat wel de bedoeling is o.O\n{}".format(filePath))
			raise


	def launchStiffnessCB(self, arg):
		filePath = "/home/jasper/omni_marco_gazebo/src/stiffness_launch/launch/omni_simple_marco.launch"
		self._startRosLaunch(filePath)


	def launchExperimentCB(self):
		filePath = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/launch/simple_experiment.launch"
		self._startRosLaunch(filePath)

	def launchRvizCB(self):
		filePath = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/launch/rviz_experiment_gui.launch"
		self._startRosLaunch(filePath)

	def killStiffnessCB(self):
		bashFile = "kill_nodes.sh"
		nodes = "draw_ellipsoid haptic_device_rotation mock_End_Effector omni1 omni_2_marker omni_feedback_force stiffness_commanding"
		bashCommand = self.pathToPkg+bashFile+" "+nodes
		subprocess.call(bashCommand, shell=True)

	def killExperimentCB(self):
		bashFile = "kill_nodes.sh"
		nodes = "simple_experiment"
		bashCommand = self.pathToPkg+bashFile+" "+nodes
		subprocess.call(bashCommand, shell=True)

	def killRvizCB(self): # need to manually close window
		bashFile = "kill_nodes.sh"
		nodes = "rviz_experiment_gui"
		bashCommand = self.pathToPkg+bashFile+" "+nodes
		subprocess.call(bashCommand, shell=True)

	######################################################## ROW 2 ########################################################
	def startExperimentCB(self,number):
		print("Start experiment {}, practice: {}".format(number,self.learning.get()))
		self.guiMsg.start_experiment = True
		self.guiPub.publish(self.guiMsg)

	def experimentNrCB(self,*args):
		nr = self.experimentNumber.get()
		print("set experiment number to {}".format(nr))
		self.guiMsg.experiment_number = nr
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		# self.fileNameCB() # prints

	def LearningPhaseCB(self,*args):
		learning = self.learning.get()
		print("set learning to {}".format(learning))
		self.guiMsg.learning = learning
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		# self.fileNameCB() # prints

	def pauseExperimentCB(self):
		self.guiMsg.start_experiment = False
		print("Trying to pause experiment")
		print("publish: {} to experiment node".format(False))
		self.guiPub.publish(self.guiMsg)


	def trialCB(self,indecrement):
		print("Publish: {}".format(indecrement))
		self.guiMsg.trial_change = indecrement
		self.guiPub.publish(self.guiMsg)
		self.guiMsg.trial_change = 0
		# self.guiPub.publish(self.guiMsg)

	######################################################## ROW 3 ########################################################
	def partNrCB(self,*args):
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		self.saveDir.set(self.getDumpPath())
		print("update fileName with partNr {} to {}".format(self.participantNumber.get(),self.fileName.get()))
		print("update dirPath to {}".format(self.saveDir.get()))

	def genderCB(self, *args):
		print("updated gender to {}".format(self.gender.get()))
	def partAgeCB(self,*args):
		print("updated age to {}".format(self.participantAge.get()))
	def experianceCB(self, *args):
		print("updated experiance to {}".format(self.experience.get()))
	def handCB(self, *args):
		print("updated handedness to {}".format(self.hand.get()))
	def saveParticipantInfoCB(self):
		dirName = "part_"+ str(self.participantNumber.get())
		dumpPath = self.pathToPkgData+'/'+dirName
		fileName = "part_"+str(self.participantNumber.get())+"_info.yaml"
		filePath = dumpPath+'/'+fileName
		try:
			os.mkdir(dumpPath)
			print("file {} is saved in directory {}".format(fileName,dirName))

		except OSError as e:
		    if e.errno == errno.EEXIST:
		        print("directiory: {} already exists and file: {} is saved as: {}".format(dirName,fileName,dumpPath))
		    else:
		        raise

		infoDict = self.generateParticipantInfoDict()

		with open(filePath,'w') as outfile:
			yaml.dump(infoDict,outfile)
		# set and update combobox
		self.saveDir.set(dumpPath)
		self.absPaths.append(dumpPath)



	def saveExperimentNotesAndLoggerCB(self):
		dumpPath = self.saveDir.get()
		if not dumpPath:
			print("specify path to dump file")
			return
		fileName = self.fileName.get()
		pathToFile = dumpPath+'/'+fileName+'.txt'

		expText= self.expInfoText.get(1.0,END)
		logText= self.loggerWindow.get(1.0,END)
		try: 
			with open(pathToFile,'w') as outfile:
				outfile.write(expText)
				outfile.write(logText)
			print("file: {} saved in {}".format(fileName,dumpPath))
			# clear
			self.loggerWindow.delete('1.0', END)
			self.expInfoText.delete('1.0', END)
		except EnvironmentError:
			print("WHOOPS!\nOp een of andere duistere reden can er niet naar: \n{} geschreven worden".format(pathToFile))

	######################################################## ROW 4 ######################################################## 
	def startLoggerCB(self, pik='pik'):
		if len(self.saveDir.get()) == 0:
			print("no directory specified!!! \nspecify directory and try again")
			return
		if not os.path.isdir(self.saveDir.get()):
			print("{} is not a existing directory.\nSpecify correct path".format(self.saveDir.get()))
			return
		bashFile = "rosbag_record.sh"
		# fix this part to part{nr}_Exp{nr}_{type}.bag
		# bagName = "my_test_bag"
		self.fileName.set(self.generateFileName()) # set variable
		bagName = self.fileName.get()
		# fix the topics that are recorded
		topics = "simple_experiment_logger omni1_lock_state omni1_button omni1_force_feedback omni1_joint_states omni_stiffness eigen_pair stiffness_command marker_visualization experiment_ellipsoid draw_ellipsoid/ellipsoid_visualization draw_ellipsoid/arrow_visualization experiment_ellipsoid gui_commands orientation_accuracy_text shape_accuracy_text tf tf_static simple_experiment_data virtual_robot_force"
		# ./rosbag_record 	saveDir bagName logNodeName topic1 	topic2 	etc
		# $0 				$1		$2 		$3 			$3		$3 		etc
		# check if self.saveDir and self.fileName are empty
		bashCommand = self.pathToPkg+bashFile+" "+self.saveDir.get()+" "+bagName+" "+self.loggerNodeName+" "+topics
		subprocess.call(bashCommand, shell=True)
		print("Logger started")

	def pathCB(self, *args):
		print("updated save Dir to {}".format(self.saveDir.get()))
	def fileNameCB(self, *args):
		print("updated file name to {}".format(self.fileName.get()))


	def stopLoggerCB(self):
		bashFile = "kill_nodes.sh"
		nodes = self.loggerNodeName
		bashCommand = self.pathToPkg+bashFile+" "+nodes
		subprocess.call(bashCommand, shell=True)



	########################################################
	# OTHER HELPER FUNCTIONS
	########################################################
	def getDumpPath(self):
		dirName = dirName = "part_"+ str(self.participantNumber.get())
		dirPath = self.pathToPkgData+'/'+dirName
		return dirPath

	def generateFileName(self):
		trial = "Real"
		if self.learning.get() == True:
			trial = "Learn"
		else:
			trial = "Real"
		t = strftime("%m%d%H%M") # month day hour minutes
		nameString= ("Part"+str(self.participantNumber.get())+"_Exp"+
			str(self.experimentNumber.get())+"_"+trial+"_"+str(t))
		return nameString

	def getPaths(self):
		completePath = os.path.abspath(__file__) #File name
		delimiter = 'stiffness_simple_experiment' #Package name
		pathToPkg = completePath.split(delimiter)[0] + delimiter + '/'
		pathToPkgData = pathToPkg +'data'
		dirsInPkgData = [Dir[0] for Dir in os.walk(pathToPkgData)]
		return pathToPkg, pathToPkgData, dirsInPkgData

	def generateParticipantInfoDict(self):
		participantInfo = {'number': self.participantNumber.get(),
							'gender': self.gender.get(),
							'age': self.participantAge.get(),
							'experience': self.experience.get(),
							'hand': self.hand.get()}
		return participantInfo



########################################################
# MAINLOOP
########################################################


if __name__ == "__main__":
	root = Tk()

	gui = GuiWindow(root)

	root.mainloop()