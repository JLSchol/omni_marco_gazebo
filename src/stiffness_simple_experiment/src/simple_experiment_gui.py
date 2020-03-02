#!/usr/bin/env python

from rospy import Subscriber, Publisher, init_node, Time
import os
import errno
import subprocess
import yaml
import roslaunch
from time import strftime
import unicodedata


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

		# traced button variables from gui
		self.participantNumber = StringVar(self)
		self.gender = StringVar()
		self.participantAge = StringVar(self)
		self.experience = StringVar(self)
		self.experimentInfo = StringVar(self)
		self.experimentNumber = IntVar(self)
		self.learning = IntVar(self)

		# other variables from gui
		self.pathToPkg, self.pathToPkgData, self.absPaths = self.getPaths()
		self.saveDir = StringVar()
		self.fileName = StringVar(self, value=self.generateFileName())
		# self.savePath = StringVar(self, value=pathToPkg)

		# intialize esthetics and button
		self.initializeWindow()

        # Initialize ROS 
        # ros variables
		self.guiMsg = []
		self.logString	= []
		self.loggerNodeName = "experiment_log"
		init_node('simple_experiment_gui')
		self._initializeGuiCommands()
		self._guiSub = Subscriber('simple_experiment_logger',String, self._guiLoggerCB)
		self._initializePublishers()


	def _initializeGuiCommands(self):
		self.guiMsg = gui_command()
		self.guiMsg.header.stamp = Time.now()
		# self.guiMsg.start_stiffness = False
		self.guiMsg.start_experiment = False
		self.guiMsg.trial_change = 0

	def _initializePublishers(self):
		self.stiffnessPub = Publisher('start_stiffness',Bool, latch=False, queue_size=1) # to stiffness code
		self.guiPub = Publisher('gui_commands',gui_command, latch=False, queue_size=1) # to experiment
		self.loggerPub= Publisher('rosbag_log',Bool, queue_size=1)
		# self.guiMsg.experiment_number = False
		# self.guiMsg.learning = False
		# self.guiMsg.trial =  +1 or -1

	def _guiLoggerCB(self,message):
		self.logString = message
		self.loggerWindow.insert(END, str(self.logString) + "\n")

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

		######## ROW 1 ########
		launchStiffness = Button(self.master, text="Launch stiffness pkg", command=partial(self.launchStiffnessCB, arg='hoi'))
		launchStiffness.place(relx=col1,rely=row1,relwidth=bw,relheight=bh)
		killStiffness = Button(self.master, text="Kill stiffness pkg", command=self.killStiffnessCB)
		killStiffness.place(relx=col2,rely=row1,relwidth=bw,relheight=bh)
		launchExp = Button(self.master, text="Launch experiment", command=self.launchExperimentCB)
		launchExp.place(relx=col1,rely=row1+2*bh,relwidth=bw,relheight=bh)


		killExp = Button(self.master, text="Kill experiment", command=self.killExperimentCB)
		killExp.place(relx=col2,rely=row1+2*bh,relwidth=bw,relheight=bh)

		self.loggerWindow = Text(self.master, yscrollcommand=True)
		self.loggerWindow.place(relx=col2+bw+0.05,rely=row1,relwidth=0.325,relheight=0.45)

		######## ROW 2 ########
		exp1 = Button(self.master, text="Start experiment", command= lambda: self.startExperimentCB(1))
		exp1.place(relx=col1,rely=row2,relwidth=bw,relheight=bh)
		comboBoxExpNr = ttk.Combobox(self.master,values=[1,2,3,4,5] ,textvariable=self.experimentNumber)
		self.experimentNumber.trace('w',self.saveExperimentNrCB)
		comboBoxExpNr.place(relx=(col1+bw),rely=row2,relwidth=0.1,relheight=bh)

		# exp2 = Button(self.master, text="Start experiment 2", command=lambda: self.startExperimentCB(2))
		# exp2.place(relx=col1,rely=(row2+0.1),relwidth=bw,relheight=bh)
		# exp3 = Button(self.master, text="Start experiment 3", command=lambda: self.startExperimentCB(3))
		# exp3.place(relx=col1,rely=(row2+0.2),relwidth=bw,relheight=bh)
		# pauseExp = Button(self.master, text="Pauze experiment", command=self.pauseExperimentCB)
		# pauseExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)
		# stopExp = Button(self.master, text="Stop experiment", command=self.stopExperimentCB)
		# stopExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)

		checkBoxLearning = Checkbutton(self.master, text="practice?", variable=self.learning)
		self.learning.trace('w', self.LearningPhaseCB)
		checkBoxLearning.place(relx=(col1+bw+0.1),rely=row2,relwidth=0.1,relheight=bh)
		stopExp = Button(self.master, text="Stop experiment", command=self.stopExperimentCB)
		stopExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)


		prevTrial = Button(self.master, text="previous trial", command= lambda: self.trialCB(-1))
		prevTrial.place(relx=col1,rely=(row2+2*bh),relwidth=bw,relheight=bh)
		nextTrial = Button(self.master, text="next trial", command=lambda: self.trialCB(1))
		nextTrial.place(relx=col2,rely=(row2+2*bh),relwidth=bw,relheight=bh)


		######## ROW 3 ########
		labelUsrinfo = Label(self.master, text='Participant nr')
		labelUsrinfo.place(relx=col1,rely=row3,relwidth=0.15,relheight=bh)
		labelUsrGender = Label(self.master, text='Gender')
		labelUsrGender.place(relx=col1,rely=row3+bh,relwidth=0.15,relheight=bh)
		labelUsrAge = Label(self.master, text='Age')
		labelUsrAge.place(relx=col1,rely=row3+2*bh,relwidth=0.15,relheight=bh)
		labelUsrExp = Label(self.master, text='Experience')
		labelUsrExp.place(relx=col1,rely=row3+3*bh,relwidth=0.15,relheight=bh)

		entryPartNr = Entry(self.master, bg='white', textvariable=self.participantNumber)
		self.participantNumber.trace('w',self.partNrCB)
		entryPartNr.place(relx=(bw +col1),rely=row3,relwidth=0.1,relheight=bh)
		comboOptionGender = ttk.Combobox(self.master,values=['male','female','other'] ,textvariable=self.gender)
		self.gender.trace('w',self.saveGenderCB)
		comboOptionGender.place(relx=(col1+bw),rely=row3+bh,relwidth=0.1,relheight=bh)
		entryPartAge = Entry(self.master, bg='white', textvariable=self.participantAge)
		self.participantAge.trace('w',self.partAgeCB)
		entryPartAge.place(relx=(bw +col1),rely=row3+2*bh,relwidth=0.1,relheight=bh)
		comboOptionExp = ttk.Combobox(self.master,values=['none','some','experienced'] ,textvariable=self.experience)
		self.experience.trace('w',self.saveExperianceCB)
		comboOptionExp.place(relx=(col1+bw),rely=row3+3*bh,relwidth=0.1,relheight=bh)
		saveParticipantButton = Button(self.master, text="save info", command=self.saveParticipantInfoCB)
		saveParticipantButton.place(relx=col1,rely=row3+4*bh,relwidth=bw,relheight=bh)


		self.expInfoText = Text(self.master, yscrollcommand=True)
		self.expInfoText.place(relx=col2,rely=row3,relwidth=0.525,relheight=0.2)
		saveExpNotesButton = Button(self.master, text="save notes/logger", command=self.saveExperimentNotesCB)
		saveExpNotesButton.place(relx=col2,rely=row3+0.2,relwidth=bw,relheight=bh)


		######## ROW 4 ########
		startLog = Button(self.master, text="Start logger", command=self.startLoggerCB)
		startLog.place(relx=col1,rely=row4,relwidth=bw,relheight=bh)

		comboOptionPath = ttk.Combobox(self.master,values=self.absPaths ,textvariable=self.saveDir, 
										postcommand=lambda: comboOptionPath.configure(values=self.absPaths))
		self.saveDir.trace('w',self.savePathCB)
		comboOptionPath.place(relx=(col1+bw),rely=row4,relwidth=0.4,relheight=bh)

		entryFileName = Entry(self.master, bg='white', textvariable=self.fileName)
		self.fileName.trace('w',self.saveFileNameCB)
		entryFileName.place(relx=(col1+bw+0.4),rely=row4,relwidth=0.2,relheight=bh)

		stopLog = Button(self.master, text="Stop logger", command=self.stopLoggerCB)
		stopLog.place(relx=col3,rely=row4,relwidth=bw,relheight=bh)



	def launchStiffnessCB(self, arg):
		print("roslaunch stiffness_launch omni_simple_marco.launch")
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)

		file_path = "/home/jasper/omni_marco_gazebo/src/stiffness_launch/launch/omni_simple_marco.launch"
		launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
		launch.start()

	def launchExperimentCB(self):
		print("roslaunch stiffness_launch omni_simple_marco.launch")
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		
		file_path = "/home/jasper/omni_marco_gazebo/src/stiffness_simple_experiment/launch/simple_experiment.launch"
		launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
		launch.start()

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

	def startExperimentCB(self,number):
		print("Start experiment {}".format(number))
		self.guiMsg.start_experiment = True
		self.guiPub.publish(self.guiMsg)

	def saveExperimentNrCB(self,*args):
		print(self.experimentNumber.get())
		self.guiMsg.experiment_number = self.experimentNumber.get()
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		# self.saveFileNameCB() # prints

	def LearningPhaseCB(self,*args):
		print(self.learning.get())
		self.guiMsg.learning = self.learning.get()
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		# self.saveFileNameCB() # prints

	def stopExperimentCB(self):
		print("STOPPPPP")
		self.guiMsg.start_experiment = False
		self.guiPub.publish(self.guiMsg)


	def trialCB(self,indecrement):
		print("Publish: {}".format(indecrement))
		self.guiMsg.trial_change = indecrement
		self.guiPub.publish(self.guiMsg)
		self.guiMsg.trial_change = 0
		# self.guiPub.publish(self.guiMsg)


	def partNrCB(self,*args):
		print(self.participantNumber.get())
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable


	def partAgeCB(self,*args):
		print(self.participantAge.get())


	def startLoggerCB(self, pik='pik'):
		if len(self.saveDir.get()) == 0:
			print("no directory specified!!! \nspecify directory and try again")
			return
		if not os.path.isdir(self.saveDir.get()):
			print("{} is not a existing directory.\nSpecify correct path".format(self.saveDir.get()))
			return
		bashFile = "rosbag_record.sh"
		bagName = "my_test_bag"
		topics = "simple_experiment_logger hoi"
		# ./rosbag_record 	saveDir bagName logNodeName topic1 	topic2 	etc
		# $0 				$1		$2 		$3 			$3		$3 		etc
		bashCommand = self.pathToPkg+bashFile+" "+self.saveDir.get()+" "+bagName+" "+self.loggerNodeName+" "+topics
		subprocess.call(bashCommand, shell=True)
		print("Logger started")


	def stopLoggerCB(self):
		bashFile = "kill_nodes.sh"
		nodes = self.loggerNodeName
		bashCommand = self.pathToPkg+bashFile+" "+nodes
		subprocess.call(bashCommand, shell=True)

	def saveRosbagsCB(self,name,location):
		pass

	def savePathCB(self, *args):
		print(self.saveDir.get())
	def saveGenderCB(self, *args):
		print(self.gender.get())
	def saveExperianceCB(self, *args):
		print(self.experience.get())
	def saveFileNameCB(self, *args):
		print(self.fileName.get())


	def saveParticipantInfoCB(self):
		dirName = "part_"+ str(self.participantNumber.get())
		dumpPath = self.pathToPkgData+'/'+dirName
		fileName = "part_"+str(self.participantNumber.get())+"_info.yaml"
		filePath = dumpPath+'/'+fileName
		try:
			os.mkdir(dumpPath)
			print("directiory: {} is created".format(dirName))
		except OSError as e:
		    if e.errno == errno.EEXIST:
		        print("directiory: {} already exists and file: {} is saved in: {}".format(dirName,fileName,dumpPath))
		    else:
		        raise

		infoDict = self.generateParticipantInfoDict()

		with open(filePath,'w') as outfile:
			yaml.dump(infoDict,outfile)
		# set and update combobox
		self.saveDir.set(dumpPath)
		self.absPaths.append(dumpPath)

	def saveExperimentNotesCB(self):
		dumpPath = self.saveDir.get()
		if not dumpPath:
			print("specify path to dump file")
			return
		fileName = self.fileName.get()

		expText= self.expInfoText.get(1.0,END)
		logText= self.loggerWindow.get(1.0,END)
		with open(dumpPath+'/'+fileName+'.txt','w') as outfile:
			outfile.write(expText)
			outfile.write(logText)
		print("file: {} saved in {}".format(fileName,dumpPath))



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
							'experience': self.experience.get()}
		return participantInfo






if __name__ == "__main__":
	root = Tk()


	gui = GuiWindow(root)

	root.mainloop()