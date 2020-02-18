#!/usr/bin/env python

from rospy import Subscriber, Publisher, init_node, Time
import os
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
		pathToPkg, self.absPaths = self.getPaths()
		self.dirs = StringVar()
		self.fileName = StringVar(self, value=self.generateFileName())
		# self.savePath = StringVar(self, value=pathToPkg)

		# intialize esthetics and button
		self.initializeWindow()

        # Initialize ROS 
        # ros variables
		self.guiMsg = []
		self.logString	= []
		init_node('simple_experiment_gui')
		self._initializeGuiCommands()
		self._guiSub = Subscriber('simple_experiment_logger',String, self._guiLoggerCB)
		self._initializePublishers()


	def _initializeGuiCommands(self):
		self.guiMsg = gui_command()
		self.guiMsg.header.stamp = Time.now()
		# self.guiMsg.start_stiffness = False
		self.guiMsg.start_experiment = False

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
		row2 = 0.20
		row3 = 0.55
		row4 = 0.9
		bw = 0.15
		bh = 0.05

		######## ROW 1 ########
		startStiffness = Button(self.master, text="Start stiffness pkg", command=partial(self.startStiffnessCB, arg='hoi'))
		startStiffness.place(relx=col1,rely=row1,relwidth=bw,relheight=bh)
		stopStiffness = Button(self.master, text="Stop stiffness pkg", command=self.stopStiffnessCB)
		stopStiffness.place(relx=col2,rely=row1,relwidth=bw,relheight=bh)

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
		stopExp = Button(self.master, text="Stop experiment", command=self.stopExperimentCB)
		stopExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)

		checkBoxLearning = Checkbutton(self.master, text="practice?", variable=self.learning)
		self.learning.trace('w', self.LearningPhaseCB)
		checkBoxLearning.place(relx=(col1+bw+0.1),rely=row2,relwidth=0.1,relheight=bh)

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

		self.expInfoText = Text(self.master, yscrollcommand=True)
		self.expInfoText.place(relx=col2,rely=row3,relwidth=0.525,relheight=0.2)

		######## ROW 4 ########
		startLog = Button(self.master, text="Start logger", command=self.startLoggerCB)
		startLog.place(relx=col1,rely=row4,relwidth=bw,relheight=bh)

		comboOptionPath = ttk.Combobox(self.master,values=self.absPaths ,textvariable=self.dirs)
		self.dirs.trace('w',self.savePathCB)
		comboOptionPath.place(relx=(col1+bw),rely=row4,relwidth=0.4,relheight=bh)

		entryFileName = Entry(self.master, bg='white', textvariable=self.fileName)
		self.fileName.trace('w',self.saveFileNameCB)
		entryFileName.place(relx=(col1+bw+0.4),rely=row4,relwidth=0.2,relheight=bh)

		stopLog = Button(self.master, text="Stop and save", command=self.stopLoggerCB)
		stopLog.place(relx=col3,rely=row4,relwidth=bw,relheight=bh)




	def startStiffnessCB(self, arg):
		print("roslaunch stiffness_launch omni_simple_marco.launch")
		# print(arg)	

	def stopStiffnessCB(self):
		print("Control c gweoon")	

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


	def trialCB(self,number):
		print("Publish: {}".format(number))
		self.guiMsg.trial = number
		self.guiPub.publish(self.guiMsg)
		print(self.guiMsg.trial)

	def partNrCB(self,*args):
		print(self.participantNumber.get())
		#update filename path
		self.fileName.set(self.generateFileName()) # set variable
		# self.saveFileNameCB() # prints 

	def partAgeCB(self,*args):
		print(self.participantAge.get())


	def startLoggerCB(self, pik='pik'):
		print("StartLog")
	def stopLoggerCB(self):
		expText= self.expInfoText.get(1.0,END)#.encode('ascii','ignore')
		print(expText)
		print("STOPPP!!1")

	def saveRosbagsCB(self,name,location):
		pass

	def savePathCB(self, *args):
		print(self.dirs.get())
	def saveGenderCB(self, *args):
		print(self.gender.get())
	def saveExperianceCB(self, *args):
		print(self.experience.get())
	def saveFileNameCB(self, *args):
		print(self.fileName.get())


	def saveParticipantInfo(self):
		# save All the info in suitable text,yaml,t=dict whatever format
		pass

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
		completePath = os.path.abspath('simple_experiment_gui.py') #File name
		delimiter = 'stiffness_simple_experiment' #Package name
		pathToPkg = completePath.split(delimiter)[0] + delimiter + '/'
		dirsInPkg = [Dir[0] for Dir in os.walk(pathToPkg)]
		return pathToPkg, dirsInPkg





if __name__ == "__main__":
	root = Tk()


	gui = GuiWindow(root)

	root.mainloop()