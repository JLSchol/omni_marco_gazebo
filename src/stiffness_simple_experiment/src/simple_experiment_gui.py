#!/usr/bin/env python

import rospy
import os
import roslaunch


from Tkinter import * 
from functools import partial


class GuiWindow(Frame):

	def __init__(self,master=None):
		Frame.__init__(self,master)
		self.master = master
		self.participantNumber = StringVar()
		self.male = IntVar()
		self.female = IntVar()
		self.other = IntVar()


        # self.label = Label(master, text="This is our first GUI!")
        # self.label.pack()

		self.initializeWindow()




	def initializeWindow(self):
		# Frame
		w=900
		h=700
		self.master.title("Simple experiment")
		self.master["bg"]="black"
		self.master.geometry("900x700")
		# canvas = Canvas(self.master, height=h, width=w, bg='#1a1a1a').pack()
		frame = Frame(self.master, bg='#ff6666').place(relx=0.02,rely=0.02,relwidth=0.96,relheight=0.96)
		# label = Label(self.master,text='hoi')


		# self.pack(fill=BOTH, expand=True)

		# buttons
		# arg = "start"
		# startStiffness = partial(self.startStiffnessCB, arg)

		col1 = 0.05
		col2 = 0.425
		col3 = 0.80

		row1 = 0.05
		row2 = 0.20
		row3 = 0.55
		row4 = 0.9

		bw = 0.15
		bh = 0.05

		# fw = 0.9
		# fh = 0.3

		# frame2 = Frame(frame, bg='green').place(relx=col1,rely=row3,relwidth=0.9,relheight=0.3)

		startStiffness = Button(self.master, text="Start stiffness pkg", command=partial(self.startStiffnessCB, arg='hoi'))
		startStiffness.place(relx=col1,rely=row1,relwidth=bw,relheight=bh)

		stopStiffness = Button(self.master, text="Stop stiffness pkg", command=self.stopStiffnessCB)
		stopStiffness.place(relx=col3,rely=row1,relwidth=bw,relheight=bh)

		exp1 = Button(self.master, text="Start experiment 1", command= lambda: self.startExperimentCB(1))
		exp1.place(relx=col1,rely=row2,relwidth=bw,relheight=bh)
		exp2 = Button(self.master, text="Start experiment 2", command=lambda: self.startExperimentCB(2))
		exp2.place(relx=col1,rely=(row2+0.1),relwidth=bw,relheight=bh)
		exp3 = Button(self.master, text="Start experiment 3", command=lambda: self.startExperimentCB(3))
		exp3.place(relx=col1,rely=(row2+0.2),relwidth=bw,relheight=bh)
		pauseExp = Button(self.master, text="Pauze experiment", command=self.pauseExperimentCB)
		pauseExp.place(relx=col2,rely=row2,relwidth=bw,relheight=bh)
		stopExp = Button(self.master, text="Stop experiment", command=self.stopExperimentCB)
		stopExp.place(relx=col3,rely=row2,relwidth=bw,relheight=bh)


		labelUsrinfo = Label(self.master, text='Participant nr')
		labelUsrinfo.place(relx=col1,rely=row3,relwidth=0.15,relheight=bh)

		entryParticipant = Entry(self.master, bg='white', textvariable=self.participantNumber)
		self.participantNumber.trace('w',self.partChangeCB)
		entryParticipant.place(relx=(0.15 +col1),rely=row3,relwidth=0.4,relheight=bh)


		checkBoxMale = Checkbutton(self.master, text="male", variable=self.male)
		self.male.trace('w', self.maleCB)
		checkBoxMale.place(relx=(0.15+0.4+col1),rely=row3,relwidth=0.1,relheight=bh)
		checkBoxFemale = Checkbutton(self.master, text="female", variable=self.female)
		self.female.trace('w', self.femaleCB)
		checkBoxFemale.place(relx=(0.15+0.4+0.1+col1),rely=row3,relwidth=0.1,relheight=bh)
		checkBoxOther = Checkbutton(self.master, text="other", variable=self.other)
		self.other.trace('w', self.otherCB)
		checkBoxOther.place(relx=(0.15+0.4+0.1+0.1+col1),rely=row3,relwidth=0.1,relheight=bh)



		startLog = Button(self.master, text="Start logger", command=self.startLoggerCB)
		startLog.place(relx=col1,rely=row4,relwidth=bw,relheight=bh)
		stopLog = Button(self.master, text="Stop logger", command=self.stopLoggerCB)
		stopLog.place(relx=col3,rely=row4,relwidth=bw,relheight=bh)

	def startStiffnessCB(self, arg):
		print("roslaunch stiffness_launch omni_simple_marco.launch")
		print(arg)	
	def stopStiffnessCB(self):
		print("Control c gweoon")	

	def startExperimentCB(self,number):
		print("Start experiment {}".format(number))
	def pauseExperimentCB(self):
		print("Rustaaaghh")
	def stopExperimentCB(self):
		print("STOPPPPP")

	def partChangeCB(self,*args):
		print(self.participantNumber.get())
	def maleCB(self,*args):
		print(self.male.get())
	def femaleCB(self,*args):
		print(self.female.get())
	def otherCB(self,*args):
		print(self.other.get())

	def startLoggerCB(self, pik='pik'):
		print("StartLog")
	def stopLoggerCB(self):
		print("StartLog")

	def saveRosbagsCB(self,name,location):
		pass



	def saveParticipantInfo(self):
		# save All the info in suitable text,yaml,t=dict whatever format
		pass




if __name__ == "__main__":
	root = Tk()


	gui = GuiWindow(root)

	root.mainloop()