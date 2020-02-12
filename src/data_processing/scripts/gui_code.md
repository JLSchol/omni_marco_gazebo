GUI CODE

```python
#!/usr/bin/env python3

import rospy
import os
import roslaunch


from tkinter import *
from tkinter import ttk
from tkinter import simpledialog
from tkinter.font import Font

from std_msgs.msg import Bool, String, Int64

class GuiWindow(Frame):

    def __init__(self,master=None):
        Frame.__init__(self, master)
        self.master = master
        self.init_window()

        # Initialize ROS publishers
        self.teleop_pub = rospy.Publisher('/teleop',Bool, latch=True, queue_size=1)
        self.coffee_pub = rospy.Publisher('/i_want_coffee',Bool, latch=True, queue_size=1)
        self.coffee_ready_pub = rospy.Publisher('/coffee_teleop',Bool, latch=False, queue_size=1)
        self.bed_pub = rospy.Publisher('/i_found_a_bed',Bool, latch=True, queue_size=1)
        self.bedteleop_pub = rospy.Publisher('/bed_teleop',Bool, latch=False, queue_size=1)

        self.label = Label(self, bg='forest green',fg="black", width=30, height=5, relief=SUNKEN, font= Font(family="Ubuntu Mono", size=20))

        self.domain_pub = rospy.Publisher('/domain',String, latch=True, queue_size=1)
        self.port_pub = rospy.Publisher('/port_number',Int64, latch=True, queue_size=1)

        self.parent =' '


    def init_window(self):
        rospy.init_node('state_machine_gui')
        self.master.title("STATE MACHINE GUI")
        self.pack(fill=BOTH, expand=True)



        frame = Frame(self, borderwidth=10)
        frame.pack()

        myfont = Font(family="Ubuntu Mono", size=16)

        teleop_button = Button(self, borderwidth=5,  text="Operator control", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Operator_button)
        teleop_button.place(x=10,y=10)

        auto_button = Button(self, borderwidth=5,text="Autonomous mode", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Autonomous_button)
        auto_button.place(x=350,y=10)

        coffee_button = Button(self,borderwidth=5, text="I want coffee", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Coffee_button)
        coffee_button.place(x=10, y=80)

        coffee_ready_button = Button(self, borderwidth=5,text="Coffee is ready", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Coffee_Ready_button)
        coffee_ready_button.place(x=350, y=80)

        bed_railing_button = Button(self, borderwidth=5,text="I found a bed", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Bed_Found_button)
        bed_railing_button.place(x=10, y=150)

        bed_processed_button = Button(self, borderwidth=5,text="Bed processed", activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.Bed_Processed_button)
        bed_processed_button.place(x=350, y=150)

        connectToServer_button = Button(self, borderwidth=5, text="Connect to cloud", activebackground='DarkOrange1', font=myfont, cursor='hand2', command = self.connectToCloud)
        connectToServer_button.place(x=10, y =220)

        Visualize_States_button = Button(self, borderwidth=5, text="State Machine Flowchart", bg=['turquoise1'], activebackground='DarkOrange1', font=myfont, cursor='hand2', command = self.VisualizeStates)
        Visualize_States_button.place(x=350, y =220)

        Cockpit_button = Button(self, borderwidth=5, text="Launch cockpit", bg=['maroon1'], activebackground='DarkOrange1', font=myfont, cursor='hand2', command = self.Cockpit_launch_button)
        Cockpit_button.place(x=10, y =550)

        Kill_cockpit_button = Button(self, borderwidth=5, text="Kill cockpit", bg=['firebrick1'], activebackground='DarkOrange1', font=myfont, cursor='hand2', command = self.Cockpit_kill_button)
        Kill_cockpit_button.place(x=350, y =550)

        start_button = Button(self, borderwidth=5,  text="Start", bg=['Green'], activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.start_gui)
        start_button.place(x=10,y=650)

        quit_button = Button(self, borderwidth=5,text="Quit GUI",bg=['red'],activebackground='DarkOrange1', font=myfont, cursor='hand2', command=self.close_window)
        quit_button.place(x=750,y=650)



    def start_gui(self):
        self.publish_on_boolean_topic(self.teleop_pub, False)
        self.publish_on_boolean_topic(self.coffee_pub,False)
        self.publish_on_boolean_topic(self.bed_pub,False)
        # self.publish_on_boolean_topic(self.bed_pub,False)


    def publish_on_boolean_topic(self, publisher, topic_value):
        topic = Bool()
        topic.data = topic_value
        publisher.publish(topic)

    def update_output_text(self,label,text, color):
        label.config(text=text, bg=color)
        label.pack(side=LEFT, padx=10,pady=300)

    def close_window(self):
        exit()

    def Operator_button(self):
        self.publish_on_boolean_topic(self.teleop_pub, True)
        self.update_output_text(self.label,"OPERATOR MODE",'orange')

    def Autonomous_button(self):
        self.publish_on_boolean_topic(self.teleop_pub, False)
        self.update_output_text(self.label,"AUTONOMOUS MODE",'green')

    def Coffee_button(self):
        self.publish_on_boolean_topic(self.coffee_pub,True)
        self.update_output_text(self.label,"GETTING COFFEE",'purple')

    def Coffee_Ready_button(self):
        self.publish_on_boolean_topic(self.coffee_pub,False)
        self.publish_on_boolean_topic(self.coffee_ready_pub,False)
        self.update_output_text(self.label,"AUTONOMOUS MODE",'green')

    def Bed_Found_button(self):
        self.publish_on_boolean_topic(self.bed_pub,True)
        self.publish_on_boolean_topic(self.bedteleop_pub,True)
        self.update_output_text(self.label,"BED DETECTED",'DarkOrange')

    def Bed_Processed_button(self):
        self.publish_on_boolean_topic(self.bed_pub,False)
        self.publish_on_boolean_topic(self.bedteleop_pub,False)
        self.update_output_text(self.label,"AUTONOMOUS MODE",'green')

    def Cockpit_launch_button(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        # os.system("roslaunch marco_cockpit_launcher cockpit.launch")
        cockpit_args = ['marco_cockpit_launcher','cockpit.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cockpit_args)
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        self.parent.start()

    def Cockpit_kill_button(self):
        self.parent.shutdown()

    def connectToCloud(self):
        Domain_msg = String()
        Port_msgs = Int64()
        domain = simpledialog.askstring("Server Connection", "Enter domain:")
        port = simpledialog.askinteger("Server Connection","Enter port:")
        Domain_msg.data = domain
        Port_msgs.data=port
        self.domain_pub.publish(Domain_msg)
        self.port_pub.publish(Port_msgs)

    def VisualizeStates(self):
        os.system("rosrun smach_viewer smach_viewer.py &")

master = Tk()
master.geometry("900x700")
master["bg"]="black"
gui = GuiWindow(master)
master.mainloop()
```
