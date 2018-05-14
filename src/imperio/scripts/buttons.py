#!/usr/bin/env python

""" Makes the buttons for starting and ending autonomy.
(The big red button)

Author: James Madison University
Date: 5/11/2018
Version: 1

"""

import Tkinter
from wave import open as waveOpen
from ossaudiodev import open as ossOpen
import rospy
import os
from std_msgs.msg import Bool

class imperio_button(object):
    def __init__(self):
        rospy.init_node("imperio_buttons")

        self.halt_publisher = rospy.Publisher('/halt_autonomy', Bool, queue_size = 1)

        button_height = 7
        button_width = 35
        border = 5

        self.window = Tkinter.Tk()
        self.window.title("Imperio : Autonomy")

        logo_path = "../include/imperio_logo_2.gif"
        logo_full_path = os.path.abspath(logo_path)
        logo_image = Tkinter.PhotoImage(file=logo_full_path)


        self.logo = Tkinter.Label(self.window, image=logo_image)
        self.logo.pack()

        self.start_imperio_button = Tkinter.Button(self.window, text="Start Imperio", command=self.start_imperio_callback)
        self.start_imperio_button.configure(bg="#009900", height=button_height, width=button_width, bd=border)

        self.halt_button = Tkinter.Button(self.window, text="Halt Imperio", command=self.halt_button_callback)
        self.halt_button.configure(bg="#cc0000", height=button_height, width=button_width, bd=border)

        self.pack_elements()

        Tkinter.mainloop()

    def pack_elements(self):
        self.logo.pack(side=Tkinter.TOP)
        self.start_imperio_button.pack(side=Tkinter.LEFT)
        self.halt_button.pack(side=Tkinter.LEFT)


    def halt_button_callback(self):
        rospy.logwarn("[IMPERIO BUTTON] : Halting Imperio")

        os.system("play ../include/halt.wav &")

        msg = Bool()
        msg.data = True
        self.halt_publisher.publish(msg)

    def start_imperio_callback(self):
        rospy.logwarn("[IMPERIO BUTTON] : Starting Imperio")
        os.system("roslaunch imperio sim_testing_imperio.launch &")

    def run(self):
        rospy.loginfo("[IMPERIO_BUTTON] : Launching the imperio buttons")
        self.window.mainloop()

if __name__ == "__main__":
    button = imperio_button()