#!/usr/bin/env python
# -*- coding: utf-8 -*-

# general
import Tkinter as tk

# my class
from bhand_node import Finger

# ros
import rospy
import serial_bhand.srv as bhs
import serial_bhand.msg as bhm

# ****** ACTION BUTTON ******

class Abuttons(tk.Button):#{{{
    """
    button introdusing class
    @param master [Tkinter frame object]: master 
    @param text [str]: button name
    @param order [serial_bhand.srv.ActionRequest]: hand service order
    """
    def __init__(self, master, text, order):
        tk.Button.__init__(self, master, text=text, command=self.bhand_action)
        self.order = order
    
    def bhand_action(self):
        """ hand service callbask method """
        rospy.wait_for_service("bhand_node/action")
        req = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        req(self.order)
    #}}}

class ActionButtons(tk.LabelFrame):#{{{
    def __init__(self, master=None):
        tk.LabelFrame.__init__(self, master=master)
        self.configure(text="Actions")
        self.btns = [
                ("Initialize", bhs.ActionRequest.INITIALIZE),
                ("Open Hand", bhs.ActionRequest.OPEN),
                ("Close Hand", bhs.ActionRequest.CLOSE),
                ("Torque Open Hand", bhs.ActionRequest.T_OPEN),
                ("Torque Close Hand", bhs.ActionRequest.T_CLOSE),
                ("Spread Open", bhs.ActionRequest.S_OPEN),
                ("Spread Close", bhs.ActionRequest.S_CLOSE),
                ]

        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        for b in self.btns:
            b = Abuttons(self, text=b[0], order=b[1])
            b.pack(fill=tk.BOTH)
    #}}}

# ****** BHAND STATE DISCRIPTER ******

class FingerSpeedDiscripter(tk.LabelFrame):#{{{
    def __init__(self, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Finger Speed")
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.sub = rospy.Subscriber("bhand_node/FingerVel", bhm.FingerInfo, self.update)

        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        for i in range(1, 5):
            label = tk.Label(self, text="F{}: ".format(i))
            label.grid(row=i-1, column=0, sticky="w")
        for i, b in enumerate(self.buffs):
            label = tk.Label(self, textvariable=b)
            label.grid(row=i, column=1, sticky="e")

    def update(self, finfo):
        """docstring for update"""
        self.buffs[0].set("{}".format(finfo.f1))
        self.buffs[1].set("{}".format(finfo.f2))
        self.buffs[2].set("{}".format(finfo.f3))
        self.buffs[3].set("{}".format(finfo.f4))
#}}}

class FingerPositionDiscripter(tk.LabelFrame):#{{{
    def __init__(self, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Finger Position")

        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.sub = rospy.Subscriber("bhand_node/FingerPos", bhm.FingerInfo, self.update)

        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        for i in range(1, 5):
            label = tk.Label(self, text="F{}: ".format(i))
            label.grid(row=i-1, column=0, sticky="w")
        for i, b in enumerate(self.buffs):
            label = tk.Label(self, textvariable=b)
            label.grid(row=i, column=1, sticky="e")

    def update(self, finfo):
        """docstring for update"""
        self.buffs[0].set("{}".format(finfo.f1))
        self.buffs[1].set("{}".format(finfo.f2))
        self.buffs[2].set("{}".format(finfo.f3))
        self.buffs[3].set("{}".format(finfo.f4))
        #}}}

# ****** BHAND CONTROLLER ******

class SpeedController(tk.LabelFrame):#{{{
    def __init__(self, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Speed Controller")

        # set default value
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        for i, b in enumerate(self.buffs):
            if i <=2 : b.set("100")
            else: b.set("30")
        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        label = tk.Label(self, text="F1-F3: 0-20000, F4: 0-3160")
        label.grid(row=0, column=0, columnspan=2, sticky="e")
        for i, b in enumerate(self.buffs):
            label = tk.Label(self, text="F{}".format(i+1))
            label.grid(row=i+1, column=0, sticky="w")

            entry = tk.Entry(self, text=b)
            entry.grid(row=i+1, column=1, sticky="w")

        button = tk.Button(self, text="send", command=self.service)
        button.grid(row=i+2, column=1, sticky="e")

    def service(self):
        """docstring for service"""
        rospy.wait_for_service("bhand_node/setFingerVel")
        srv = rospy.ServiceProxy("bhand_node/setFingerVel", bhs.SetFingerInfo)
        req = srv(
                int(self.buffs[0].get()),
                int(self.buffs[1].get()),
                int(self.buffs[2].get()),
                int(self.buffs[3].get()),
                )
    #}}}

class PositionController(tk.LabelFrame):#{{{
    def __init__(self, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Position Controller")

        # set default value
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        # HOME Position
        for i, b in enumerate(self.buffs):
            if i <=2 : b.set("0")
            else: b.set("0")

        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        label = tk.Label(self, text="F1-F4: 0-100")
        label.grid(row=0, column=0, columnspan=2, sticky="e")
        for i, b in enumerate(self.buffs):
            label = tk.Label(self, text="F{}".format(i+1))
            label.grid(row=i+1, column=0, sticky="w")

            entry = tk.Entry(self, text=b)
            entry.grid(row=i+1, column=1, sticky="w")

        button = tk.Button(self, text="send", command=self.service)
        button.grid(row=i+2, column=1, sticky="e")

    def service(self):
        """docstring for service"""
        rospy.wait_for_service("bhand_node/setFingerPos")
        srv = rospy.ServiceProxy("bhand_node/setFingerPos", bhs.SetFingerInfo)
        req = srv(
                int(self.buffs[0].get()),
                int(self.buffs[1].get()),
                int(self.buffs[2].get()),
                int(self.buffs[3].get()),
                )
    #}}}

if __name__ == '__main__':
    # init window
    rospy.init_node("bhand_gui")
    root = tk.Tk()
    root.title("Barrett Controller")

    # create each Frame instance 
    actionBtn = ActionButtons(root)
    spdDis = FingerSpeedDiscripter(root)
    posDis = FingerPositionDiscripter(root)
    spdCtrl = SpeedController(root)
    posCtrl = PositionController(root)

    # layout instance
    actionBtn.grid(row=0, column=0, rowspan=2, sticky="n")
    spdCtrl.grid(row=0, column=1, sticky="n")
    posCtrl.grid(row=1, column=1, sticky="n")
    spdDis.grid(row=0, column=2, sticky="n")
    posDis.grid(row=1, column=2, sticky="n")

    root.mainloop()
