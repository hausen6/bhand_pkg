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
    def __init__(self, master, target_node, text, order):
        tk.Button.__init__(self, master, text=text, command=self.bhand_action)
        self.order = order
        self.target_node = target_node
    
    def bhand_action(self):
        """ hand service callbask method """
        rospy.wait_for_service("{0}/action".format(self.target_node))
        req = rospy.ServiceProxy("{0}/action".format(self.target_node), bhs.Action)
        req(self.order)
    #}}}

class ActionButtons(tk.LabelFrame):#{{{
    def __init__(self, nodename, master=None):
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

        self.nodename = nodename
        self.initUI()

    def initUI(self):
        """docstring for initUI"""
        for b in self.btns:
            b = Abuttons(self, self.nodename, text=b[0], order=b[1])
            b.pack(fill=tk.BOTH)
    #}}}

# ****** BHAND STATE DISCRIPTER ******

class FingerSpeedDiscripter(tk.LabelFrame):#{{{
    def __init__(self, target_node, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Finger Speed")
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.target_node = target_node
        rospy.logdebug("{0}/FingerVel".format(self.target_node))
        self.sub = rospy.Subscriber("{0}/FingerVel".format(self.target_node), bhm.FingerInfo, self.update)

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
    def __init__(self, target_node, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Finger Position")

        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.target_node = target_node
        self.sub = rospy.Subscriber("{}/FingerPos".format(self.target_node), bhm.FingerInfo, self.update)

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
    def __init__(self, target_node, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Speed Controller")

        # set default value
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.target_node = target_node
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
        rospy.wait_for_service("{}/setFingerVel".format(self.target_node))
        srv = rospy.ServiceProxy("{}/setFingerVel".format(self.target_node), bhs.SetFingerInfo)
        req = srv(
                int(self.buffs[0].get()),
                int(self.buffs[1].get()),
                int(self.buffs[2].get()),
                int(self.buffs[3].get()),
                )
    #}}}

class PositionController(tk.LabelFrame):#{{{
    def __init__(self, target_node, master=None):
        tk.LabelFrame.__init__(self, master=master, text="Position Controller")

        # set default value
        self.buffs = [
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                tk.StringVar(),
                ]
        self.target_node = target_node
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
        rospy.wait_for_service("{}/setFingerPos".format(self.target_node))
        srv = rospy.ServiceProxy("{}/setFingerPos".format(self.target_node), bhs.SetFingerInfo)
        req = srv(
                int(self.buffs[0].get()),
                int(self.buffs[1].get()),
                int(self.buffs[2].get()),
                int(self.buffs[3].get()),
                )
    #}}}

if __name__ == '__main__':
    # param get
    rospy.init_node("bhand_gui")
    default_params = {
            "target_node": "bhand_node",
            }
    nodename = rospy.get_name()
    for k, v in default_params.iteritems():
        try:
            if rospy.search_param(k):
                target_node = rospy.get_param("{0}/{1}".format(nodename, k))
                rospy.loginfo(target_node)
            else:
                rospy.loginfo("params are not found")
                target_node = default_params[k]
        except rospy.ROSException, e:
            rospy.loginfo(e)
    # init window
    root = tk.Tk()
    root.title("Barrett Controller")

    # create each Frame instance 
    actionBtn = ActionButtons(target_node, root)
    spdDis = FingerSpeedDiscripter(target_node, root)
    posDis = FingerPositionDiscripter(target_node, root)
    spdCtrl = SpeedController(target_node, root)
    posCtrl = PositionController(target_node, root)

    # layout instance
    actionBtn.grid(row=0, column=0, rowspan=2, sticky="n")
    spdCtrl.grid(row=0, column=1, sticky="n")
    posCtrl.grid(row=1, column=1, sticky="n")
    spdDis.grid(row=0, column=2, sticky="n")
    posDis.grid(row=1, column=2, sticky="n")

    root.mainloop()
