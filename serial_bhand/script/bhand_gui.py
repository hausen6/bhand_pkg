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

class BHandGUI(tk.Frame):
    # init
    #{{{
    def __init__(self, ):
        # tkinter
        self.root = tk.Tk()
        # *** button ***
        # ボタンのコールバックに指定したい場合はこれに追加する必要がある
        self.bottonCallbacks = [
                self.HandInit,
                self.HandOpen,
                self.HandClose,
                self.HandTorqueOpen,
                self.HandTorqueClose,
                self.SpreadOpen,
                self.SpreadClose,
                ]

        # edit box
        f1vel = tk.StringVar()
        f2vel = tk.StringVar()
        f3vel = tk.StringVar()
        f4vel = tk.StringVar()
        self.vels = [
                f1vel,
                f2vel,
                f3vel,
                f4vel,
                ]
        for v in self.vels:
            v.set("40")
        self.vel = Finger()

        # ros
        rospy.init_node("bhand_gui")
        self.velPub = rospy.Publisher("bhand_node/FingerVel", bhm.FingerInfo)
        #}}}

    #  callbacks method 
    def HandInit(self): #{{{
        """docstring for initCall"""
        print("init is pushed")
        print("wait ...")
        rospy.wait_for_service("bhand_node/action")
        print("Service!!")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.INITIALIZE)
        #}}}
    def HandOpen(self):#{{{
        """docstring for opencall"""
        print("open is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.OPEN)
#}}}
    def HandClose(self):#{{{
        """docstring for closeCall"""
        print("close is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.CLOSE)
#}}}
    def HandTorqueOpen(self):#{{{
        """docstring for topenCall"""
        print("torque open is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.T_OPEN)
#}}}
    def HandTorqueClose(self):#{{{
        """docstring for topenCall"""
        print("torque close is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.T_CLOSE)
        # }}}
    def SpreadOpen(self):#{{{
        """docstring for topenCall"""
        print("spread open is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.S_OPEN)
        # }}}
    def SpreadClose(self):#{{{
        """docstring for topenCall"""
        print("spread close is pushed")
        rospy.wait_for_service("bhand_node/action")
        serv = rospy.ServiceProxy("bhand_node/action", bhs.Action)
        res = serv(bhs.ActionRequest.S_CLOSE)
        #}}}
    def sendFingerVel(self):#{{{
        """docstring for sendFingerVel"""
       
        # print int(self.vels[0].get())
        # print int(self.vels[1].get())
        # print int(self.vels[2].get())
        # print int(self.vels[3].get())
        self.velPub.publish(
                int(self.vels[0].get()),
                int(self.vels[1].get()),
                int(self.vels[2].get()),
                int(self.vels[3].get()),
                )
    #}}}

    # gui grid method
    def gridActionButton(self):#{{{
        """ method for showing barrett hand control buttons """
        for i, c in enumerate(self.bottonCallbacks):
            button = tk.Button(self.root, text=c.__name__, command=c)
            button.grid(row=i, column=0, sticky="ew")
    #}}}
    def gridFinLabel(self):#{{{
        """docstring for gridLabel"""
        for i in range(4):
            label = tk.Label(self.root, text="F{0} speed".format(i+1))
            label.grid(row=i, column=1, sticky="e")
    #}}}
    def gridFingerVel(self):#{{{
        """docstring for setFingerVel"""
        label = "finger {0} speed set"
        for i, v in enumerate(self.vels):
            e = tk.Entry(self.root, textvariable=v)
            e.grid(row=i, column=2, sticky="ns")
        button = tk.Button(self.root, text="send", command=self.sendFingerVel)
        button.grid(row=4, column=2, sticky="e")
    #}}}

# ********* TO HERE BHandGUI class ***********

if __name__ == '__main__':
    try:
        g = BHandGUI()
        g.gridActionButton()
        g.gridFinLabel()
        g.gridFingerVel()
        g.root.mainloop()
    except KeyboardInterrupt:
        pass
