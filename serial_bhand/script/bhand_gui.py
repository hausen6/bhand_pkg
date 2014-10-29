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
        self.bFrame = tk.LabelFrame(self.root,  text="Barrett Action")
        self.spFrame = tk.LabelFrame(self.root, text="Finger Speed")
        self.posFrame = tk.LabelFrame(self.root, text="Finger Position")
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
        # speed #{{{
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
        f1setvel = tk.StringVar()
        f2setvel = tk.StringVar()
        f3setvel = tk.StringVar()
        f4setvel = tk.StringVar()
        self.setvels = [
                f1setvel,
                f2setvel,
                f3setvel,
                f4setvel,
                ]
        for v, default in zip(self.setvels, ["100", "100", "100", "40"]):
            v.set(default)
        #}}}
        # position #{{{
        f1pos = tk.StringVar()
        f2pos = tk.StringVar()
        f3pos = tk.StringVar()
        f4pos = tk.StringVar()
        self.poss = [
                f1pos,
                f2pos,
                f3pos,
                f4pos,
                ]
        for p in self.poss:
            p.set("0")
        #}}}

        # ros
        rospy.init_node("bhand_gui")
        self.velSub = rospy.Subscriber("bhand_node/FingerVel", bhm.FingerInfo, self.getFingerVel)
        self.posSub = rospy.Subscriber("bhand_node/FingerPos", bhm.FingerInfo, self.getFingerPos)
        self.vel = Finger()
        self.pos = Finger()
        #}}}

    #  callbacks method 
    #{{{
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
        rospy.wait_for_service("bhand_node/setFingerVel")
        velSrv = rospy.ServiceProxy("bhand_node/setFingerVel", bhs.SetFingerInfo)
        req = velSrv(
                int(self.setvels[0].get()),
                int(self.setvels[1].get()),
                int(self.setvels[2].get()),
                int(self.setvels[3].get()),
                )
    #}}}
    def getFingerVel(self, fInfo):#{{{
        """docstring for viewFingerInfo"""
        # finger position view
        self.vels[0].set(str(fInfo.f1))
        self.vels[1].set(str(fInfo.f2))
        self.vels[2].set(str(fInfo.f3))
        self.vels[3].set(str(fInfo.f4))
    #}}}
    def getFingerPos(self, fInfo):#{{{
        """docstring for viewFingerInfo"""
        # finger position view
        self.poss[0].set(str(fInfo.f1))
        self.poss[1].set(str(fInfo.f2))
        self.poss[2].set(str(fInfo.f3))
        self.poss[3].set(str(fInfo.f4))
    #}}}
    #}}}
    # gui grid method
    def gridActionButton(self):#{{{
        """ method for showing barrett hand control buttons """
        for i, c in enumerate(self.bottonCallbacks):
            button = tk.Button(self.bFrame, text=c.__name__, command=c)
            # button.grid(row=i, column=0, sticky="ew")
            button.pack(fill=tk.BOTH)
    #}}}
    def gridFinLabel(self):#{{{
        """docstring for gridLabel"""
        for i in range(4):
            label = tk.Label(self.spFrame, text="F{0} speed".format(i+1))
            label.grid(row=i, column=0, sticky="e")
    #}}}
    def gridFingerVel(self):#{{{
        """docstring for setFingerVel"""
        label = "finger {0} speed set"
        for i, v in enumerate(self.setvels):
            e = tk.Entry(self.spFrame, text=v.get())
            e.grid(row=i, column=1, sticky="ns")
        button = tk.Button(self.spFrame, text="send", command=self.sendFingerVel)
        button.grid(row=4, column=1, sticky="e")
    #}}}
    def gridFingerInfo(self):#{{{
        """docstring for gredFingerInfo"""
        frame = tk.LabelFrame(self.root, text="Finger Info")
        for i, p in enumerate(self.poss):
            lbl = tk.Label(frame, textvariable="finger {0} pos: {1}".format(i, p.get()))
            lbl.grid(row=i, column=0)
        for i, p in enumerate(self.vels):
            label = "finger {0} speed: {1}".format(i, p.get())
            lbl = tk.Label(frame, textvariable=label)
            lbl.grid(row=i, column=1)
        frame.pack(side=tk.RIGHT)
#}}}

    def framePuck(self):
        # widget pucking
        self.gridActionButton()
        self.gridFinLabel()
        self.gridFingerVel()
        self.gridFingerInfo()

        # frame pucking
        self.spFrame.pack(side=tk.RIGHT)
        self.bFrame.pack(side=tk.LEFT)
    

# ********* TO HERE BHandGUI class ***********

if __name__ == '__main__':
    try:
        g = BHandGUI()
        g.framePuck()
        g.root.mainloop()
    except rospy.ROSInterruptException:
        pass
