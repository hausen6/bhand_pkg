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

class BHandGUI():
    # init
    #{{{
    def __init__(self, ):
        # tkinter
        self.root = tk.Tk()
        self.bFrame = tk.LabelFrame(self.root,  text="Barrett Action")
        self.spFrame = tk.LabelFrame(self.root, text="Speed Control")
        self.posFrame = tk.LabelFrame(self.root, text="Finger Position")
        self.velFrame = tk.LabelFrame(self.root, text="Finger Speed")
        self.posSliderFrame = tk.LabelFrame(self.root, text="Position Control")
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
        for i, v in enumerate(self.vels):
            v.set("{0} Speed: {1}".format(i + 1, 0))
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
        for i, p in enumerate(self.poss):
            p.set("{0} Position: {1}".format(i + 1, 0))
        f1pos_s = tk.StringVar()
        f2pos_s = tk.StringVar()
        f3pos_s = tk.StringVar()
        f4pos_s = tk.StringVar()
        self.poss_s = [
                f1pos_s,
                f2pos_s,
                f3pos_s,
                f4pos_s,
                ]
        for i, p in enumerate(self.poss_s):
            p.set("{}".format(0))
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
        # finger position view
        self.vels[0].set("Speed 1: {0}".format(fInfo.f1))
        self.vels[1].set("Speed 2: {0}".format(fInfo.f2))
        self.vels[2].set("Speed 3: {0}".format(fInfo.f3))
        self.vels[3].set("Speed 4: {0}".format(fInfo.f4))
    #}}}
    def getFingerPos(self, fInfo):#{{{
        """docstring for viewFingerInfo"""
        # finger position view
        self.poss[0].set("Position 1: {0}".format(fInfo.f1))
        self.poss[1].set("Position 2: {0}".format(fInfo.f2))
        self.poss[2].set("Position 3: {0}".format(fInfo.f3))
        self.poss[3].set("Position 4: {0}".format(fInfo.f4))
    #}}}
    def setFingerPos(self):#{{{
        """docstring for SetSliderPos"""
        rospy.wait_for_service("bhand_node/setFingerPos")
        posSrv = rospy.ServiceProxy("bhand_node/setFingerPos", bhs.SetFingerInfo)
        req = posSrv(
                int(self.poss_s[0].get()),
                int(self.poss_s[1].get()),
                int(self.poss_s[2].get()),
                int(self.poss_s[3].get()),
                )
        #}}}
    #}}}
    # gui grid method
    def ActionButton(self):#{{{
        """ method for showing barrett hand control buttons """
        for i, c in enumerate(self.bottonCallbacks):
            button = tk.Button(self.bFrame, text=c.__name__, command=c)
            # button.(row=i, column=0, sticky="ew")
            button.pack(fill=tk.BOTH)
    #}}}
    def FinLabel(self):#{{{
        """docstring for Label"""
        for i in range(4):
            label = tk.Label(self.spFrame, text="F{0} speed".format(i+1))
            label.grid(row=i, column=0, sticky="e")
    #}}}
    def FingerVel(self):#{{{
        """docstring for setFingerVel"""
        label = "finger {0} speed set"
        for i, v in enumerate(self.setvels):
            e = tk.Entry(self.spFrame, text=v)
            e.grid(row=i, column=1, sticky="ns")
        button = tk.Button(self.spFrame, text="send", command=self.sendFingerVel)
        button.grid(row=4, column=1, sticky="e")
    #}}}
    def FingerInfo(self):#{{{
        """docstring for gredFingerInfo"""
        for i, p in enumerate(self.poss):
            # lbl = tk.Label(frame, text=p.get())
            lbl = tk.Label(self.posFrame, textvariable=p)
            lbl.grid(row=i, column=0, sticky="w")
        for i, p in enumerate(self.vels):
            # lbl = tk.Label(frame, text=p.get())
            lbl = tk.Label(self.velFrame, textvariable=p)
            lbl.grid(row=i+5, column=0, sticky="w")
#}}}
    def PosSetter(self):#{{{
        """docstring for posSlider"""
        lbl = tk.Label(self.posSliderFrame, text="F1~F3 (0 - 20000)\tF4 (0 - 3160)")
        lbl.grid(row=0, column=0, columnspan=2, sticky="w")
        for i, s in enumerate(self.poss_s):
            ent = tk.Entry(self.posSliderFrame, text=s)
            ent.grid(row=i+2, column=1, sticky="ns")
            lbl = tk.Label(self.posSliderFrame, text="F{0} position".format(i+1))
            lbl.grid(row=i+2, column=0, sticky="e")
        btn = tk.Button(self.posSliderFrame, text="send", command=self.setFingerPos)
        btn.grid(row=6, column=1, sticky="e")
#}}}

    def framePuck(self):
        # widget pucking
        self.ActionButton()
        self.FinLabel()
        self.FingerVel()
        self.FingerInfo()
        self.PosSetter()

        # frame pucking
        self.spFrame.pack(side=tk.BOTTOM)
        self.posSliderFrame.pack(side=tk.BOTTOM)
        self.bFrame.pack(side=tk.LEFT)
        self.posFrame.pack(anchor=tk.SE)
        self.velFrame.pack(anchor=tk.SE)
    

# ********* TO HERE BHandGUI class ***********

if __name__ == '__main__':
    try:
        g = BHandGUI()
        g.framePuck()
        g.root.mainloop()
    except rospy.ROSInterruptException:
        pass
