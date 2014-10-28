#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from bhand_connect import BHandConnect
import re

from serial_bhand.msg import State
import serial_bhand.msg as bhm
import serial_bhand.srv as bhs

class Finger():
    """ class of Finger """
    def __init__(self, ):
        #{{{
        # position
        self.f1 = 0
        self.f2 = 0
        self.f3 = 0
        self.f4 = 0
        self.fs = (self.f1, self.f2, self.f3, self.f4)#}}}

class BHandStatus():
    """ BHand status publisher """#{{{
    def __init__(self,):
        #{{{
        # ros
        rospy.init_node("bhand_node")
        self.pub       = rospy.Publisher("{0}/status".format(rospy.get_name()), State)
        self.action    = rospy.Service("{0}/action".format(rospy.get_name()), bhs.Action, self.action)
        self.subSetPos = rospy.Subscriber("{0}/FingerPos".format(rospy.get_name()), bhm.FingerInfo, self.setPos)
        self.subSetVel = rospy.Subscriber("{0}/FingerVel".format(rospy.get_name()), bhm.FingerInfo, self.setVel)

        self.pos = Finger()
        self.Openvel = Finger()
        self.Closevel = Finger()
        self.connect = BHandConnect()
        self.Init = False
        self.Fstate = False
        self.Sstate = False
        #}}}

    def intResultFilter(self, command):
        while 1:#{{{
            res_filter = re.compile(r"¥d+")
            self.connect.serial_handle.readall()
            self.connect.command(command)
            self.connect.serial_handle.readline()
            res = self.connect.serial_handle.readall()
            searchRes = res_filter.search(res)
            if searchRes:
                searchRes = searchRes.group(0)
                return searchRes#}}}

    def getPos(self):
        # TODO: command CHECK!!!#{{{
        tmp = range(1, 4)
        tmp.append("S")
        tmpRes = []
        for i, f in zip(tmp, self.pos.fs):
            cmd = "{0}FGET P".format(i)
            res = self.intResultFilter(cmd)
            tmpRes.append(int(res))
        self.pos.f1 = tmpRes[0]
        self.pos.f2 = tmpRes[1]
        self.pos.f3 = tmpRes[2]
        self.pos.f4 = tmpRes[3]#}}}

    def getVel(self):
        tmp = range(1, 4)#{{{
        tmp.append("S")
        tmpRes = []
        # TODO: command CHECK!!!
        for i, f in zip(tmp, self.Openvel.fs):
            self.connect.command("{0}FGET MOV".format(i))
            cmd = "{0}FGET MOV".format(i)
            res = self.intResultFilter(cmd)
            tmpRes.append(int(res))
        self.Openvel.f1 = tmpRes[0]
        self.Openvel.f2 = tmpRes[1]
        self.Openvel.f3 = tmpRes[2]
        self.Openvel.f4 = tmpRes[3]
        tmpRes = []
        for i, f in zip(tmp, self.Closevel.fs):
            cmd = "{0}FGET MCV".format(i)
            res = self.intResultFilter(cmd)
            tmpRes.append(int(res))
        self.Closevel.f1 = tmpRes[0]
        self.Closevel.f2 = tmpRes[1]
        self.Closevel.f3 = tmpRes[2]
        self.Closevel.f4 = tmpRes[3]#}}}

    def setPos(self, fInfo):
        #{{{
        """
        @param _f1: [int] target position
        @param _f2: [int] target position
        @param _f3: [int] target position
        @param _f4: [int] target position
        """
        tmp = range(1, 4)
        tmp.append("S")
        args = [fInfo.f1, fInfo.f2, fInfo.f3, fInfo.f4]
        for i, f, arg in zip(tmp, self.pos.fs, args):
            self.connect.command("{0}M {1}".format(i, arg))
        self.connect.serial_handle.readall()
        return 0
        #}}}

    def setVel(self, fInfo):
        # TODO: command CHECK!!!#{{{
        """
        @param _f1: [int] target velosity
        @param _f2: [int] target velosity
        @param _f3: [int] target velosity
        @param _f4: [int] target velosity
        """
        tmp = range(1, 4)
        tmp.append("S")
        args = [fInfo.f1, fInfo.f2, fInfo.f3, fInfo.f4]
        for i, f, arg in zip(tmp, self.Openvel.fs, args):
            self.connect.command("{0}FSET MOV {1}".format(i, arg))
        for i, f, arg in zip(tmp, self.Openvel.fs, args):
            self.connect.command("{0}FSET MCV {1}".format(i, arg))
        self.connect.serial_handle.readall()
        return 0
        #}}}

    def action(self, req):
        if req.order == req.INITIALIZE:#{{{
            self.connect.command("HI")
            self.Init = True
            self.Fstate = True
            self.Sstate = True
        elif req.order == req.OPEN:
            self.connect.command("123O")
            self.Fstate = True
        elif req.order == req.CLOSE:
            self.connect.command("123C")
            self.Fstate = False
        elif req.order == req.T_OPEN:
            self.connect.command("123TO")
            self.Fstate = True
        elif req.order == req.T_CLOSE:
            self.connect.command("123TC")
            self.Fstate = False
        elif req.order == req.S_OPEN:
            self.connect.command("SO")
            self.Sstate = True
        elif req.order == req.S_CLOSE:
            self.connect.command("SC")
            self.Sstate = False
        elif req.order == req.COMMAND:
            self.connect.command(req.s_order)
        self.connect.serial_handle.readall()
        return bhs.ActionResponse(True)
    #}}}

    def run(self, _port, _boud):
        self.connect.connectPort(_port, _boud)#{{{
        self.action
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            res = self.connect.serial_handle.readall()
            while not res == "":
                res = self.connect.serial_handle.readline()

            self.getPos()
            self.getVel()
            self.pub.publish(
                # state
                self.Init,
                self.Fstate,
                self.Sstate,
                # finger positions
                self.pos.f1,
                self.pos.f2,
                self.pos.f3,
                self.pos.f4,
                # finger open velosity
                self.Openvel.f1,
                self.Openvel.f4,
                # finger close velosity
                self.Closevel.f1,
                self.Closevel.f4,
                )
            r.sleep()#}}}#}}}

if __name__ == '__main__':
    hand = BHandStatus()
    hand.run("/dev/ttyUSB2", 9600)
    rospy.spin()

# vim:set modeline
# vim:set foldmethod=marker: # %s
