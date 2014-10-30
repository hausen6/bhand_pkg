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
        self.pubPos = rospy.Publisher("{0}/FingerPos".format(rospy.get_name()), bhm.FingerInfo)
        self.pubVel = rospy.Publisher("{0}/FingerVel".format(rospy.get_name()), bhm.FingerInfo)
        self.srvSetPos = rospy.Service("{0}/setFingerPos".format(rospy.get_name()), bhs.SetFingerInfo, self.setPos)
        self.srvSetVel = rospy.Service("{0}/setFingerVel".format(rospy.get_name()), bhs.SetFingerInfo, self.setVel)
        self.action    = rospy.Service("{0}/action".format(rospy.get_name()), bhs.Action, self.action)

        self.pos = Finger()
        self.vel = Finger()
        self.connect = BHandConnect()
        self.Init = False
        self.Fstate = False
        self.Sstate = False

        self.lock = False
        #}}}

    def intResultFilter(self, command):
        #{{{
        int_filter = re.compile(r"\d+")
        word_filter = re.compile(r"[a-zA-Z]+")

        # blocking other process is running
        while self.lock:
            rospy.loginfo("wait ...")
            rospy.sleep(0.5)
            
        self.lock = True
        self.connect.command(command)
        try:
            lines = self.connect.serial_handle.readlines()
            res = -1
            for l in lines:
                # search words include or not
                l = l.replace("\r", "").replace("\n", "")
                test = word_filter.search(l)
                if test:
                    continue
                # get result
                test = int_filter.search(l)
                if test:
                    rospy.loginfo(" Matching => {}".format(l))
                    res = int(l)
            self.lock = False
            return res
        except Exception, e:
            print(e)
            exit(0)
            #}}}

    def getPos(self):
        #{{{
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
        self.pos.f4 = tmpRes[3]
        #}}}

    def getVel(self):
        tmp = range(1, 4)#{{{
        tmp.append("S")
        tmpRes = []
        for i, f in zip(tmp, self.vel.fs):
            cmd = "{0}FGET MOV".format(i)
            res = self.intResultFilter(cmd)
            tmpRes.append(int(res))
        self.vel.f1 = tmpRes[0]
        self.vel.f2 = tmpRes[1]
        self.vel.f3 = tmpRes[2]
        self.vel.f4 = tmpRes[3]
        tmpRes = []
        # }}}

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
            print f
            self.connect.command("{0}M {1}".format(i, arg))
        return bhs.SetFingerInfoResponse(True)
        #}}}

    def setVel(self, fInfo):
        # {{{
        """
        @param _f1: [int] target velosity
        @param _f2: [int] target velosity
        @param _f3: [int] target velosity
        @param _f4: [int] target velosity
        """
        tmp = range(1, 4)
        tmp.append("S")
        args = [fInfo.f1, fInfo.f2, fInfo.f3, fInfo.f4]
        for i, f, arg in zip(tmp, self.vel.fs, args):
            self.connect.command("{0}FSET MOV {1}".format(i, arg))
        for i, f, arg in zip(tmp, self.vel.fs, args):
            self.connect.command("{0}FSET MCV {1}".format(i, arg))
        return bhs.SetFingerInfoResponse(True)
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
        return bhs.ActionResponse(True)
    #}}}

    def run(self, _port, _boud):
        self.connect.connectPort(_port, _boud)#{{{
        self.action
        r = rospy.Rate(1)
        res = self.connect.serial_handle.readline()
        while not rospy.is_shutdown():
            while not res == "":
                res = self.connect.serial_handle.readline()

            self.pub.publish(
                    self.Init, self.Fstate, self.Sstate
                    )
            # self.pub.publish(
            #     # state
            #     self.Init, self.Fstate, self.Sstate,
            #     # finger positions
            #     self.pos.f1, self.pos.f2, self.pos.f3, self.pos.f4,
            #     # finger open velosity
            #     self.Openvel.f1, self.Openvel.f4,
            #     # finger close velosity
            #     self.Closevel.f1, self.Closevel.f4,)
            self.getPos()
            self.pubPos.publish(
                    self.pos.f1, self.pos.f2, self.pos.f3, self.pos.f4,
                    )
            self.getVel()
            self.pubVel.publish(
                    self.vel.f1, self.vel.f2, self.vel.f3, self.vel.f4,
                    )
            r.sleep()#}}}#}}}

if __name__ == '__main__':
    try:
        hand = BHandStatus()
        default_args = {"port": "/dev/ttyUSB0", "boud": 9600}
        args = {}
        nodename = rospy.get_name()
        for k, v in default_args.iteritems():
            try:
                if rospy.search_param(k):
                    args[k] = rospy.get_param("{0}/{1}".format(nodename, k))
                else:
                    rospy.loginfo("not found key => dealut_args set")
                    args[k] = v
            except rospy.ROSException, e:
                rospy.logerr(e)
        hand.run(
                args["port"], 
                args["boud"],
                )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# vim:set modeline
# vim:set foldmethod=marker: # %s
