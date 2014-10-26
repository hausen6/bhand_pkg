#!/usr/bin/env python
# coding: utf8
from BHandConnect import BHandConnect

# import rospy
# from mybhand.msg import State

class Finger():
	""" class of Finger """
	def __init__(self, ):
		# position
		self.f1 = 0
		self.f2 = 0
		self.f3 = 0
		self.f4 = 0
		self.fs = (self.f1, self.f2, self.f3, self.f4)

class BHandStatus():
	""" BHand status publisher """
	def __init__(self,):
		# ros
		rospy.init_node("bhand_status", anonymous=True)
		self.pub = rospy.Publisher("bhand/status", State)

		self.pos = Finger()
		self.vel = Finger()
		self.connect = BHandConnect()

	def getPos(self):
		# TODO: command CHECK!!!
		for i, f in zip(range(1, 5), self.pos.fs):
			self.connect.command("GET {0}MOV".format(i))
			res = self.connect.read()
			f = int(res)

	def getVel(self):
		# TODO: command CHECK!!!
		for i, f in zip(range(1, 5), self.vel.fs):
			self.connect.command("GET {0}MOV".format(i))
			res = self.connect.read()
			f = int(res)

	def setPos(self, _f1, _f2, _f3, _f4):
		# TODO: command CHECK!!!
		"""
		@param _f1: [int] target position 
		@param _f2: [int] target position 
		@param _f3: [int] target position 
		@param _f4: [int] target position 
		"""
		args = [_f1, _f2, _f3, _f4]
		for i, f, arg in zip(range(1, 5), self.pos.fs, args):
			self.connect.command("SET {0}MOP {1}".format(i, arg))
			res = self.connect.read()
			f = int(res)

	def setVel(self, _f1, _f2, _f3, _f4):
		# TODO: command CHECK!!!
		"""
		@param _f1: [int] target velosity 
		@param _f2: [int] target velosity 
		@param _f3: [int] target velosity 
		@param _f4: [int] target velosity 
		"""
		args = [_f1, _f2, _f3, _f4]
		for i, f, arg in zip(range(1, 5), self.vel.fs, args):
			self.connect.command("SET {0}MOV {1}".format(i, arg))
			res = self.connect.read()
			f = int(res)

	def statusPub(self):
		r = rospy.Rate(1)
		while rospy.is_shutdown():
			
			r.sleep()
if __name__ == '__main__':
	hand = BHandStatus()