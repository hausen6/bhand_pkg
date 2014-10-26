# -*- coding: u8 -*-

import serial


class BHandConnect():
	""" Class for barrett hand control """

	def __init__(self, ):
		self.port = ""
		self.boud = 0
		self.status = ""
		self.serial_handle = None

	def connectPort(self, _port="/dev/ttyUSB0", _boud=9600):
		""" 
		connect port method 
		@param _port: [string] target COM port 	
		@param _boud: [int] boud
		"""

		self.port = _port
		self.boud = _boud 
		self.serial_handle =  serial.Serial(self.port, self.boud, timeout=10)

	def checkConnect(self):
		"""
		check Port connection method
		"""
		if not isinstance(self.serial_handle, SerialBHand):
			print("serial port handle is not Created!!!")
			return False
		responce = self.serial_handle.read()
		if responce == "" or responce is None:
			print("Port {0} is not connected")
			return False
		else:
			return True

	def command(self, command=""):
		"""
		@param command: [str] command to Barrett Hand
		"""
		self.checkConnect()
		print "serial port handle is not Created!!!"
		return False

if __name__ == '__main__':
	bhc = BHandConnect()