#!/usr/bin/env python

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
        self.serial_handle =  serial.Serial(self.port, self.boud, timeout=0.1)

    def checkConnect(self):
        """
        check Port connection method
        """
        try:
            responce = self.serial_handle.read()
            return True
        except:
            return False

    def command(self, command=""):
        """
        @param command: [str] command to Barrett Hand
        """
        if self.checkConnect():
            self.serial_handle.write(command.decode("u8") + u"\r")
            return True
        else:
        	return False


if __name__ == '__main__':
    bhc = BHandConnect()
    bhc.connectPort("/dev/ttyUSB1", 9600)
