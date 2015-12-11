#!/usr/bin/python

import sys
from PyQt4 import QtCore, QtGui
import serial
import time
import thread, threading, Queue
import simplejson as json
import collections
from PyQt4.QtGui import QApplication, QWidget
from PyQt4.QtCore import QObject, pyqtSignal
from PyQt4 import QtCore

from ui_out import Ui_Dialog

def convert(data):
    if isinstance(data, basestring):
        return str(data)
    elif isinstance(data, collections.Mapping):
        return dict(map(convert, data.iteritems()))
    elif isinstance(data, collections.Iterable):
        return type(data)(map(convert, data))
    else:
        return data

def parseStr(tstStr):
    testDict = {}
    theStr = QtCore.QString(tstStr)
    theStr.replace(':', '": "')
    theStr.replace('{', '{"')
    theStr.replace(',', '", "')
    theStr.replace('}', '"}')
    if '{' in theStr:
        if '}' in theStr:
            print theStr
            try:
                testDict = json.loads(str(theStr))
            except:
                print "failed to convert to JSON"
                return None
            return convert(testDict)
    return None

class StoppableThread(threading.Thread):
    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()

class SerialThread(StoppableThread):
    def setParentAndPort(self, parentGui, port):
        self.serPort = port
        self.theGui = parentGui
    def run(self):
        print "running"
        self.reading = False
        self.inStr = ""
        while not self.stopped():
            self.inChar = self.serPort.read()
#            print self.inChar
            if self.inChar == '{' or self.reading:
                if not self.reading:
                    self.reading = True
                self.inStr += self.inChar
                if self.inChar == '}':
#                    self.serPort.write('{"AK":"RPi"}')
                    self.reading = False
                    print self.inStr
                    if "EM" in self.inStr or "ES" in self.inStr or "EF" in self.inStr:
                        print self.inStr
                        self.theGui.errorPacketReceived.emit(self.inStr)
                    elif "SR" in self.inStr:
                        print self.inStr
                        self.theGui.sensorPacketReceived.emit(self.inStr)
                    elif "AM" in self.inStr:
                        self.theGui.errorPacketReceived.emit(self.inStr)
                    self.inStr = ""




class DataDisplay(Ui_Dialog):
    errorPacketReceived  = pyqtSignal(str, name='errorPacketReceived')
    sensorPacketReceived = pyqtSignal(str, name='sensorPacketReceived')
    def linkButtons(self):
        # self.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.last100Lines = QtCore.QStringList()
        self.listModel = QtGui.QStringListModel()
        self.listView.setModel(self.listModel)
        self.roverPic = QtGui.QPixmap("rover.png")
        self.label.setPixmap(self.roverPic)
        self.label_3.setPixmap(self.roverPic)
        self.pushButton_2.clicked.connect(self.button_Stop_clicked)
        self.pushButton.clicked.connect(self.button_Forward_clicked)
        self.pushButton_5.clicked.connect(self.button_Back_clicked)
        self.pushButton_3.clicked.connect(self.button_SpinR_clicked)
        self.pushButton_4.clicked.connect(self.button_SpinL_clicked)
        self.pushButton_7.clicked.connect(self.button_Left_clicked)
        self.pushButton_8.clicked.connect(self.button_Right_clicked)
        self.pushButton_6.clicked.connect(self.closeCleanly)
        self.label.setAlignment(QtCore.Qt.AlignHCenter)
        self.label_3.setAlignment(QtCore.Qt.AlignHCenter)
        self.errorPacketReceived.connect(self.handleErrorPacket)
        self.sensorPacketReceived.connect(self.handleSensorPacket)
        

    def setSerThread(self, theThread):
        self.serThread = theThread

    def setSerPort(self, port):
        self.serPort = port

    def button_Right_clicked(self):
        self.serPort.write('d')
        self.serPort.flush()
        self.appendToList("Sent: " + 'd')
    def button_Forward_clicked(self):
        self.serPort.write('w')
        self.serPort.flush()
        self.appendToList("Sent: " + 'w')
    def button_Left_clicked(self):
        self.serPort.write('a')
        self.serPort.flush()
        self.appendToList("Sent: " + 'a')
    def button_Back_clicked(self):
        self.serPort.write('s')
        self.serPort.flush()
        self.appendToList("Sent: " + 's')
    def button_SpinL_clicked(self):
        self.serPort.write('q')
        self.serPort.flush()
        self.appendToList("Sent: " + 'q')
    def button_SpinR_clicked(self):
        self.serPort.write('e')
        self.serPort.flush()
        self.appendToList("Sent: " + 'e')
    def button_Stop_clicked(self):
        self.serPort.write('?')
        self.appendToList("Sent: " + '?')
    def keyPressEvent(self, event):
        print "key pressed"
        if event.key() == QtCore.Qt.Key_W:
            self.button_Up_pressed()
            print "key up pressed"
        if event.key() == QtCore.Qt.Key_Left:
            self.button_Left_pressed()
        if event.key() == QtCore.Qt.Key_Right:
            self.button_Right_pressed()
        if event.key() == QtCore.Qt.Key_Down:
            self.button_Back_pressed()
        return True
    def appendToList(self, text):
        # print self.last100Lines.count()
        if self.last100Lines.count() == 100:
            self.last100Lines.takeLast()
        self.last100Lines.prepend(QtCore.QString(text))
        self.listModel.setStringList(self.last100Lines)
        # self.listView.setModel(self.listModel)
    def handleErrorPacket(self, inStr):
        # print "handle: " + inStr
        self.appendToList(inStr)
    def handleSensorPacket(self, inStr):
        inDic = parseStr(inStr)
        if inDic:
            if 'RS' in inDic.keys():
                self.label_2.setText(inDic['RS'])
                #self.label_2.setText('Right')
            if 'FS' in inDic.keys():
                self.label_4.setText(inDic['FS'])
                #self.label_4.setText('Center')
            if 'LS' in inDic.keys():
                self.label_5.setText(inDic['LS'])
                #self.label_5.setText('Left')
    def setParentWindow(self, wind):
        self.parentWindow = wind
    def closeCleanly(self, event):
        self.serThread.stop()
        self.serThread.join()
        print "stopped"
        self.parentWindow.close()

def main():
    ser = serial.Serial(port='/dev/ttyUSB0', baudrate=57600, timeout=1)
    ser2 = serial.Serial(port='/dev/ttyUSB1',  baudrate=57600, timeout=1)
    # print ser.isOpen()
    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowState(QtCore.Qt.WindowMaximized)
    ui = DataDisplay()
    serThread = SerialThread()
    serThread.setParentAndPort(ui, ser)
    ui.setupUi(window)
    ui.setParentWindow(window)
    ui.linkButtons()
    ui.setSerThread(serThread)
    ui.setSerPort(ser)
    serThread.start()

    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
  main()
