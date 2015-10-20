#!/usr/bin/python

import sys
from PyQt4 import QtCore, QtGui
import serial
import time
import thread, threading, Queue
import json
import collections

ser = serial.Serial(port='/dev/ttyUSB4', baudrate=57600)

running = True
theQueue = Queue.Queue(10)
stopQueue = Queue.Queue(10)
serThread = None

def serialThread(threadName):
  print "in serial thread"
  y = ""
  start = False
  while stopQueue.empty:
    if start:
      x = ser.read()
      y += x
      if x == '}':
        theQueue.put(y)
        y = ""
    else:
      if ser.read() == '{':
        start = True
        print "started"



def updateThread(threadName, ui):
  internalTxt = ""
  internalDic = {}
  displayString = ""
  print "in update thread"
  while stopQueue.empty:
    string = theQueue.get()
    # ui.sensorDataLabel.setText(string)
    dic = parseStr(string)
    if dic:
      if 'FF' in dic.keys():
        ui.ff_pacs += 1
      if 'MR' in dic.keys():
        ui.MR_pacs += 1
      if 'AM' in dic.keys():
        ui.Ack_pacs += 1
      if 'SR' in dic.keys():
        ui.SR_pacs += 1
      if 'EM' in dic.keys():
        ui.EM_pacs += 1
      internalDic.update(dic)
      displayString = ""
      try:
        FF_str = "Find and Follow - RF: " + internalDic["RF"] + " LF: " + internalDic["LF"] + " CF: " + internalDic["CF"] + " NF: " + internalDic["NF"] + "\n"
        displayString = displayString + FF_str #+ "Total Find and Follow Packets: " + str(ff_pacs ) + "\n"
      except:
        print "key error"
      try:
        Ack_str = "Ack             - AM: " + internalDic["AM"] + " AN: " + internalDic["AN"] + " NU: " + internalDic["NU"] + "\n"
        displayString = displayString + Ack_str #+ "Total Ack Packets:             " + str(Ack_pacs) + "\n"
      except:
        print "key error"
      try:
        SR_str = "Sensor          - SR: " + internalDic["SR"] + " RS: " + internalDic["RS"] + " LS: " + internalDic["LS"] + " FS: " + internalDic["FS"] + " BS: " + internalDic["BS"] + " NS: " + internalDic["NS"] + "\n"
        displayString = displayString + SR_str #+ "Total Sensor Packets:          " + str(SR_pacs ) + "\n"
      except:
        print "key error"
      try:
        MR_str = "Motor           - MR: " + internalDic["MR"] + " QW: " + internalDic["QW"] + " QE: " + internalDic["QE"] + " QT: " + internalDic["QT"] + " QY: " + internalDic["QY"] + " NM: " + internalDic["NM"] + "\n"
        displayString = displayString + MR_str #+ "Total Motor Packets:           " + str(MR_pacs ) + "\n"
      except:
        print "key error"
      try:
        EM_str = "Motor Error     - EM: " + internalDic["EM"] + " XX: " + internalDic["XX"] + "\n"
        displayString = displayString + EM_str #+ "Total Motor Error Packets:     " + str(EM_pacs ) + "\n"
      except:
        print "key error"
      ui.sensorDataLabel.setText(displayString)
      totalPacsStr = "Total Find and Follow Packets: " + str(ui.ff_pacs ) + "\n" \
                   + "Total Ack Packets:             " + str(ui.Ack_pacs) + "\n" \
                   + "Total Sensor Packets:          " + str(ui.SR_pacs ) + "\n" \
                   + "Total Motor Packets:           " + str(ui.MR_pacs ) + "\n" \
                   + "Total Motor Error Packets:     " + str(ui.EM_pacs )
      ui.errorDataLabel.setText(totalPacsStr)

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
  # print tstStr
  theStr = QtCore.QString(tstStr)
  theList = theStr.split('}')
  theList.replaceInStrings(':', '": "')
  theList.replaceInStrings('{', '{"')
  theList.replaceInStrings(',', '", "')
  list2 = []
  for i in theList:
    j = i.append('"}]')
    list2.append(str(j.prepend('[')))
  for i in list2:
    if '{' in i:
      if '}' in i:
        print i
        try:
          testDict = json.loads(i)[0]
        except:
          return None
        return convert(testDict)
  return None
    



class DataDisplay(QtGui.QWidget):
  def __init__(self):
    super(DataDisplay, self).__init__()
    try:
      thread.start_new_thread(updateThread, ("Update_Thread", self) )
    except:
      print "unable to start update thread"
      running = False
      self.close()
    self.initUI()
    self.count = 0

    
    
      

  def initUI(self):
    self.sensorDataLabel = QtGui.QLabel('', self)
    self.sensorDataLabel.setStyleSheet(" font-size: 16px; qproperty-alignment: AlignJustify; font-family: Courier New;")

    self.errorDataLabel = QtGui.QLabel('', self)
    self.errorDataLabel.setStyleSheet(" font-size: 16px; qproperty-alignment: AlignJustify; font-family: Courier New;")

    errorRequestBtn = QtGui.QPushButton('Request Error Data', self)
    errorRequestBtn.clicked.connect(self.requestErrorData)
    
    vbox = QtGui.QVBoxLayout()
    vbox.addWidget(self.sensorDataLabel)
    vbox.addWidget(self.errorDataLabel)
    vbox.addWidget(errorRequestBtn)
    self.ff_pacs = 0
    self.Ack_pacs = 0
    self.SR_pacs = 0
    self.MR_pacs = 0
    self.EM_pacs = 0

    self.setLayout(vbox)


    
    self.setGeometry(20, 20, 700, 400)
    self.setWindowTitle('Follow the Leader')
    self.show()

  def requestErrorData(self):
    # ser.write('error_request')
    self.count += 1
    # self.sensorDataLabel.setText('btnPressed' + str(self.count))
    ser.write('btnPressed' + str(self.count))
    self.ff_pacs = 0
    self.Ack_pacs = 0
    self.SR_pacs = 0
    self.MR_pacs = 0
    self.EM_pacs = 0

  def closeEvent(self,event):
    reply=QtGui.QMessageBox.question(self,'Message',"Are you sure to quit?",QtGui.QMessageBox.Yes,QtGui.QMessageBox.No)
    if reply==QtGui.QMessageBox.Yes:
      stopQueue.put("stop")

      event.accept()
    else:
       event.ignore()

def main():
  app = QtGui.QApplication(sys.argv)
  DD = DataDisplay()
  serThead = threading.Thread(target=serialThread, args=("serialThread",))
  serThead.start()
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
