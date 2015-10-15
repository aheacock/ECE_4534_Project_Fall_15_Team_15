#!/usr/local/bin/python

import pygtk
import serial
import gtk
import threading
import time
serialThread = None


# ser = serial.Serial('/dev/ttyUSB0')
# def waitForData(data):



class SerialMonitorThread(threading.Thread):
  def __init__(self, threadID, name):
    super(SerialMonitorThread, self).__init__()
    threading.Thread.__init__(self)
    self.threadID = threadID
    self.name = name
    self._stop = threading.Event()
    self.run()

  def run(self):
      print "Starting " + self.name
      print "Exiting " + self.name
  def stop(self):
    self._stop.set()
  def stopped(self):
    return self._stop.isSet()



class GUI:
  def hello(self, widget, data=None):
    print "Hello World"

  def delete_event(self, widget, event, data=None):
    print "delete event occurred"
    return False

  def destroy(self, widget, data=None):
    serialThread.stop()
    serialThread.join()
    gtk.main_quit()    

  def __init__(self):
    self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
    self.window.connect("delete_event", self.delete_event)
    self.window.connect("destroy", self.destroy)
    self.window.set_border_width(10)
    self.button = gtk.Button("Hello World")
    self.button.connect("clicked", self.hello, None)
    self.window.add(self.button)
    self.button.show()
    self.window.show()

  def main(self):
    gtk.main()

if __name__ == "__main__":
  gui = GUI()
  serialThread = SerialMonitorThread(1, "Thread-1")
  serialThread.start()
  gui.main()