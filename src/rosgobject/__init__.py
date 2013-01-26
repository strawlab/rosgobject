# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import rospy

import logging
import threading

from gi.repository import GLib, Gtk, GObject, Gdk

GObject.threads_init()
Gdk.threads_init()

LOG = logging.getLogger(__name__)

class _ROSThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, name="Thread-rospy.spin")
        self.daemon = True
        LOG.getChild("ROSThread").info("creating thread")
        self.start()

    def _check_running(self):
        #this lets us die when the ROS thread dies
        if not self.is_alive():
            Gtk.main_quit()
            return False
        return True

    def run(self):
        GLib.timeout_add(1000/10, self._check_running)
        LOG.getChild("ROSThread").info("calling rospy.spin")
        rospy.spin()
        LOG.getChild("ROSThread").info("rospy.spin() exited")

__ros_thread = None
def get_ros_thread():
    import rosgobject
    if not rosgobject.__ros_thread:
        rosgobject.__ros_thread = _ROSThread()
    return rosgobject.__ros_thread

def main_quit(*args):
    rospy.signal_shutdown('quit by GUI')
    Gtk.main_quit()
    return False


