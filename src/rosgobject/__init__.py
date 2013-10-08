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
        self.daemon = False #insist we must be shut down properly (use rosgobject.main_quit)
        self._log = LOG.getChild("ROSThread")
        self._log.debug("creating rospy.spin() thread")
        self.start()

    def _check_running(self):
        #this lets us die when the ROS thread dies
        if not self.is_alive():
            Gtk.main_quit()
            return False
        return True

    def run(self):
        GLib.timeout_add(1000/10, self._check_running)
        rospy.spin()
        self._log.info("rospy.spin() exited")

__ros_thread = None
def get_ros_thread():
    import rosgobject
    if not rosgobject.__ros_thread:
        rosgobject.__ros_thread = _ROSThread()
    return rosgobject.__ros_thread

__parameter_lock = threading.Lock()
def get_parameter_lock():
    return __parameter_lock

def main_quit(*args):
    rospy.signal_shutdown('quit by GUI')
    Gtk.main_quit()
    return False

def add_console_logger(level=logging.INFO, logger=''):
    #also redirect logging to stderr
    console = logging.StreamHandler()
    console.setLevel(level)
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger(logger).addHandler(console)

#rospy API compatibility
from . import core
Subscriber = core.SubscriberGObject

def init_node(*args, **kwargs):
    rospy.init_node(*args, **kwargs)
    get_ros_thread()

def spin():
    try:
        Gtk.main()
    except:
        rospy.logfatal("crash in UI:\n%s" % traceback.format_exc())
    finally:
        main_quit()    


