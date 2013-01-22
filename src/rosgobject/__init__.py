# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib
roslib.load_manifest('rospy')

import rospy

import logging
import threading

LOG = logging.getLogger(__name__)

class _ROSThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, name="Thread-rospy.spin")
        self.daemon = True
        LOG.getChild("ROSThread").info("creating thread")
        self.start()

    def run(self):
        LOG.getChild("ROSThread").info("calling rospy.spin")
        rospy.spin()
        LOG.getChild("ROSThread").info("rospy.spin() exited")

__ros_thread = None
def get_ros_thread():
    import rosgobject
    if not rosgobject.__ros_thread:
        rosgobject.__ros_thread = _ROSThread()
    return rosgobject.__ros_thread

