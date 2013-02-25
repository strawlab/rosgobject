# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import os.path
import traceback

import rospy
import std_msgs.msg

import rosgobject.wrappers
import rosgobject.graph

from gi.repository import Gtk

class UI:
    def __init__(self):
        me = os.path.dirname(os.path.abspath(__file__))
        self._w = Gtk.Window()
        self._build_ui()
        self._w.connect("delete-event", rosgobject.main_quit)
        self._w.show_all()

    def _build_ui(self):
        vb = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self._g = rosgobject.graph.Plotter(
                    nodepath="/testnode/float",
                    msgclass=std_msgs.msg.Float32)
        self._g.ax.set_xlabel('time (s)')
        self._g.ax.set_ylabel('/tesnode/float')
        vb.pack_start(self._g, True, True, 0)

        vb.pack_start(Gtk.Label("X Control"), False, False, 0)
        vb.pack_start(self._g.xlabel_controller, False, False, 0)

        vb.pack_start(Gtk.Label("Y Control"), False, False, 0)
        vb.pack_start(self._g.ylabel_controller, False, False, 0)

        self._w.add(vb)

if __name__ == "__main__":
    rospy.init_node("testgraph", anonymous=True)
    rosgobject.get_ros_thread() #ensure ros is spinning
    rosgobject.add_console_logger()
    try:
        u = UI()
        Gtk.main()
    except:
        rospy.logfatal("crash in UI:\n%s" % traceback.format_exc())
    finally:
        rosgobject.main_quit()

