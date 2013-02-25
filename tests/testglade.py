# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import os.path
import traceback

import rospy
import std_msgs.msg

import rosgobject.wrappers

from gi.repository import Gtk

class UI:
    def __init__(self):
        me = os.path.dirname(os.path.abspath(__file__))
        self._ui = Gtk.Builder()
        self._ui.add_from_file(os.path.join(me,"main.ui"))

        self._build_ui()
        
        w = self._ui.get_object("window1")
        w.connect("delete-event", rosgobject.main_quit)
        w.show_all()

    def _build_ui(self):
        rosgobject.wrappers.GtkEntryTopicWidget(
                    widget=self._ui.get_object("entry1"),
                    nodepath="/testnode/float",
                    msgclass=std_msgs.msg.Float32,
                    format_func=lambda x:"{0:5.1f} fps".format(x.data))
        rosgobject.wrappers.GtkEntryTopicWidget(
                    widget=self._ui.get_object("entry2"),
                    nodepath="/testnode/string",
                    msgclass=std_msgs.msg.String)





if __name__ == "__main__":
    rospy.init_node("testglade", anonymous=True)
    rosgobject.get_ros_thread() #ensure ros is spinning
    rosgobject.add_console_logger()
    try:
        u = UI()
        Gtk.main()
    except:
        rospy.logfatal("crash in UI:\n%s" % traceback.format_exc())
    finally:
        rosgobject.main_quit()


