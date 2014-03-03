# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')
import std_msgs.msg
import rosgobject.wrappers
from gi.repository import Gtk

if __name__ == "__main__":
    rosgobject.init_node("testminimal")
    w = Gtk.Window()
    w.connect("delete-event", rosgobject.main_quit)
    e = rosgobject.wrappers.GtkEntryTopicWidget(
                nodepath="/testnode/string",
                msgclass=std_msgs.msg.String)
    w.add(e.widget) #note the .widget!
    w.show_all()
    rosgobject.spin()

