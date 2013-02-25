#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import argparse
import logging
import traceback

import rospy
import std_msgs.msg

import rosgobject.managers
import rosgobject.gtk

from gi.repository import Gtk, GObject, Gdk

GObject.threads_init()
Gdk.threads_init()

class UI:
    def __init__(self):
        self._w = Gtk.Window()
        self._w.set_default_size(600,400)
        self._w.connect("delete-event", rosgobject.main_quit)
        self._grid = Gtk.Grid(orientation=Gtk.Orientation.VERTICAL)

        self._manager = rosgobject.managers.ROSNodeManager()
        self._build_ui()

        self._w.add(self._grid)
        self._w.show_all()


    def _build_ui(self):
        
        self._grid.add(Gtk.Label("Show Only std_msgs/Float32 Topics"))
        model = rosgobject.gtk.ROSTopicListModel(self._manager, "std_msgs/Float32")
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.props.hexpand = True
        self._grid.add(tree)

        self._grid.add(Gtk.Label("Show Nodes And Their Topics"))
        model = rosgobject.gtk.ROSNodeTreeModel(self._manager)
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.append_column(
                Gtk.TreeViewColumn("Type", Gtk.CellRendererText(),
                text=model.COL_ID_TYPE))
        tree.props.hexpand = True
        self._grid.add(tree)


if __name__ == "__main__":
    rospy.init_node("testgui", anonymous=True)
    rosgobject.get_ros_thread() #ensure ros is spinning
    rosgobject.add_console_logger()
    try:
        u = UI()
        Gtk.main()
    except:
        rospy.logfatal("crash in UI:\n%s" % traceback.format_exc())
    finally:
        rosgobject.main_quit()

