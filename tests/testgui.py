#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import argparse

import rospy
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
        self._grid = Gtk.Grid(orientation=Gtk.Orientation.HORIZONTAL)
        self._w.add(self._grid)

        self._add_treeviews()

        self._w.show_all()

    def _add_treeviews(self):
        manager = rosgobject.managers.ROSNodeManager()

        model = rosgobject.gtk.ROSTopicListModel(manager, "std_msgs/Float32")
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.props.hexpand = True
        tree.props.vexpand = True
        self._grid.add(tree)

        model = rosgobject.gtk.ROSNodeTreeModel(manager)
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.append_column(
                Gtk.TreeViewColumn("Type", Gtk.CellRendererText(),
                text=model.COL_ID_TYPE))
        tree.props.hexpand = True
        tree.props.vexpand = True
        self._grid.add(tree)

if __name__ == "__main__":
    rospy.init_node("testgui", anonymous=True)
    ros = rosgobject.get_ros_thread() #ensure ros is spinning
    u = UI()
    Gtk.main()
