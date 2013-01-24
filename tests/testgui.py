#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import argparse

import rospy
import std_msgs.msg

import rosgobject.managers
import rosgobject.gtk

from gi.repository import Gtk, GObject, Gdk

GObject.threads_init()
Gdk.threads_init()

class CompositeTopicServiceWidget:
    def __init__(self, basepath):
        pass

class UI:
    def __init__(self):
        self._w = Gtk.Window()
        self._w.set_default_size(600,400)
        self._w.connect("delete-event", Gtk.main_quit)
        self._grid = Gtk.Grid()
        self._w.add(self._grid)

        self._add_topic_widgets()
        self._add_treeviews()

        self._w.show_all()

    def _add_topic_widgets(self):
        w = rosgobject.gtk.GtkEntryTopicDisplay(
                "/testnode/string",
                std_msgs.msg.String)
        self._grid.insert_row(0)
        self._grid.attach(Gtk.Label(w.name,xalign=0.0),0,0,1,1)
        self._grid.attach(w,1,0,1,1)

        w = rosgobject.gtk.GtkEntryTopicDisplay(
                "/testnode/float",
                std_msgs.msg.Float32,
                format_func=lambda x:"{0:5.1f} fps".format(x.data),
                name="Framerate")
        self._grid.insert_row(0)
        self._grid.attach(Gtk.Label(w.name,xalign=0.0),0,0,1,1)
        self._grid.attach(w,1,0,1,1)

        w = rosgobject.gtk.GtkComboBoxTextPublisher(
                "~pub_string",
                std_msgs.msg.String,
                ("foo","bar","baz"),
                name="Publish String")
        self._grid.insert_row(0)
        self._grid.attach(Gtk.Label(w.name,xalign=0.0),0,0,1,1)
        self._grid.attach(w,1,0,1,1)

        w = rosgobject.gtk.GtkComboBoxTextPublisher(
                "~pub_float",
                std_msgs.msg.Float32,
                ("-1","2.3","1234"),
                type_conv_func=float,
                name="Publish Float")
        self._grid.insert_row(0)
        self._grid.attach(Gtk.Label(w.name,xalign=0.0),0,0,1,1)
        self._grid.attach(w,1,0,1,1)

    def _add_treeviews(self):
        manager = rosgobject.managers.ROSNodeManager()

        model = rosgobject.gtk.ROSTopicListModel(manager, "std_msgs/Float32")
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.props.hexpand = True
        self._grid.insert_row(0)
        self._grid.attach(tree,0,0,2,1)

        model = rosgobject.gtk.ROSNodeTreeModel(manager)
        tree = Gtk.TreeView(model)
        tree.append_column(
                Gtk.TreeViewColumn("Path", Gtk.CellRendererText(),
                text=model.COL_ID_PATH))
        tree.append_column(
                Gtk.TreeViewColumn("Type", Gtk.CellRendererText(),
                text=model.COL_ID_TYPE))
        tree.props.hexpand = True
        self._grid.insert_row(0)
        self._grid.attach(tree,0,0,2,1)

if __name__ == "__main__":
    rospy.init_node("testgui", anonymous=True)
    ros = rosgobject.get_ros_thread() #ensure ros is spinning
    u = UI()
    Gtk.main()

