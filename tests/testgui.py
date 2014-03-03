# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import argparse
import logging
import traceback

import rospy
import std_msgs.msg
import std_srvs.srv

import rosgobject.managers
import rosgobject.gtk
from rosgobject.wrappers import *

from gi.repository import Gtk

class UI:
    def __init__(self):
        self._w = Gtk.Window()
        self._w.connect("delete-event", rosgobject.main_quit)
        self._grid = Gtk.Grid()
        self._widgets = []

        self._manager = rosgobject.managers.ROSNodeManager()
        self._build_ui()

        self._w.add(self._grid)
        self._w.show_all()

    def _add_widget(self, w):
        self._widgets.append(w)
        self._grid.insert_row(0)
        self._grid.attach(w.label,0,0,1,1)
        self._grid.attach(w.widget,1,0,1,1)

    def _build_ui(self):
        w = GtkEntryTopicWidget(
                nodepath="/testnode/string",
                msgclass=std_msgs.msg.String)
        self._add_widget(w)
        w = GtkEntryTopicWidget(
                nodepath="/testnode/float",
                msgclass=std_msgs.msg.Float32)
        self._add_widget(w)
        w = GtkEntryTopicWidget(
                nodepath="/testnode/int",
                msgclass=std_msgs.msg.Int32)
        self._add_widget(w)

        w = GtkEntryTopicWidget(
                nodepath="/testnode/float",
                msgclass=std_msgs.msg.Float32,
                format_func=lambda x:"{0:5.1f} fps".format(x.data),
                name="Framerate")
        self._add_widget(w)

        w = GtkSwitchTopicWidget(
                nodepath="/testnode/bool",
                msgclass=std_msgs.msg.Bool,
                name="Power")
        self._add_widget(w)

        w = GtkImageTopicWidget(
                nodepath="/testnode/bool",
                msgclass=std_msgs.msg.Bool)
        self._add_widget(w)

        w = GtkComboBoxTextPublisherWidget(
                options=("foo","bar","baz"),
                nodepath="~pub_string",
                msgclass=std_msgs.msg.String)
        self._add_widget(w)

        w = GtkSpinButtonPublisherWidget(
                min=10,
                max=100,
                step=0.1,
                nodepath="/some_float",
                msgclass=std_msgs.msg.Float32,
                name="Publish a Float")
        self._add_widget(w)

        w = GtkButtonPublisherWidget(
                nodepath="/some_uint",
                msgclass=std_msgs.msg.UInt32,
                name="Click to Publish")
        self._add_widget(w)

        w = GtkButtonPublisherWidget(
                widget=Gtk.ToggleButton("Toggle!"),
                nodepath="/some_bool",
                msgclass=std_msgs.msg.Bool)
        self._add_widget(w)

        w = GtkEntryViewParam(
                nodepath="/param_b")
        self._add_widget(w)
        w = GtkSpinButtonParam(
                nodepath="/param_b",
                min=0.5,
                max=5,
                step=0.01)
        self._add_widget(w)
        w = GtkEntryViewParam(
                nodepath="/param_a")
        self._add_widget(w)
        w = GtkEntryChangeParam(
                nodepath="/param_a")
        self._add_widget(w)
        w = GtkEntryChangeParam(
                nodepath="/param_c",
                create="created")
        self._add_widget(w)

        w = GtkButtonKillNode(
                nodepath="/newtestnode",
                nodemanager=self._manager,
                name="Kill /newtestnode")
        self._add_widget(w)

        w = GtkButtonStartNode(
                nodepath="newtestnode",
                nodemanager=self._manager,
                package="rosgobject",
                node_type="testnode.py",
                name="Start /newtestnode")
        self._add_widget(w)

        w = GtkButtonStartNode(
                nodemanager=self._manager,
                package="rosgobject",
                node_type="testnode.py",
                name="Start /newtestnode (launch cb)",
                launch_callback=lambda : {"nodepath":"newtestnode"})
        self._add_widget(w)

        w = GtkButtonSetServiceWidget(
                nodepath="/testnode/empty_service",
                srvclass=std_srvs.srv.Empty,
                name="Call /testnode/empty_service")
        self._add_widget(w)
        w = GtkEntryTopicWidget(
                nodepath="/testnode/change_on_service_call",
                msgclass=std_msgs.msg.Float32)
        self._add_widget(w)

if __name__ == "__main__":
    rosgobject.init_node("testgui", anonymous=True)
    u = UI()
    rosgobject.spin()

