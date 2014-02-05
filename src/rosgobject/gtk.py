# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import logging
import math

import rospy

from gi.repository import GObject, Gtk, Gdk

from rosgobject.core import SubscriberGObject

LOG = logging.getLogger(__name__)

class ROSNodeTreeModel(Gtk.TreeStore):

    COL_ID_PATH =  0
    COL_ID_TYPE =  1

    def __init__(self, rosnodemanager):
        Gtk.TreeStore.__init__(self, str, str)
        self._iters = {}
        self._rosnodemanager = rosnodemanager
        self._rosnodemanager.connect("node-appeared", self._on_node_appeared)
        self._rosnodemanager.connect("node-disappeared", self._on_node_disappeared)
        self._rosnodemanager.connect("topic-appeared", self._on_topic_appeared)
        self._rosnodemanager.connect("topic-disappeared", self._on_topic_disappeared)

        self._log = LOG.getChild("ROSNodeTreeModel")

    def _on_node_appeared(self, nm, nodename):
        self._iters["node_"+nodename] = self.append(None, [nodename, ""])

    def _on_node_disappeared(self, nm, nodename):
        iter_ = self._iters.pop("node_"+nodename)
        self.remove(iter_)

    def _on_topic_appeared(self, nm, topicpath, topictype, nodepath):
        parent = self._iters["node_"+nodepath]
        self._iters["topic_"+topicpath+nodepath] = self.append(parent, [topicpath, topictype])

    def _on_topic_disappeared(self, nm, topicpath, topictype, nodepath):
        iter_ = self._iters.pop("topic_"+topicpath+nodepath)
        self.remove(iter_)

class ROSTopicListModel(Gtk.ListStore):

    COL_ID_PATH =  0
    COL_ID_TYPE =  1
    COL_ID_NODE_PATH =  2

    def __init__(self, rosnodemanager, *only_topic_types):
        Gtk.ListStore.__init__(self, str, str, str)
        self._iters = {}
        self._only_topic_types = only_topic_types
        self._rosnodemanager = rosnodemanager
        self._rosnodemanager.connect("topic-appeared", self._on_topic_appeared)
        self._rosnodemanager.connect("topic-disappeared", self._on_topic_disappeared)

        self._log = LOG.getChild("ROSTopicListModel")

    def _on_topic_appeared(self, nm, topicpath, topictype, nodepath):
        if not self._only_topic_types or topictype in self._only_topic_types:
            self._iters["topic_"+topicpath+nodepath] = self.append([topicpath, topictype, nodepath])

    def _on_topic_disappeared(self, nm, topicpath, topictype, nodepath):
        if not self._only_topic_types or topictype in self._only_topic_types:
            iter_ = self._iters.pop("topic_"+topicpath+nodepath)
            self.remove(iter_)

class UpdateableGtkEntry(Gtk.Entry):
    def __init__(self, *args, **kwargs):
        Gtk.Entry.__init__(self, *args, **kwargs)
        self.add_events(Gdk.EventMask.FOCUS_CHANGE_MASK)
        self._focused = False
        self._ros_txt = ""
        self.connect_after("focus-in-event", self._focus_in)
        self.connect_after("focus-out-event", self._focus_out)

    def _focus_in(self, *args):
        self._focused = True

    def _focus_out(self, *args):
        self.set_icon_from_stock(Gtk.EntryIconPosition.SECONDARY, None)
        self._focused = False
        if self._ros_txt:
            self.set_text(self._ros_txt, take_my_changes=True)

    def set_text(self, txt, take_my_changes=False):
        if take_my_changes:
            Gtk.Entry.set_text(self, txt)
            self._ros_txt = ""
        elif self._focused and (self.get_text() != txt):
            self._ros_txt = txt
            self.set_icon_from_stock(Gtk.EntryIconPosition.SECONDARY,
                                     Gtk.STOCK_INFO)
            self.set_icon_tooltip_text(Gtk.EntryIconPosition.SECONDARY,
                                       "This parameter changed while you were "\
                                       "editing it")
        else:
            self._ros_txt = ""
            Gtk.Entry.set_text(self, txt)

class UpdateableGtkSwitch(Gtk.Switch):
    def __init__(self, *args, **kwargs):
        Gtk.Switch.__init__(self, *args, **kwargs)
        self._changing = False
        self.connect_after("notify::active", self._changed)

    def _changed(self, *args):
        if self._changing:
            self.stop_emission("notify::active")

    def set_active(self, is_active):
        self._changing = True
        Gtk.Switch.set_active(self, is_active)
        self._changing = False

    def connect(self, *args, **kwargs):
        self.connect_after(*args, **kwargs)

class UpdateableGtkSpinButton(Gtk.SpinButton):
    def __init__(self, *args, **kwargs):
        Gtk.SpinButton.__init__(self, *args, **kwargs)
        self._changing = False
        self.connect_after("value-changed", self._changed)

    def _changed(self, *args):
        if self._changing:
            self.stop_emission("value-changed")

    def set_value(self, value):
        self._changing = True
        Gtk.SpinButton.set_value(self, value)
        self._changing = False

    def connect(self, *args, **kwargs):
        self.connect_after(*args, **kwargs)
        
    @staticmethod
    def new_with_range(min,max,step):
        #copied from gtkspinbutton.c
        MAX_DIGITS = 20
        
        adj = Gtk.Adjustment(
                    value=min,lower=min,upper=max,
                    step_increment=step,page_increment=10*step,page_size=0)
        sb = UpdateableGtkSpinButton()

        if math.fabs(step) >= 1.0 or step == 0.0:
            digits = 0
        else:
            digits = abs(int(math.floor(math.log10(math.fabs(step)))))
        if digits > MAX_DIGITS:
            digits = MAX_DIGITS

        sb.configure(adj,step,digits)

        return sb

