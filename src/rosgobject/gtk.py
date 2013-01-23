# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import logging
import math

from gi.repository import GObject, Gtk

from rosgobject.wrappers import SubscriberGObject

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

class GtkEntryTopicDisplay(Gtk.Entry):
    def __init__(self, nodepath, msgclass, format_func=None, **kwargs):
        Gtk.Entry.__init__(self, **kwargs)
        self._log = LOG.getChild("TopicDisplay+%s" % nodepath)
        self._sub = SubscriberGObject(nodepath, msgclass)
        self._sub.connect("message", self._on_message)
        self._format_func = format_func or str

    def format_message(self, msg):
        try:
            txt = self._format_func(msg)
        except:
            self._log.warn("Error calling format function", exc_info=True)
            txt = str(msg)
        return txt

    def _on_message(self, sub, msg):
        self.set_text(self.format_message(msg))

class ServiceWidget:

    widget = None
    label = None

    def __init__(self, servicegobj):
        servicegobj.connect("node-error", self._on_node_error)
        servicegobj.connect("node-acquired", self._on_node_available)
        if self.widget:
            self.widget.set_sensitive(False)

    def _on_node_available(self, node, name):
        if self.widget:
            self.widget.set_sensitive(True)

    def _on_node_error(self, go, name, res):
        pass

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

