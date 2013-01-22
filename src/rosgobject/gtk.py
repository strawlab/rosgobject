# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import logging

from gi.repository import GObject, Gtk

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

