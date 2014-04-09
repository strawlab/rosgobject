# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import rospy
import rosnode
import rostopic
import rosgraph.masterapi
import roslaunch.scriptapi
import roslaunch.core

import string
import time
import threading
import logging
import random
import platform

from gi.repository import GObject

LOG = logging.getLogger(__name__)

class ROSNodeManager(GObject.GObject):

    __gsignals__ =  { 
            "node-appeared": (
                GObject.SignalFlags.RUN_LAST, None, [
                str]),  #node path
            "node-disappeared": (
                GObject.SignalFlags.RUN_LAST, None, [
                str]),  #node path
            "topic-appeared": (
                GObject.SignalFlags.RUN_LAST, None, [
                str, str, str]),    #topic path, topic type, node path
            "topic-disappeared": (
                GObject.SignalFlags.RUN_LAST, None, [
                str, str, str]),    #topic path, topic type, node path
            }

    def __init__(self, this_machine=None):
        GObject.GObject.__init__(self)

        self._log = LOG.getChild("ROSNodeManager")

        self._nodes = []
        self._node_topics = {}
        self._topics = {}

        if this_machine is None:
            this_machine = platform.node()

        self._this_machine = this_machine
        self._thread = threading.Thread(target=self._thread_func)
        self._thread.daemon = True
        self._thread.start()

        self._launch = roslaunch.scriptapi.ROSLaunch()
        self._launch.start()

    def _thread_func(self):
        def topic_type(t, topic_types):
            matches = [t_type for t_name, t_type in topic_types if t_name == t]
            if matches:
                return matches[0]
            return 'unknown type'

        self._mastertopic = rosgraph.masterapi.Master('/rostopic')
        while 1:
            nodes = rosnode.get_node_names(); new_nodes = nodes[:]
            old_nodes = self._nodes[:]
            disappeared_nodes = []

            for n in old_nodes:
                try:
                    nodes.remove(n)
                except ValueError:
                    disappeared_nodes.append(n)

            time.sleep(1)
            pubs,subs,_ = self._mastertopic.getSystemState()
            time.sleep(1)
            topic_types = self._mastertopic.getTopicTypes()

            topics = {}
            node_topics = {}
            for t, l in pubs:
                if len(l) >= 1:
                    for n in l:
                        ttype = topic_type(t, topic_types)
                        try:
                            node_topics[n].append((t, ttype))
                        except KeyError:
                            node_topics[n] = [(t, ttype)]
                        topics[t] = (ttype,n)
            self._node_topics = node_topics

            new_topics = topics.copy()
            old_topics = self._topics.copy()
            disappeared_topics = {}
            for n in old_topics:
                try:
                    topics.pop(n)
                except KeyError:
                    disappeared_topics[n] = old_topics[n]

            time.sleep(1)

            #sometimes topics appear before nodes
            for t in new_topics:
                node = new_topics[t][1]
                if node not in new_nodes[:]:
                    self._log.info("Topic appeared before node... marking node %s new" % node)
                    new_nodes.append(node)
                    nodes.append(node)

            #the order of emission here is so that we can maintain a tree model easier, that is
            #so the signal order is
            # new node -> new topic(s)
            #or
            # died topic(s) -> died node

            #anything left in nodes has appeared since last time
            for n in nodes:
                GObject.idle_add(GObject.GObject.emit,self,"node-appeared",n)
                self._log.info("Node Appeared: %s" % n)
            #anything left in topics has appeared since last time
            for t in topics:
                GObject.idle_add(GObject.GObject.emit,self,"topic-appeared",t,*topics[t])
                self._log.info("Topic Appeared: %s" % t)
            #anything in disappeared has died since last time
            for t in disappeared_topics:
                GObject.idle_add(GObject.GObject.emit,self,"topic-disappeared",t,*disappeared_topics[t])
                self._log.info("Topic Died: %s" % t)
            #anything in disappeared has died since last time
            for n in disappeared_nodes:
                GObject.idle_add(GObject.GObject.emit,self,"node-disappeared",n)
                self._log.info("Node Died: %s" % n)

            self._nodes = new_nodes
            self._topics = new_topics

    @staticmethod
    def random_node_name(N, prefix=""):
        return prefix + ''.join(random.choice(string.ascii_uppercase + string.digits) for x in range(N))

    def has_topic(self, name):
        return name in self._topics

    def kill_nodes(self, *nodes):
        rosnode.kill_nodes(nodes)

    def get_node_topics(self, name):
        return self._node_topics.get(name,[])

    def launch_node(self, package, node_type, machine_name="", name="", args='', env_args=None, remap_args=None, **rosparams):

        n = roslaunch.core.Node(
                package=package,
                node_type=node_type,
                name=name or self.random_node_name(10, package),
                machine_name=machine_name or self._this_machine,
                args=args,
                env_args=env_args or [],
                remap_args=remap_args or [])

        self._log.info("Launching node: %s %s (name %s, on machine %s, args %r, env_args %r, remap_args %r)" % (
                    n.package,n.type,
                    n.name,n.machine_name,
                    n.args,n.env_args,n.remap_args))

        for k,v in rosparams.items():
            self._log.info("Setting ROS param: %s = %s" % (k, v))
            rospy.set_param(k,v)

        self._launch.launch(n)

    def find_topic(self, topic_type='sensor_msgs/Image'):
        return rostopic.find_by_type(topic_type)

