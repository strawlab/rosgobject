# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')
import roslib.scriptutil
import roslib.names
import roslib.message
import rosgraph.masterapi
import rospy

import xmlrpclib
import logging
import threading
import socket
import Queue
import time
import sys

from gi.repository import GObject

import rosgobject

LOG = logging.getLogger(__name__)

## from rxplot
##
## subroutine for getting the topic type
## (nearly identical to rostopic._get_topic_type, except it returns rest of name instead of fn)
## @return str, str, str: topic type, real topic name, and rest of name referenced
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def _get_topic_type(topic):
    code, msg, val = roslib.scriptutil.get_master().getPublishedTopics('/', '/')
    if code != 1:
        raise Exception("unable to get list of topics from master")
    matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t+'/')]
    if matches:
        t, t_type = matches[0]
        if t_type == roslib.names.ANYTYPE:
            return None, None, None
        if t_type == topic:
            return t_type, None
        return t_type, t, topic[len(t):]
    else:
        return None, None, None

## from rxplot
##
## get the topic type (nearly identical to rostopic.get_topic_type, except it doesn't return a fn)
## @return str, str, str: topic type, real topic name, and rest of name referenced
## if the \a topic points to a field within a topic, e.g. /rosout/msg
def get_topic_type(topic):
    topic_type, real_topic, rest = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, rest
    else:
        print >> sys.stderr, "WARNING: topic [%s] does not appear to be published yet"%topic
        while not rospy.is_shutdown():
            topic_type, real_topic, rest = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, rest
            else:
                time.sleep(0.1)
        return None, None, None


## from rostopic.py
##
## does not require topics be published yet
def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except xmlrpclib.Fault:
        #TODO: remove, this is for 1.1
        sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
        val = master.getPublishedTopics('/')
    return val


def _get_topic_type_subscribers(error_callback, topic):
    master = rosgraph.masterapi.Master('/rostopic')
    try:
        state = master.getSystemState()
        pubs, subs, _ = state
        topic_types = _master_get_topic_types(master)
        subs = [x[0] for x in subs if x[0] == topic]
        pubs = [x[0] for x in pubs if x[0] == topic]
        topic_types = [x[1] for x in topic_types if x[0] == topic]
    except socket.error:
        error_callback("Could not communicate with master")

    try:
        return topic_types[0],subs[0]
    except IndexError:
        error_callback("Noone is subscribed to %s yet" % topic)

## from rxplot
##
## requires topics to be published
def parse_topics(error_callback, topics, nodename, published=True):

    topic_list = []
    range_list = []
    for t in topics:
        try:
            sub_t,tmin,tmax = t.split(',')
            tmin = float(tmin)
            tmax = float(tmax)
        except ValueError:
            sub_t = t
            tmin = 0
            tmax = 1000

        range_list.append( (tmin,tmax) )

        c_topics = []
        # check for shorthand '/foo/field1:field2:field3'
        if ':' in sub_t:
            base = sub_t[:sub_t.find(':')]
            # the first prefix includes a field name, so save then strip it off
            c_topics.append(base)
            if not '/' in base:
                error_callback("%s must contain a topic and field name"%sub_t)
            base = base[:base.rfind('/')]

            # compute the rest of the field names
            fields = sub_t.split(':')[1:]
            c_topics.extend(["%s/%s"%(base, f) for f in fields if f])
        else:
            c_topics.append(sub_t)

        # #1053: resolve command-line topic names
        c_topics = [roslib.scriptutil.script_resolve_name(nodename, n) for n in c_topics]

        topic_list.extend(c_topics)

    #flatten for printing
    print_topic_list = []
    for l in topic_list:
        if type(l) == list:
            print_topic_list.extend(l)
        else:
            print_topic_list.append(l)

    #look up the list of classes
    nodepaths, msgclasses, dataattrs = [],[],[]
    for t in topic_list:
        if published:
            #look for published nodes and block if a node is not
            #published
            clsname,path,attr = get_topic_type(t)
            dataattrs.append(attr[1:])
        else:
            #look for subscribed topics
            clsname,path = _get_topic_type_subscribers(error_callback, t)
            dataattrs.append('data')

        nodepaths.append(path)
        msgclasses.append(roslib.message.get_message_class(clsname))

    return nodepaths, msgclasses, dataattrs, range_list

class _ParameterPollThread(threading.Thread):
    def __init__(self, path, freq, existscallback, changecallback, create=None):
        threading.Thread.__init__(self, name="ParameterThread-%s" % path)
        self._path = path
        self._freq = freq
        self._existscallback = existscallback
        self._changecallback = changecallback
        self._create = create
        self._log = LOG.getChild("ParameterThread")
        self._log.info("wrapping %s @ %dHz" % (self._path, self._freq))
        self._val = None
        self.daemon = True
        self.queue = Queue.Queue()

    def run(self):
        while 1:
            try:
                path,val = self.queue.get(block=True, timeout=1.0/self._freq)
                with rosgobject.get_parameter_lock():
                    rospy.set_param(path,val)
            except Queue.Empty:
                pass

            with rosgobject.get_parameter_lock():
                try:
                    val = rospy.get_param(self._path)
                    has_param = True
                except KeyError:
                    val = None
                    has_param = False

            create_param = False
            if has_param:
                if self._val is None:
                    self._existscallback(self._path)
                if val != self._val:
                    self._changecallback(self._path, val)
                    self._val = val
            elif self._create is not None:
                with rosgobject.get_parameter_lock():
                    rospy.set_param(self._path, self._create)
                    create_param = True

            if create_param:
                self._log.info("set parameter %s to %r" % (self._path, self._create))
                self._existscallback(self._path)
                self._changecallback(self._path, self._create)

class _ServiceThread(threading.Thread):
    def __init__(self, servicecls, servicename, freq, acqback, callback, errback):
        threading.Thread.__init__(self, name="ServiceThread-%s" % servicename)
        self.servicecls = servicecls
        self.servicename = str(servicename)
        self._freq = freq
        self._ab = acqback
        self._cb = callback
        self._eb = errback
        self._log = LOG.getChild("ServiceThread")
        self._log.info("wrapping %s %s @ %dHz" % (self.servicecls, self.servicename, self._freq))
        self.daemon = True
        self.queue = Queue.Queue()

    def run(self):
        self._log.info("waiting for service: %s" % self.servicename)
        try:
            rospy.wait_for_service(self.servicename)
        except rospy.exceptions.ROSInterruptException:
            self._log.debug("quit while waiting for service")
            return

        func = rospy.ServiceProxy(self.servicename, self.servicecls)
        self._log.info("acquired service: %s" % self.servicename)
        self._ab(self.servicename)
        while 1:
            args = tuple()
            if self._freq > 0.0:
                time.sleep(1.0/self._freq)
            else:
                args = self.queue.get(block=True)
                self._log.info("calling %s function with args: %s" % (self.servicename,str(args)))
            try:
                self._log.debug("calling %s function with args: %s" % (self.servicename,str(args)))
                res = func(*args)
                self._cb(self.servicename, res)
            except rospy.ServiceException, e:
                self._log.info("error calling %s function" % self.servicename,exc_info=True)
                if self._eb:
                    self._eb(self.servicename, str(e))


class ServiceGObject(GObject.GObject):

    __gsignals__ =  { 
            "node-acquired": (
                GObject.SignalFlags.RUN_LAST, None, [
                str]),
            "node-error": (
                GObject.SignalFlags.RUN_LAST, None, [
                str,str]),
            "node-message": (
                GObject.SignalFlags.RUN_LAST, None, [
                str,object])
            }

    nodepath = None
    node = None

    def __init__(self, servicecls, servicename, freq):
        GObject.GObject.__init__(self)
        self.node = _ServiceThread(
                        servicecls, servicename,
                        freq, self._service_acquired, self._service_ok, self._service_err)
        self.nodepath = servicename
        self.node.start()

    @property
    def queue(self):
        return self.node.queue

    def _service_acquired(self, name):
        GObject.idle_add(GObject.GObject.emit,self,"node-acquired",name)

    def _service_ok(self, name, result):
        GObject.idle_add(GObject.GObject.emit,self,"node-message",name,result)

    def _service_err(self, name, result):
        GObject.idle_add(GObject.GObject.emit,self,"node-error",name,result)

class SubscriberGObject(GObject.GObject):

    __gsignals__ =  { 
            "message": (
                GObject.SignalFlags.RUN_LAST, None, [
                object])
            }

    def __init__(self, nodepath, msg, subscribe=True):
        GObject.GObject.__init__(self)
        self._log = LOG.getChild("SubscriberGObject")
        self._msg = msg
        self._sub = None
        self._nodepath = nodepath
        if subscribe:
            self.subscribe()

    @property
    def nodepath(self):
        if self._sub is not None:
            return self._sub.resolved_name
        return self._nodepath

    def subscribe(self):
        if self._sub:
            self.unsubscribe()
        self._log.info("subscribing to %s (%s)" % (self._nodepath, self._msg))
        self._sub = rospy.Subscriber(self._nodepath, self._msg, self._cb)

    def unsubscribe(self):
        if self._sub:
            self._sub.unregister()
    
    def _cb(self, data):
        GObject.idle_add(GObject.GObject.emit,self,"message",data)

class ParameterGObject(GObject.GObject):

    __gsignals__ =  { 
            "parameter-exists": (
                GObject.SignalFlags.RUN_LAST, None, [
                str]),
            "parameter-changed": (
                GObject.SignalFlags.RUN_LAST, None, [
                str,object])
            }

    nodepath = None

    def __init__(self, path, create=None):
        GObject.GObject.__init__(self)
        self.node = _ParameterPollThread(
                        path, freq=1.0,
                        existscallback=self._parameter_exists,
                        changecallback=self._parameter_changed,
                        create=create)
        self.nodepath = path
        self.node.start()

    @property
    def queue(self):
        return self.node.queue

    def set_param(self, val):
        self.node.queue.put((self.nodepath,val))

    def _parameter_exists(self, path):
        GObject.idle_add(GObject.GObject.emit,self,"parameter-exists", path)

    def _parameter_changed(self, path, val):
        GObject.idle_add(GObject.GObject.emit,self,"parameter-changed", path, val)

