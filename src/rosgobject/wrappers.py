# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import rospy

import logging
import threading
import Queue
import time

from gi.repository import GObject

LOG = logging.getLogger(__name__)

class _ServiceThread(threading.Thread):
    def __init__(self, servicecls, servicename, freq, acqback, callback, errback):
        threading.Thread.__init__(self, name="Thread-%s" % servicename)
        self.servicecls = servicecls
        self.servicename = str(servicename)
        self._freq = freq
        self._ab = acqback
        self._cb = callback
        self._eb = errback
        self._log = LOG.getChild("ServiceThread")
        self.daemon = True
        self.queue = Queue.Queue()

    def run(self):
        self._log.info("waiting for service: %s" % self.servicename)
        rospy.wait_for_service(self.servicename)
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
        LOG.getChild("ServiceGObject").info("wrapping %s %s @ %dHz" % (servicecls, servicename, freq))
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

    nodepath = None

    def __init__(self, nodepath, msg, subscribe=True):
        GObject.GObject.__init__(self)
        LOG.getChild("SubscriberGObject").info("wrapping %s msg %s" % (nodepath, msg))
        self.nodepath = nodepath
        self._msg = msg
        self._sub = None
        if subscribe:
            self.subscribe()

    def subscribe(self):
        if self._sub:
            self.unsubscribe()
        self._sub = rospy.Subscriber(self.nodepath, self._msg, self._cb)

    def unsubscribe(self):
        if self._sub:
            self._sub.unregister()
    
    def _cb(self, data):
        GObject.idle_add(GObject.GObject.emit,self,"message",data)

