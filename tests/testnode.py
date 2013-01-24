#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-

import roslib; roslib.load_manifest('rosgobject')

import rospy
import std_msgs.msg
import std_srvs.srv

import random
import string

class Tester:
    def __init__(self):
        self._pubs = []
        self._pubst = {}
        for n,t in (("bool",std_msgs.msg.Bool),("float",std_msgs.msg.Float32),
                    ("int",std_msgs.msg.Int32),("string",std_msgs.msg.String)):
            p = rospy.Publisher("~%s"%n,t)
            self._pubs.append( p )
            self._pubst[p] = t

        self._fs = rospy.Publisher("~foo",std_msgs.msg.Float32)
        s = rospy.Service('~set_foo', std_srvs.srv.Empty, self._change_foo)

        self._timer = rospy.Timer(rospy.Duration(0.2),
                                    self._new_msg)

    def _change_foo(self, req):
        self._fs.publish(random.random())
        return std_srvs.srv.EmptyResponse()

    def _new_msg(self, evt):
        p = random.choice( self._pubs )
        msg = self._pubst[p]()
        if type(msg.data) == bool:
            msg.data = bool(random.randint(0,1))
        elif type(msg.data) == int:
            msg.data = random.randint(0,10)
        elif type(msg.data) == float:
            msg.data = random.random()
        elif type(msg.data) == str:
            msg.data = ''.join(random.choice(string.ascii_letters) for i in range(10))
        p.publish(msg)
        print msg

if __name__ == "__main__":
    rospy.init_node('testnode')
    t = Tester()
    rospy.spin()

