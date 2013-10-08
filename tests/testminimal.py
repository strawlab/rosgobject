# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import std_msgs.msg
import rosgobject

class UI:
    def __init__(self):
        self._sub = rosgobject.Subscriber("/testnode/string",std_msgs.msg.String)
        self._sub.connect("message", self._on_message)

    def _on_message(self, sub, msg):
        print msg.data

if __name__ == "__main__":
    rosgobject.init_node("testminimal")
    u = UI()
    rosgobject.spin()

