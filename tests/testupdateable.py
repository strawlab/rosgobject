import roslib; roslib.load_manifest('rosgobject')

import sys
import random

from gi.repository import Gtk

import rosgobject
from rosgobject.gtk import UpdateableGtkSpinButton, UpdateableGtkSwitch

def _clicked_change_sb(btn,_sb):
    _sb.set_value(10 + random.random()*10)
def _clicked_change_sw(btn,_sw):
    _sw.set_active(random.randint(0,1) == 1)

if __name__ == "__main__":

    rosgobject.init_node("testupdateable")

    w = Gtk.Window()
    w.connect("delete-event", rosgobject.main_quit)
    g = Gtk.Grid()
    w.add(g)

    sb = UpdateableGtkSpinButton.new_with_range(10,30,1)
    sb.connect("value-changed", lambda *args: sys.stdout.write("sb changed\n"))
    b = Gtk.Button("<-- Random Change")
    b.connect("clicked", _clicked_change_sb, sb)
    g.add(sb)
    g.add(b)

    sw = UpdateableGtkSwitch()
    sw.connect("notify::active", lambda *args: sys.stdout.write("sw changed\n"))
    b = Gtk.Button("<-- Random Change")
    b.connect("clicked", _clicked_change_sw, sw)
    g.add(sw)
    g.add(b)

    w.show_all()

    rosgobject.spin()
