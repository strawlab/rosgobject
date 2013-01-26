# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')
import rospy

import logging
import math

from gi.repository import GObject, Gtk

from rosgobject.core import ServiceGObject, SubscriberGObject, ParameterGObject
from rosgobject.gtk import UpdateableGtkSpinButton

LOG = logging.getLogger(__name__)

def get_std_msgs_python_type(msgclass):
    t = msgclass._type
    if "Array" in t:
        raise Exception("Array type autoconverstion not supported")
    elif t == "std_msgs/Bool":
        return bool
    elif t == "std_msgs/String":
        return str
    elif t.startswith("std_msgs/Float"):
        return float
    elif t.startswith("std_msgs/Int"):
        return int
    elif t.startswith("std_msgs/UInt"):
        return int
    else:
        raise Exception("Type conversion not supported")

class _MagicLabel(object):

    _label = None

    @property
    def label(self):
        if not self._label:
            self._label = Gtk.Label(self.name, xalign=0.0)
        return self._label

class ParamWidget(_MagicLabel):

    widget = None
    name = None
    nodepath = None

    def __init__(self, widget, nodepath, name=None, **kwargs):
        self._node = ParameterGObject(nodepath)
        self._node.connect("parameter-exists", self._on_parameter_exists)
        self._node.connect("parameter-changed", self._on_parameter_changed)
        self.widget = widget
        self.nodepath = nodepath
        self.name = name or self.nodepath

    def _on_parameter_exists(self, node, path):
        self.widget.set_sensitive(True)

    def _on_parameter_changed(self, node, path, val):
        self.show_parameter(val)

    def show_parameter(self, val):
        raise NotImplementedError

class GtkEntryViewParam(ParamWidget):
    def __init__(self, format_func=None, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Entry(editable=False, sensitive=False))
        ParamWidget.__init__(self, **kwargs)
        self._format_func = format_func or str

    def show_parameter(self, val):
        self.widget.set_text(self._format_func(val))

class GtkSpinButtonParam(ParamWidget):
    def __init__(self, conv_func=None, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=UpdateableGtkSpinButton.new_with_range(
                                        min=kwargs["min"],
                                        max=kwargs["max"],
                                        step=kwargs["step"]))
            kwargs["widget"].set_sensitive(False)
        ParamWidget.__init__(self, **kwargs)
        self._conv_func = conv_func or float
        self.widget.connect("value-changed", self._on_val_changed)

    def _on_val_changed(self, sb):
        self._node.set_param(self._conv_func(sb.get_value()))

    def show_parameter(self, val):
        self.widget.set_value(self._conv_func(val))

class PublisherWidget(_MagicLabel):

    widget = None
    name = None
    nodepath = None
    conv_func = None

    def __init__(self, widget, nodepath, msgclass, conv_func=None, latch=False, name=None, **kwargs):
        self._msgclass = msgclass
        self._pub = rospy.Publisher(nodepath, msgclass, latch=latch)
        self.widget = widget
        self.nodepath = self._pub.resolved_name
        self.name = name or self.nodepath
        self.conv_func = conv_func or get_std_msgs_python_type(msgclass)

    def publish_message(self, *args):
        msg = self._msgclass(*args)
        self._pub.publish(msg)

class GtkComboBoxTextPublisherWidget(PublisherWidget):
    def __init__(self, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.ComboBoxText())
        PublisherWidget.__init__(self, **kwargs)
        for o in kwargs.get("options", []):
            self.widget.append_text(o)
        self.widget.connect("changed", self._on_combo_changed)

    def _on_combo_changed(self, cb):
        self.publish_message(self.conv_func(cb.get_active_text()))

class GtkSpinButtonPublisherWidget(PublisherWidget):
    def __init__(self, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.SpinButton.new_with_range(
                                        min=kwargs["min"],
                                        max=kwargs["max"],
                                        step=kwargs["step"]))
        PublisherWidget.__init__(self, **kwargs)
        self.widget.connect("value-changed", self._on_val_changed)

    def _on_val_changed(self, sb):
        self.publish_message(self.conv_func(sb.get_value()))

class GtkButtonPublisherWidget(PublisherWidget):
    def __init__(self, **kwargs):
        _add_name = False
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Button())
            _add_name = True
        PublisherWidget.__init__(self, **kwargs)
        if _add_name:
            self.widget.set_label(self.name)
        self.widget.connect("clicked", self._on_clicked)

    def _on_clicked(self, btn):
        try:
            active = btn.props.active
        except AttributeError: #not a toggle/checkbutton
            active = True
        self.publish_message(self.conv_func(active))

class ServiceWidget(_MagicLabel):

    widget = None
    name = None
    nodepath = None

    def __init__(self, servicegobj, widget=None, name=None):
        self.widget = widget
        self.nodepath = servicegobj.nodepath
        self.name = name or self.nodepath
        self._node = servicegobj

        self._log = LOG.getChild("ServiceWidget+%s" % self.nodepath)
        self.set_sensitive(False)

        self._node.connect("node-error", self._on_node_error)
        self._node.connect("node-acquired", self._on_node_available)

    def _on_node_available(self, node, name):
        self.set_sensitive(True)

    def _on_node_error(self, go, name, res):
        self._log.warn("error making service call to %s: %s" % (name,res))

    def set_sensitive(self, v):
        if self.widget is not None:
            self.widget.set_sensitive(v)

    def service_call(self, *args):
        self._node.queue.put(args)

class GtkButtonSetServiceWidget(ServiceWidget):
    def __init__(self, nodepath, srvclass, **kwargs):
        _add_name = False
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Button())
            _add_name = True
        gobj = ServiceGObject(srvclass, nodepath, 0.0)
        ServiceWidget.__init__(self, gobj, **kwargs)
        if _add_name:
            self.widget.set_label(self.name)
        self.widget.connect("clicked", self._on_clicked)

    def _on_clicked(self, btn):
        self.service_call()

class TopicWidget(_MagicLabel):

    widget = None
    name = None
    nodepath = None

    def __init__(self, widget, nodepath, msgclass, name=None):
        self._log = LOG.getChild("TopicWidget+%s" % nodepath)
        self._sub = SubscriberGObject(nodepath, msgclass)
        self._sub.connect("message", self._on_message)
        self.widget = widget
        self.nodepath = self._sub.nodepath
        self.name = name or self.nodepath

    def _on_message(self, sub, msg):
        self.set_message(msg)

    def set_message(self, msg):
        raise NotImplementedError

class GtkEntryTopicWidget(TopicWidget):
    def __init__(self, format_func=None, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Entry())
        TopicWidget.__init__(self, **kwargs)
        self._format_func = format_func or (lambda x: str(x.data))

    def set_message(self, msg):
        self.widget.set_text(self._format_func(msg))

class GtkSwitchTopicWidget(TopicWidget):
    def __init__(self, getter_func=None, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Switch(sensitive=False))
        TopicWidget.__init__(self, **kwargs)
        self._getter_func = getter_func or (lambda x: x.data)

    def set_message(self, msg):
        on = self._getter_func(msg)
        self.widget.set_active(True if on else False)

class GtkImageTopicWidget(TopicWidget):
    def __init__(self, getter_func=None, on_icon=Gtk.STOCK_YES, off_icon=Gtk.STOCK_NO, icon_size=Gtk.IconSize.MENU, **kwargs):
        if "widget" not in kwargs:
            kwargs.update(widget=Gtk.Image.new_from_stock(off_icon, icon_size))
        TopicWidget.__init__(self, **kwargs)
        self._getter_func = getter_func or (lambda x: x.data)
        self._on_icon = on_icon
        self._off_icon = off_icon
        self._icon_size = icon_size

    def set_message(self, msg):
        on = self._getter_func(msg)
        self.widget.set_from_stock(self._on_icon if on else self._off_icon, self._icon_size)

class GtkButtonKillNode(_MagicLabel):

    widget = None
    name = None
    nodepath = None

    def __init__(self, widget=None, nodemanager=None, nodepath=None, name=None):
        assert nodemanager is not None
        assert nodepath is not None
        self.nodepath = nodepath
        self.name = name or self.nodepath
        if widget is None:
            widget = Gtk.Button(self.name)
        self.widget = widget
        self.widget.set_sensitive(False)
        self._nodemanager = nodemanager
        self._nodemanager.connect("node-appeared", self._node_appeared)
        self._nodemanager.connect("node-disappeared", self._node_disappeared)
        self.widget.connect("clicked", self._on_clicked)

    def _node_appeared(self, mgr, path):
        if self.nodepath == path:
            self.widget.set_sensitive(True)

    def _node_disappeared(self, mgr, path):
        if self.nodepath == path:
            self.widget.set_sensitive(False)

    def _on_clicked(self, *args):
        self._nodemanager.kill_nodes(self.nodepath)

class GtkButtonStartNode(_MagicLabel):

    widget = None
    name = None
    nodepath = None

    def __init__(self, widget=None, nodemanager=None, nodepath=None, package=None, node_type=None, machine_name='', name=None, **kwargs):
        assert nodemanager is not None
        assert nodepath is not None
        assert package is not None
        assert node_type is not None
        self.nodepath = nodepath
        self.name = name or self.nodepath
        if widget is None:
            widget = Gtk.Button(self.name)
        self.widget = widget
        self.widget.connect("clicked", self._on_clicked)
        self._nodemanager = nodemanager
        self._package = package
        self._node_type = node_type
        self._machine_name = machine_name
        self._kwargs = kwargs

    def _on_clicked(self, *args):
        self._nodemanager.launch_node(
                            package=self._package,
                            node_type=self._node_type,
                            machine_name=self._machine_name,
                            name=self.nodepath,
                            **self._kwargs)


