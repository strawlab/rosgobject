# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil; indent-offset: 4 -*-
import roslib; roslib.load_manifest('rosgobject')

import collections
import numpy as np
import threading
import time
import random

import rospy

from gi.repository import Gtk, GLib

class mpl:
    from matplotlib.figure import Figure
    from matplotlib.gridspec import GridSpec
    from matplotlib.backends.backend_gtk3agg import FigureCanvasGTK3Agg as FigureCanvas

class FIFO(object):

    def __init__(self, N, dtype, binning=1):
        assert type(N) is int
        self._dtype = dtype
        self._lock = threading.Lock()
        self._fifo = collections.deque([0 for i in range(N)], N)
        self._newdata_available = True
        self._bin = binning
        self._N = N
        self._currentpos = 0

    @property
    def length(self):
        return self._N/self._bin

    @property
    def newdata_available(self):
        with self._lock:
            b = self._newdata_avialable
        return b

    def update_data(self, *d):
        with self._lock:
            self._currentpos = (self._currentpos + len(d)) % self._N
            self._fifo.extend(d)
            self._newdata_available = True
        return True

    def get_data(self):
        with self._lock:
            if self._newdata_available:
                if self._bin != 1:
                    b = np.fromiter(self._fifo, dtype=self._dtype)
                    b = np.roll(b,(self._currentpos % self._bin))
                    b[:(self._currentpos % self._bin)] = 0
                    data = b.reshape((self._N/self._bin, self._bin)).sum(1)
                else:
                    data = np.fromiter(self._fifo, dtype=self._dtype)
                self._newdata_available = False
            else:
                data = None
        return data

class Plotter(Gtk.Box):
    def __init__(self, nodepath, msgclass, data_func=None, N=300, ymax=1.0, ymin=0.0, binning=1, dtype=np.float, freq=20):
        Gtk.Box.__init__(self, orientation=Gtk.Orientation.VERTICAL)

        self._N = int(N)
        self._bin = int(binning)
        self._dtype = dtype
        self._data_func = data_func or (lambda coll,msg: coll.update_data(msg.data))

        if N % self._bin != 0:
            raise ValueError('N % binning != 0')

        self.coll = FIFO(self._N, self._dtype, self._bin)

        interval = 1.0
        level=-1.0

        self._interval = float(interval*(10**level)*10)
        self._time = np.arange(-1*N,0,self._bin)*self._interval
        self._vx = self._time[0]
        self._vy = int(ymax)
        self._my = int(ymin)

        self._xsb = None
        self._ysb = None

        figure = mpl.Figure()
        self._plt = figure.add_subplot(111)
        self._canvas = mpl.FigureCanvas(figure)
        self._canvas.props.hexpand = True
        self._canvas.props.vexpand = True
        self._plt.set_ylim(self._my, self._vy)
        self._line, = self._plt.plot(
                                self._time,
                                np.zeros(self.coll.length, dtype=self._dtype)
        )
        self.add(self._canvas)

        rospy.Subscriber(nodepath, msgclass, self._cb)
        GLib.timeout_add(freq, self._update_data)

    @property
    def canvas(self):
        return self._canvas

    @property
    def ax(self):
        return self._plt

    @property
    def xlabel_controller(self):
        if self._xsb is None:
            self._xsb = Gtk.SpinButton.new_with_range(self._time[0], self._time[-1], self._interval)
            self._xsb.props.value = self._time[0]
            self._xsb.connect("value-changed", self._on_val_changed, 'x')
        return self._xsb

    @property
    def ylabel_controller(self):
        if self._ysb is None:
            self._ysb = Gtk.SpinButton.new_with_range(1, 1000000, 1)
            self._ysb.props.value = self._vy
            self._ysb.connect("value-changed", self._on_val_changed, 'y')
        return self._ysb

    def _cb(self, msg):
        self._data_func(self.coll, msg)

    def _on_val_changed(self, sb, ident):
        if ident == 'y':
            self._vy = sb.get_value()
            self._plt.set_ylim(self._my, self._vy)
            self._update_data()
        if ident == 'x':
            self._vx = sb.get_value()
            self._plt.set_xlim(self._vx,0)
            self._update_data()

    def _update_data(self):
        b = self.coll.get_data()
        if b is not None:
            self._line.set_ydata(b)
        self._canvas.draw()
        return True

    def do_get_request_mode(self):
        return Gtk.SizeRequestMode.CONSTANT_SIZE

    def do_get_preferred_width(self):
        return 300, 900

    def do_get_preferred_height(self):
        return 100, 300

