#!/usr/bin/env python2.7

import rospy
import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
import pylab as p
from matplotlib.text import Text

class DragHandler(object):
    def __init__(self, figure=None) :
        if figure is None : figure = p.gcf()
        self.dragged = None
        figure.canvas.mpl_connect("pick_event", self.on_pick_event)
        figure.canvas.mpl_connect("button_release_event", self.on_release_event)

    def on_pick_event(self, event):
        if isinstance(event.artist, Text):
            self.dragged = event.artist
            self.pick_pos = (event.mouseevent.xdata, event.mouseevent.ydata)
        return True

    def on_release_event(self, event):
        if self.dragged is not None :
            old_pos = self.dragged.get_position()
            new_pos = (old_pos[0] + event.xdata - self.pick_pos[0],
                       old_pos[1] + event.ydata - self.pick_pos[1])
            a = str(self.dragged)
            b = a.split('\'')
            print(b[1])
            self.dragged.set_position(new_pos)
            index = int(b[1])
            p.waypoint_dx[index] = old_pos[0] + event.xdata - self.pick_pos[0]
            p.waypoint_dy[index] = old_pos[1] + event.ydata - self.pick_pos[1]
            print(new_pos)
            self.dragged = None
            p.draw()
            p.close()
            for a,b,l in zip(p.waypoint_dx, p.waypoint_dy, p.waypoint_label):
                p.text(a, b, l, picker=True)
            p.scatter(p.waypoint_dx, p.waypoint_dy)
            p.grid()
            dragh = DragHandler()
            p.title('Race Track')
            p.xlabel('[map_frame] x in m')
            p.ylabel('[map_frame] y in m')
            p.show()
        return True


def main():
    rospy.init_node('track_editor')
    p.waypoint_dx = []
    p.waypoint_dy = []
    p.waypoint_dx = genfromtxt('waypoint_file_race_x.csv',delimiter = '')
    p.waypoint_dy = genfromtxt('waypoint_file_race_y.csv', delimiter = '')
    p.waypoint_label = ["%d" % i for i in xrange(p.waypoint_dx.size)]

    p.scatter(p.waypoint_dx, p.waypoint_dy)

    for a,b,l in zip(p.waypoint_dx, p.waypoint_dy, p.waypoint_label):
        p.text(a, b, l, picker=True)

    dragh = DragHandler()
    p.grid()
    p.title('Race Track')
    p.xlabel('[map_frame] x in m')
    p.ylabel('[map_frame] y in m')
    p.show()

if __name__ == '__main__':
    main()
