#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division    # Standardmäßig float division - Ganzzahldivision kann man explizit mit '//' durchführen
import numpy as np
import os

class ReadNumber(object):
    """Read number from key presses"""
    def __init__(self):
        self.typed = ""

        # allowed key codes: '0'-'9' and '.' and ','
        self.allowed = map(ord,map(str,range(10))+['.',','])

    def feed(self, key):
        """Feed key into number reader.
        Returns whether text input is complete"""
        if key in self.allowed:
            self.typed += chr(key)
        elif key == 8: # backspace
            self.typed = self.typed[:-1]
        elif key == 10: # return
            return True

        return False

    def as_float(self, default = 0):
        # parse number
        try:
            return float(self.typed)
        except Exception, e:
            return default

class Input(object):
    """Handles input for IRTracking."""
    def __init__(self, irtracking):
        super(Input, self).__init__()
        self.ir = irtracking
        self.filename = "homography.npy"
        if os.path.isfile(self.filename):
            self.ir.homography = np.load(self.filename)
            self.state = "done"
            self.ir.selected_marker = None
        else:
            self.state = "select_fix"
            self.ir.selected_marker = 0
        self.distance_between = None
        self.readnumber = ReadNumber()
        self.descriptions = {
            "select_fix": [
                "Select marker that represents (0cm, 0cm).",
                "[TAB]:   next marker",
                "[ENTER]: confirm selection"
                ],
            "select_xaxis": [
                "Select marker that lies on x-axis.",
                "[TAB]:   next marker",
                "[ENTER]: confirm selection"
                ],
            "select_ypositive": [
                "Select marker with positive y-value.",
                "[TAB]:   next marker",
                "[ENTER]: confirm selection"
                ],
            "distance": [
                "Enter distance between marker {self.distance_between[0]} and {self.distance_between[1]} in cm.",
                "[TAB]:       next distance",
                "0-9 and '.': enter number",
                "[ENTER]:     confirm number"
                ],
            "compute_homography": [
                "Computing homography..."
                ],
            "done": [
                "Done.",
                "s : Save homography matrix"
                ]
            ,
            "saved": [
                "Homography matrix saved",
                "s : Save homography matrix"
                ]
        }


    def process(self, key):
        if self.state in ["select_fix","select_xaxis","select_ypositive"]:
            if key == 9: # tab
                # select next marker
                if self.ir.selected_marker is None:
                    self.ir.selected_marker = 0
                else:
                    self.ir.selected_marker += 1
                    self.ir.selected_marker %= len(self.ir.markers)

            if self.ir.selected_marker is not None:
                if key == 10: # return
                    # confirm selection
                    if self.state == "select_fix":
                        # set fixed marker
                        self.ir.fixed_marker = self.ir.selected_marker
                        self.ir.markers[self.ir.selected_marker]['fixed'] = True
                        
                        # proceed with selecting x-axis marker
                        self.state = "select_xaxis"
                        self.ir.selected_marker = min([marker['id'] for marker in self.ir.markers 
                                                        if marker['id'] != self.ir.fixed_marker])

                    elif self.state == "select_xaxis":
                        # set x-axis marker
                        self.ir.xaxis_marker = self.ir.selected_marker
                        self.ir.markers[self.ir.selected_marker]['xaxis'] = True
                        
                        # proceed with selecting y positive marker
                        self.state = "select_ypositive"
                        self.ir.selected_marker = min([marker['id'] for marker in self.ir.markers 
                                                                    if marker['id'] != self.ir.fixed_marker 
                                                                       and marker['id'] != self.ir.xaxis_marker])
                    elif self.state == "select_ypositive":
                        # set y positive marker
                        self.ir.ypositive_marker = self.ir.selected_marker
                        self.ir.markers[self.ir.selected_marker]['ypositive'] = True

                        # proceed with distance input
                        self.state = "distance"
                        
                        # first input distance between fixed marker and x-axis marker
                        self.distance_between = (self.ir.fixed_marker, self.ir.xaxis_marker)
                        self.distance_between = min(self.distance_between),max(self.distance_between)

                        # populate text input string
                        if self.ir.distances[self.ir.fixed_marker, self.ir.xaxis_marker] is not None:
                            self.readnumber.typed = str(self.ir.distances[self.ir.fixed_marker, self.ir.xaxis_marker])
                        else:
                            self.readnumber.typed = ""

        elif self.state == "distance":
            if len([marker for marker in self.ir.markers if 'transformed_pos' in marker]) >= 4:
                self.state = "compute_homography"

            if self.distance_between is None:
                self.distance_between = (0,1) 

            a,b = self.distance_between
            assert (a,b) == (min(a,b),max(a,b))

            # feed key into number reading
            if key == 9: # tab
                self.next_distance()
            elif self.readnumber.feed(key): 
                # if number reading is complete set distance 

                # parse number
                dist = self.readnumber.as_float()
                self.ir.distances[a,b] = dist
                self.ir.distances[b,a] = dist

                # select next distance
                self.next_distance()

                # populate text input string
                if self.ir.distances[a,b] is not None:
                    self.readnumber.typed = str(self.ir.distances[a,b])
                else:
                    self.readnumber.typed = ""
        elif self.state == "compute_homography":
            if self.ir.homography is not None:
                self.state = "done"
        elif self.state == "done":
            if key == ord('s'):
                # save
                if self.ir.homography is not None:
                    np.save(self.filename, self.ir.homography)
                    self.state = "saved"

        # quit on 'q'
        if key == ord('q'):
            self.ir.quit()

    def next_distance(self):
        a,b = self.distance_between
        assert (a,b) == (min(a,b),max(a,b))
        a += 1
        if a>=b:
            b += 1
            a = 0
            if b >= len(self.ir.markers):
                b = 0
        if a==0 and b==0:
            b = 1
        self.distance_between = a,b

    def current_description(self):
        if self.state in self.descriptions:
            return map(lambda line: line.format(self=self), self.descriptions[self.state])
        else:
            return ""