#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division    # Standardmäßig float division - Ganzzahldivision kann man explizit mit '//' durchführen
#from wx.lib.pubsub.core.arg1 import publisher

"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs
CompressedImage. It converts the CompressedImage into a numpy.ndarray,
then detects and marks features in that image. It finally displays
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time
import cProfile

# numpy and scipy
import numpy as np
from threading import Thread
from geometry_msgs.msg import Pose2D


import cv2

import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import copy
import datetime


def msg_to_img(msg):
    arr = np.frombuffer(msg.data, dtype='uint8')
    arr = arr.reshape((msg.height,msg.width)) # reshape to image (width, height and 3 color channels)
    # arr = arr[:,:,[2,1,0]] # reverse order of color channeles (rgb->bgr)
    return arr

VERBOSE=True

circles2detect=2

from Input import Input
import ImageProcessing

import math
from scipy.optimize import minimize

class IRTracking:
    def __init__(self, display=True):
        self.fps = 0
        self.markers = None
        self.last_markers = None
        self.transformed_markers = None
        self.homography = None
        self.selected_marker = None
        self.fixed_marker = None
        self.xaxis_marker = None
        self.ypositive_marker = None
        self.distances = None
        self.last_time = None
        self.blur = None
#        self.profiler = cProfile.Profile()
        self.profiler = None
        self.counter = 0
        self.input = Input(self)
        self.display = display
        self.publisher_list = list()

        self.pixel_per_cm = 1

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /camera/image"

    def quit(self):
        if self.profiler is not None:
            self.profiler.disable()
            self.profiler.dump_stats("profiling-{0}.log".format(str(datetime.datetime.today()).replace(' ','-')[:19]))

        cv2.destroyAllWindows()
        rospy.signal_shutdown("quit")
        sys.exit(0)

    def update_marker_thread(self, gray_img, dt, marker):
        id = marker['id']
        if 'velocity' in marker:
            point_estimate = marker['pos'] + marker['velocity'] * dt
        else:
            point_estimate = marker['pos']

        # generate roi
        halfsize = 20
        from_x = max(0,int(point_estimate[0]-halfsize+0.5))
        from_y = max(0,int(point_estimate[1]-halfsize+0.5))
        to_x =  min(gray_img.shape[1],int(point_estimate[0]+halfsize+0.5))
        to_y =  min(gray_img.shape[0],int(point_estimate[1]+halfsize+0.5))
        roi = gray_img[from_y:to_y,from_x:to_x]
        pointFound = False

        offset = np.array([from_x,from_y])
        mean = roi.mean()
        print "mean: ", mean
        # if roi is not black then update position
        if mean >= 4:                                   #TODO depends on halfsize and is critical if ledpoint is to small!!! also depends on camera ExposureTime!!!
            # calculate mass center with moments
            pointFound = True
            central_point = ImageProcessing.find_central_point(roi,offset)

            if central_point is not None:
                # update
                self.markers[id]['pos'] = central_point
            else:
                # found no central point (why ever)
                # fall back to posiition finding with findContours
                bw_img = cv2.threshold(gray_img, 0.5*255, 255, cv2.THRESH_BINARY)[1]
                points = self.find_points_in(bw_img[from_y:to_y,from_x:to_x],offset)
                if len(points)>1:
                    # find point that is nearest to last point
                    diff = points - point_estimate
                    error = np.sum(diff*diff, axis=1)
                    best = points[np.argmin(error)]

                    # update
                    self.markers[id]['pos'] = best

                elif len(points)==1:
                    # update
                    # just take first contour
                    self.markers[id]['pos'] = points[0]

                # else: no update

        if dt != 0:
            if self.last_markers is not None:
                # calculate velocity
                self.markers[id]['velocity'] = (marker['pos'] - self.last_markers[id]['pos']) / dt
                #print  self.markers[id]['velocity']

        return pointFound

    def find_markers(self, gray_img):
            bw_img = cv2.threshold(gray_img, 0.5*255, 255, cv2.THRESH_BINARY)[1]
            points = ImageProcessing.find_points_in(bw_img)

            # remove duplicates
            points_cleared = list()
            points_cleared.append(points[0])
            for point in points:
                add=True
                for cpoint in points_cleared:
                     if abs(point[0]-cpoint[0]) < 10 and abs(point[1]-cpoint[1]) < 10:
                         add=False
                if add:
                    points_cleared.append(point)

            self.markers = [dict({'id': id, 'pos': point}) for (id,point) in enumerate(points_cleared)]
            self.distances = np.empty(shape=(len(self.markers), len(self.markers)),dtype=np.dtype(object))

    def update_markers(self, gray_img, dt):
        if self.markers is None:
            self.find_markers(gray_img)
            # initialize
#            bw_img = cv2.threshold(gray_img, 0.5*255, 255, cv2.THRESH_BINARY)[1]
#            points = ImageProcessing.find_points_in(bw_img)

            # remove duplicates
#            points_cleared = list()
#            points_cleared.append(points[0])
#            for point in points:
#                add=True
#                for cpoint in points_cleared:
#                     if abs(point[0]-cpoint[0]) < 10 and abs(point[1]-cpoint[1]) < 10:
#                         add=False
#                if add:
#                    points_cleared.append(point)
#
#            self.markers = [dict({'id': id, 'pos': point}) for (id,point) in enumerate(points_cleared)]
 #           self.distances = np.empty(shape=(len(self.markers), len(self.markers)),dtype=np.dtype(object))

        else:
            self.transformed_markers = []
            success = True
            for marker in self.markers:
                #t = Thread(target=self.update_marker_thread, args=(gray_img,dt, marker))
                #t.start()
                if(not self.update_marker_thread(gray_img,dt, marker)):
                    success = False
                if self.homography is not None:
                    self.transformed_markers+=[cv2.perspectiveTransform(np.array([[marker["pos"]]], dtype="float32"), np.array(self.homography, dtype="float32"))[0][0]]
                else:
                    self.transformed_markers+=[marker['pos']]
            if(not success):
                self.find_markers(gray_img)
            if self.last_markers is None:
                self.create_pub()

            self.last_markers = [copy.copy(marker) for marker in self.markers]
            self.pubish_marker()

    def pubish_marker(self):
        tmp=Pose2D()
        tmp.theta=0
        tmp.x=0
        tmp.y=0
        i=0
        for pub in self.publisher_list:
            if len(self.transformed_markers) > i:
		tmp.x=self.transformed_markers[i][0]
            	tmp.y=self.transformed_markers[i][1]
            	pub.publish(tmp)
            i+=1

    def create_pub(self):
        for marker in self.markers:
            self.publisher_list.append(rospy.Publisher('Marker'+str(marker['id']), Pose2D, queue_size=10))


    def callback(self, msg):
        if self.profiler is not None:
            self.profiler.enable()
        # update dt and fps
        dt = 0
        self.counter+=1
        if self.last_time is not None:
            dt = time.time() - self.last_time
            if dt != 0:
                if self.fps == None:
                    self.fps=1/dt
                else:
                    self.fps=0.9*self.fps+0.1*(1/dt)
                if self.counter % 50 == 0:
                    print self.fps

        self.last_time = time.time()

        # get image
        img=msg_to_img(msg)

        #if self.homography is not None:
        #    img = self.to_topview(self.homography, img)
            ##cv2.imshow('topview', topview)

        # optional: apply blur
        if self.blur > 0:
            img = cv2.GaussianBlur(img, (self.blur,self.blur), 0)

        # update markers
        self.update_markers(img, dt)

        # update transformed positions of markers
        self.update_transformed_positions()

        if self.profiler is not None:
            self.profiler.disable()

        if self.homography is None:
            self.display=True

        if self.display:
            self.draw(img)                  # draw markers and ui
            key = cv2.waitKey(2) & 0xFF     # get user input
            self.input.process(key)







        if self.homography is None and self.input.state == "compute_homography":
            used_markers = [marker for marker in self.markers if 'transformed_pos' in marker]
            src_points = np.array([marker['pos'] for marker in used_markers])
            dst_points = np.array([marker['transformed_pos'] for marker in used_markers])
            self.homography, _ = cv2.findHomography(src_points, dst_points)
            # print self.homography
            # self.quit()

    def draw(self, gray_img):
        # convert to color, so we can draw with colors
        colored = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)

        # draw markers
        for marker in self.markers:
            id = marker['id']
            x,y = map(int,marker['pos']+0.5) # pixel position

            # highlight selected marker
            if self.selected_marker == id:
                cv2.circle(colored, (x,y), radius=10, color=(0,0,255), thickness = 2)

            # draw marker
            cv2.circle(colored, (x,y), radius=5, color=(0,0,255))

            # draw marker label
            string = "Point"+str(id)
            if id == self.fixed_marker:
                string += " fix"
            if id == self.xaxis_marker:
                string += " xaxis"
            if id == self.ypositive_marker:
                string += " y>0"
            if 'transformed_pos' in marker:
                string += " " + str(list(marker['transformed_pos']))
            cv2.putText(colored,string,(x,y),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,0,255))

        # draw distances between markers
        if self.distances is not None:
            # determine which distance the user has to inputcv2.imshow('cv_img', colored)
            current_a,current_b = None,None
            if self.input.state == "distance":
                if self.input.distance_between is not None:
                    current_a,current_b = self.input.distance_between

            # for all distances with a<b
            for b in range(len(self.markers)):
                for a in range(b):
                    # position between marker a and marker b
                    mid = (self.markers[a]['pos']+self.markers[b]['pos']) / 2

                    if self.input.state == "distance" and (current_a == a) and (current_b == b):
                        # distance is the currently selected
                        color = (0,255,255)
                        string = self.input.readnumber.typed

                        # draw line between marker a and b
                        pt_a = tuple(map(int,0.5+self.markers[a]['pos']))
                        pt_b = tuple(map(int,0.5+self.markers[b]['pos']))
                        cv2.line(colored, pt_a, pt_b, color, 1)
                    else:
                        # all other distances
                        color = (0,0,255)
                        if self.distances[a,b] is not None:
                            string = str(self.distances[a,b])
                        else:
                            string = "?"

                    # display distance
                    cv2.putText(colored,string ,tuple(map(int,mid+0.5)),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,color)

        # display current input state
        # string = str(self.input.state)
        # if self.input.state == "distance":
            # string += str(self.input.distance_between)
        # cv2.putText(colored,string ,(100,200),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,0,255))

        line_height = 16
        description = self.input.current_description()
        for k,line in enumerate(description):
            cv2.putText(colored,line,(100,200+k*line_height),cv2.cv.CV_FONT_HERSHEY_PLAIN,1,(0,0,255))

        # show image
        cv2.imshow('cv_img', colored)

    def update_transformed_positions(self):
        for marker in self.markers:
            if 'transformed_pos' in marker:
                continue

            if 'fixed' in marker and marker['fixed']:
                marker['transformed_pos'] =  np.array([0,0])

            if 'xaxis' in marker and marker['xaxis']:
                if self.distances[self.fixed_marker][self.xaxis_marker]:
                    marker['transformed_pos'] =  np.array([self.distances[self.fixed_marker][self.xaxis_marker],0])

            if ('ypositive' in marker
                and self.distances[marker['id']][self.fixed_marker] is not None
                and self.distances[marker['id']][self.xaxis_marker] is not None):
                # http://www.arndt-bruenner.de/mathe/scripts/Dreiecksberechnung.htm
                # triangle with points self.fixed_marker, self.xaxis_marker and marker
                a = self.distances[marker['id']][self.xaxis_marker]
                b = self.distances[marker['id']][self.fixed_marker]
                c = self.distances[self.fixed_marker][self.xaxis_marker]
                # alpha is angle at point self.fixed_marker
                cosalpha = (a*a-b*b-c*c)/(-2*b*c)
                alpha = math.acos(cosalpha)
                x,y = cosalpha * b, math.sin(alpha) * b
                marker['transformed_pos'] = np.round(np.array([x,y]),2)


            references = [other_marker for other_marker in self.markers
                        if other_marker['id'] != marker['id']
                        and 'transformed_pos' in other_marker
                        and self.distances[marker['id'],other_marker['id']] is not None]
            n_references = len(references)
            positions = np.array([reference['transformed_pos'] for reference in references])
            dists = np.array([self.distances[marker['id'],reference['id']] for reference in references])
            if n_references >= 3:
                def error_function(p):
                    x,y = p
                    pos = np.array([x,y])
                    diffs = positions - pos
                    transformed_distances = np.sqrt(np.sum(np.square(diffs),axis=1))
                    error = np.sum(np.square(dists - transformed_distances))
                    return error
                result = minimize(error_function, (0,0))
                marker['transformed_pos'] = np.round(result.x,2)

    def roi_points_complete(self, img):
        h,w = img.shape
        roi_points = [[0,0],
                      [w-1,0],
                      [w-1,h-1],
                      [0,h-1]]
        return roi_points

    def to_topview(self, homography, img, roi_points=None):
        if roi_points is None:
            roi_points = self.roi_points_complete(img)

        scale = np.array([[self.pixel_per_cm,0,0],
                          [0,self.pixel_per_cm,0],
                          [0,0,1]])

        roi_bounding_rect = cv2.boundingRect(np.array([roi_points],'float32'))
        transformed_roi_bounding_rect = cv2.boundingRect(self.points_to_topview(scale.dot(homography),roi_points))

        x,y,w,h = transformed_roi_bounding_rect
        topview_size = (w,h)
        translate =  np.array([[1,0,-x],
                               [0,1,-y],
                               [0,0,1]])
        m = translate.dot(scale.dot(homography))
        return cv2.warpPerspective(img, m , topview_size, flags=cv2.INTER_NEAREST)

    def points_to_topview(self, homography, points):
        p = np.asarray([points]).astype('float32')
        transformed = cv2.perspectiveTransform(p,homography)
        return transformed

    def point_to_topview(self, homography, point):
        return self.points_to_topview(homography,[point])


def main(args):
    display=True
    rospy.init_node('image_feature', anonymous=True)
    if (len(args) > 1) and (args[1] == "--nodisplay"):
        display=False
    ir = IRTracking(display)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
