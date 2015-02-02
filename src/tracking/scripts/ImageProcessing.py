#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division    # Standardmäßig float division - Ganzzahldivision kann man explizit mit '//' durchführen
import cv2
import numpy as np

def find_central_point(gray_img, offset=None):
    # if image has zero size return None
    if gray_img.shape[0] * gray_img.shape[1] == 0:
        return None

    # calculate mass center with moments
    m = cv2.moments(gray_img)
    xy = np.array([m['m10'] / m['m00'],  m['m01'] / m['m00']])

    if offset is None:
        return xy
    else:
        return xy + offset
    


def find_points_in(bw_img, offset = None):
    '''
    :returns: all found points in bw_img
    :rtype: list of np.array
    '''

    if offset is None:
        offset = np.zeros(shape=(2,))
    if bw_img.shape[0] * bw_img.shape[1] == 0:
        return []

    contours = cv2.findContours(bw_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    for i in range(len(contours)):
        contours[i] = 1.0 * contours[i] # convert to float 

        contours[i] = np.mean(contours[i],axis=0) 
        # reshape
        contours[i] = contours[i][0,:]

        contours[i] += offset

    print contours
    return contours
