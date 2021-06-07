#import cv2
import numpy as np
import scipy.linalg as la
import copy
from math import asin, pi, sqrt

def intersect_over_union_2circs(circle1, circle2):
    '''
    This function takes two circles and computes the intersection 
    over union metric for them.

    Args: circle1, circle2:
        Each circle is tuple consisting of (center_loc,radius)
    '''
    # First, compute the distance between the two circles
    cent1 = np.array(circle1[0])
    cent2 = np.array(circle2[0])
    cents = np.array([cent1,cent2])
    distance = la.norm(cent1-cent2)
    rad1 = circle1[1] #radius1
    rad2 = circle2[1] #radius2
    rads = np.array([rad1,rad2])

    # Detect the easiest case ... no intersection
    if rad1+rad2 <= distance:
        return 0

    #Let us make the first one the bigger one
    if rad1<rad2:
        cents[0] = cent2
        cents[1] = cent1
        rads[0] = rad2
        rads[1] = rad1
        
    area1 = pi*rads[0]**2
    area2 = pi*rads[1]**2
    total_area= area1+area2


    #### Check for one circle completely inside another
    # Return the area of smaller divided by the bigger
    if rads[0] >= distance+rads[1]:
        return area2/area1
    
    #### We should now be to the case where there are two intersection
    # points between the two circles.
    # Using the algorithm described at:
    # https://www.xarg.org/2016/07/calculate-the-intersection-points-of-two-circles/
    # It doesn't say it on the page, but the A^2 - B^2... formula for x assumes
    # (I think) that A_r is greater than B_r, so I made sure that is true above.
    x = (rads[0]**2 - rads[1]**2 + distance**2)/(2*distance)
    y = sqrt(rads[0]**2 - x**2)
    ang1 = asin(y/rads[0])
    ang2 = asin(y/rads[1])
    if x < distance:
        # Intersection forms a "football" made of two arcs
        # ... less their triangles. See pictures under case 3 at webpage
        # Compute arc - rectangle
        seg1= ang1*rads[0]**2 - x*y
        x2 = distance-x
        seg2 = ang2*rads[1]**2 - x2*y
        intersection = seg1+seg2
    else:
        # cent2 is in the first circle but the "angle" of 
        # what is enclosed is greater than 180 degrees
        seg1= ang1*rads[0]**2 - x*y
        x2 = x-distance
        seg2 = ang2*rads[1]**2 - x2*y
        # intersection is all of circle 2 except for a "toenail" of
        # 2 sitting outside the bigger circle
        intersection = area2-seg2 + seg1
    return intersection/ (total_area-intersection)
         
