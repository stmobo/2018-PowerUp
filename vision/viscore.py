# viscore.py -- core vision processing functions
# Written for the 2017 FRC Season (Steamworks), updated for 2018 (Power Up)
# By Sebastian Mobo

import numpy as np
import math
import cv2
import sys

#
# Vision Target Specs:
#
# There are two vision targets on the side of the Switch:
#
# |X|   |X|
# |X|   |X|
# |X|   |X|
# |X|   |X|
# |X|   |X|
#
# Each target is a 2" by 15.3" (about ~5 cm by ~39 cm) rectangle.
# They are spaced about 4 in. away from each other
#  (measuring between the inside edges of the tape)
#


# Field-of-View Angles
# (Note: these might be a bit screwed up, the distance math is a bit wonky)
#
# Empirically found, for the LiveCam Studio:
# Horizontal FOV (radians): 0.776417
# Vertical FOV (radians): 0.551077
#
# Found on ChiefDelphi, for the LifeCam HD-3000:
# Horizontal FOV (degrees): 60
# Vertical FOV (degrees): 34.3
#
# Emprically found for the LifeCam HD-3000:
# Horizontal FOV (radians): 0.623525
# Vertical FOV (radians): 0.594530

fovHoriz = 0.623525
fovVert = 0.594530

# Target retroreflector sizes:
# Both are in inches.
# Note that the units used here will be the same units used in get_target_distance
# and get_lateral_offset.
targetWidth = 2.0
targetHeight = 15.3

def get_minimal_rect_dims(ct):
    min_rect = cv2.minAreaRect(ct)
    pts = np.int0(cv2.boxPoints(min_rect))
    min_width = abs(pts[0][0] - pts[2][0])
    min_height = abs(pts[0][1] - pts[2][1])

    return min_width, min_height

def get_minimal_rect_pts(ct):
    min_rect = cv2.minAreaRect(ct)
    return np.int0(cv2.boxPoints(min_rect))

def get_bounding_rect_dims(ct):
    bb = cv2.boundingRect(ct)
    bb_width = bb[2]
    bb_height = bb[3]

    return bb_width, bb_height

def get_target_center(ct):
    moments = cv2.moments(ct)
    center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))

    return center

#
# Gets distance to observed target.
def get_target_distance(ct, imgWidth, imgHeight):
    width = get_bounding_rect_dims(ct)

    # According to the documentation on ScreenStepsLive, the divisor should be multiplied
    # by 2, but doing that breaks the calculations for some reason.
    return targetWidth * imgWidth / (width * math.tan(fovHoriz))

#
# Get camera lateral offset to the target center.
# Assumes that the camera centerline is perpendicular to the target.
def get_lateral_offset(ct, imgWidth, dist):
    center = get_target_center(ct)

    # positive values = target is right/under of center
    centerOffsetHoriz = center[0] - (imgWidth/2.0)

    return centerOffsetHoriz * ((dist*math.tan(fovHoriz)) / imgWidth)

#
# Returns estimated FOV angles based on observed target size with a known, fixed
# distance to the target.
# Returns the horizontal FOV first, then the vertical.
#
# Use for calibration.
def get_fov_angles(ct, imgWidth, imgHeight, dist):
    width, height = get_bounding_rect_dims(ct)

    return math.atan2(targetWidth*imgWidth, width*dist), math.atan2(targetHeight*imgHeight, height*dist)

#
# Performs image preprocessing and contour finding.
def preprocess(frame):
    filtered = cv2.boxFilter(frame, -1, (7,7))

    # Filter by HSV:
    filtered = cv2.cvtColor(filtered, cv2.COLOR_BGR2HLS)
    filtered = cv2.inRange(filtered, np.array([0, 94, 0]), np.array([60, 255, 255])) # Green targets (actual)
    #filtered = cv2.inRange(filtered, np.array([90, 0, 105]), np.array([180, 255, 231])) # Red targets (testing)

    _, contours, _ = cv2.findContours(filtered, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    return contours, filtered

#
# Picks out the two best candidate contours as targets.
def select_contours(contours):
    scoredContours = []
    failedContours = []
    for ct in contours:
        ct_area = cv2.contourArea(ct)

        if ct_area < 100:
            failedContours.append(ct)
            continue

        min_width, min_height = get_minimal_rect_dims(ct)
        if min_width > 550:
            failedContours.append(ct)
            continue

        # Aspect Ratio test
        # Ideal AS = (2/5)
        AS = min_width / min_height
        as_score = 100/math.exp(abs(AS-(2/5)))
        if AS < 0.1 or AS > 0.6:
            failedContours.append(ct)
            continue

        # Coverage area (Bounding Box area vs particle area) test:
        # Ideal Cvg. area ratio = 1
        bb_area = min_width*min_height
        if bb_area < 10:
            failedContours.append(ct)
            continue

        cvg_ratio = ct_area / bb_area
        cvg_score = 100/math.exp(abs(cvg_ratio-1))

        scoredContours.append( (ct, cvg_score+as_score, AS, cvg_ratio) )

    scoredContours.sort(key=lambda ct: ct[1])

    if len(scoredContours) >= 2:
        tgt1 = scoredContours.pop()
        tgt2 = scoredContours.pop()
        return tgt1, tgt2, scoredContours, failedContours
    return None, None, scoredContours, failedContours

def do_the_vision_stuff(frame):
    contours = preprocess(frame)
    tgt1, tgt2 = select_contours(contours)

    if (tgt1 is not None) and (tgt2 is not None):
        # Calculate distance to targets:
        #print(frame.shape)
        dist1 = getTargetDistance(tgt1[0], frame.shape[1], frame.shape[0])
        dist2 = getTargetDistance(tgt2[0], frame.shape[1], frame.shape[0])
        avgDist = (dist1+dist2)/2.0

        # Get target offsets:
        offset1 = getTargetOffset(tgt1[0], frame.shape[1], dist1)
        offset2 = getTargetOffset(tgt2[0], frame.shape[1], dist2)
        avgOffset = (offset1+offset2)/2.0

        return avgDist, avgOffset
    # else return None, None
