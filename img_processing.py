import cv2
import numpy as np
import math

def findObstacle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([155, 43, 35])
    upper_red = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    # cv2.imshow("mask_red", mask_red)
    ret, thresh_gray = cv2.threshold(mask_red, 200, 255, cv2.THRESH_BINARY)
    #cv2.imshow("obstacle: before close", thresh_gray)
    thresh_gray = cv2.morphologyEx(thresh_gray, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (50, 50)))
    #cv2.imshow("obstacle: after close", thresh_gray)
    image_grey, contours, hier = cv2.findContours(thresh_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # cv2.drawContours(frame,contours,-1,(255, 255, 0), 10)
    obstacle_datas = []
    for c in contours:
        area = cv2.contourArea(c)
        # Small contours are filled.
        if area < 500:
            cv2.fillPoly(thresh_gray, pts=[c], color=0)
            continue
        rect = cv2.minAreaRect(c)
        # box = cv2.boxPoints(rect)
        # convert all coordinates floating point values to int
        # box = np.int0(box)
        # cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
        x, y, w, h = cv2.boundingRect(c)
        # cv2.putText(frame, "obstacle" + "(" + str(x) + "," + str(y) + ") Size: " + str(w * h), (x, y + h),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        obstacle_data = [x,y,w,h]
        obstacle_datas.append(obstacle_data)
    return obstacle_datas


def findGoal(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([110, 50, 50])
    upper = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    # cv2.imshow('mask', mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_BayerRG2GRAY)
    # use opening to get rid of some small noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # use thresh to turn mask into binary
    res, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    # find contours
    goal_image, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow("goal", goal_image)
    # res_blue = cv2.drawContours(frame.copy(), contours_blue, -1, (0, 255, 0), 1)
    if len(contours) > 0:
        cnt = contours[0]
        x, y, w, h = cv2.boundingRect(cnt)
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        temp = abs(frame.shape[0] - h)
        # print(temp)
        if temp < 110 or w*h > 0.8*(frame.shape[0]*frame.shape[1]):
            return None
        return x+ 0.5*w
    else :
        return -1
