#!/usr/bin/env python

import numpy as np
import cv2
import time
import base64
# start the video
cap = cv2.VideoCapture(0)


def rescale_frame(frame, percent=75):
    #width = int(frame.shape[1] * percent / 100)
    #height = int(frame.shape[0] * percent / 100)
    dim = (400, 400)
    return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)


def getVideo():
    rect, frame = cap.read()
    f2 = rescale_frame(frame, 20)
    retval, buffer = cv2.imencode('.jpg', f2)
    return base64.b64encode(buffer)


def cleanUp():
    cap.release()
    cv2.destroyAllWindows()
