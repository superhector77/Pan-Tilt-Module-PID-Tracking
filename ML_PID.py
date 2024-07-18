#-------------------------------Imports------------------------------#
from picamera2 import Picamera2, Preview
import time

import sys
sys.path.insert(1, "/home/pi/PCA9685/example/Jetson")
from ServoKit import *
from simple_pid import PID

import cv2 as cv
import numpy as np
import math
import random


#--------------------------------Setup-------------------------------#
"""
Variables for general use
Params:
    target: Placeholder for target color the camera will seek
    threshold: How many points each RGB value can be from the target
    h,w: Default height and width dimensions of picamera array
    center: Center of the image
    default_map: Weights for pixel values sorrounding detection center
"""
target = [0,0,0]
threshold = 50
h, w = 480, 640
center = [w/2, h/2]
default_mat = [0.3333/16, 0.3333/16, 0.3333/16, 0.3333/16, 0.3333/16,
               0.3333/16, 0.3333/8, 0.3333/8, 0.3333/8, 0.3333/16,
               0.3333/16, 0.3333/8, 0.3333/1, 0.3333/8, 0.3333/16,
               0.3333/16, 0.3333/8, 0.3333/8, 0.3333/8, 0.3333/16,
               0.3333/16, 0.3333/16, 0.3333/16, 0.3333/16, 0.3333/16]

"""
Setup for YoloV3 Detection
Params:
    classes: Human-readable labels for items being observed
    net: Loaded YoloV3 Neural Network
    ln: Layer Names for the model
"""
classes = open('/home/pi/Desktop/coco.names').read().strip().split('\n')
net = cv.dnn.readNetFromDarknet('/home/pi/Desktop/yolov3.cfg', 
                                '/home/pi/Desktop/yolov3.weights')
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
ln = net.getLayerNames()
ln = [ln[i - 1] for i in net.getUnconnectedOutLayers()]

"""
Sets up and starts Picamera modules.
Params:
    picam2a: Corresponds to camera 1
    picam2b: Corresponds to camera 2
    servoKit: Necessary for controlling servo motors
"""
picam2a = Picamera2(0)
camera_config = picam2a.create_preview_configuration()
picam2a.configure(camera_config)
picam2a.start_preview(Preview.QTGL)
picam2a.start()

picam2b = Picamera2(1)
picam2b.configure(camera_config)
picam2b.start_preview(Preview.QTGL)
picam2b.start()

servoKit = ServoKit(4)


#----------------------Image Processing Functions--------------------#
def np_arr(arr):
    """
    Returns Numpy array from Python list
    Params:
        arr: Multidimensional Python list which represents an image
        ( arr[row][col][0-3] )
    Returns:
        np_arr: Multidimensional Numpy array which contains only RGB
        values for each pixel ( np_arr[row][col][0-2] )
    """
    arr = [[[p[0], p[1], p[2]] for p in row] for row in arr]
    np_arr = np.array(arr, dtype=np.uint8)
    return np_arr

def img_detect(img, conf, goal_objs):
    """
    Detects objects in the image, and returns those with desired
    labels and high confidence
    Args:
        img: Numpy Array representing an image
        conf: Confidence threshold for detections.
        goal_objs: Sets what objects to look for. 
    Returns:
        processed: List which contains the center, confidence and label
                   for each valid detection.
    """
    blob = cv.dnn.blobFromImage(img, 1/255.0, (h, w), 
                                            swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward(ln)
    processed = []
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            if confidence > conf:
                box = detection[0:4] * np.array([w,h,w,h])
                (centerX, centerY, _, _) = box.astype("int")
                if classes[classID] in goal_objs or goal_objs == "ALL":
                    processed.append([(centerX,centerY), confidence,
                                                    classes[classID]])
    return processed

def color_finder(conf = 0.8, goal_objs = "ALL", mat = default_mat): 
    """
    Attempts to find objects using camera footage. When something
    is detected, mat is used to find calculate the weighted sum of the
    RGB values, which will be seeked by the cameras.
    Args:
        conf: Confidence threshold for detections.
        goal_objs: Sets what objects to look for. 
                   "ALL" objects by default.
        mat: List of float values which add up to ~1. Dimensions must
             be a square of an odd number. See default_mat 
    Returns:
        target: RGB color which will be seeked by the camera.
    
    WARNING: Make sure mat has ODD Dimensions (1, 3, 5, ...)
    """
    data_a = []
    data_b = []

    while len(data_a) == 0 and len(data_b) == 0:
        rand_ls = [int(random.random()* 180) for _ in range(4)]

        set_servo(0, rand_ls[0])
        set_servo(1, rand_ls[1])
        set_servo(2, rand_ls[2])
        set_servo(3, rand_ls[3])

        array_a = picam2a.capture_array("main")
        img_a = np_arr(array_a)
        data_a = img_detect(img_a, conf, goal_objs)

        array_b = picam2b.capture_array("main")
        img_b = np_arr(array_b)
        data_b = img_detect(img_b, conf, goal_objs)

    if len(data_a) != 0:
        target = color_extractor(array_a, data_a[0][0], mat)
    else:
        target = color_extractor(array_b, data_b[0][0], mat)
    return target

def color_extractor(arr, center, mat):
    """
    Finds color which will be seeked by taking a weighted sum of the
    colors around the center of the detected object
    Args:
        arr: Multidimensional Python list representing an image
        center: Center of the detected object
        mat: List of float values which add up to ~1.
    Returns:
        target: Color which will be seeked by the cameras
    """
    sz = math.sqrt(len(mat))
    colors = [arr[center[1]+y][center[0]+x] 
              for x in range(-math.ceil(sz/2), math.ceil(sz/2)) 
              for y in range(-math.ceil(sz/2), math.ceil(sz/2))]
    mult = [[colors[i][0]*mat[i], colors[i][1]*mat[i], colors[i][2]*mat[i]] for i in range(len(mat))]
    target = [ int(sum([i[0] for i in mult])),
               int(sum([j[1] for j in mult])),
               int(sum([k[2] for k in mult]))]
    return target

def cm_calculator(arr, target, res):
    """
    Finds center of mass of target color at a specified resolution.
    Args:
        arr: Mulidimensional Python list representing an image
        target: Color we wish to find the CM of
        res: Resolution at which we will look for target
    Returns:
        cm: Center of mass of target color
    """
    coord_array = [[],[]]
    for y_index in range(0,h//res):
        for x_index in range(0,w//res):
            c_r = arr[y_index*res][x_index*res]
            thing = [abs(c_r[i] - target[i]) for i in range(0,3)]
            if max(thing) < threshold:
                coord_array[0].append(x_index*res)
                coord_array[1].append(y_index*res)

    if len(coord_array[0]) != 0:
        cm = [int(sum(coord_array[0])/len(coord_array[0])), 
            int(sum(coord_array[1])/len(coord_array[1]))]
    else:
        cm = [3/4*w, h/2]
    return cm

def mat_maker(side, current_side=False):
    """
    Creates a multidimensional Python list which contains the weights
    for different pixel values around the center of the detection.
    Args:
        side: Side length of matrix
        current_side: counts layers of recursion in Python.
    Returns:
        mat: Multidimensional Python list with weights.
    """
    if not current_side:
        current_side = side
    if current_side == 1:
        return [[1/math.ceil(side/2)]]
    else:
        smaller = mat_maker(side,current_side-2)
        num = 1/(math.ceil(side/2)*4*(current_side-1))
        mat= [[num]*current_side] +[[num]+row+[num] for row in 
                                       smaller] + [[num]*current_side]
        return mat

def flatten(mat):
    """
    Flattens mat to be a one-dimensional array
    Args:
        mat: Output from mat_maker
    Returns:
        new_list: Flat copy of mat
    """
    new_list = []
    for el in mat:
        if type(el) == list:
            new_list += flatten(el)
        else:
            new_list.append(el)
    return new_list

#----------------------------Servo Controls--------------------------#
def set_servo(motor_id, angle):
    """
    Moves specified servo by specified step.
    Args:
        motor__id: 0 (tilt 1), 1 (pan 1), 2 (tilt 2), 3 (pan 2)
        angle: Goal for servo.
    """
    servoKit.setAngle(motor_id, angle)

def move_servo(motor_id, motor_step):
    """
    Moves specified servo by specified step.
    Args:
        motor__id: 0 (tilt 1), 1 (pan 1), 2 (tilt 2), 3 (pan 2)
        motor_step: Offset from current angle
    """
    set_servo(motor_id, servoKit.getAngle(motor_id) + motor_step)

class PID:
    """
    PID object which calculates adequate steps for motor based on
    detected cm vs center of camera.
    """
    def __init__(self, Kp, Kd, Ki, dt=2):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.last_error = 0
        self.integral = 0

    def update(self,error):
        deriv = (error-self.last_error)/self.dt
        self.integral += error *self.dt
        output = self.Kp* error + self.Kd * deriv + self.Ki * self.integral
        self.last_error = error
        return output

pan_pid1 = PID(0.025,0.003,0)
pan_pid2 = PID(0.025,0.003,0)
tilt_pid1 = PID(-0.025,0.003,0)
tilt_pid2 = PID(-0.025,0.003,0)


#---------------------------------MAIN-------------------------------#
auto_mat = flatten(mat_maker(9))
target = color_finder(mat=auto_mat)

while True:
    array_a = picam2a.capture_array("main")
    cm_a = cm_calculator(array_a, target, 5)

    array_b = picam2b.capture_array("main")
    cm_b = cm_calculator(array_b, target, 5)

    ##pan_motor pid 1
    pan_step1 = pan_pid1.update(cm_a[0]-center[0])
    move_servo(1, int(pan_step1))

    ##pan_motor pid 2
    pan_step2 = pan_pid2.update(cm_b[0]-center[0])
    move_servo(3, int(pan_step2))

    ##tilt_motor pid 1
    tilt_step1 = tilt_pid1.update(cm_a[1]-center[1])
    move_servo(0, int(tilt_step1))

    ##tilt_motor pid 2
    tilt_step2 = tilt_pid2.update(cm_b[1]-center[1])
    move_servo(2, int(tilt_step2))