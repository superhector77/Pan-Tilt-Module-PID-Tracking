from picamera2 import Picamera2, Preview
import time

import sys
sys.path.insert(1, "/home/pi/PCA9685/example/Jetson")
from ServoKit import *
from simple_pid import PID


# Variables globales para manipulacion de imagenes
gc = [227,25,25]
threshold = 50
h, w = 480, 640
center = [w/2, h/2]


# Setup para PiCamera
picam2a = Picamera2(0)
camera_config = picam2a.create_preview_configuration()
picam2a.configure(camera_config)
picam2a.start_preview(Preview.QTGL)
picam2a.start()

picam2b = Picamera2(1)
picam2b.configure(camera_config)
picam2b.start_preview(Preview.QTGL)
picam2b.start()


# Calcula y devuelve el centro de masa de un objeto con el color gc con resolucion reducida
def cm_calculator(img_arr, res):
    coord_array = [[],[]]
    for y_index in range(0,h//res):
        for x_index in range(0,w//res):
            c_r = img_arr[y_index*res][x_index*res]
            thing = [abs(c_r[i] - gc[i]) for i in range(0,3)]
            if max(thing) < threshold:
                coord_array[0].append(x_index*res)
                coord_array[1].append(y_index*res)

    if len(coord_array[0]) != 0:
        cm = [int(sum(coord_array[0])/len(coord_array[0])), 
            int(sum(coord_array[1])/len(coord_array[1]))]
        return cm
    else:
        return [3/4*w, h/2]


servoKit = ServoKit(4)
def move_servo(motor_id, motor_step):
#motor__id: 0 (tilt 1), 1 (pan 1), 2 (tilt 2), 3 (pan 2)
#motor_step: Offset from current angle
    servoKit.setAngle(motor_id, servoKit.getAngle(motor_id) + motor_step)

class PID:
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self,error,dt):
        deriv = (error-self.last_error)/dt
        self.integral += error *dt
        output = self.Kp* error + self.Kd * deriv + self.Ki * self.integral
        self.last_error = error
        return output

pan_pid1 = PID(0.025,0.003,0)
pan_pid2 = PID(0.025,0.003,0)
tilt_pid1 = PID(-0.025,0.003,0)
tilt_pid2 = PID(-0.025,0.003,0)
dt = 2

def basic_control(error, threshold):
    if abs(error) <= threshold:
        return 0
    if error > 0:
        return 1
    if error < 0:
        return -1



def set_servo(motor_id, angle):
    """
    Moves specified servo by specified step.
    Args:
        motor__id: 0 (tilt 1), 1 (pan 1), 2 (tilt 2), 3 (pan 2)
        angle: Goal for servo.
    """
    servoKit.setAngle(motor_id, angle)


set_servo(3,0)
while True:
    move_servo(3,5)
    time.sleep(1)

"""
while True:
    array_a = picam2a.capture_array("main")
    cm_a = cm_calculator(array_a,5)

    array_b = picam2b.capture_array("main")
    cm_b = cm_calculator(array_b,5)

    ##pan_motor pid 1
    pan_step1 = pan_pid1.update(cm_a[0]-center[0], dt)
    #pan_step = basic_control(current_cm[0]-center[0], 10)
    #print(pan_step)
    move_servo(1, int(pan_step1))

    ##pan_motor pid 2
    pan_step2 = pan_pid2.update(cm_b[0]-center[0], dt)
    move_servo(3, int(pan_step2))

    ##tilt_motor pid 1
    tilt_step1 = tilt_pid1.update(cm_a[1]-center[1], dt)
    #tilt_step = basic_control(current_cm[1]-center[1], 10)
    #print(tilt_step)
    move_servo(0, int(tilt_step1))

    ##tilt_motor pid 2
    tilt_step2 = tilt_pid2.update(cm_b[1]-center[1], dt)
    move_servo(2, int(tilt_step2))
"""