
# Pan-Tilt-Module-PID-Tracking

## Overview

This project is designed to work with the Arducam Pan-Tilt Platform Kit for the Raspberry Pi Camera, the Adafruit PCA9685, and the Raspberry Pi Camera Module 3. The goal of this project is to develop Python (3.11.2) code that allows two Pan-Tilt platforms to track objects.

## Table of Contents

1.  [Hardware](#hardware)
2.  [Setup](#setup)
3.  [Software](#software)
    -   [PID_control](#pid_control)
    -   [ML_PID](#ml_pid)
4.  [Installation](#installation)
5.  [Acknowledgements](#acknowledgements)
6.  [Contact Information](#contact-information)

## Hardware

-   **Raspberry Pi 5**
-   **Arducam Pan-Tilt Platform Kit**
-   **Raspberry Pi Camera Module 3**
-   **Adafruit PCA9685**

## Setup

To avoid potential issues with the Raspberry Pi hardware and the I2C pins, we use the PCA9685 circuit board by Adafruit. This allows us to connect all motors directly to it and use previously published code. This is effectively a fork of the [ArduCAM/PCA9685](https://github.com/ArduCAM/PCA9685) repository (but because I am new to git and GitHub, I do not know how to set that up), so please check that out for more documentation and a wiring diagram (The one for Jetson platforms works great except for how you connect the circuit board to the Raspberry Pi).

The OS used is `Debian GNU/Linux 12 (bookworm)`, the latest version of Raspbian available in June 1 2024, and it was installed via the [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

## Software

An additional challenge was that the Raspberry Pi Camera Module 3 has compatibility issues with OpenCV. To address this, we used the Picamera2 Python module, utilizing the `capture_array` function to extract images and perform the necessary processing.

### PID_control

`PID_control` was our initial attempt at writing code capable of tracking objects. It searches for a specified RGB value (within an adjustable threshold and resolution) in the image and calculates the center of mass (CoM) to locate the target. The difference between the CoM and the center of the camera is used to actuate the pan and tilt motors via a PID algorithm.

### ML_PID

`ML_PID` has much more detailed comments. Initially, it used a YOLOv3 model for object detection on each captured frame, but this proved too slow for practical application. To improve performance, we search for objects until one is found (with an adjustable confidence threshold) by moving the cameras in randomized directions. Then, we take a weighted average of the colors surrounding the center of the detection area. For each captured frame, we seek the CoM of this average color. This approach works best when the target object is a different color from its environment.

## Installation

Some of the required Python packages include:

- `picamera2`
- `time`
- `sys`
- `cv2`
- `numpy`
- `math`
- `random`
  
I unfortunately did not keep track of all the files, packages and libraries I installed, but I wanted to include that if you are having issues with installing the required Python packages with `pip` outside a virtual environment (research before taking this route, but it worked for me), you could use the `--break-system-packages` argument.

The `yolov3.weights` file was too large to upload to GitHub, so download it from [here](https://www.dropbox.com/scl/fi/nzwga4u87ytv7dowcynsp/yolov3.weights?rlkey=d117ida2e4fid4mrq9ov31rg5&st=tpxar9ye&dl=0), or look for a different set of weights and configurations online.

## Acknowledgements

-   Thanks to Dr. Hiram Ponce for his guidance with the project, and the Universidad Panamericana for their hospitality.
-   MIT's MISTI Mexico Program for the opportunity.

## Contact Information

If you have any questions or feedback, feel free to reach out at hlugaro@mit.edu.
