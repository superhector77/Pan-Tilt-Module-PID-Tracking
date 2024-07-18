# Pan-Tilt-Module-PID-Tracking
For use with the Arducam Pan-Tilt Platform Kit for Raspberry Pi Camera, Adafruit PCA9685, and Raspberry Pi Camera Module 3. 

The goal of this project was to write code for the Pan-Tilt Platform Kit by Arducam that allows 2 of these to track objects. 

To avoid messing up the Raspberry Pi hardware and the I2C pins, we used the PCA9685 circuit board by Adafruit and connected all the motors directly into it, which allows us to use Adafruit drivers.

An additional issue was that PiCamera 3 has issues with OpenCV. To get around this, we used the Picamera2 Python module, and extracted the capture_array function to extract images and do any processing

PID_control was the first attempt at writing code that can track objects. It searches for a specified RGB value (within an adjustable threshold and resolution) in the image, and does a center of mass (CoM) calculation to find our target for the camera. Then, using the difference between the CoM and the center of the camera, the pan and tilt motors are actuated using a PID algorithm. 

ML_PID has much more detailed documentation. Initially, it used a yolov3 model for object detection on each captured frame, but this was far too slow for any practical application. To improve performance, we search for objects until one is found (with an adjustable confidence threshold), and then take a weighted average of the colors sorrounding the center of the detection area. Then, for each captured frame, we seek the CoM of this average color. This of course works best in cases where the object you look for is a different color from your environment.

