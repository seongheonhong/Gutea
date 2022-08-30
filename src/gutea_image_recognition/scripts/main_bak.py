#!/home/sbc-07/catkin_ws/src/gutea_image_recognition/scripts/venv/bin/python
import rospy
from sensor_msgs.msg import Image
from custom_msgs.srv import *
import ros_numpy
import numpy as np
import ml_model
import time
import threading
print("BEGIN: main.py")

LISTEN = True
def handle_img(req):
    print("THREAD (CALLBACK - MAIN):", threading.get_ident())
    global LISTEN
    print("Listen")
    if LISTEN:
        print("Checking image...")
        LISTEN = False
        img = ros_numpy.numpify(req.image)
        res = ml_model.run_classification(img)
        LISTEN = True
    else:
        print("Ignoring...")
        res = -1
    return ImageClassificationResponse(res)

if __name__ == '__main__':
    print("Starting...")
    print("THREAD (MAIN):", threading.get_ident())
    rospy.init_node('gutea_image_recognition')
    rospy.Service('image_classification', ImageClassification, handle_img)
    rospy.spin()
print("END: main.py")
