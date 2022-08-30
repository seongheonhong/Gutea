#!/home/sbc-07/catkin_ws/src/gutea_image_recognition/scripts/venv/bin/python
import rospy
from sensor_msgs.msg import Image
import ros_numpy
import numpy as np
import ml_model
import time
print("BEGIN: main.py")


def handle_img(img_msg):
    print("Checking image...")
    img = ros_numpy.numpify(img_msg)
    ml_model.run_classification(img)

if __name__ == '__main__':
    print("Starting...")
    rospy.init_node('gutea_image_recognition')
    while True:
        print("Waiting for messages..")
        msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        print("Message received!")
        handle_img(msg)
    #sub = rospy.Subscriber('/camera/color/image_raw', Image, handle_img, queue_size=1)
    #rospy.spin()
print("END: main.py")
