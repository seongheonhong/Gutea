#!/home/sbc-07/catkin_ws/src/gutea_image_recognition/scripts/venv/bin/python
import rospy
from sensor_msgs.msg import Joy, Image
from custom_msgs.srv import *
import ros_numpy
import numpy as np
import PIL.Image


button_names = ['A', 'B', 'X', 'Y']
pressed = {b:False for b in button_names}

filename = '/tmp/cat.jpg'

def send_image():
    print("Sending image...")
    #img_msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    rospy.wait_for_service('image_classification')
    print("Sending...")
    try:
        img = np.asarray(PIL.Image.open(filename).resize((224,224)))
        img_msg = ros_numpy.msgify(img)
        print("Sending...")
        image_classification = rospy.ServiceProxy('image_classification', ImageClassification)
        resp = image_classification(img_msg)
        print(resp) 
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)



def joy_handle(data):
    global pressed
    global button_names

    for i,bn in enumerate(button_names):
        b = data.buttons[i]
        if b == 1 and pressed[bn] is False:
            print(bn, 'pressed!')
            pressed[bn] = True
            if bn == 'A':
                send_image()
        elif b == 0 and pressed[bn] is True:
            pressed[bn] = False


if __name__ == "__main__":
    print("Starting...")
    rospy.init_node('gutea_test', anonymous=True)
    while True:
        cmd = input("waiting...")
        if cmd == "":
            send_image()

    rospy.Subscriber("/joy", Joy, joy_handle)
    rospy.spin()
