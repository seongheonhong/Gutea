#!/home/sbc-07/catkin_ws/src/gutea_image_recognition/scripts/venv/bin/python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ros_numpy
import ml_model


TOPIC_INPUT_IMG = '/gutea_brain/image'
TOPIC_OUTPUT_RESULT = '/gutea_image_recognition/result'

def handle_img(img_msg):
    print("Checking image...")
    img = ros_numpy.numpify(img_msg)
    return ml_model.run_classification(img)

if __name__ == '__main__':
    print("Starting...")
    pub = rospy.Publisher(TOPIC_OUTPUT_RESULT, String, queue_size=1)
    rospy.init_node('gutea_image_recognition')
    while True:
        # wait for message
        print("Waiting for messages..")
        msg = rospy.wait_for_message(TOPIC_INPUT_IMG, Image)
        print("Message received!")
        res = handle_img(msg)

        # publish result
        pub.publish(String(res))
