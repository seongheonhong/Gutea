#!/home/sbc-07/catkin_ws/src/robby_python/scripts/venv/bin/python
import rospy
from sensor_msgs.msg import Joy
import speech.client

JOY_ENABLED = True

def handle_joy(data):
    global JOY_ENABLED
    if data.buttons[2] == 1 and JOY_ENABLED:
        JOY_ENABLED = False
        rospy.loginfo("X Pressed!")
        speech.client.run_client()
    elif data.buttons[2] == 0 and not JOY_ENABLED:
        JOY_ENABLED = True

def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("joy", Joy, handle_joy)
    rospy.spin()


if __name__ == '__main__':
    listener()

