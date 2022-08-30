#!/home/sbc-07/catkin_ws/src/gutea_speech/scripts/.venv/bin/python
import rospy
import speech_client
from gutea_msgs.srv import DoSpeech, DoSpeechResponse


PKG_NAME = 'gutea_speech'
SERVICE_NAME = 'gutea_do_speech'

INSTRUCT_MODE = 0
END_MODE = 1


def handle_speech(req):
    if req.mode == INSTRUCT_MODE:
        rooms = speech_client.speak_instructions()
    elif req.mode == END_MODE:
        speech_client.speak_results(req.results)
        rooms = []
    print("handle_speech", rooms)

    return DoSpeechResponse(rooms)


if __name__ == '__main__':
    rospy.init_node(PKG_NAME)
    rospy.Service(SERVICE_NAME, DoSpeech, handle_speech)
    rospy.spin()
