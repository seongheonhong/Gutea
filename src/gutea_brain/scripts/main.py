#!/home/sbc-07/catkin_ws/src/gutea_brain/scripts/.venv/bin/python
import rospy, rospkg
import pickle
import time
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from gutea_msgs.srv import DoSpeech
from gutea_msgs.msg import ClassificationResult
import algorithms


PKG_NAME = 'gutea_brain'
PKG_PATH = rospkg.RosPack().get_path(PKG_NAME)


##------------------- JOYSTICK ---------------------##

BUTTON_NAMES = ['A', 'B', 'X', 'Y']
PRESSED = {b:False for b in BUTTON_NAMES}
TOPIC_JOY = ('/joy', Joy)

def joy_handle(data):
    global PRESSED
    global BUTTON_NAMES

    for i, bn in enumerate(BUTTON_NAMES):
        b = data.buttons[i]
        if b == 1 and PRESSED[bn] is False:
            print(bn, 'pressed!')
            PRESSED[bn] = True
            if bn == 'A':
                pass
                # send_image()
            elif bn == 'B':
                pass
                # set_goal()
            elif bn == 'X':
                pass
                # navigate_rooms([1, 2, 3])
            elif bn == 'Y':
                start_program()


        elif b == 0 and PRESSED[bn] is True:
            PRESSED[bn] = False

##--------------------------------------------------##


##-------------- IMAGE CLASSIFICATION --------------##

TOPIC_INPUT_IMG = ('/gutea_brain/image', Image)
TOPIC_OUTPUT_RESULT = ('/gutea_image_recognition/result', String)
TOPIC_CAMERA = ('/usb_cam/image_raw', Image)
#TOPIC_CAMERA = ('/camera/color/image_raw', Image)

def send_image():
    print("Sending image...")
    global pub_img
    img_msg = rospy.wait_for_message(*TOPIC_CAMERA)
    print("Image fetched from Camera!")
    pub_img.publish(img_msg)


def classify_image():
    print("Classifying image...")
    send_image()
    data = rospy.wait_for_message(*TOPIC_OUTPUT_RESULT)
    return data.data


##--------------------------------------------------##


##----------------- GOAL SETTING  ------------------##

TOPIC_GOAL = ('move_base_simple/goal', PoseStamped)
# GOAL_ROOMS = [None, None, None, None, None, None]
# GOAL_HOME = None
GOAL_SAVE_PATH = PKG_PATH + '/goals.pkl'

def set_goal():
    global GOAL_HOME
    global GOAL_ROOMS
    while True:
        cmd = input("Specify which room you want to set goal:")
        try:
            cmd = int(cmd)
            if 0 <= cmd <= 6:
                break
            else:
                print("Not a valid index!")
                break
        except:
            print("Not a valid command!")
            pass
    try:
        pose_msg = rospy.wait_for_message(*TOPIC_GOAL)
        if cmd == 0:
            GOAL_HOME = pose_msg
        else:
            GOAL_ROOMS[cmd-1] = pose_msg
        print("Goal set!")
        print(pose_msg)

        # Save as pickle
        with open(GOAL_SAVE_PATH, 'wb') as f:
            pickle.dump((GOAL_ROOMS, GOAL_HOME), f)

    except Exception as e:
        print("ERROR:", e)

##--------------------------------------------------##


##------------------ NAVIGATION --------------------##

TOPIC_GOAL_REACHED = ('gutea_isGoalReached', Bool)
TOPIC_GOAL_REACHED_ONCE = ('gutty_goal_finish', Bool)

def navigate_to_goal(goal_msg):
    global pub_goal

    print("Publishing goal...")
    pub_goal.publish(goal_msg)
    print("Goal published")

    wait_until_goal_reached()

def wait_until_goal_reached():
    print("Waiting to reach goal...")
    goal_reached = False
    while goal_reached is False:
        print(goal_reached)
        reached_msg = rospy.wait_for_message(*TOPIC_GOAL_REACHED)
        goal_reached = reached_msg.data
    print("Goal reached!")

def goal_reached_handle(data):
    global pub_goal_finish
    if data.data == True:
        print("GOAL REACHED!!!")
        pub_goal_finish.publish(Bool(True))


##--------------------------------------------------##


##--------------------- SPEECH ---------------------##

SERV_SPEECH = 'gutea_do_speech'

def run_speech(mode, data=[]):
    print("Requesting to speech service...")
    rospy.wait_for_service(SERV_SPEECH)
    try:
        do_speech = rospy.ServiceProxy(SERV_SPEECH, DoSpeech)
        print("Performing speech module...")
        res = do_speech(mode, data)
        print("run_speech", res)
        print("Done!")
        return res.rooms
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


##--------------------------------------------------##


##---------------- MAIN ALGORITHM ------------------##

INSTRUCT_MODE = 0
END_MODE = 1

def start_program():
    global GOAL_ROOMS
    print("[BRAIN]: Starting program!!")

    # Receive commands
    rooms = run_speech(INSTRUCT_MODE)
    sorted_rooms = algorithms.sort_rooms(rooms)
    print(sorted_rooms)
    sorted_results = []

    # Navigate to all destinations
    for r in sorted_rooms:
        # cnt = 5
        # while cnt > 0:
            # print(cnt)
            # time.sleep(1)
            # cnt -= 1
        navigate_to_goal(GOAL_ROOMS[r-1])
        time.sleep(2)
        prediction = classify_image()
        sorted_results.append(ClassificationResult(r, prediction))

    results = sorted(sorted_results, key=lambda x: rooms.index(x.roomnum))
    
    # Return home
    navigate_to_goal(GOAL_HOME)
    run_speech(END_MODE, results)

    print("[BRAIN]: Program finished!!")

##--------------------------------------------------##


if __name__ == "__main__":
    print("Starting...")
    
    # Load saved goals
    with open(GOAL_SAVE_PATH, 'rb') as f:
        (GOAL_ROOMS, GOAL_HOME) = pickle.load(f)
        print("Goals loaded!")

    # Set publishers
    pub_img = rospy.Publisher(*TOPIC_INPUT_IMG, queue_size=1)
    pub_goal = rospy.Publisher(*TOPIC_GOAL, queue_size=1)
    pub_goal_finish = rospy.Publisher(*TOPIC_GOAL_REACHED_ONCE, queue_size=1)

    # Set subscribers
    rospy.Subscriber(*TOPIC_JOY, joy_handle)
    rospy.Subscriber(*TOPIC_GOAL_REACHED, goal_reached_handle)

    # Start node
    rospy.init_node(PKG_NAME, anonymous=True)
    
    # start_program()

    # Keep it spinning
    rospy.spin()

