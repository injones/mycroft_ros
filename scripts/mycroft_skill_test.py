#!/usr/bin/env python
import rospy
from mycroft_ros.msg import *
from mycroft_ros.srv import *
from std_msgs.msg import String
import actionlib
from mycroft_helpers.mycroft_skill import get_response, ask_yesno, Manager

from mycroft.messagebus.client.ws import WebsocketClient
from mycroft.messagebus.message import Message
from mycroft.util import (
    connected, wait_while_speaking, reset_sigint_handler,
    create_echo_function, create_daemon, wait_for_exit_signal
)

bus = None  # Mycroft messagebus reference, see "mycroft.messagebus"

def connect():
    global bus
    bus.run_forever()

def check_working(message):
    if message is not None:
        print('do somethingggg')
        print(message.data)
    else:
        print('do something else')

def get_response2(skill_path, dialog):
    client = actionlib.SimpleActionClient('get_response', GetResponseAction)
    client.wait_for_server()
    goal = GetResponseGoal(skill_path, dialog)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result.response

def ask_yesno2(skill_path, dialog):
    client = actionlib.SimpleActionClient('ask_yesno', GetResponseAction)
    client.wait_for_server()
    goal = GetResponseGoal(skill_path, dialog)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result.response

def intent_callback(message):
    intent = message.intent_name
    #print(','.join([print('Entity: ' + i.entity + 'Value: ' + i.value) for i in message.entities)]))
    [print('Entity: ' + i.entity + ' Value: ' + i.value) for i in message.entities]
    if intent == 'testertesting':
        print('testing says: ' + message.utterance)
        user_response = get_response('/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/tester', 'testing user response')
        print('lol ' + user_response)
    elif intent == 'testerMycroftRos':
        print('MycroftRos says: ' + message.utterance)
        yesno = ask_yesno('/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/tester', 'yes or no?')
        print('lol ' + yesno)
    elif intent == 'testermytest':
        print('mytest says: ' + message.utterance)
    elif intent == 'testerrandomlol':
        print('randomlol says: ' + message.utterance)

def register_skill():
    rospy.wait_for_service('mycroft/register_skill')
    try:
        req = IntentRequire(entity="tests")
        intObj = Intent(name="testing", requires=[req])
        skill = Mycroft(path="/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/tester", intents=[intObj, Intent(name="MycroftRos", requires=[IntentRequire(entity="mycroft"), IntentRequire(entity="ros")])], intent_files=["mytest.intent", "random.lol.intent"], entities=["thing.entity"])
        register = rospy.ServiceProxy('mycroft/register_skill', MycroftSkill)
        response = register(skill)
        return response.initialised
    except rospy.ServiceException:
        print("Service call failed")

def second_intent(message):
    print('hello')

def main():
    #pub = rospy.Publisher('mycroft/register_node', Mycroft, queue_size=10)
    rospy.init_node('mycroft_skill_test')
    rospy.loginfo(rospy.get_caller_id() + " started")
    manager = Manager().addIntent('testerMycroftRos', second_intent)
    rospy.Subscriber('mycroft/tester', IntentResponse, manager.handleIntent)
    #rospy.Subscriber('mycroft/tester', IntentResponse, intent_callback)
    global bus
    reset_sigint_handler()
    bus = WebsocketClient()
    payload = {
        'utterances': ["this is a test of my skill"],
        'lang': "en-us"
    }
    create_daemon(bus.run_forever)
    #req = IntentRequire(entity="tests")
    #intObj = Intent(name="testing", requires=[req])
    #skill = Mycroft(path="/home/vagrant/dev/catkin_ws/src/mycroft_test/scripts/tester", intents=[intObj, Intent(name="MycroftRos", requires=[IntentRequire(entity="mycroft"), IntentRequire(entity="ros")])])
    #pub.publish(skill)
    if register_skill():
        print("Successfully created MycroftSkill")
    else:
        print("Error occurred")
    wait_for_exit_signal()
    rospy.spin()

if __name__ == '__main__':
    main()
