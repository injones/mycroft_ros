#!/usr/bin/env python
import rospy
from mycroft_ros.msg import Mycroft, Intent, GetResponse
from std_msgs.msg import String

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

def main():
    pub = rospy.Publisher('mycroft/response', GetResponse, queue_size=10)
    rospy.init_node('mycroft_skill_test2')
    rospy.loginfo(rospy.get_caller_id() + " started")
    global bus
    reset_sigint_handler()
    bus = WebsocketClient()
    payload = {
        'utterances': ["this is a test of my skill"],
        'lang': "en-us"
    }
    create_daemon(bus.run_forever)
    bus.on('speak', check_working)
    #bus.emit(Message("recognizer_loop:utterance", payload))
    #bus.emit(Message("recognizer_loop:utterance", {'utterances': ["mycroft with ros"], 'lang': "en-us"}))
    bus.emit(Message("recognizer_loop:utterance", {'utterances': ["testing the test intent file LULW"], 'lang': "en-us"}))
    #pub.publish(GetResponse("/home/vagrant/dev/catkin_ws/src/mycroft_test/scripts/tester", "say something pls?"))
    wait_for_exit_signal()
    rospy.spin()

if __name__ == '__main__':
    main()
