#!/usr/bin/env python
import rospy
from mycroft_ros.msg import *
from mycroft_ros.srv import *
from std_msgs.msg import String
import actionlib
from mycroft_ros.helpers import *

path = "/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/tester"

def mycroft_ros_callback(message):
    print('hello')
    user_response = get_response(skill_path=path, dialog='can you say something to me?')
    rospy.loginfo(user_response)

def main():
    rospy.init_node('mycroft_skill_test')
    rospy.loginfo(rospy.get_caller_id() + " started")
    manager = IntentManager().register_callback('testerMycroftRos', mycroft_ros_callback) \
       .register_callback('testermytest', mycroft_ros_callback)
    rospy.Subscriber('mycroft/tester', IntentResponse, manager.handle_intent)
    my_intent = IntentBuilder("MycroftRos").require("mycroft").require("ros").build()
    initialised = SkillBuilder(path).intent(my_intent).intent_file("mytest.intent") \
    .intent_file("random.lol.intent").entity("thing.entity").buildAndRegister()
    if initialised:
       print("created")
    else:
       print("not created")
    rospy.spin()

if __name__ == '__main__':
    main()
