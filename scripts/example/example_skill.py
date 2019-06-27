#!/usr/bin/env python
import rospy
from mycroft_ros.msg import *
from mycroft_ros.srv import *
from std_msgs.msg import String
import actionlib
from mycroft_ros.helpers import RosMycroftSkill, IntentBuilder

class ExampleSkill(RosMycroftSkill):

    def __init__(self):
        super(ExampleSkill, self).__init__("/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/example")
        self.register_intent(IntentBuilder("MycroftRos").require("mycroft").require("ros"), self.example)
        self.register_intent_file("mytest.intent", self.mytest)
        self.register_intent_file("random.lol.intent", self.random)
        self.register_entity_file("thing.entity")
        if self.initialise():
            rospy.loginfo("created")
        else:
            rospy.loginfo("not created")

    def example(self, data):
        rospy.loginfo("MycroftRos callback")
        user_response = self.get_response(dialog="say something cool to me")
        rospy.loginfo("Response: " + user_response)

    def mytest(self, data):
        rospy.loginfo("mytest callback")

    def random(self, data):
        rospy.loginfo("random.lol.intent callback")

if __name__ == "__main__":
    rospy.init_node('mycroft_skill_test')
    example = ExampleSkill()
    rospy.spin()
    rospy.on_shutdown(example.shutdown)

