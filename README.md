# mycroft_ros
This package provides capability to run the Text-To-Speech, Speech-To-Text and Skill functionality of Mycroft AI as ROS nodes providing topics, services and actions for communication.

The package makes use of the catkin_virtualenv library to run the Mycroft nodes within a seperate Python virtual environment
## Installation
1. Install Python virtualenvironment
```
sudo apt install python3-venv
```
2. Install packaging using pip
```
python -m pip install packaging --user
```
3. Clone catkin_virtualenv into your workspace 
```
cd workspace/src
git clone https://github.com/locusrobotics/catkin_virtualenv.git
cd catkin_virtualenv
git checkout 0.5.2
```
4. If you're using Python 3 you will need to edit the `catkin_virtualenv/catkin_virtualenv/scripts/global_requirements` file of the catkin_virtualenv package by changing the line ```import Queue as Queue``` at the top of the file to ```from queue import Queue```

5. Install catkin_virtualenv using pip.
```
cd workspace/src/catkin_virtualenv/catkin_virtualenv/
python setup.py install
```
6. Install catkin_virtualenv
```
cd workspace
catkin_make install --only-pkg-with-deps catkin_virtualenv
```
7. Run the dev_setup.sh
```
./workspace/src/mycroft_ros/dev.setup.sh
```
8. Build mycroft_ros
```
catkin_make --only-pkg-with-deps mycroft_ros
```

## Mycroft Setup
Mycroft can be configured using either **~/.mycroft/mycroft.conf** or **/etc/mycroft/mycroft.conf**
## Launching ROS Nodes
To launch all Mycroft nodes run mycroft.launch which will lanch the following nodes
* mycroft_message_bus.py
* mycroft_tts.py
* mycroft_skills.py
* mycroft_stt.py
```
roslaunch mycroft_ros mycroft.launch
```
## Topics, Services and Actions
### Topics
* **mycroft/speak (std_msgs/String)** - String for Mycroft TTS to read
* **mycroft/utterance (std_msgs/String)** - String to pass to the intent service of Mycroft
* **mycroft/speech (mycroft_ros/Speech)** - List of Strings received from Mycroft STT
* **mycroft/remove_skill (std_msgs/String)** - Path of skill to remove from SkillManager as a String
### Services
* **mycroft/register_skill (mycroft_ros/MycroftSkill)** - Register the skill in the SkillManager
### Actions
* **mycroft/get_response (mycroft_ros/GetResponse)** - Reads dialog before recording and returning user response
* **mycroft/ask_yesno (mycroft_ros/GetResponse)** - Reads dialog before recording and returning "yes" or "no" based on user 
response
## Example
### Skill/Node Structure
ROS nodes that are to be used as MycroftSkill's should be placed within their own directory with relevent 'vocab', 'dialog' and 'regex' directories to use for intents and entities
### Using the RosMycroftSkill
The mycroft_ros Python module provides helper functions and classes for convenience such as the RosMycroftSkill which can be used to register a node as a MycroftSkill in the Mycroft SkillManager
``` python
class ExampleSkill(RosMycroftSkill):

    def __init__(self):
        super(ExampleSkill, self).__init__("/home/vagrant/dev/catkin_ws/src/mycroft_ros/scripts/example")
        self.register_intent(IntentBuilder("MycroftRos").require("mycroft").require("ros"), self.example)
        self.register_intent_file("mytest.intent", self.mytest)
        if self.initialise():
            rospy.loginfo("MycroftSkill created")
        else:
            rospy.loginfo("Error creating MycroftSkill")

    def example(self, data):
        rospy.loginfo("MycroftRos callback")
        user_response = self.get_response(dialog="say something cool to me")
        rospy.loginfo("Response: " + user_response)

    def mytest(self, data):
        rospy.loginfo("mytest callback")
        rospy.loginfo("testentity: " + data.entities.get("testentity", ""))

if __name__ == "__main__":
    rospy.init_node('mycroft_skill_test')
    example = ExampleSkill()
    rospy.spin()
    rospy.on_shutdown(example.shutdown)
```
## TODO
* add context to Skills
* add speak_dialog to Skills
* add events / repeating events to Skills
* error handling for skills
