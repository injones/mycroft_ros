# mycroft_ros
This package provides the capability to run the Text-To-Speech, Speech-To-Text and Skill functionality of Mycroft AI as ROS nodes providing topics, services and actions for communication.

The package makes use of the catkin_virtualenv library to run each node within a seperate Python virtual environment
## Installation
The dev_setup.sh can be used to install dependencies and configure the package for use. 
## Mycroft Setup
Mycroft can be configured as normal using the mycroft.conf
## TODO
* STT
* add context to Skills
* add speak_dialog
* add Skill events / repeating events
* add support for Streamlabs TTS?
