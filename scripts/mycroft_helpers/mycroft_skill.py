import rospy
from mycroft_ros.msg import *
from mycroft_ros.srv import *
from std_msgs.msg import String
import actionlib

response_client = None
ask_yesno_client = None

class IntentManager:
    
    def __init__(self, intents={}):
        self.intents = intents

    def remove_callback(self, intent_name):
        del self.intents[intent_name]

    def register_callbacks(self, intents):
        self.intents.update(intents)
        return self

    def handle_intent(self, data):
        intent_func = self.intents.get(data.intent_name, None)
        if intent_func is not None:
            intent_func(data)
  
    def register_callback(self, intent_name, func):
        self.intents[intent_name] = func
        return self

class SkillBuilder:

    def __init__(self, path=""):
        self.path = path
        self.intents = []
        self.intent_files = []
        self.entities = []

    def intent(self, intent):
        self.intents.append(intent)
        return self

    def intent_file(self, intent_file):
        self.intent_files.append(intent_file)
        return self

    def entity(self, entity):
        self.entities.append(entity)
        return self

    def build(self):
        return Mycroft(self.path, self.intents, self.intent_files, self.entities)

    def buildAndRegister(self):
        return register_skill(Mycroft(self.path, self.intents, self.intent_files, self.entities))


class IntentBuilder:
    
    def __init__(self, intent_name):
        self.at_least_one = []
        self.requires = []
        self.optional = []
        self.name = intent_name

    def one_of(self, *args):
        self.at_least_one.append(args)
        return self

    def require(self, entity_type, attribute_name=None):
        if not attribute_name:
            attribute_name = entity_type
        self.requires.append(IntentRequire(entity=entity_type, attribute_name=attribute_name))
        return self

    def optionally(self, entity_type, attribute_name=None):
        if not attribute_name:
            attribute_name = entity_type
        self.optional.append(IntentOptional(entity=entity_type, attribute_name=attribute_name))
        return self

    def build(self):
        return Intent(self.name, self.requires, self.at_least_one, self.optional)

def get_response(skill_path, dialog, client=None):
    if client is None:
        client = actionlib.SimpleActionClient('get_response', GetResponseAction)

    client.wait_for_server()
    goal = GetResponseGoal(skill_path, dialog)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result.response

def ask_yesno(skill_path, dialog, client=None):
    if client is None:
        client = actionlib.SimpleActionClient('ask_yesno', GetResponseAction)

    client.wait_for_server()
    goal = GetResponseGoal(skill_path, dialog)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    return result.response

def register_skill(mycroft_skill):
    rospy.wait_for_service('mycroft/register_skill')
    try:
        register = rospy.ServiceProxy('mycroft/register_skill', MycroftSkill)
        response = register(mycroft_skill)
        return response.initialised
    except rospy.ServiceException:
        rospy.loginfo("Service call failed")
