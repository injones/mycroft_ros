#!/usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import String, Bool
from mycroft_ros.msg import Mycroft, IntentResponse, IntentResponseEntity as Entity, GetResponseAction, GetResponseResult
from mycroft_ros.srv import MycroftSkill as MycroftService, MycroftSkillResponse, MycroftSkillRequest

import time
import re
from threading import Timer
import mycroft.lock
import pathlib
from os.path import basename
from mycroft import dialog
from mycroft.api import is_paired, BackendDown
from mycroft.enclosure.api import EnclosureAPI
from mycroft.configuration import Configuration
from mycroft.messagebus.client.ws import WebsocketClient
from mycroft.messagebus.message import Message
from mycroft.util import (
    connected, wait_while_speaking, reset_sigint_handler,
    create_echo_function, create_daemon, wait_for_exit_signal
)
from mycroft.util.log import LOG
from mycroft.util.lang import set_active_lang
from mycroft.skills.skill_manager import SkillManager, MsmException
from mycroft.skills.core import FallbackSkill, MycroftSkill
from mycroft.skills.event_scheduler import EventScheduler
from mycroft.skills.intent_service import IntentService
from mycroft.skills.padatious_service import PadatiousService
from adapt.intent import Intent, IntentBuilder

bus = None  # Mycroft messagebus reference, see "mycroft.messagebus"
event_scheduler = None
skill_manager = None
topic_name = "checking"
connect_to_mycroft_backend = True
get_response_server = None
ask_yesno_server = None

# Remember "now" at startup.  Used to detect clock changes.
start_ticks = time.monotonic()
start_clock = time.time()

class GetResponseServer:
    bus = None
    manager = None
    result = GetResponseResult()

    def __init__(self, manager):
        self.manager = manager
        self.server = actionlib.SimpleActionServer('get_response', GetResponseAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        global skill_manager
        skill_path = goal.skill_path
        #skill = skill_manager.loaded_skills.get(skill_path, {})
        skill = self.manager.loaded_skills.get(skill_path, {})
        self.result.response = skill["instance"].get_response(dialog=goal.dialog)
        self.server.set_succeeded(self.result)

class AskYesNoServer:
    bus = None
    manager = None
    result = GetResponseResult()

    def __init__(self, manager):
        self.manager = manager
        self.server = actionlib.SimpleActionServer('ask_yesno', GetResponseAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        skill_path = goal.skill_path
        skill = self.manager.loaded_skills.get(skill_path, {})
        self.result.response = skill["instance"].ask_yesno(prompt=goal.dialog)
        self.server.set_succeeded(self.result)

def handle_get_response(data):
    global bus, skill_manager
    skill_path = data.skill_path
    skill = skill_manager.loaded_skills.get(skill_path, {})
    print(data.dialog)
    if skill["instance"] is not None:
        text = skill["instance"].get_response(dialog=data.dialog)
        if text is not None:
            print(text)
        else:
            print("was none")

def connect():
    global bus
    bus.run_forever()

def _starting_up():
    """
        Start loading skills.
        Starts
        - SkillManager to load/reloading of skills when needed
        - a timer to check for internet connection
        - adapt intent service
        - padatious intent service
    """
    global bus, skill_manager, event_scheduler, connect_to_mycroft_backend

    bus.on('intent_failure', FallbackSkill.make_intent_failure_handler(bus))

    # Create the Intent manager, which converts utterances to intents
    # This is the heart of the voice invoked skill system
    service = IntentService(bus)
    try:
        PadatiousService(bus, service)
    except Exception as e:
        LOG.exception('Failed to create padatious handlers '
                      '({})'.format(repr(e)))
    event_scheduler = EventScheduler(bus)

    # Create a thread that monitors the loaded skills, looking for updates
    try:
        skill_manager = SkillManager(bus)
    except MsmException:
        # skill manager couldn't be created, wait for network connection and
        # retry
        LOG.info('Msm is uninitialized and requires network connection',
                 'to fetch skill information\n'
                 'Waiting for network connection...')
        while not connected():
            time.sleep(30)
        skill_manager = SkillManager(bus)

    skill_manager.daemon = True
    # Wait until priority skills have been loaded before checking
    # network connection
    # print(skill_manager.msm.repo.get_default_skill_names())
    skill_manager.load_priority()
    skill_manager.start()
    bus.emit(Message('skill.manager.initialised'))
    if connect_to_mycroft_backend:
       check_connection()
    else:
       check_connection_without_backend()

def shutdown():
    if event_scheduler:
        event_scheduler.shutdown()

    # Terminate all running threads that update skills
    if skill_manager:
        skill_manager.stop()
        skill_manager.join()

def try_update_system(platform):
    bus.emit(Message('system.update'))
    msg = Message('system.update', {
        'paired': is_paired(),
        'platform': platform
    })
    resp = bus.wait_for_response(msg, 'system.update.processing')

    if resp and (resp.data or {}).get('processing', True):
        bus.wait_for_response(Message('system.update.waiting'),
                              'system.update.complete', 1000)

def check_connection_without_backend():
    if connected():
        LOG.info('here')
        bus.emit(Message('mycroft.internet.connected'))

def handle_utterance(data):
    global bus
    bus.emit(Message("recognizer_loop:utterance", {'utterances': [data.data], 'lang': "en-us"}))

def register_intents(instance, skill_topic, intent_files=None, entities=None, intents=None):
    pub = rospy.Publisher(skill_topic, IntentResponse, queue_size=10)
    if intent_files:
        for intent_file in intent_files:
            instance.register_intent_file(intent_file, lambda m : pub.publish(IntentResponse(re.sub(r'[\.:](intent)?', '', m.type), m.data['utterance'], [Entity(entity, value) for entity, value in m.data.items() if entity != 'utterance'])))
    if intents:
        for intent in intents:
            instance.register_intent(Intent(name=intent.name, requires=[(req.entity, req.attribute_name) if req.attribute_name != '' else (req.entity, req.entity) for req in intent.requires], at_least_one=intent.at_least_one, optional=[(opt.entity, opt.attribute_name) if opt.attribute_name != '' else (opt.entity, opt.entity) for opt in intent.optional]), lambda m : pub.publish(IntentResponse(m.data['intent_type'].replace(':', ''), m.data['utterance'], [Entity(entity, value) for entity, value in m.data.items() if entity not in ('intent_type', 'target', 'confidence', '__tags__', 'utterance')])))
    if entities:
        for entity in entities:
            instance.register_entity_file(entity)

def handle_remove_skill(data):
    """
        Removes the skill using its path.
        Arguements:
            data (String): Skill directory path
    """
    global bus, skill_manager
    skill_path = data.data
    skill = skill_manager.loaded_skills.get(skill_path, {})
    # set is_ros_node to False to let the SkillManager handle skill removal
    skill["is_ros_node"] = False

def handle_register_skill(data):
    global bus, skill_manager
    mycroft_skill = data.skill
    skill_path = mycroft_skill.path.rstrip('/')
    skill_id = basename(skill_path)
    skill = skill_manager.loaded_skills.setdefault(skill_path, {})
    skill.update({"id": skill_id, "path": skill_path})
    skill["loaded"] = True
    skill["is_ros_node"] = True
    instance = MycroftSkill(name=skill_id)
    instance.skill_id = skill_id
    instance.bind(bus)
    try:
        instance.load_data_files(skill_path)
        skill_topic = 'mycroft/' + skill_id
        register_intents(instance=instance, intents=mycroft_skill.intents, skill_topic=skill_topic, intent_files=mycroft_skill.intent_files, entities=mycroft_skill.entities)
        instance.initialize()
    except Exception as e:
        instance.default_shutdown()
        raise e
    skill["instance"] = instance
    modified = 0
    if skill['instance'] is not None:
        bus.emit(Message('mycroft.skills.loaded', {'path': skill_path, 'id': skill['id'], 'name': skill['instance'].name, 'modified': modified}))
        return MycroftSkillResponse(True)
    else:
         bus.emit(Message('mycroft.skills.loading_failure', {'path': skill_path, 'id': skill['id']}))
         return MycroftSkillResponse(False)

def mycroft_register_node(data):
    """ Create new MycroftSkill for ROS node and load into SkillManager

    Args:
        data: mycroft/register_node topic payload containing Mycroft msg

    """
    global bus, skill_manager
    skill_path = data.path.rstrip('/')
    skill_id = basename(skill_path)
    skill = skill_manager.loaded_skills.setdefault(skill_path, {})
    skill.update({"id": skill_id, "path": skill_path})
    skill["loaded"] = True
    skill["is_ros_node"] = True
    instance = MycroftSkill(name=skill_id)
    instance.bind(bus)
    try:
        instance.load_data_files(skill_path)
        skill_topic = 'mycroft/' + skill_id
        register_intents(instance, data.intents, skill_topic)
        instance.initialize()
    except Exception as e:
        instance.default_shutdown()
        raise e
    skill["instance"] = instance
    modified = 0
    if skill['instance'] is not None:
        bus.emit(Message('mycroft.skills.loaded', {'path': skill_path, 'id': skill['id'], 'name': skill['instance'].name, 'modified': modified}))
        return
    else:
         bus.emit(Message('mycroft.skills.loading_failure', {'path': skill_path, 'id': skill['id']}))

def ros_skill(data):
    global skill_manager, bus
    skill_path = '/home/vagrant/dev/catkin_ws/src/mycroft_test/scripts/tester'
    skill = skill_manager.loaded_skills.setdefault('tester', {})
    skill.update({"id": 'tester', "path": '/home/vagrant/dev/catkin_ws/src/mycroft_test/scripts/tester'})
    skill["loaded"] = True
    skill["is_ros_node"] = True
    instance = MycroftSkill(name='tester')
    instance.bind(bus)
    try:
        instance.load_data_files('/home/vagrant/dev/catkin_ws/src/mycroft_test/scripts/tester')
        # Set up intent handlers
        #instance._register_decorated()
        #instance.register_intent(IntentBuilder("TesterSkill").require("tests"), intent_callback)
        instance.register_intent(IntentBuilder("TesterSkill").require("tests"), lambda m : rospy.loginfo("logged"))
        #instance.register_resting_screen()
        instance.initialize()
    except Exception as e:
        # If an exception occurs, make sure to clean up the skill
        instance.default_shutdown()
        raise e
    skill["instance"] = instance
    modified = 0
    if skill['instance'] is not None:
        bus.emit(Message('mycroft.skills.loaded', {'path': skill_path, 'id': skill['id'], 'name': skill['instance'].name, 'modified': modified}))
        return True
    else:
         bus.emit(Message('mycroft.skills.loading_failure', {'path': skill_path, 'id': skill['id']}))
    return False

def check_connection():
    """
        Check for network connection. If not paired trigger pairing.
        Runs as a Timer every second until connection is detected.
    """
    if connected():
        enclosure = EnclosureAPI(bus)

        if is_paired():
            # Skip the sync message when unpaired because the prompt to go to
            # home.mycrof.ai will be displayed by the pairing skill
            enclosure.mouth_text(dialog.get("message_synching.clock"))

        # Force a sync of the local clock with the internet
        config = Configuration.get()
        platform = config['enclosure'].get("platform", "unknown")
        if platform in ['mycroft_mark_1', 'picroft']:
            bus.wait_for_response(Message('system.ntp.sync'),
                                  'system.ntp.sync.complete', 15)

        if not is_paired():
            try_update_system(platform)

        # Check if the time skewed significantly.  If so, reboot
        skew = abs((time.monotonic() - start_ticks) -
                   (time.time() - start_clock))
        if skew > 60 * 60:
            # Time moved by over an hour in the NTP sync. Force a reboot to
            # prevent weird things from occcurring due to the 'time warp'.
            #
            data = {'utterance': dialog.get("time.changed.reboot")}
            bus.emit(Message("speak", data))
            wait_while_speaking()

            # provide visual indicators of the reboot
            enclosure.mouth_text(dialog.get("message_rebooting"))
            enclosure.eyes_color(70, 65, 69)  # soft gray
            enclosure.eyes_spin()

            # give the system time to finish processing enclosure messages
            time.sleep(1.0)

            # reboot
            bus.emit(Message("system.reboot"))
            return
        else:
            bus.emit(Message("enclosure.mouth.reset"))
            time.sleep(0.5)

        enclosure.eyes_color(189, 183, 107)  # dark khaki
        enclosure.mouth_text(dialog.get("message_loading.skills"))

        bus.emit(Message('mycroft.internet.connected'))
        # check for pairing, if not automatically start pairing
        try:
            if not is_paired(ignore_errors=False):
                payload = {
                    'utterances': ["pair my device"],
                    'lang': "en-us"
                }
                bus.emit(Message("recognizer_loop:utterance", payload))
            else:
                from mycroft.api import DeviceApi
                api = DeviceApi()
                api.update_version()
        except BackendDown:
            data = {'utterance': dialog.get("backend.down")}
            bus.emit(Message("speak", data))
            bus.emit(Message("backend.down"))

    else:
        thread = Timer(1, check_connection)
        thread.daemon = True
        thread.start()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

def check_working(message):
    if message is not None:
        print('do somethingggg')
        print(message.data)
    else:
        print('do something else')

def initialise_response_server(message):
    global skill_manager
    get_response_server = GetResponseServer(manager=skill_manager)
    ask_yesno_server = AskYesNoServer(manager=skill_manager)
    

def listener():
    rospy.init_node('mycroft_skills')
    rospy.loginfo(rospy.get_caller_id() + " started")
    #rospy.Subscriber("mycroft/register_node", Mycroft, mycroft_register_node)
    #rospy.Subscriber("mycroft/response", GetResponse, handle_get_response)
    rospy.Subscriber("mycroft/utterance", String, handle_utterance)
    rospy.Subscriber("mycroft/remove_skill", String, handle_remove_skill)
    s = rospy.Service('mycroft/register_skill', MycroftService, handle_register_skill)
    
    global bus
    reset_sigint_handler()
    # Create PID file, prevent multiple instancesof this service
    mycroft.lock.Lock('skills')
    # Connect this Skill management process to the Mycroft Messagebus
    bus = WebsocketClient()
    Configuration.init(bus)
    config = Configuration.get()
    # Set the active lang to match the configured one
    set_active_lang(config.get('lang', 'en-us'))

    bus.on('skill.manager.initialised', initialise_response_server)
    bus.on('message', create_echo_function('SKILLS'))
    # Startup will be called after the connection with the Messagebus is done
    bus.once('open', _starting_up)
    #bus.on('mycroft.skills.loaded', check_working)
    bus.on('skill.converse.request', check_working)

    create_daemon(bus.run_forever)
    wait_for_exit_signal()
    shutdown()

    rospy.spin()

if __name__ == '__main__':
    listener()
