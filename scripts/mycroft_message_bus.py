#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from tornado import autoreload, web, ioloop

from mycroft.configuration import Configuration
from mycroft.lock import Lock  # creates/supports PID locking file
from mycroft.messagebus.service.ws import WebsocketEventHandler
from mycroft.util import validate_param, reset_sigint_handler, create_daemon, \
    wait_for_exit_signal

settings = {
    'debug': True
}


def main():
    rospy.init_node('mycroft_message_bus')
    rospy.loginfo(rospy.get_caller_id() + " started")

    import tornado.options
    reset_sigint_handler()
    lock = Lock("service")
    tornado.options.parse_command_line()

    def reload_hook():
        """ Hook to release lock when autoreload is triggered. """
        lock.delete()

    autoreload.add_reload_hook(reload_hook)

    config = Configuration.get().get("websocket")

    host = config.get("host")
    port = config.get("port")
    route = config.get("route")
    validate_param(host, "websocket.host")
    validate_param(port, "websocket.port")
    validate_param(route, "websocket.route")

    routes = [
        (route, WebsocketEventHandler)
    ]
    application = web.Application(routes, **settings)
    application.listen(port, host)
    create_daemon(ioloop.IOLoop.instance().start)

    wait_for_exit_signal()
    rospy.spin()


if __name__ == "__main__":
    main()
