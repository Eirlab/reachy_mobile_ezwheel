#!/usr/bin/env python3
import os

from flask import Flask, request, Blueprint
from flask_cors import CORS
from flask_socketio import SocketIO

import rospy
from simple_navigation_goal import SimpleNavigationGoals

move_base = None


class FlaskAPI:
    """
    Define and setup flask server and ros topic subscriber / publisher for ronoco-vm
    """

    def __init__(self):
        """
        Launch flask server when RonocoVm is created (this constructor uses SocketIO)
        """
        global move_base
        self.app = None
        self.create_app()
        socketio = SocketIO(self.app, logger=False, cors_allowed_origins='*')
        self.setup_app()
        rospy.init_node('user')
        move_base = SimpleNavigationGoals()
        rospy.loginfo("User root is serving the Web app")
        socketio.run(self.app, host="0.0.0.0")

    def create_app(self, test_config=None):
        """
        Build a Flask instance and configure it
        :param test_config: path to configuration file (Default : None)
        :return: a Flask instance
        """
        # create and configure the app
        self.app = Flask(__name__, instance_relative_config=True)
        self.app.config.from_mapping(
            SECRET_KEY='dev',
        )
        self.app.debug = True

        if test_config is None:
            # load the instance config, if it exists, when not testing
            self.app.config.from_pyfile('config.py', silent=True)
        else:
            # load the test config if passed in
            self.app.config.from_mapping(test_config)

        # ensure the instance folder exists
        try:
            os.makedirs(self.app.instance_path)
        except OSError:
            pass

    def setup_app(self):
        """
        Register blueprint in app.
        The class attribute "app" must contain an Flask instance
        :return: None
        """
        self.app.register_blueprint(API().bp)
        CORS(self.app)


class API:
    def __init__(self):
        self.bp = Blueprint('api', __name__)
        self.bp.route('/goal', methods=['POST'])(self.goal)
        self.bp.route('/cancel', methods=['GET'])(self.cancel)
        self.bp.route('/arrived', methods=['GET'])(self.arrived)

    def goal(self):
        if request.method == 'POST':
            global move_base
            data = request.get_json()
            print(data)
            x = data['x']
            y = data['y']
            theta = data['theta']
            move_base.go_to(x, y, theta, blocking=False)
            return "Go to {} {} {}".format(x, y, theta)

    def cancel(self):
        if request.method == 'GET':
            global move_base
            move_base.cancel_all_goals()
            return 'cancelling'

    def arrived(self):
        if request.method == 'GET':
            global move_base
            arrived = move_base.is_arrived()
            return arrived


if __name__ == '__main__':
    FlaskAPI()
