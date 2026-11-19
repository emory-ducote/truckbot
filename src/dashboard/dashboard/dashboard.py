import logging
import threading
import time

from flask import Flask, jsonify, render_template


class Dashboard:

    def __init__(self, deliver=None):
        # deliver is called with no arguments and returns (success, message)
        self.deliver = deliver
        self.lock = threading.Lock()
        self.state = None
        self.modules = []
        self.received = None

    def updateStatus(self, state, modules):
        # state is a name like 'GOOD', modules is a list of {'name', 'state', 'detail', 'stamp'} dicts
        with self.lock:
            self.state = state
            self.modules = modules
            self.received = time.monotonic()

    def getStatus(self):
        with self.lock:
            if self.received is None:
                return {'age': None, 'state': 'INIT', 'modules': []}
            return {
                'age': time.monotonic() - self.received,
                'state': self.state,
                'modules': self.modules,
            }

    def createApp(self):
        app = Flask(__name__)
        logging.getLogger('werkzeug').setLevel(logging.ERROR)  # silence per-request logs from polling

        @app.route('/')
        def index():
            return render_template('index.html')

        @app.route('/status')
        def status():
            return jsonify(self.getStatus())

        @app.route('/deliver', methods=['POST'])
        def deliver():
            if self.deliver is None:
                return jsonify({'success': False, 'message': 'deliver not available'})
            success, message = self.deliver()
            return jsonify({'success': success, 'message': message})

        return app
