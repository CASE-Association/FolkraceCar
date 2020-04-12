#!/usr/bin/env python3
from importlib import import_module
from module.WebControl import camera_opencv
import os
from flask import Flask, render_template, Response
from module import shared, config as conf
import cv2
from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_cors import CORS
from multiprocessing import Queue
from module import config as conf
from module import shared
import logging
log = logging.getLogger('werkzeug')
log.setLevel(shared.LOG_LEVEL)

Camera = camera_opencv.Camera

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
CORS(app)
socketio = SocketIO(app)

_debug = conf.DEBUG


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR")
    print(request.event["message"])
    print(request.event["args"])
    print(e)


@socketio.on('control', namespace='/control')
def control(message):
    data = message["data"]
    if "left" in data.keys():
        x = data["left"][0]
        y = data["left"][1]
        if _debug:
            print("[Server] Left: ", x, ",", y)
        # linear.q.put(("left", x, y))
    elif "right" in data.keys():
        x = data["right"][0]
        y = data["right"][1]
        with shared.steer.get_lock():
            shared.steer.value = x
        with shared.speed.get_lock():
            shared.speed.value = -y
        if _debug:
            print("[Server] Right: ", x, ",", y)
        # servo.q.put(("right", x, y))
        # servo2.q.put(("right", y, x))
    elif "A" in data.keys():
        if _debug:
            print("[Server] A")
        # binary.q.put(("A", 1, 0))
    elif "B" in data.keys():
        if _debug:
            print("[Server] B")
        # binary2.q.put(("B", 1, 0))


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def main():
    app.run(host='0.0.0.0', threaded=True)


if __name__ == '__main__':
    main()
