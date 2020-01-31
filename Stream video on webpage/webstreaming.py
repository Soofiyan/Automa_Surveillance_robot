# /*
# *
# * Project Name: 	Web streaming in the network
# * Author List: 	Soofiyan Atar
# * Filename: 		webstreaming.py
# * Functions: 		index(), detect_motion(), generate(), register(), video_feed1()
# * Global Variables:	vs, outputFrame, lock
# *
# */

from imutils.video import VideoStream
from flask import Response
from flask import Flask, render_template, request
import threading
import argparse
import imutils
import time
import cv2
outputFrame = None
lock = threading.Lock()
app = Flask(__name__)
vs = VideoStream(src=0).start()
time.sleep(2.0)


@app.route("/")
def index():
    return render_template("index.html")


def detect_motion(frameCount):
    global vs, outputFrame, lock
    while True:
        frame = vs.read()
        frame = imutils.resize(frame, width=1280, height=640)
        frame = cv2.flip(frame, 1)
        with lock:
            outputFrame = frame.copy()


def generate():
    global outputFrame, lock

    while True:
        with lock:
            if outputFrame is None:
                continue

            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            if not flag:
                continue

        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')


@app.route("/register", methods=['POST', 'GET'])
def register():
    if request.method == 'POST':
        if request.form['submit_b'] == 'Do Something':
            print("first")  # do something
            pass
        elif request.form['submit_b'] == 'Do Something Else':
            print("second")  # do something else
            pass
        elif request.form['submit_b'] == 'Do third':
            print("third")  # do something else
            pass
        else:
            pass

        return render_template('index.html')

@app.route("/video_feed1")
def video_feed1():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


# check to see if this is the main thread of execution
if __name__ == '__main__':
    # construct the argument parser and parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, default="",
                    help="ip address of the device")
    ap.add_argument("-o", "--port", type=int, default=8080,
                    help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=120,
                    help="# of frames used to construct the background model")
    args = vars(ap.parse_args())

    # start a thread that will perform motion detection
    t = threading.Thread(target=detect_motion, args=(
        args["frame_count"],))
    t.daemon = True
    t.start()

    # start the flask app
    app.run(host=args["ip"], port=args["port"], debug=True,
            threaded=True, use_reloader=False)

# release the video stream pointer
vs.stop()
