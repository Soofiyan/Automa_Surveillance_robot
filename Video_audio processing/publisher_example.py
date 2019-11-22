#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import cv2
import paho.mqtt.client as mqtt
import numpy as np
import sys
import tkinter as tk
from tkinter import filedialog
import matplotlib.image as img
import scipy.misc
import imageio
from matplotlib.image import imread
import base64

def image_to_string(image):
    import cv2
    import base64
    encoded, buffer = cv2.imencode('.jpg', image)
    return base64.b64encode(buffer)


def string_to_image(string):
    import numpy as np
    import cv2
    import base64
    img = base64.b64decode(string)
    npimg = np.fromstring(img, dtype=np.uint8)
    return cv2.imdecode(npimg, 1)

def rescale_frame(frame, percent=10):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # print(frame)
    frame = cv2.resize(frame, (240, 64), interpolation = cv2.INTER_AREA)
    # frame = rescale_frame(frame, 15)
    # frame = np.array(frame)
    frame = image_to_string(frame)
    # frame = frame.decode("utf-8")
    # horizontal_img = np.moveaxis(horizontal_img.reshape(640,), 0, -1)
    # print(str)
    # scipy.misc.imread(frame)
    # horizontal_img = scipy.misc.imread(frame)
    # horizontal_img = cv2.flip(frame, 1)
    # f2 = np.int(horizontal_img)
    # horizontal_img = np.int(f2)
    # broker="soldier.cloudmqtt.com"
    broker="test.mosquitto.org"
    # port=12043
    # username="exyxiena"
    # username1="automa"
    # password="QQI5rhyzZHyc"
    # client = mqtt.Client()
    # client.connect("soldier.cloudmqtt.com",12043,60)
    # client.publish("topic/test", "Hello world!")
    # client.disconnect()
    client = mqtt.Client("Python1")
    # client.username_pw_set(username, password)
    # client.connect(broker,port)
    client.connect(broker)
    # print(sys.getsizeof(frame))

    client.publish("topic/test",frame)
    client.disconnect()