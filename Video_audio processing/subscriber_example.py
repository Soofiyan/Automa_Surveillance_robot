#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import cv2

# This is the Subscriber
import socket
import pyaudio
import wave
import sys
import pickle
import time

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 1

def record(sock):
    def callback_record(in_data, frame_count, time_info, status):
        # print len(in_data)
        output = base64.b64encode(in_data)
        footage_socket.send(output)
        # sock.send(in_data)

        return (in_data, pyaudio.paContinue)

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    output=False,
                    stream_callback=callback_record)

    stream.start_stream()
    return stream


def play(in_data):
    def callback_play(in_data, frame_count, time_info, status):
        # msg=recv_all(sock)
        # in_data = footage_socket.recv_string()
        # output = base64.b64decode(in_data)
        output = get.str
        return (in_data, pyaudio.paContinue)

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=False,
                    output=True,
                    stream_callback=callback_play)

    stream.start_stream()
    return stream

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("topic/test",qos=0)

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

def on_message(client, userdata, msg):
    output = (msg.payload.decode())
    output = string_to_image(output)
    window = cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 900, 1800)
    cv2.imshow('image', output)
    # cv2.moveWindow(MAZE_NAME, 100, 100)
    # cv2.namedWindow('image', WINDOW_NORMAL)
    # cv2.resizeWindow('image', (600, 600))
    # cv2.imshow('image',output)
    cv2.waitKey(1)
    # if msg.payload.decode() == "Hello world!":
    #     print("Yes!")
        # client.disconnect()


# client.connect("test.mosquitto.org", 1883, 60)
# client.on_connect = on_connect
# client.on_message = on_message

# client.loop_forever()

# broker="soldier.cloudmqtt.com"
broker="test.mosquitto.org"
port=12043
username="exyxiena"
username1="automa"
password="QQI5rhyzZHyc"
# client = mqtt.Client()
# client.connect("soldier.cloudmqtt.com",12043,60)
# client.publish("topic/test", "Hello world!")
# client.disconnect()
client = mqtt.Client()
# client.username_pw_set(username, password)
client.connect(broker)

client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()