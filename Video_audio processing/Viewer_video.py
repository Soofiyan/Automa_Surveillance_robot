# import cv2
# import zmq
# import base64
# import numpy as np
# import socket
# import pyaudio
# import wave
# import sys
# import pickle
# import time
# 
# CHUNK = 1024
# FORMAT = pyaudio.paInt16
# CHANNELS = 1
# RATE = 44100
# RECORD_SECONDS = 3
# 
# context = zmq.Context()
# footage_socket = context.socket(zmq.SUB)
# footage_socket.bind('tcp://192.168.43.144:12043')
# footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))
# 
# def record(sock):
#     def callback_record(in_data, frame_count, time_info, status):
#         # print len(in_data)
#         output = base64.b64encode(in_data)
#         footage_socket.send(output)
#         # sock.send(in_data)
# 
#         return (in_data, pyaudio.paContinue)
# 
#     p = pyaudio.PyAudio()
#     stream = p.open(format=FORMAT,
#                     channels=CHANNELS,
#                     rate=RATE,
#                     input=True,
#                     output=False,
#                     stream_callback=callback_record)
# 
#     stream.start_stream()
#     return stream
# 
# 
# def play(sock):
#     def callback_play(in_data, frame_count, time_info, status):
#         # msg=recv_all(sock)
#         in_data = footage_socket.recv_string()
#         output = base64.b64decode(in_data)
#         return (in_data, pyaudio.paContinue)
# 
#     p = pyaudio.PyAudio()
#     stream = p.open(format=FORMAT,
#                     channels=CHANNELS,
#                     rate=RATE,
#                     input=False,
#                     output=True,
#                     stream_callback=callback_play)
# 
#     stream.start_stream()
#     return stream
# 
# while True:
#     try:
#         frame = footage_socket.recv_string()
#         img = base64.b64decode(frame)
#         npimg = np.fromstring(img, dtype=np.uint8)
#         source = cv2.imdecode(npimg, 1)
#         source = cv2.flip(source,1)
#         try:
#             cv2.imshow("Stream", source)
#         except:
#             stream_play = play(s)
#         # time.sleep(5)
#         # stream_record=record(s)
# 
#         # while stream_play.is_active():
#         #     # time.sleep(0.0)
#         #     pass
# 
#         # stream_record.stop_stream()
#         # stream_record.close()
#         # stream_play.stop_stream()
#         # stream_play.close()
#         cv2.waitKey(1)
# 
#     except KeyboardInterrupt:
#         cv2.destroyAllWindows()
#         break

import cv2
import zmq
import base64
import numpy as np

context = zmq.Context()
footage_socket = context.socket(zmq.SUB)
footage_socket.bind('tcp://*:12043')
footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

while True:
    try:
        frame = footage_socket.recv_string()
        img = base64.b64decode(frame)
        npimg = np.fromstring(img, dtype=np.uint8)
        source = cv2.imdecode(npimg, 1)
        source = cv2.flip(source,1)
        cv2.imshow("Stream", source)
        cv2.waitKey(1)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        break