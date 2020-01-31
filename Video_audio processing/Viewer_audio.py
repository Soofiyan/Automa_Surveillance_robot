# /*
# *
# * Project Name: 	audio viewing using sockets
# * Author List: 	Soofiyan Atar
# * Filename: 		Viewer_audio.py
# * Functions: 		None
# * Global Variables:	None
# *
# */
import socket
import pyaudio
import wave

#record
CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 40

HOST = ''    # The remote host
PORT = 8080              # The same port as used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index = 2,
                frames_per_buffer=CHUNK)

print("*recording")

frames = []

# for i in range(0, int(RATE/CHUNK*RECORD_SECONDS)):
while True:
    data  = stream.read(CHUNK)
    frames.append(data)
    s.sendall(data)

print("*done recording")

stream.stop_stream()
stream.close()
p.terminate()
s.close()

print("*closed")