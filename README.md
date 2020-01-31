# Automa Surevillance Robot

This bot is used in surevillance in various categories and being 2 wheel robot it can move freely at any direction with many functions such as object detction, face recognition, foot step recognition and many more...

### Prerequisites
Install python3 to run all these files after that, 

Libraries to be install are in the requirements.txt

```
pip3 install -r requirements.txt
```

### Running the codes

To run the arduino code upload it on the board but without hardware there won't be much understanding

Chatbot
To run chatbot enter this command and say "left", "right", "forward", etc basic commands to navigate robot.

```
cd chatbot
python3 speech.py
```

Face detection and recognition
To run face recogntiion and detection first we have to collect all custom face so first run

```
cd FaceDetection_and_recogniton
python3 face_detection_opencv_dnn.py.py
```

then for training run
```
python3 face_training.py
```

Then for testing and recognition run
```
python3 face_recognition_dnn.py
```

Object Detection and tracking 
For this example you have to download yolo weights and coco names files which are pretrained weights and names respectively.

Then run
```
cd ObjectDetection-YOLO
python3 object_detection_yolo.py.py
```

For tracking
run
```
python3 tracking_with_yolo.py.py
```


To send sms 

run
```
cd SMS
python3 send_sms.py
```
then enter your number

To video web stream run 
```
cd Stream video on webpage
python3 webstreaming.py
```

To live stream audio as well as video both the devices must be in the same network and then run all the files simultaneously i.e. stramers in one laptop and viewer in another
Run this in one laptop
```
cd Video_audio processing
python3 Streamer_audio.py
python3 Streamer_video.py
```
And run this in another laptop
```
cd Video_audio processing
python3 Viewer_audio.py
python3 Viewer_video.py
```

## Authors

* **Soofiyan Atar**


## Acknowledgments

* pyimagesearch

 
