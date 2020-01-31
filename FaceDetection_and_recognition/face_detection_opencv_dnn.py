# /*
# *
# * Project Name:   face detection using deep neural network
# * Author List: 	Soofiyan Atar
# * Filename: 		face_detection_opencv_dnn.py
# * Functions: 		detectFaceOpenCVDnn(net, frame)
# * Global Variables:	count
# *
# */

from __future__ import division
import cv2
import time
import sys

def detectFaceOpenCVDnn(net, frame):
    global count
    frameOpencvDnn = frame.copy()
    frameHeight = frameOpencvDnn.shape[0]
    frameWidth = frameOpencvDnn.shape[1]
    blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], False, False)
    
    net.setInput(blob)
    detections = net.forward()
    bboxes = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            gray = cv2.cvtColor(frameOpencvDnn, cv2.COLOR_BGR2GRAY)
            x1 = int(detections[0, 0, i, 3] * frameWidth)
            y1 = int(detections[0, 0, i, 4] * frameHeight)
            x2 = int(detections[0, 0, i, 5] * frameWidth)
            y2 = int(detections[0, 0, i, 6] * frameHeight)
            bboxes.append([x1, y1, x2, y2])
            cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight/150)), 8)
            face_id = 2
            try :
                cv2.imwrite("/FaceDetection_and_recognition/dataset/" + str(count) + '.' + str(face_id) + ".jpg", gray[y1:y2,x1:x2])
            except Exception as e:
                print(e)
            count += 1
    return frameOpencvDnn, bboxes

if __name__ == "__main__" :

    # OpenCV DNN supports 2 networks.
    # 1. FP16 version of the original caffe implementation ( 5.4 MB )
    # 2. 8 bit Quantized version using Tensorflow ( 2.7 MB )
    modelFile = "/FaceDetection_and_recognition/models/opencv_face_detector_uint8.pb"
    configFile = "/FaceDetection_and_recognition/models/opencv_face_detector.pbtxt"
    net = cv2.dnn.readNetFromTensorflow(modelFile, configFile)

    conf_threshold = 0.6

    source = 0
    if len(sys.argv) > 1:
        source = sys.argv[1]

    cap = cv2.VideoCapture(source)
    hasFrame, frame = cap.read()
    tt_opencvDnn = 0
    count = 0
    while(1):
        hasFrame, frame = cap.read()
        if not hasFrame:
            break

        t = time.time()
        outOpencvDnn, bboxes = detectFaceOpenCVDnn(net,frame)
        cv2.imshow("Face Detection", outOpencvDnn)

        k = cv2.waitKey(10)
        if k == 27:
            break
    cv2.destroyAllWindows()
