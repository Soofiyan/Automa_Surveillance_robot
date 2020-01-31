# /*
# *
# * Project Name:   face recognition using deep neural network
# * Author List: 	Soofiyan Atar
# * Filename: 		face_recognition_dnn.py
# * Functions: 		detectFaceOpenCVDnn(net, frame)
# * Global Variables:	count,id,ids
# *
# */

from __future__ import division
import cv2
import time
import sys

recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('/Applications/Automa/FaceDetectionComparison/training/trainer.yml')
font = cv2.FONT_HERSHEY_SIMPLEX

#iniciate id counter
id = 1

# names related to ids: example ==> Marcelo: id=1,  etc
names = ['None', 'C', 'D', 'E', 'Z', 'W'] 

def detectFaceOpenCVDnn(net, frame):
    global count
    global id
    global ids
    ids = ""
    frame = cv2.flip(frame, 1)
    frameOpencvDnn = frame.copy()
    frameHeight = frameOpencvDnn.shape[0]
    frameWidth = frameOpencvDnn.shape[1]
    blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], False, False)
    
    net.setInput(blob)
    detections = net.forward()
    bboxes = []
    gray = cv2.cvtColor(frameOpencvDnn,cv2.COLOR_BGR2GRAY)
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            gray = cv2.cvtColor(frameOpencvDnn, cv2.COLOR_BGR2GRAY)
            x1 = int(detections[0, 0, i, 3] * frameWidth)
            y1 = int(detections[0, 0, i, 4] * frameHeight)
            x2 = int(detections[0, 0, i, 5] * frameWidth)
            y2 = int(detections[0, 0, i, 6] * frameHeight)
            bboxes.append([x1, y1, x2, y2])
            cv2.rectangle(frameOpencvDnn, (x1,y1), (x2,y2), (0,255,0), 2)
            try :
                id, confidence = recognizer.predict(gray[y1:y2,x1:x2])
            except Exception as e:
                l = 0
            # Check if confidence is less them 100 ==> "0" is perfect match 
            if (confidence < 100):
                ids = names[id]
                confidence = "  {0}%".format(round(100 - confidence))
            else:
                ids = "unknown"
                confidence = "  {0}%".format(round(100 - confidence))
            
            cv2.putText(frameOpencvDnn, str(ids), (x1+5,y1-5), font, 1, (255,255,255), 2)
            cv2.putText(frameOpencvDnn, str(confidence), (x1+5,y2-5), font, 1, (255,255,0), 1)  
    cv2.imshow("Face Detection Comparison", frameOpencvDnn)
    return frameOpencvDnn, bboxes


if __name__ == "__main__" :

    # OpenCV DNN supports 2 networks.
    # 1. FP16 version of the original caffe implementation ( 5.4 MB )
    # 2. 8 bit Quantized version using Tensorflow ( 2.7 MB )
    modelFile = "/Applications/Codes/learnopencv-master/FaceDetectionComparison/models/opencv_face_detector_uint8.pb"
    configFile = "/Applications/Codes/learnopencv-master/FaceDetectionComparison/models/opencv_face_detector.pbtxt"
    net = cv2.dnn.readNetFromTensorflow(modelFile, configFile)

    conf_threshold = 0.6

    source = 0
    if len(sys.argv) > 1:
        source = sys.argv[1]

    cap = cv2.VideoCapture(source)
    hasFrame, frame = cap.read()

    vid_writer = cv2.VideoWriter('output-dnn-{}.avi'.format(str(source).split(".")[0]),cv2.VideoWriter_fourcc('M','J','P','G'), 15, (frame.shape[1],frame.shape[0]))

    frame_count = 0
    tt_opencvDnn = 0
    count = 0
    while(1):
        hasFrame, frame = cap.read()
        if not hasFrame:
            break
        frame_count += 1

        t = time.time()
        outOpencvDnn, bboxes = detectFaceOpenCVDnn(net,frame)
        # tt_opencvDnn += time.time() - t
        # fpsOpencvDnn = frame_count / tt_opencvDnn
        # label = "OpenCV DNN ; FPS : {:.2f}".format(fpsOpencvDnn)
        # cv2.putText(outOpencvDnn, label, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, cv2.LINE_AA)

        

        # vid_writer.write(outOpencvDnn)
        # if frame_count == 1:
        #     tt_opencvDnn = 0

        k = cv2.waitKey(10)
        if k == 27:
            break
    cv2.destroyAllWindows()
    vid_writer.release()
