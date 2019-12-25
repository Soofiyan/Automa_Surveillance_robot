# Usage example:  python3 object_detection_yolo.py --video=run.mp4
#                 python3 object_detection_yolo.py --image=bird.jpg

import cv2
import argparse
import sys
import numpy as np
import os.path
import math

# Initialize the parameters
confThreshold = 0.2  #Confidence threshold
nmsThreshold = 0.2   #Non-maximum suppression threshold
inpWidth = 416       #Width of network's input image
inpHeight = 416      #Height of network's input image
tracking_gone = 0
first_in_loop = 1
first_loop = 1
person_detected = 0


parser = argparse.ArgumentParser(description='Object Detection using YOLO in OPENCV')
parser.add_argument('--image', help='Path to image file.')
parser.add_argument('--video', help='Path to video file.')
args = parser.parse_args()
        
# Load names of classes
classesFile = "coco.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

# Give the configuration and weight files for the model and load the network using them.
modelConfiguration = "yolov3.cfg"
modelWeights = "yolov3.weights"

net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Get the names of the output layers
def getOutputsNames(net):
    # Get the names of all the layers in the network
    layersNames = net.getLayerNames()

    # Get the names of the output layers, i.e. the layers with unconnected outputs
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

def compute_center(left,right,top,bottom):
    centre = (((right-left)/2+left),((bottom-top)/2+top))
    return centre

# Draw the predicted bounding box
def drawPred(classId, conf, left, top, right, bottom,prev_centre):
    # Draw a bounding box.
    class_object = 'yyy'
    cv2.rectangle(frame, (left, top), (right, bottom), (255, 178, 50), 3)
    centre = compute_center(left,right,top,bottom)
    label = '%.2f' % conf
    # Get the label for the class name and its confidence
    class_object = classes[classId]
    if (prev_centre[0] - centre[0])>10:
        class_object = classes[classId]+'1'

    if classes:
        assert(classId < len(classes))
        label = '%s:%s' % (class_object, label)

    print(class_object)
    centre_x = int(centre[0])
    centre_y = int(centre[1])
    #Display the label at the top of the bounding box
    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    top = max(top, labelSize[1])
    cv2.rectangle(frame, (left, top - round(1.5*labelSize[1])), (left + round(1.5*labelSize[0]), top + baseLine), (255, 255, 255), cv2.FILLED)
    cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 1)
    cv2.circle(frame, (centre_x,centre_y), 1, (255, 255, 255), 10)
    return centre

def compute_min_error(prev,curr,max_bbox,min_err,j,prev_j):
    curr_min_err = np.square(int(prev[0])-int(curr[0])) + np.square(int(prev[1])-int(curr[1])) + np.square(int(prev[2])-int(curr[2])) + np.square(int(prev[3])-int(curr[3]))
    if(min_err > curr_min_err):
        return curr,curr_min_err,j
    else:
        return max_bbox,min_err,prev_j

def compute_area(prev,curr):
    error = prev[2]*prev[3] - curr[2]*curr[3]
    return error

# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(frame, outs, first_in_loop,first_loop):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]

    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    classIds = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                center_x = int(detection[0] * frameWidth)
                center_y = int(detection[1] * frameHeight)
                width = int(detection[2] * frameWidth)
                height = int(detection[3] * frameHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])
    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    global tracking_gone
    global max_i
    max_i = 0
    global bbox
    global prev_bbox
    global bbox_max
    global min_error
    global person_detected
    global error_area
    error_area = 0
    person_detected = 0

    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        if(first_loop):
            prev_bbox = (left,top,width,height)
            first_loop = 0
        if(first_in_loop):
            bbox_max = (left,top,width,height)
            min_error = 10000000
            first_in_loop = 0
        global prev_centre
        prev_centre = (0,0)
        centre = drawPred(classIds[i], confidences[i], left, top, left + width, top + height,prev_centre)
        prev_centre = centre
        if(classes[classIds[i]] == 'person'):
            global bbox
            bbox = (left,top,width,height)
            bbox_max,min_error,max_i = compute_min_error(prev_bbox,bbox,bbox_max,min_error,i,max_i)
            person_detected = 1
            error_area = compute_area(prev_bbox,bbox_max)
    if tracking_gone:
        ok = tracker.init(frame, bbox_max)
        tracking_gone = 0
    
    ok, bbox = tracker.update(frame)
    prev_bbox = bbox
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    else:
        tracking_gone = 1
    cv2.imshow("Tracking", frame)
        
        # cv2.imshow(winName, frame)

#tracking block
tracker = cv2.TrackerCSRT_create()

# Process inputs
winName = 'Deep learning object detection in OpenCV'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    
    # get frame from the video
    hasFrame, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.merge((frame, frame, frame))

    cv2.waitKey(1)
    # Create a 4D blob from a frame.
    blob = cv2.dnn.blobFromImage(frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)

    # Sets the input to the network
    net.setInput(blob)

    # Runs the forward pass to get output of the output layers
    outs = net.forward(getOutputsNames(net))

    # Remove the bounding boxes with low confidence
    postprocess(frame, outs, first_in_loop, first_loop)
    first_loop = 0
    first_in_loop = 1
    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27: break
