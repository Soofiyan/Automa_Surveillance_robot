# /*
# *
# * Project Name: 	Tracking using yolo algorithm
# * Author List: 	Soofiyan Atar
# * Filename: 		tracking_with_yolo.py
# * Functions: 		IoU(), getOutputsNames(net), compute_center(), drawPred(), postprocess()
# * Global Variables:	prev_center, tracking_gone, bbox, prev_bbox, bbox_max,
# *                     iou_b, max_iou, object_detect, tracker   
# *
# */

import cv2
import argparse
import sys
import numpy as np
import os.path
import math

# Initialize the parameters
confThreshold = 0.2  #Confidence threshold
nmsThreshold = 0.2   #Non-maxsimum suppression threshold
inpWidth = 416       #Width of network's input image
inpHeight = 416      #Height of network's input image
tracking_gone = 0
first_in_loop = 1
first_loop = 1
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

def IoU(curr,prev):
    x1_curr = curr[0]
    x1_prev = prev[0]
    y1_curr = curr[1]
    y1_prev = prev[1]
    x2_curr = curr[0] + curr[2]
    x2_prev = prev[0] + prev[2]
    y2_curr = curr[1] + curr[3]
    y2_prev = prev[1] + prev[3]
    x_left = max(x1_curr,x1_prev)
    y_top = max(y1_curr,y1_prev)
    x_right = min(x2_curr,x2_prev)
    y_bottom = min(y2_curr,y2_prev)
    if x_right < x_left or y_bottom < y_top:
        return 0.0
    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    area_curr = (x2_curr - x1_curr)*(y2_curr - y1_curr)
    area_prev = (x2_prev - x1_prev)*(y2_prev - y1_prev)
    iou = intersection_area / float(area_curr + area_prev - intersection_area)
    if iou <= 0.0:
        iou = 0.0
    if iou >= 1.0:
        iou = 1.0
    return iou


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
    global iou_b
    global max_iou
    global object_detect
    global tracker
    # global centres
    # global centre_track
    # centres = (0.0,0.0)
    object_detect = 0
    max_iou = -1.0
    iou_b = 0.0

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
            first_in_loop = 0
        global prev_centre
        prev_centre = (0,0)
        centre = drawPred(classIds[i], confidences[i], left, top, left + width, top + height,prev_centre)
        prev_centre = centre
        # if(classes[classIds[i]] == 'person'):
        global bbox
        bbox = (left,top,width,height)
        iou_b = IoU(bbox,prev_bbox)
        # centres = compute_center(left,top,width,height)
        # centre_track = compute_center(prev_bbox[0],prev_bbox[1],prev_bbox[2],prev_bbox[3])
        # diff_centre = (abs(int(centres[0])- int(centre_track[0])),abs(int(centres[1])- int(centre_track[1])))
        # centre_diff_value = np.sqrt(np.square(diff_centre[0]) + np.square(diff_centre[1]))
        # print(centre_diff_value)
        if(iou_b > max_iou):
            bbox_max = bbox
            max_i = i
            max_iou = iou_b
        object_detect = 1
            # print(bbox)
            # print()
        # print(max_iou)
    if(max_iou < 0.25):
        tracking_gone = 1
    if object_detect == 1:
        if tracking_gone:
            tracker = cv2.TrackerCSRT_create()
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
        winname = "Tracking"
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, 130,130)
        cv2.imshow(winname, frame)
        
        # cv2.imshow(winName, frame)
    else:
        winname = "Tracking"
        cv2.namedWindow(winname)
        cv2.moveWindow(winname, 130,130)
        cv2.imshow(winname, frame)

#tracking block
tracker = cv2.TrackerCSRT_create()

# Process inputs

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
