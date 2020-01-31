# /*
# *
# * Project Name: 	Object detection using yolo algorithm
# * Author List: 	Soofiyan Atar
# * Filename: 		object_detection_yolo.py
# * Functions: 		getOutputsNames(net), compute_center(), drawPred(), postprocess()
# * Global Variables:	prev_center
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
nmsThreshold = 0.2   #Non-maximum suppression threshold
inpWidth = 416       #Width of network's input image
inpHeight = 416      #Height of network's input image

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
    print(centre[1])
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

# Remove the bounding boxes with low confidence using non-maxima suppression
def postprocess(frame, outs):
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
    print(boxes)
    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
        global prev_centre
        centre = drawPred(classIds[i], confidences[i], left, top, left + width, top + height,prev_centre)
        prev_centre = centre
# Process inputs
winName = 'Deep learning object detection in OpenCV'
cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

outputFile = "yolo_out_py.avi"
if (args.image):
    # Open the image file
    if not os.path.isfile(args.image):
        print("Input image file ", args.image, " doesn't exist")
        sys.exit(1)
    cap = cv2.VideoCapture(args.image)
    outputFile = args.image[:-4]+'_yolo_out_py.jpg'
elif (args.video):
    # Open the video file
    if not os.path.isfile(args.video):
        print("Input video file ", args.video, " doesn't exist")
        sys.exit(1)
    cap = cv2.VideoCapture(args.video)
    outputFile = args.video[:-4]+'_yolo_out_py.avi'
else:
    # Webcam input
    cap = cv2.VideoCapture(0)

# Get the video writer initialized to save the output video
if (not args.image):
    vid_writer = cv2.VideoWriter(outputFile, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))
prev_centre = (0,0)
while(cap.isOpened()):
    
    # get frame from the video
    hasFrame, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.merge((frame, frame, frame))
    # Stop the program if reached end of video
    # if not hasFrame:
    #     print("Done processing !!!")
    #     print("Output file is stored as ", outputFile)
    #     cv.waitKey(3000)
    #     # Release device
    #     cap.release()
    #     break
    cv2.waitKey(1)
    # Create a 4D blob from a frame.
    blob = cv2.dnn.blobFromImage(frame, 1/255, (inpWidth, inpHeight), [0,0,0], 1, crop=False)

    # Sets the input to the network
    net.setInput(blob)

    # Runs the forward pass to get output of the output layers
    outs = net.forward(getOutputsNames(net))

    # Remove the bounding boxes with low confidence
    postprocess(frame, outs)


    # Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
    # t, _ = net.getPerfProfile()
    # label = 'Inference time: %.2f ms' % (t * 1000.0 / cv2.getTickFrequency())
    # cv2.putText(frame, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

    if (args.image):
        cv2.imwrite(outputFile, frame.astype(np.uint8))
    else:
        vid_writer.write(frame.astype(np.uint8))

    cv2.imshow(winName, frame)

