import cv2
import sys

# Set up tracker.
# Instead of MIL, you can also use

tracker_types = [
    'BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'CSRT'
]
tracker_type = tracker_types[6]
tracker = cv2.TrackerMOSSE_create()

# Read video
video = cv2.VideoCapture(0)

# Exit if video not opened.
# if not video.isOpened():
#     print("Could not open video")
#     sys.exit()

# Read first frame.
ok, frame = video.read()

# Define an initial bounding box
bbox = (430, 69, 400, 700)

# Uncomment the line below to select a different bounding box
# bbox = cv2.selectROI(frame, False)
print(bbox)
# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)

while True:
    # Read a new frame
    ok, frame = video.read()

    # Update tracker
    ok, bbox = tracker.update(frame)

    # Draw bounding box
    if ok:
        # Tracking success
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    # else:

    # Display result
    cv2.imshow("Tracking", frame)

    # Exit if ESC pressed
    k = cv2.waitKey(1) & 0xff
    if k == 27: break
