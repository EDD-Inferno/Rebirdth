import cv2
import numpy as np

# GStreamer pipeline to receive H.264 stream from port 5600
pipeline = "udpsrc port=5600 ! application/x-rtp,media=video,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink"

# OpenCV VideoCapture with the GStreamer pipeline
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open the video stream")
    exit()

# ArUco marker detection setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  # Choose the dictionary based on your markers
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Detect ArUco markers in the frame
    corners, ids, rejected_img_points = detector.detectMarkers(frame)

    # If markers are detected, draw them on the frame
    if len(corners) > 0:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Display the frame with ArUco markers
    cv2.imshow("H.264 Stream with ArUco Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
