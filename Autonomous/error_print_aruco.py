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

frame_center = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Get the center of the frame
    frame_height, frame_width = frame.shape[:2]
    if frame_center is None:
        frame_center = (frame_width // 2, frame_height // 2)

    # Detect ArUco markers in the frame
    corners, ids, rejected_img_points = detector.detectMarkers(frame)

    # If markers are detected, draw them on the frame
    if len(corners) > 0:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Loop over each detected marker
        for corner in corners:
            # Calculate the center of the marker (average of corner points)
            marker_center = np.mean(corner[0], axis=0)

            # Calculate the error (x and y distance) from the marker center to the frame center
            x_error = marker_center[0] - frame_center[0]
            y_error = marker_center[1] - frame_center[1]

            # Print the X and Y error
            print(f"X Error: {x_error:.2f}, Y Error: {y_error:.2f}")

            # Draw a circle at the marker center and display the X, Y error
            marker_center = tuple(np.int32(marker_center))
            cv2.circle(frame, marker_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"X: {x_error:.2f}, Y: {y_error:.2f}", marker_center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with ArUco markers
    cv2.imshow("H.264 Stream with ArUco Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
