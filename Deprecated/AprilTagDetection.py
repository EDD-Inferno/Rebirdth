import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2
import numpy as np
import math as math

# Initialize AprilTag detector
at_detector = Detector(families="tag36h11")

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())
picam2.start()

print("Press 'q' to quit.")
camera_params = (600, 600, 320, 240)
tag_size = 0.1

while True:
    # Capture frame from the camera
    frame = picam2.capture_array()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the grayscale frame
    tags = at_detector.detect(gray_frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

    for tag in tags:
        # Extract position and orientation data
        x = tag.pose_t[0][0]
        y = tag.pose_t[1][0]
        z = tag.pose_t[2][0]
        
        rol = np.degrees(np.arctan2(tag.pose_R[2][1], tag.pose_R[2][2]))
        yaw = np.degrees(np.arctan2(tag.pose_R[1][0], tag.pose_R[0][0]))  # Yaw in degrees

        # Draw the detected tag on the frame
        (ptA, ptB, ptC, ptD) = tag.corners
        ptA = tuple(map(int, ptA))
        ptB = tuple(map(int, ptB))
        ptC = tuple(map(int, ptC))
        ptD = tuple(map(int, ptD))

        # Draw bounding box
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        # Display the tag ID and position on the frame
        cv2.putText(frame, f"ID: {tag.tag_id}", (ptA[0], ptA[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}", 
                    (ptA[0], ptA[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #cv2.putText(frame, f"yaw: {yaw:.2f}", (ptA[0], ptA[1] + 40),
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Print the data to the console
        #print(f"Tag ID: {tag.tag_id}")
        #print(f"Position - x: {x:.2f} m, y: {y:.2f} m, z: {z:.2f} m")
        #print(f"Yaw: {yaw:.2f} degrees")
        #print("-" * 30)

    # Display the camera feed with detections
    cv2.imshow("AprilTag Detection", frame)

    # Quit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
picam2.stop()
