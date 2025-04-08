import cv2
import numpy as np
from picamera2 import Picamera2

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())
picam2.start()


# Set up the ArUco detector
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

try:
    while True:
        # Read frame
        frame = picam2.capture_array()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = gray_frame
        # print(frame)
        
        if frame is None:
            print("Failed to capture frame")
            continue
            
        # Determine frame center
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        # Detect ArUco markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        
        if corners and len(corners) > 0:
            # Assume the first detected marker is our target
            marker_corners = corners[0]
            # Compute the center of the marker
            marker_center = np.mean(marker_corners[0], axis=0)
            marker_center = tuple(np.int32(marker_center))

            # Calculate errors in X and Y (in pixels)
            error_x = marker_center[0] - frame_center[0]
            error_y = marker_center[1] - frame_center[1]
            
            error_x = error_x
            error_y = error_y

            # Draw marker and error information on frame
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, marker_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Must move forward: {error_x:.1f}", (marker_center[0]+10, marker_center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"Must move right: {error_y:.1f}", (marker_center[0]+10, marker_center[1]+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Print errors to console
            print(f"X Error: {error_x:.1f} px, Y Error: {error_y:.1f} px")

        # Display the frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()