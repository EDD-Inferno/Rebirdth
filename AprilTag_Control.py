import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2
import numpy as np
import math
import asyncio
from mavsdk import System
import mavsdk.offboard

# Initialize AprilTag detector
at_detector = Detector(families="tag36h11")

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())
picam2.start()

# Camera parameters and tag size
camera_params = (600, 600, 320, 240)  # fx, fy, cx, cy
tag_size = 0.1  # Tag size in meters

# Desired hover position (relative to the tag)
HOVER_X = 0  # Hover directly above the tag
HOVER_Y = 0  # Center horizontally with the tag
HOVER_Z = 2  # Hover 2 meters above the tag
HOVER_YAW = 0  # Align yaw to match tag orientation


async def control_drone(drone, x, y, z, yaw):
    """
    Adjust drone position and yaw to hover over the detected tag.
    """
    # Calculate position adjustments
    x_adjustment = HOVER_X - x
    y_adjustment = HOVER_Y - y
    z_adjustment = HOVER_Z - z
    yaw_adjustment = HOVER_YAW - yaw

    # Send position and yaw commands to the drone
    await drone.offboard.set_position_ned(
        mavsdk.offboard.PositionNedYaw(
            x_adjustment, y_adjustment, -z_adjustment, yaw_adjustment
        )
    )


async def main():
    # Initialize and connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for the drone to be ready
    print("Connecting to the drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Set up offboard mode
    print("Starting offboard mode...")
    await drone.offboard.start()

    print("Press 'q' to quit.")
    while True:
        # Capture frame from the camera
        frame = picam2.capture_array()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the grayscale frame
        tags = at_detector.detect(
            gray_frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size
        )

        if tags:
            tag = tags[0]  # Use the first detected tag (can be improved to handle multiple)
            # Extract position and orientation data
            x = tag.pose_t[0][0]
            y = tag.pose_t[1][0]
            z = tag.pose_t[2][0]
            yaw = np.degrees(np.arctan2(tag.pose_R[1][0], tag.pose_R[0][0]))

            # Print data to console
            print(f"Tag ID: {tag.tag_id}, x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, yaw: {yaw:.2f}")

            # Control the drone to hover over the tag
            await control_drone(drone, x, y, z, yaw)

            # Draw the tag on the frame
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

            # Display tag ID and position on the frame
            cv2.putText(frame, f"ID: {tag.tag_id}", (ptA[0], ptA[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"x: {x:.2f}, y: {y:.2f}, z: {z:.2f}",
                        (ptA[0], ptA[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        else:
            print("No tag detected. Hovering in place.")
            # Stop movement if no tags are detected
            await drone.offboard.set_velocity_body(
                mavsdk.offboard.VelocityBodyYawspeed(0, 0, 0, 0)
            )

        # Display the camera feed
        cv2.imshow("AprilTag Detection", frame)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    print("Stopping offboard mode...")
    await drone.offboard.stop()
    cv2.destroyAllWindows()
    picam2.stop()


if __name__ == "__main__":
    asyncio.run(main())
