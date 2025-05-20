import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from picamera2 import Picamera2

# --- PID Controller Class ---
import asyncio
import cv2
import numpy as np
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# --- PID Controller Class ---
class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float=0.1, integral_limit: float=100, derivative_noise_frequency: float=50.0):
        """
        @param kp kp
        @param ki ki
        @param kd kd
        @param dt change in time between iterations
        @param integral_limit maximum integral value
        @param derivative_noise_frequency: frequency of oscillations in the derivative term (take the fft of the error signal and then set this to the most prominent high frequency)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral_limit = integral_limit
    
        self.prev_error = float("NaN")
        self._integral = 0.0
        self._derivative = 0.0
        self._derivative_alpha = dt * derivative_noise_frequency / (1. + dt * derivative_noise_frequency)

    def update(self, error):
        if (math.isnan(self.prev_error)):
            self.prev_error = error
        
        # Proportional term
        p = self.kp * error

        # Integral term with anti-windup
        if (error > 0) != (self.prev_error > 0):
            # reset on sign crossing
            self._integral = 0
        else:
            self._integral += error
            # Clamp the integral to avoid windup
            self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)
        
        i = self.ki * self._integral * self.dt

        # Derivative term
        self._derivative += ((error - self.prev_error) - self._derivative) * self._derivative_alpha
        d = self.kd * self._derivative / self.dt
        self.prev_error = error

        # PID output
        return p + i + d

# --- Precision Landing Routine ---
## alt_threshold 
async def precision_landing(drone, alt_threshold=0.5):
    print("-- Starting precision landing using ArUco marker and PID")
    
    # Initialize camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration())
    picam2.start()

    # Set up the ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Create PID controllers for X and Y axes.
    # NOTE: You must tune these gains based on your system.
    pid_x = PID(kp=0.001, ki=0.0000, kd=0.000, dt=0.1)
    pid_y = PID(kp=0.001, ki=0.0000, kd=0.000, dt=0.1)
    pid_z = PID(kp=0.00025, ki=0.0000, kd=0.000, dt=0.1)

   # Threshold in pixels under which we consider the drone to be aligned
    threshold = 15 # pixels

    aligned_counter = 0
    required_alignments = 20  # Require several consecutive frames to confirm alignment
    failed_detection_counter = 0
    iteration = 0

    # Parameters for drone descent
    camFOV = math.radians(75)
    safety_factor = 1.5
    tag_size = 0.183 # Length/Width of AprilTag in meters
    tag_circumradius = tag_size * math.sqrt(2)/2
    max_descent_rate = 0.5 #m/s
    error_z = None

    # Original Function: safety_factor * (dist_to_tag + tag_circumradius) * (1 / math.tan(half_camFOV))
    min_altitude_calc_const = safety_factor * safety_factor * tag_circumradius * (1 / math.tan(camFOV / 2.0))

    while True:
        use_altitude_reading = altitude != None or altitude == 0.0
        # Read frame in a non-blocking way
        frame = picam2.capture_array()
        if frame is None:
            print("Failed to capture frame")
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Determine frame center (assume camera is calibrated so that frame center aligns with desired landing point)
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        # Detect ArUco markers in the frame
        corners, ids, _ = detector.detectMarkers(frame)
        if corners and len(corners) > 0:
            failed_detection_counter = 0
            # Assume the first detected marker is our target
            marker_corners = corners[0]
            # Compute the center of the marker (average of its corner points)
            marker_center = np.mean(marker_corners[0], axis=0)
            marker_center = tuple(np.int32(marker_center))

            # Calculate errors in X and Y (in pixels)
            error_x = marker_center[0] - frame_center[0]
            error_y = marker_center[1] - frame_center[1]
            if use_altitude_reading:
                horizon_dist_to_tag = math.sqrt(error_x**2 + error_y**2)
                min_altitude = horizon_dist_to_tag * min_altitude_calc_const
                print(f'UnclampedMinAlt: {min_altitude} Alt: {altitude}')
                min_altitude = max(min_altitude, 0.5) 
                min_altitude = min(min_altitude, 20)
                
                error_z = altitude - min_altitude
            
            # PID correction outputs (mapping pixel error to m/s command, adjust gains as needed)
            control_x = pid_x.update(error_x)  # Negative sign if image x error is opposite to drone's right movement
            control_y = pid_y.update(error_y)  # Adjust sign based on camera mounting and coordinate frame
            if use_altitude_reading:
                control_z = pid_z.update(error_z)  # Adjust sign based on camera mounting and coordinate frame
            else: 
                control_z = 0.0

            print(f'Velocities X: {control_x} Y: {control_y} Z: {control_z}')
            # Draw marker and error information on frame (for debugging)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, marker_center, 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ErrX: {error_x:.1f}", (marker_center[0]+10, marker_center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"ErrY: {error_y:.1f}", (marker_center[0]+10, marker_center[1]+15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"ErrZ: {error_z:.1f}", (altitude, min_altitude),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Command the drone with the computed corrections.
            # Here, control_x adjusts forward/backward and control_y adjusts left/right.
            # We keep vertical velocity zero as we want to continue descending by landing command.
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(control_x, control_y, control_z, 0.0)
            )

            # Check if errors are within the threshold.
            if abs(error_x) < threshold and abs(error_y) < threshold and abs(error_z) < alt_threshold:
                aligned_counter += 1
                print(f'Aligned Counter: {aligned_counter}')
            elif aligned_counter != 0:
                aligned_counter = 0
                print('Aligned Counter Reset')

            # If aligned for several consecutive frames, exit loop and initiate landing.
            if aligned_counter >= required_alignments:
                print("Marker aligned! Initiating landing...")
                break

        else:
            # If no marker is detected, hover in place before landing
            print("-- No Marker, Hovering")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            if (failed_detection_counter == 10):
                print("-- No Marker, Landing")
                await drone.action.land()
            # Optionally show frame as is
            else:
                failed_detection_counter += 1  
                print("Failed Detection Counter: " + str(failed_detection_counter))
        if(iteration % 10 ==0):
            # Show the frame with detected markers
            cv2.imwrite("~/Rebirdth/Frames/", frame)
        iteration +=1

    cv2.destroyAllWindows()
    return

# --- Helper functions for telemetry ---
altitude = None
async def print_altitude(drone):
    previous_altitude = None
    async for position in drone.telemetry.position():
        altitude = position.relative_altitude_m
        altitudeRound = round(altitude)
        if altitudeRound != previous_altitude:
            previous_altitude = altitudeRound
            print(f"Altitude: {altitude}")

async def print_flight_mode(drone):
    previous_flight_mode = None
    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def observe_is_in_air(drone, running_tasks):
    was_in_air = False
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air
        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()
            return

# --- Main function ---
async def run():


    # Initialize the drone
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyS0:57600")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # Start parallel tasks to print telemetry
    print_altitude_task = asyncio.create_task(print_altitude(drone))
    print_flight_mode_task = asyncio.create_task(print_flight_mode(drone))
    running_tasks = [print_altitude_task, print_flight_mode_task]
    termination_task = asyncio.create_task(observe_is_in_air(drone, running_tasks))

    # Wait until the drone is ready to fly
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # Arm the drone and start offboard mode
    print("-- Arming")
    await drone.action.arm()
    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Execute initial maneuvers: climb while turning
    print("-- climb")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1, 0))
    await asyncio.sleep(10)

    print("-- Wait for a bit")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    # # print("-- forward & side")
    # # await drone.offboard.set_velocity_body(
    # #     VelocityBodyYawspeed(0.5, 0.5, 0.0, 0.0))
    # # await asyncio.sleep(3)

    # print("-- Wait for a bit")
    # await drone.offboard.set_velocity_body(
    #     VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    # await asyncio.sleep(5)

    
    # Call precision landing routine using computer vision and PID control
    try:
        await precision_landing(drone)
    except Exception as e:
        print(f'ERROR: {e}')
        drone.action.land()
    # Finally, command the drone to land
    print("-- Landing")
    await drone.action.land()

    # Wait until the drone is landed
    await termination_task

if __name__ == "__main__":
    asyncio.run(run())
