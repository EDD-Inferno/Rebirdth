import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import keyboard

from Autonomous import cv_landing_pi

#--- GPIO Definitions ---
MOSFET_PIN = 17 #TODO: put the correct pin #

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

async def print_position(drone):
    async for position in drone.telemetry.position():
        print(f"Lat: {position.latitude_deg}, Lon: {position.longitude_deg}, Alt: {position.absolute_altitude_m}")

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

async def initDrone():
    await drone.connect(system_address="serial:///dev/ttyS0:57600")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    # Start parallel tasks to print telemetry
    print_altitude_task = asyncio.create_task(print_altitude(drone))
    print_flight_mode_task = asyncio.create_task(print_flight_mode(drone))
    print_position = asyncio.create_task(print_position(drone))

    running_tasks = [print_altitude_task, print_flight_mode_task, print_position]
    termination_task = asyncio.create_task(observe_is_in_air(drone, running_tasks))

    # Wait until the drone is ready to fly
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    # Wait for GPS fix
    print("Waiting for GPS Fix...")
    async for gps_info in drone.telemetry.gps_info():
        if gps_info.num_satellites >= 6:  # Ensure sufficient satellites for positioning
            print(f"GPS Fix acquired: {gps_info}")
            break
    
async def armDrone():
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
    
    print("-- Wait for a bit")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1)

async def check_vert_vel():
    timeout = 30 * 10 # seconds
    # Check current vertical velocity
    
    async for velocity in drone.telemetry.velocity_ned():
        print(f"Velocity NED: North: {velocity.north_m_s:.2f} m/s, "
              f"East: {velocity.east_m_s:.2f} m/s, "
              f"Down: {velocity.down_m_s:.2f} m/s")
        if abs(velocity.down_m_s) < 1: # or timeout <= 0:
            break
        await asyncio.sleep(0.1)

async def wait_for_enter():
    enter_pressed = False
    while True:
        if keyboard.is_pressed('enter'):
            print("Enter Key Was Pressed")
            enter_pressed = True
            break
        await asyncio.sleep(0.1)

# --- Main function ---
async def run():
    # Initialize the drone
    global drone 
    global termination_task
    global enter_pressed

    drone = System()

    #Turn on Electromagnets
    GPIO.output(MOSFET_PIN, GPIO.HIGH)

    await initDrone()

################### Prints before launch 1 #########################
    wait_for_enter()
    battery_ready = False # TODO: Implement these three ready checks
    gyro_ready = False
    magnet_ready = False
    while True:
        battery = await drone.telemetry.battery()
        print(f"Battery Voltage: {battery.voltage_v:.2f} V")

        gyro = await drone.telemetry.attitude_euler()
        print(f"Gyroscope (angular velocity): Roll: {gyro.roll_deg:.2f} deg/s, "
            f"Pitch: {gyro.pitch_deg:.2f} deg/s, "
            f"Yaw: {gyro.yaw_deg:.2f} deg/s")
        
        print(f"Electromagnet: {GPIO.input(MOSFET_PIN)}")
        
        await cv_landing_pi.test_camera()

        print()

        if battery_ready and gyro_ready and magnet_ready: 
            print("ALL SYSTEMS ARE GOOD")

        print("############################################################\n")

        if enter_pressed:
            break  # Exit after ready to launch

        await asyncio.sleep(0.5)

####################  During Launch 1 #####################################3
    #       Receive Signal from GroundStation for termination altitude
    if (False):
        termination_alt = altitude
        print(f"Termination signal received. Current alt: {termination_alt}")

    print("-- Fetching current velocity...")
    await check_vert_vel()
    
    print("-- ARMING DRONE")
    await armDrone()

    print("-- Wait for a bit")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    print("-- Setting to Hold (GPS Position Mode)")
    await drone.action.set_flight_mode(mavsdk.action.FlightMode.HOLD)

    # Fly to drop tag with GPS
    print("-- Going to target location")
    await drone.action.goto_location(
        latitude_deg=0,      # Replace with target lat
        longitude_deg=0,      # Replace with target lon
        absolute_altitude_m=200,    # Replace with target absolute altitude (not relative! sea level is 0)
        yaw_deg=0                     # Set desired heading (yaw)
    )

    # Wait for some time or monitor position
    print("Flying to target...")
    await asyncio.sleep(20)

    # Align to Drop location AprilTag
    try:
        await cv_landing_pi.precision_landing(drone, True)
    except Exception as e:
        print(f'ERROR: {e}')
        await drone.action.land()
    
    await wait_for_enter()
    await drone.action.land()
    GPIO.output(MOSFET_PIN, GPIO.LOW)

    # GPS To RB2 Apriltag

    # Call precision landing routine using computer vision and PID control
    try:
        await cv_landing_pi.precision_landing(drone, False, 0.25)
    except Exception as e:
        print(f'ERROR: {e}')
        await drone.action.land()
    # Finally, command the drone to land
    print("-- Landing")
    await drone.action.land()


    ###################### Launch 2 Ready Checks #############################
    #     Gyro orientation
    #     Drone connection
    #     Magnet engagement
    #     Battery

    ####################  During Launch 2 #####################################3
    #     if (apogee near, determined by velocity (everything going upwards at around 5 mph))
    await armDrone()

    ##################### Descent and Land 2 ####################
    #     GPS Fly to second spot
    try:
        await cv_landing_pi.precision_landing(drone, True)
    except Exception as e:
        print(f'ERROR: {e}')
        await drone.action.land()
    #     Stop at minaltitude

    # Finally, command the drone to land TODO: Check if this landing is slow enough to safely land with rocket
    print("-- Landing")
    await drone.action.land()


    # Wait until the drone is landed
    await termination_task

if __name__ == "__main__":
    asyncio.run(run())