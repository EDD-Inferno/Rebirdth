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
    running_tasks = [print_altitude_task, print_flight_mode_task]
    termination_task = asyncio.create_task(observe_is_in_air(drone, running_tasks))

    # Wait until the drone is ready to fly
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
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
    await asyncio.sleep(5)

async def wait_for_enter():
    while True:
        if keyboard.is_pressed('enter'):
            print("Enter Key Was Pressed")
            break
        await asyncio.sleep(0.1)

# --- Main function ---
async def run():
    # Initialize the drone
    global drone 
    global termination_task

    drone = System()

    #Turn on Electromagnets
    GPIO.output(MOSFET_PIN, GPIO.HIGH)

    initDrone()

################### Prints before launch #########################
    #     Battery
    #     Altitude
    #     Gyro reading
    #     Flight Mode
    #     Camera Vision active
    #     Magnet Engagement
    
###################### Launch Ready Checks #############################
    #   { 
    #     Gyro orientation
    #     Drone connection
    #     Magnet engagement
    #     Battery

####################  During Launch #####################################3
    #       record termination altitude
    #     
    #     if (apogee near, determined by velocity (everything going upwards at around 5 mph))
    #     {
    armDrone()
    
    #Align to Receiver AprilTag
    try:
        await cv_landing_pi.precision_landing(drone, True)
    except Exception as e:
        print(f'ERROR: {e}')
    await wait_for_enter()
    GPIO.output(MOSFET_PIN, GPIO.LOW)

    # GPS To Other Apriltag

    # Call precision landing routine using computer vision and PID control
    try:
        await cv_landing_pi.precision_landing(drone, False)
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
