import traceback
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import asyncio

import cv_landing_pi

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

async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery Voltage: {battery.voltage_v:.2f} V")

async def print_stuff(drone):
    previous_altitude = None
    while(True):
        print("--------------")
        async for battery in drone.telemetry.battery():
            print(f"Battery Voltage: {battery.voltage_v:.2f} V")
            if battery.voltage_v < 14.85:
                raise ValueError("BATTERY BELOW 14.85")
            break

        async for position in drone.telemetry.position():
            altitude = position.relative_altitude_m
            altitudeRound = round(altitude)
            if altitudeRound != previous_altitude:
                previous_altitude = altitudeRound
                print(f"Altitude: {altitude}")
            break

        async for position in drone.telemetry.position():
            print(f"Lat: {position.latitude_deg}, Lon: {position.longitude_deg}, Alt: {position.absolute_altitude_m}")
            break

        print("--------------")


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

async def connect_drone():
    await drone.connect(system_address="serial:///dev/ttyS0:57600")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break
    global print_stuff
    global termination_task
    # Start parallel tasks to print telemetry
    print_stuff = asyncio.create_task(print_stuff(drone))

    running_tasks = [print_stuff]
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
    
async def arm_drone():
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
    
    # --- Main function ---
async def run():
    # Initialize the drone
    print("Script Started")
    global drone 
    global termination_task
    global termination_alt
    termination_alt = 0

    print("Drone = System()")
    drone = System()

    #Turn on Electromagnets
    # GPIO.output(MOSFET_PIN, GPIO.HIGH)
    
    print("Connecting to drone")
    await connect_drone()

    abs_alt = None
    async for position in drone.telemetry.position():
        abs_alt = position.absolute_altitude_m
        break

    await arm_drone()

    print("-- climb")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1, 0))
    await asyncio.sleep(6)

    print("-- Wait for a bit")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(3)

    # Fly to drop tag with GPS
    print("-- Going to dropoff location")
    await drone.action.goto_location(
        latitude_deg=33.7926172,      # TODO: Replace with target lat
        longitude_deg=-118.3432752,      # TODO: Replace with target lon
        absolute_altitude_m=abs_alt + 6,    # TODO: Replace with target absolute altitude (not relative! sea level is 0)
        yaw_deg=0                     # Set desired heading (yaw)
    )

    # Wait for some time or monitor position
    print("Flying to dropoff...")
    await asyncio.sleep(15)

    # Align to Drop location AprilTag
    print("Precision Landing...")
    try:
        await cv_landing_pi.precision_landing(drone, False)
    except Exception as e:
        print(print(traceback.format_exc()))
        await drone.action.land()

    # Finally, command the drone to land TODO: Check if this landing is slow enough to safely land with rocket
    print("-- Landing")
    await drone.action.land()

    # Wait until the drone is landed
    await termination_task

if __name__ == "__main__":
    asyncio.run(run())
