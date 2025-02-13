import asyncio
from mavsdk import System
from CustomRylr.py import recvMessage


async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    async for state in drone.core.connection_state():
		if state.is_connected:
			print("Drone Connected")
			break
			
	print("Arming the drone...")
	await drone.action.arm()
	
	print("Starting manual control...")
	await drone.manual_control.start_position_control()

    # Start the tasks, Commented out for now to make testing easier
    #asyncio.ensure_future(print_battery(drone))
    #asyncio.ensure_future(print_gps_info(drone))
    #asyncio.ensure_future(print_in_air(drone))
    #asyncio.ensure_future(print_position(drone))
    
    control_task = asyncrio.create_task(control_drone(drone))
    try:
		await asyncio.Event().wait()
	except KeyboardInterrupt:
		print("Exiting...")
		
	finally:
		control_task.cancel()
		await control_task
		
		print("Stopping manual control...")
		await drone.manual_control.stop_position_control()
		print("Landing...")
		await drone.action.land()

async def control_drone(drone):
	while True:
			try:
				parts = await recvMessage(reader)
				data = parts[2]
				#print("data: " + data)
				split_data = data.split('/')
				throttle, yaw, pitch, roll = map(float, split_data)
				normalized_roll = roll / 1000
				normalized_pitch = pitch / 1000
				normalized_yaw = yaw / 1000
				normalized_throttle = throttle / 1000
				print(f"Throttle: {normalized_throttle}, Yaw: {normalized_yaw}, Pitch: {normalized_pitch}, Roll: {normalized_roll}")
					
				await drone.manual_control.set_manual_control_input(
					roll, pitch, throttle, yaw
				)
			
			except ValueError:
				print("Error: Invalid data format, skipping...")	
			except asyncio.CancelledError:
				print("Stopping control loop...")
				break
					
			await asyncio.sleep(0.1)
				


async def print_battery(drone):
    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")

    # Could take out
async def print_gps_info(drone):
    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")


async def print_in_air(drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")


async def print_position(drone):
    async for position in drone.telemetry.position():
        print(position)


if __name__ == "__main__":
    # Start the main function
    asyncio.run(run())
