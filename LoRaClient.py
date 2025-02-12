import pygame
import asyncio
from asyncio import get_event_loop
from serial_asyncio import open_serial_connection
from CustomRylr import sendMessage, recvMessage, checkParams, setParams

async def run():

    numTimeouts = 0

    # Initialize Pygame and the joystick
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0) #assuming only one joystick is connected
    joystick.init()

    # Initialize LoRa communication
    reader, writer = await open_serial_connection(url='/dev/ttyUSB0', baudrate=115200)
    await setParams(writer, 9, 8, 1, 4)
    #await checkParams(reader, writer)

    try:
        while True:
            pygame.event.pump()  # Process events

            # Read joystick axes (adjust indices based on your joystick)
            roll = int(joystick.get_axis(0)*10000)  # Typically axis 0 for roll
            pitch = int(joystick.get_axis(1))*1000  # Typically axis 1 for pitch
            throttle = int(joystick.get_axis(2)*1000)  # Typically axis 2 for throttle
            yaw = int(joystick.get_axis(3)*1000)  # Typically axis 3 for yaw

            # Send control commands over LoRa
            command = f"{roll}/{pitch}/{throttle}/{yaw}\n"
            await sendMessage(command, 1, writer)

            # Receive and print incoming messages
            
            try:
                async with asyncio.timeout(0.3): #wait for 0.5 seconds for a response
                    parts = await recvMessage(reader)
                    #data = parts[2]
                    #print("RCV: " + str(data))
            except TimeoutError:
                print("Receiver Timeout... Continuing")
                numTimeouts+=1
                continue
            except:
                print("Malformed Data")
                continue
            print("debug")
            
            
            

            


    
    except KeyboardInterrupt:
        print("Exiting...")
        pygame.quit()
    finally:
        pygame.quit()

loop = get_event_loop()
loop.run_until_complete(run())