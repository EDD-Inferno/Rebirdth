import pygame
import serial

# Initialize Pygame and the joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0) #assuming only one joystick is connected
joystick.init()

# Initialize LoRa communication
lora = serial.Serial('/dev/ttyUSB0', 115200)  # port, baud rate (change based on setup)

try:
    while True:
        pygame.event.pump()  # Process events

        # Read joystick axes (adjust indices based on your joystick)
        roll = joystick.get_axis(0)  # Typically axis 0 for roll
        pitch = joystick.get_axis(1)  # Typically axis 1 for pitch
        throttle = joystick.get_axis(2)  # Typically axis 2 for throttle
        yaw = joystick.get_axis(3)  # Typically axis 3 for yaw

        # Send control commands over LoRa
        command = f"{roll},{pitch},{throttle},{yaw}\n"
        lora.write(command.encode())

except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.quit()