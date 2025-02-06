import asyncio
from rylr import RYLR

async def main(rylr):
    await rylr.init()
    while True:
        # Receive data
        data = await rylr.recv()
        # Print to terminal
        print(data)
        # Echo data
        await rylr.send(data)

# Get second UART device (rx=16, tx=17 on ESP32 devkitc)
rylr = RYLR('/dev/ttyAMA2')

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)

# Create RYLR background task
loop.create_task(rylr.loop())

# Create main task
loop.create_task(main(rylr))

# Start IO loop
loop.run_forever()