import asyncio
from rylr import RYLR

async def main():
    rylr = RYLR('/dev/ttyUSB0')
    await rylr.init()

    # Create a background task for the RYLR loop
    loop_task = asyncio.create_task(rylr.loop())

    try:
        while True:
            # Receive data
            data = await rylr.recv()
            # Print to terminal
            print(data)
            # Echo data
            await rylr.send(data)
    except asyncio.CancelledError:
        # Handle cancellation (e.g., cleanup)
        print("Terminating program...")
    finally:
        # Cancel the background task
        loop_task.cancel()
        await loop_task

# Run the main function
if __name__ == "__main__":
    asyncio.run(main())
