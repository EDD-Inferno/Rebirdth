from asyncio import get_event_loop
from serial_asyncio import open_serial_connection

async def run():
    reader, writer = await open_serial_connection(url='/dev/ttyUSB0', baudrate=115200)
    while True:
        line = await reader.readline()
        line=line.decode().strip()[5:]
        parts=line.split(',', 3)
        print(str(parts))
        data = parts[2]
        print(data)
        writer.write(data.encode())
        await writer.drain()


loop = get_event_loop()
loop.run_until_complete(run())