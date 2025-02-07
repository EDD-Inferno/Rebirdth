from asyncio import get_event_loop
from serial_asyncio import open_serial_connection

async def run():
    data = "hello glorious world"
    reader, writer = await open_serial_connection(url='/dev/ttyUSB0', baudrate=115200)
    while True:
        parts = await recvMessage(reader)
        print(str(parts))
        data = parts[2]
        sendMessage(data,parts[0], writer)

async def sendMessage(msg, addr, writer):
    writer.write((f'AT+SEND={addr},{len(msg)},{msg}\r\n').encode())
    await writer.drain()

async def recvMessage(reader):
    line = await reader.readline()
    line=line.decode().strip()[5:]
    parts=line.split(',', 3)
    return parts

    
loop = get_event_loop()
loop.run_until_complete(run())