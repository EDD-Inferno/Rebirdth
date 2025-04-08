from asyncio import get_event_loop
from serial_asyncio import open_serial_connection

async def sendMessage(msg, addr, writer):
    writer.write((f'AT+SEND={addr},{len(msg)},{msg}\r\n').encode())
    await writer.drain()

async def recvMessage(reader):
    line = await reader.readline()
    line=line.decode().strip()[5:]
    parts=line.split(',', 3)
    return parts

async def checkParams(reader, writer):
    writer.write(('AT+PARAMETER?\r\n').encode())
    params = await reader.readline()
    print(params.decode())

async def setParams(writer, spreadingFactor, bandwith, codingRate, preambleLength):
    writer.write((f'AT+PARAMETER={spreadingFactor},{bandwith},{codingRate},{preambleLength}\r\n').encode())
    await writer.drain()