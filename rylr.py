import asyncio
import serial_asyncio

class Packet:
    def __init__(self, data, addr=0, rssi=0, snr=0):
        self.data = data
        self.addr = addr
        self.rssi = rssi
        self.snr = snr

    def __str__(self):
        return self.data


class RYLR:
    def __init__(self, port, **kw):
        self.port = port  # Port name (e.g., '/dev/ttyUSB0')
        self.transport = None
        self.protocol = None
        self._packet = None
        self._resp = None
        self._waiting = []
        self._frequency = kw.get('frequency', 915.0)
        self._bandwidth = kw.get('bandwidth', 250000)
        self._spreading_factor = kw.get('spreading_factor', 10)
        self._coding_rate = kw.get('coding_rate', 8)
        self._preamble_length = kw.get('preamble_length', 4)

    async def init(self):
        # Create the serial connection
        self.transport, self.protocol = await serial_asyncio.create_serial_connection(
            asyncio.get_event_loop(),
            lambda: SerialProtocol(self),
            self.port,  # Pass the port name as a string
            baudrate=115200
        )
        await self.set_frequency(self._frequency)
        await self._set_parameters()

    async def send(self, msg, addr=2):
        await self._cmd(f'AT+SEND={addr},{len(msg)},{msg}')

    async def recv_packet(self):
        while self._packet is None:
            await asyncio.sleep(0.1)
        data = self._packet
        self._packet = None
        return data

    async def recv(self):
        pkt = await self.recv_packet()
        return pkt.data

    async def _cmd(self, x):
        self.transport.write((x + '\r\n').encode())  # Use transport for writing
        e = asyncio.Event()
        self._waiting.append(e)
        await e.wait()
        return self._resp

    async def loop(self):
        while True:
            try:
                data = await self.protocol.reader.readline()
                if data:
                    x = data.decode().strip()
                    if x.startswith('+RCV='):
                        self._recv(x[5:])
                    else:
                        self._resp = x
                        if self._waiting:
                            e = self._waiting.pop(0)
                            e.set()
            except Exception as e:
                print(f"Error reading from serial: {e}")
                break

    def _recv(self, x):
        try:
            # Split the data into addr, n, and the rest
            parts = x.split(',', 2)
            if len(parts) < 3:
                print(f"Malformed data received: {x}")
                return

            addr, n, rest = parts
            n = int(n)

            # Split the rest into data and rssi/snr
            data = rest[:n]
            rssi_snr = rest[n+1:].split(',')

            if len(rssi_snr) < 2:
                print(f"Malformed RSSI/SNR data: {rest}")
                return

            rssi, snr = rssi_snr
            self._packet = Packet(data, int(addr), int(rssi), int(snr))
        except Exception as e:
            print(f"Error processing received data: {e}")

    async def set_frequency(self, x):
        return await self._cmd(f'AT+BAND={round(x * 1000000)}')

    async def _set_parameters(self):
        sf = self._spreading_factor
        bws = (7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000)
        bw = 9
        for i in range(len(bws)):
            if self._bandwidth <= bws[i]:
                bw = i
                break
        cr = self._coding_rate - 4
        pl = self._preamble_length
        return await self._cmd(f'AT+PARAMETER={sf},{bw},{cr},{pl}')


class SerialProtocol(asyncio.Protocol):
    def __init__(self, rylr):
        self.rylr = rylr
        self.reader = None

    def connection_made(self, transport):
        self.transport = transport
        self.reader = asyncio.StreamReader()
        asyncio.StreamReaderProtocol(self.reader).connection_made(transport)
        print("Serial connection opened")

    def data_received(self, data):
        self.rylr._recv(data.decode())

    def connection_lost(self, exc):
        print("Serial connection closed")


async def main():
    port = '/dev/ttyUSB0'  # Replace with your serial port
    rylr = RYLR(port)
    await rylr.init()

    # Example usage
    await rylr.send("Hello, World!")
    response = await rylr.recv()
    print(f"Received: {response}")

    # Run the loop to handle incoming data
    await rylr.loop()


if __name__ == "__main__":
    asyncio.run(main())