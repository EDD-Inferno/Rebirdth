import serial
import time
test_string = "[serial port test]".encode('utf-8')
port_list = ["/dev/ttyAMA0","/dev/ttyAMA1"]
for port in port_list:
  ok = False
  try:
    buff = bytearray(len(test_string))
    serialPort = serial.Serial(port, 115200, timeout = 2, writeTimeout = 2)
    bytes_sent = serialPort.write(test_string)
    time.sleep(1)
    bytes_read = serialPort.readinto(buff)
    ok = bytes_read == bytes_sent
    serialPort.close()
  except IOError:
    pass
  print("port %s is %s" % (port, "OK" if ok else "NOT OK"))
