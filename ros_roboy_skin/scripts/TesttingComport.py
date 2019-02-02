import numpy as np

import serial
import serial.tools.list_ports

#from std_msgs.msg import String
#from std_msgs.msg import Float32MultiArray
#from std_msgs.msg import MultiArrayDimension

ports = list(serial.tools.list_ports.comports())
port = None
varM = []  # initialize empty array
varN = []  # initialize empty array
for p in ports:
    print('Checking port %s / %s' % (p[0], p[1]))
    if "uino" in p[1].lower():  # Find "ardUINO" and "genUINO" boards
        port = p
        break

if port is None:
    print('Could not find a connected Arduino')
    exit(0)

print('Using the Arduino connected on:')
print(port[0] + ' / ' + port[1])

portPath = port[0]  # Must match value shown on Arduino IDE
baud = 1000000  # Must match Arduino baud rate
timeout = 5  # Seconds
max_num_readings = 16000
num_signals = 1


def create_serial_obj(portPath, baud_rate, tout):
    """
    Given the port path, baud rate, and timeout value, creates
    and returns a pyserial object.
    """
    return serial.Serial(portPath, baud_rate, timeout=tout)


def read_serial_line(serial):
    serial_data = serial.readline()
    serial_data = serial_data.decode("utf-8")  # ser.readline returns a binary, convert to string
    # print (serial_data)
    return serial_data


print("Creating serial object...")
serial_obj = create_serial_obj(portPath, baud, timeout)

print("hello world")

# while(True):
#     serial_data = read_serial_line(serial_obj)
#
#     if "Snapshot" in serial_data:
#         #varM = serial_data.split('\r\n')
#         #varN.append(varM)
#         m = serial_data[-3]
#         n = serial_data[-5]
#         print(m)
#         break
m = 7
while(True):
    serial_data = read_serial_line(serial_obj)
    if "Snapshot" not in serial_data:
        for m in range(m):
            print(serial_data)
        print("---------------------")
