import numpy as np

import serial
import serial.tools.list_ports
import re

# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import MultiArrayDimension

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
max_num_readings = 1000
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


def read_serial_data(serial):
    """
    Given a pyserial object (serial). Outputs a list of lines read in
    from the serial port
    """
    serial.flushInput()

    serial_data = []
    readings_left = True
    timeout_reached = False

    while readings_left and not timeout_reached:
        serial_line = serial.readline()
        if serial_line == '':
            timeout_reached = True
        else:
            serial_data.append(serial_line)
            if len(serial_data) == max_num_readings:
                readings_left = False

    return serial_data


def is_number(string):
    """
    Given a string returns True if the string represents a number.
    Returns False otherwise.
    """
    try:
        float(string)
        return True
    except ValueError:
        return False


def clean_serial_data(data):
    """
    Given a list of serial lines (data). Removes all characters.
    Returns the cleaned list of lists of digits.
    Given something like: ['0.5000,33\r\n', '1.0000,283\r\n']
    Returns: [[0.5,33.0], [1.0,283.0]]
    """
    clean_data = []

    for line in data:
        line_data = re.findall("\d*\.\d*|\d*", line)  # Find all digits
        line_data = [float(element) for element in line_data if is_number(element)]  # Convert strings to float
        if len(line_data) >= 2:
            clean_data.append(line_data)

    return clean_data


print("Creating serial object...")
serial_obj = create_serial_obj(portPath, baud, timeout)

print("hello world")

# print ("Reading serial data...")
# serial_data = read_serial_data(serial_obj)
# print (serial_data)


# print ("Cleaning data.................")
# clean_data =  clean_serial_data(serial_data)
# print (clean_data)
m = 0
n = 0

while (True):

    serial_data = read_serial_line(serial_obj)

    if "Snapshot" in serial_data:
        m = int(serial_data[-3])
        n = int(serial_data[-5])
        print(m, n)
        break

while (True):
    serial_data = read_serial_line(serial_obj)
    row_data = []
    unclean_data = []
    clean_data = []

    first_unclean_data = []
    first_clean_data = []

    if "Snapshot" not in serial_data:

        first_unclean_data = (serial_data.split(','))
        first_unclean_data.pop()

        #         print(first_unclean_data)
        #         print("++++++++++++++++++++++++++++++++")
        for x in range(m):
            #             print(x)
            serial_data = read_serial_line(serial_obj)

            #             print(serial_data)
            #             print("----------------------------")
            unclean_data = (serial_data.split(','))
            unclean_data.pop()

            if unclean_data:
                clean_data.append(unclean_data)

        clean_data.insert(0, first_unclean_data)

        print(clean_data)
        print("***********************")