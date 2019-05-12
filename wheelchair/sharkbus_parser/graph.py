import sys
import binascii
import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt

filename = sys.argv[1]

stream = []

# Call repeatedly
def parseStream(byte):
    global stream

    if byte == 0x0F:
        return stream[:]

    elif byte & 0b10000000 != 0b10000000: # First byte has bit 7 as 0
        stream = []
        stream.append(byte)

    else:
        stream.append(byte)
        return None

speed_arr = []
angle_arr = []

def printPacket(bytes):
    global speed_arr, angle_arr

    packet_type = bytes[0] & 0b1111

    data = bytes[1:]

    if packet_type == 0: # SR General Info
        speed_msb = data[0] & 0b01111111
        angle_msb = data[1] & 0b01111111
        maxspeed_msb = data[2] & 0b01111111
        lsbs = data[3] & 0b01111111
        maxspeed_lsb = (lsbs & 0b01000000) >> 6
        speed_lsb = (lsbs & 0b00111000) >> 3
        angle_lsb = (lsbs & 0b00000111)

        speed = (speed_msb << 3) | speed_lsb
        angle = (angle_msb << 3) | angle_lsb
        maxspeed = (maxspeed_msb << 1) | maxspeed_lsb

        states = {0b1000000: "Joystick Error", 0b0100000: "Speed Pot Error", 0b0010000: "Local Fault", 0b0001000: "Battery Charger Inhibit", 0b0000100: "Power Switch", 0b0000010: "Horn", 0b0000001: "Lock"}

        current_states = []

        for state in states:
            if data[4] & state == state:
                current_states.append(states[state])

        angle_arr.append(angle)
        speed_arr.append(speed)

        print("SR General Information")
        print("    Speed: {}".format(speed))
        print("    Angle: {}".format(angle))
        print("    Max Speed: {}".format(maxspeed))
        print("    States: {}".format(", ".join(current_states)))
        print("")

    else:
        print("Unrecognized Packet Type {}\n".format(str(packet_type).zfill(2)))

file = open(filename, "rb")
with file:
    while True:
        byte = file.read(1)
        try:
            x = parseStream(int(binascii.hexlify(byte), 16))

            if x is not None and len(x) > 0:
                printPacket(x)
        except:
            break

    print("Graphing")
    plt.plot(speed_arr, "r")
    plt.plot(angle_arr, "g")
    plt.show()
