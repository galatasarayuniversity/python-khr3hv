#!/usr/bin/env python

import serial
import time
import sys

######################################################
# define code byte sequences for the commands
ack_command = "\x04\xfe\x06\x08" # use for PING. will send 4 bytes and receive another 4 bytes.
get_options = "\x0a\x00\x20\x00\x00\x00\x00\x00\x02\x2c" # not used.
stop_motion = "\x13\x00\x02\x00\x00\x00\x21\x87\xfd\x03\x00\x00\x00\x00\x00\x00\x00\x00\xbd"
restore_status = "\x09\x00\x02\x00\x00\x00\x23\x87\xb5"
query_status = "\x0a\x00\x20\x00\x00\x00\x00\x00\x02\x2c"

# Motions
play_motion_bytes = [None] * 101
play_motion_bytes[0] = "\x07\x0c\xb8\x0b\x00\x00\xd6"
play_motion_bytes[1] = "\x07\x0c\xb8\x13\x00\x00\xde"
play_motion_bytes[2] = "\x07\x0c\xb8\x1b\x00\x00\xe6"
play_motion_bytes[3] = "\x07\x0c\xb8\x23\x00\x00\xee"
play_motion_bytes[4] = "\x07\x0c\xb8\x2b\x00\x00\xf6"
play_motion_bytes[5] = "\x07\x0c\xb8\x33\x00\x00\xfe"
play_motion_bytes[6] = "\x07\x0c\xb8\x3b\x00\x00\x06"
play_motion_bytes[7] = "\x07\x0c\xb8\x43\x00\x00\x0e"
play_motion_bytes[8] = "\x07\x0c\xb8\x4b\x00\x00\x16"
play_motion_bytes[9] = "\x07\x0c\xb8\x53\x00\x00\x1e"
play_motion_bytes[10] = "\x07\x0c\xb8\x5b\x00\x00\x26"
play_motion_bytes[11] = "\x07\x0c\xb8\x63\x00\x00\x2e"
play_motion_bytes[12] = "\x07\x0c\xb8\x6b\x00\x00\x36"
play_motion_bytes[13] = "\x07\x0c\xb8\x73\x00\x00\x3e"
play_motion_bytes[14] = "\x07\x0c\xb8\x7b\x00\x00\x46"
play_motion_bytes[15] = "\x07\x0c\xb8\x83\x00\x00\x4e"
play_motion_bytes[16] = "\x07\x0c\xb8\x8b\x00\x00\x56"
play_motion_bytes[17] = "\x07\x0c\xb8\x93\x00\x00\x5e"
play_motion_bytes[18] = "\x07\x0c\xb8\x9b\x00\x00\x66"
play_motion_bytes[19] = "\x07\x0c\xb8\xa3\x00\x00\x6e"
play_motion_bytes[20] = "\x07\x0c\xb8\xab\x00\x00\x76"
play_motion_bytes[21] = "\x07\x0c\xb8\xb3\x00\x00\x7e"
play_motion_bytes[22] = "\x07\x0c\xb8\xbb\x00\x00\x86"
play_motion_bytes[23] = "\x07\x0c\xb8\xc3\x00\x00\x8e"
play_motion_bytes[24] = "\x07\x0c\xb8\xcb\x00\x00\x96"
play_motion_bytes[25] = "\x07\x0c\xb8\xd3\x00\x00\x9e"
play_motion_bytes[26] = "\x07\x0c\xb8\xdb\x00\x00\xa6"
play_motion_bytes[27] = "\x07\x0c\xb8\xe3\x00\x00\xae"
play_motion_bytes[28] = "\x07\x0c\xb8\xeb\x00\x00\xb6"
play_motion_bytes[29] = "\x07\x0c\xb8\xf3\x00\x00\xbe"
play_motion_bytes[30] = "\x07\x0c\xb8\xfb\x00\x00\xc6"
play_motion_bytes[31] = "\x07\x0c\xb8\x03\x01\x00\xcf"
play_motion_bytes[32] = "\x07\x0c\xb8\x0b\x01\x00\xd7"
play_motion_bytes[33] = "\x07\x0c\xb8\x13\x01\x00\xdf"
play_motion_bytes[34] = "\x07\x0c\xb8\x1b\x01\x00\xe7"
play_motion_bytes[35] = "\x07\x0c\xb8\x23\x01\x00\xef"
play_motion_bytes[36] = "\x07\x0c\xb8\x2b\x01\x00\xf7"
play_motion_bytes[37] = "\x07\x0c\xb8\x33\x01\x00\xff"
play_motion_bytes[38] = "\x07\x0c\xb8\x3b\x01\x00\x07"
play_motion_bytes[39] = "\x07\x0c\xb8\x43\x01\x00\x0f"
play_motion_bytes[40] = "\x07\x0c\xb8\x4b\x01\x00\x17"
play_motion_bytes[41] = "\x07\x0c\xb8\x53\x01\x00\x1f"
play_motion_bytes[42] = "\x07\x0c\xb8\x5b\x01\x00\x27"
play_motion_bytes[43] = "\x07\x0c\xb8\x63\x01\x00\x2f"
play_motion_bytes[44] = "\x07\x0c\xb8\x6b\x01\x00\x37"
play_motion_bytes[45] = "\x07\x0c\xb8\x73\x01\x00\x3f"
play_motion_bytes[46] = "\x07\x0c\xb8\x7b\x01\x00\x47"
play_motion_bytes[47] = "\x07\x0c\xb8\x83\x01\x00\x4f"
play_motion_bytes[48] = "\x07\x0c\xb8\x8b\x01\x00\x57"
play_motion_bytes[49] = "\x07\x0c\xb8\x93\x01\x00\x5f"
play_motion_bytes[50] = "\x07\x0c\xb8\x9b\x01\x00\x67"
play_motion_bytes[51] = "\x07\x0c\xb8\xa3\x01\x00\x6f"
play_motion_bytes[52] = "\x07\x0c\xb8\xab\x01\x00\x77"
play_motion_bytes[53] = "\x07\x0c\xb8\xb3\x01\x00\x7f"
play_motion_bytes[54] = "\x07\x0c\xb8\xbb\x01\x00\x87"
play_motion_bytes[55] = "\x07\x0c\xb8\xc3\x01\x00\x8f"
play_motion_bytes[56] = "\x07\x0c\xb8\xcb\x01\x00\x97"
play_motion_bytes[57] = "\x07\x0c\xb8\xd3\x01\x00\x9f"
play_motion_bytes[58] = "\x07\x0c\xb8\xdb\x01\x00\xa7"
play_motion_bytes[59] = "\x07\x0c\xb8\xe3\x01\x00\xaf"
play_motion_bytes[60] = "\x07\x0c\xb8\xeb\x01\x00\xb7"
play_motion_bytes[61] = "\x07\x0c\xb8\xf3\x01\x00\xbf"
play_motion_bytes[62] = "\x07\x0c\xb8\xfb\x01\x00\xc7"
play_motion_bytes[63] = "\x07\x0c\xb8\x03\x02\x00\xd0"
play_motion_bytes[64] = "\x07\x0c\xb8\x0b\x02\x00\xd8"
play_motion_bytes[65] = "\x07\x0c\xb8\x13\x02\x00\xe0"
play_motion_bytes[66] = "\x07\x0c\xb8\x1b\x02\x00\xe8"
play_motion_bytes[67] = "\x07\x0c\xb8\x23\x02\x00\xf0"
play_motion_bytes[68] = "\x07\x0c\xb8\x2b\x02\x00\xf8"
play_motion_bytes[69] = "\x07\x0c\xb8\x33\x02\x00\x00"
play_motion_bytes[70] = "\x07\x0c\xb8\x3b\x02\x00\x08"
play_motion_bytes[71] = "\x07\x0c\xb8\x43\x02\x00\x10"
play_motion_bytes[72] = "\x07\x0c\xb8\x4b\x02\x00\x18"
play_motion_bytes[73] = "\x07\x0c\xb8\x53\x02\x00\x20"
play_motion_bytes[74] = "\x07\x0c\xb8\x5b\x02\x00\x28"
play_motion_bytes[75] = "\x07\x0c\xb8\x63\x02\x00\x30"
play_motion_bytes[76] = "\x07\x0c\xb8\x6b\x02\x00\x38"
play_motion_bytes[77] = "\x07\x0c\xb8\x73\x02\x00\x40"
play_motion_bytes[78] = "\x07\x0c\xb8\x7b\x02\x00\x48"
play_motion_bytes[79] = "\x07\x0c\xb8\x83\x02\x00\x50"
play_motion_bytes[80] = "\x07\x0c\xb8\x8b\x02\x00\x58"
play_motion_bytes[81] = "\x07\x0c\xb8\x93\x02\x00\x60"
play_motion_bytes[82] = "\x07\x0c\xb8\x9b\x02\x00\x68"
play_motion_bytes[83] = "\x07\x0c\xb8\xa3\x02\x00\x70"
play_motion_bytes[84] = "\x07\x0c\xb8\xab\x02\x00\x78"
play_motion_bytes[85] = "\x07\x0c\xb8\xb3\x02\x00\x80"
play_motion_bytes[86] = "\x07\x0c\xb8\xbb\x02\x00\x88"
play_motion_bytes[87] = "\x07\x0c\xb8\xc3\x02\x00\x90"
play_motion_bytes[88] = "\x07\x0c\xb8\xcb\x02\x00\x98"
play_motion_bytes[89] = "\x07\x0c\xb8\xd3\x02\x00\xa0"
play_motion_bytes[90] = "\x07\x0c\xb8\xdb\x02\x00\xa8"
play_motion_bytes[91] = "\x07\x0c\xb8\xe3\x02\x00\xb0"
play_motion_bytes[92] = "\x07\x0c\xb8\xeb\x02\x00\xb8"
play_motion_bytes[93] = "\x07\x0c\xb8\xf3\x02\x00\xc0"
play_motion_bytes[94] = "\x07\x0c\xb8\xfb\x02\x00\xc8"
play_motion_bytes[95] = "\x07\x0c\xb8\x03\x03\x00\xd1"
play_motion_bytes[96] = "\x07\x0c\xb8\x0b\x03\x00\xd9"
play_motion_bytes[97] = "\x07\x0c\xb8\x13\x03\x00\xe1"
play_motion_bytes[98] = "\x07\x0c\xb8\x1b\x03\x00\xe9"
play_motion_bytes[99] = "\x07\x0c\xb8\x23\x03\x00\xf1"
play_motion_bytes[100] = "\x07\x0c\xb8\x2b\x03\x00\xf9"

def play_rcb4_motion(mtn_num, sleep_wait=0):
    # write array for Stop Motion
    print("Stopping Motion", stop_motion)
    ser.write(stop_motion)
    a = ser.read(4)
    print a.encode("hex")

    # write array for Play Motion
    print ("Playing Motion", play_motion_bytes[mtn_num])
    ser.write(play_motion_bytes[mtn_num])
    a = ser.read(4)
    print a.encode("hex")

    # write array for Restore memory
    print("Restoring Status %s\n", restore_status)
    ser.write(restore_status)
    a = ser.read(4)
    print a.encode("hex")

    # new feature to test: Query for execution status
    print("Restoring Status %s\n", restore_status)

    while 1:
        ser.write(query_status)
        a = ser.read(5)
        if len(a) < 5:
            print "Invalid response to Execution Status Query. exiting"
            break

        print ("Received ", a.encode("hex"), "  Key Byte: ", a[2].encode("hex"))
        if a[2] != "#": break
        time.sleep(0.05)


    # wait/sleep until the motion ends. this should be hard coded/timed and passed into this function.
    print ("Sleeping ", sleep_wait)
    time.sleep(sleep_wait)

    print "Done."

######################################################3
# MAIN CODE

if __name__ == "__main__":
    if len(sys.argv) != 3 and len(sys.argv) != 4:
        print "Usage: %s <serial port> <motion number> [timeout]" % sys.argv[0]
        sys.exit(1)
    else:
        ser = serial.Serial(sys.argv[1], 115200, serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE, 1)

        if len(sys.argv) == 4:
            timeout = float(sys.argv[3])
        else:
            timeout = 0

        mtn_number = int(sys.argv[2])

        play_rcb4_motion(mtn_number, timeout)

        ser.close()
        sys.exit(0)