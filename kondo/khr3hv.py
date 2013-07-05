#!/usr/bin/env python

import serial
import time

import xml.etree.ElementTree as ET

class KHR3HV(object):
    """Class for accessing Kondo KHR3HV humanoid robot."""
    # Byte sequences defining motions embedded in the device
    __motion_bytes = [
                        "\x07\x0c\xb8\x0b\x00\x00\xd6",     # 0
                        "\x07\x0c\xb8\x13\x00\x00\xde",
                        "\x07\x0c\xb8\x1b\x00\x00\xe6",
                        "\x07\x0c\xb8\x23\x00\x00\xee",
                        "\x07\x0c\xb8\x2b\x00\x00\xf6",
                        "\x07\x0c\xb8\x33\x00\x00\xfe",
                        "\x07\x0c\xb8\x3b\x00\x00\x06",
                        "\x07\x0c\xb8\x43\x00\x00\x0e",
                        "\x07\x0c\xb8\x4b\x00\x00\x16",
                        "\x07\x0c\xb8\x53\x00\x00\x1e",
                        "\x07\x0c\xb8\x5b\x00\x00\x26",     # 10
                        "\x07\x0c\xb8\x63\x00\x00\x2e",
                        "\x07\x0c\xb8\x6b\x00\x00\x36",
                        "\x07\x0c\xb8\x73\x00\x00\x3e",
                        "\x07\x0c\xb8\x7b\x00\x00\x46",
                        "\x07\x0c\xb8\x83\x00\x00\x4e",
                        "\x07\x0c\xb8\x8b\x00\x00\x56",
                        "\x07\x0c\xb8\x93\x00\x00\x5e",
                        "\x07\x0c\xb8\x9b\x00\x00\x66",
                        "\x07\x0c\xb8\xa3\x00\x00\x6e",
                        "\x07\x0c\xb8\xab\x00\x00\x76",     # 20
                        "\x07\x0c\xb8\xb3\x00\x00\x7e",
                        "\x07\x0c\xb8\xbb\x00\x00\x86",
                        "\x07\x0c\xb8\xc3\x00\x00\x8e",
                        "\x07\x0c\xb8\xcb\x00\x00\x96",
                        "\x07\x0c\xb8\xd3\x00\x00\x9e",
                        "\x07\x0c\xb8\xdb\x00\x00\xa6",
                        "\x07\x0c\xb8\xe3\x00\x00\xae",
                        "\x07\x0c\xb8\xeb\x00\x00\xb6",
                        "\x07\x0c\xb8\xf3\x00\x00\xbe",
                        "\x07\x0c\xb8\xfb\x00\x00\xc6",     # 30
                        "\x07\x0c\xb8\x03\x01\x00\xcf",
                        "\x07\x0c\xb8\x0b\x01\x00\xd7",
                        "\x07\x0c\xb8\x13\x01\x00\xdf",
                        "\x07\x0c\xb8\x1b\x01\x00\xe7",
                        "\x07\x0c\xb8\x23\x01\x00\xef",
                        "\x07\x0c\xb8\x2b\x01\x00\xf7",
                        "\x07\x0c\xb8\x33\x01\x00\xff",
                        "\x07\x0c\xb8\x3b\x01\x00\x07",
                        "\x07\x0c\xb8\x43\x01\x00\x0f",
                        "\x07\x0c\xb8\x4b\x01\x00\x17",     # 40
                        "\x07\x0c\xb8\x53\x01\x00\x1f",
                        "\x07\x0c\xb8\x5b\x01\x00\x27",
                        "\x07\x0c\xb8\x63\x01\x00\x2f",
                        "\x07\x0c\xb8\x6b\x01\x00\x37",
                        "\x07\x0c\xb8\x73\x01\x00\x3f",
                        "\x07\x0c\xb8\x7b\x01\x00\x47",
                        "\x07\x0c\xb8\x83\x01\x00\x4f",
                        "\x07\x0c\xb8\x8b\x01\x00\x57",
                        "\x07\x0c\xb8\x93\x01\x00\x5f",
                        "\x07\x0c\xb8\x9b\x01\x00\x67",     # 50
                        "\x07\x0c\xb8\xa3\x01\x00\x6f",
                        "\x07\x0c\xb8\xab\x01\x00\x77",
                        "\x07\x0c\xb8\xb3\x01\x00\x7f",
                        "\x07\x0c\xb8\xbb\x01\x00\x87",
                        "\x07\x0c\xb8\xc3\x01\x00\x8f",
                        "\x07\x0c\xb8\xcb\x01\x00\x97",
                        "\x07\x0c\xb8\xd3\x01\x00\x9f",
                        "\x07\x0c\xb8\xdb\x01\x00\xa7",
                        "\x07\x0c\xb8\xe3\x01\x00\xaf",
                        "\x07\x0c\xb8\xeb\x01\x00\xb7",     # 60
                        "\x07\x0c\xb8\xf3\x01\x00\xbf",
                        "\x07\x0c\xb8\xfb\x01\x00\xc7",
                        "\x07\x0c\xb8\x03\x02\x00\xd0",
                        "\x07\x0c\xb8\x0b\x02\x00\xd8",
                        "\x07\x0c\xb8\x13\x02\x00\xe0",
                        "\x07\x0c\xb8\x1b\x02\x00\xe8",
                        "\x07\x0c\xb8\x23\x02\x00\xf0",
                        "\x07\x0c\xb8\x2b\x02\x00\xf8",
                        "\x07\x0c\xb8\x33\x02\x00\x00",
                        "\x07\x0c\xb8\x3b\x02\x00\x08",     # 70
                        "\x07\x0c\xb8\x43\x02\x00\x10",
                        "\x07\x0c\xb8\x4b\x02\x00\x18",
                        "\x07\x0c\xb8\x53\x02\x00\x20",
                        "\x07\x0c\xb8\x5b\x02\x00\x28",
                        "\x07\x0c\xb8\x63\x02\x00\x30",
                        "\x07\x0c\xb8\x6b\x02\x00\x38",
                        "\x07\x0c\xb8\x73\x02\x00\x40",
                        "\x07\x0c\xb8\x7b\x02\x00\x48",
                        "\x07\x0c\xb8\x83\x02\x00\x50",
                        "\x07\x0c\xb8\x8b\x02\x00\x58",     # 80
                        "\x07\x0c\xb8\x93\x02\x00\x60",
                        "\x07\x0c\xb8\x9b\x02\x00\x68",
                        "\x07\x0c\xb8\xa3\x02\x00\x70",
                        "\x07\x0c\xb8\xab\x02\x00\x78",
                        "\x07\x0c\xb8\xb3\x02\x00\x80",
                        "\x07\x0c\xb8\xbb\x02\x00\x88",
                        "\x07\x0c\xb8\xc3\x02\x00\x90",
                        "\x07\x0c\xb8\xcb\x02\x00\x98",
                        "\x07\x0c\xb8\xd3\x02\x00\xa0",
                        "\x07\x0c\xb8\xdb\x02\x00\xa8",     # 90
                        "\x07\x0c\xb8\xe3\x02\x00\xb0",
                        "\x07\x0c\xb8\xeb\x02\x00\xb8",
                        "\x07\x0c\xb8\xf3\x02\x00\xc0",
                        "\x07\x0c\xb8\xfb\x02\x00\xc8",
                        "\x07\x0c\xb8\x03\x03\x00\xd1",
                        "\x07\x0c\xb8\x0b\x03\x00\xd9",
                        "\x07\x0c\xb8\x13\x03\x00\xe1",
                        "\x07\x0c\xb8\x1b\x03\x00\xe9",
                        "\x07\x0c\xb8\x23\x03\x00\xf1",
                        "\x07\x0c\xb8\x2b\x03\x00\xf9",     # 100
                    ]

    # Commands
    # Use for PING. will send 4 bytes and receive another 4 bytes
    __cmd_ack = "\x04\xfe\x06\x08"
    __cmd_getoptions = "\x0a\x00\x20\x00\x00\x00\x00\x00\x02\x2c"
    __cmd_stopmotion = "\x13\x00\x02\x00\x00\x00\x21\x87\xfd\x03\x00\x00\x00\x00\x00\x00\x00\x00\xbd"
    __cmd_restorestatus = "\x09\x00\x02\x00\x00\x00\x23\x87\xb5"
    __cmd_querystatus = "\x0a\x00\x20\x00\x00\x00\x00\x00\x02\x2c"

    def __init__(self, port="/dev/ttyUSB0"):
        self.port = port
        self.__motion_desc = {}

    def open(self):
        """Open the device."""
        self.device = serial.Serial(self.port, 115200,
                                    serial.EIGHTBITS,
                                    serial.PARITY_EVEN,
                                    serial.STOPBITS_ONE, 1)

    def parse_h4p(self, path):
        """Parse Heart-to-Heart project file for motion information."""
        tree = ET.parse(path)
        root = tree.getroot()
        if root.tag != "Rcb4":
            # Invalid file, skip
            print "Invalid .h4p file %s, skipping." % path
            return

        motions = root.find("MotionDataCollection")

        for motion in motions.getchildren():
            key, value = motion.getchildren()
            # key's tag is slot name, e.g. M008
            # value contains Number, Name, Date, MotionData, etc.

            motion_data = value.find("MotionData")
            # If MotionData is empty, the slot is not used.
            if motion_data.getchildren():
                # Slot is occupied, let's continue.
                name = value.find("Name").text
                number = int(value.find("Number").text)
                self.__motion_desc[name] = number

                # Inject lambda function for short-hand
                method_name = name.split("_", 1)[-1].split("(")[0].lower()
                method_name = method_name.replace(" ", "_").replace("-", "_")
                setattr(self, method_name, lambda m: self.play_motion(m))

    def play_motion(self, motion, timeout=None):
        """Initiate motion given by its number."""

        self.device.write(self.__cmd_stopmotion)
        ret = self.device.read(4)
        #print a.encode("hex")

        # write array for Play Motion
        self.device.write(self.__motion_bytes[motion])
        ret = self.device.read(4)

        # write array for Restore memory
        self.device.write(self.__cmd_restorestatus)
        ret = self.device.read(4)

        # new feature to test: Query for execution status
        while 1:
            self.device.write(self.__cmd_querystatus)
            ret = self.device.read(5)
            if len(ret) < 5:
                print "Invalid response to Execution Status Query. exiting"
                break

            # print "Received ", a.encode("hex"), " Key Byte: ",
            # a[2].encode("hex"))
            if ret[2] != "#":
                break
            time.sleep(0.05)

        # Optional wait until the motion ends
        if timeout:
            time.sleep(timeout)

    def close(self):
        """Close serial port."""
        self.device.close()