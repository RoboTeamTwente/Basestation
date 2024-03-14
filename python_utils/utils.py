import enum
import os
import re
import time
from inspect import getmembers

import serial
import usb.core
import usb.util


class Basestation(object):
    class BasestationV1(object):
        """
        The basestation with hardware version 1.

        This basestation uses a serial connection in order to communicate with the computer.
        """

        def __init__(self):
            self.byte_buffer = bytes()
            self.connection = openContinuous(timeout=0.01)

        def write(self, data) -> int:
            return self.connection.write(data)

        def read(self) -> int:
            bytes_waiting = self.connection.inWaiting()
            if bytes_waiting > 0:
                self.byte_buffer += self.connection.read(bytes_waiting)
            return bytes_waiting

    class BasestationV2(object):
        """
        The basestation with hardware version 2.

        This basestation uses a high-speed usb connection with multiple interfaces (low and high priority).
        """

        def __init__(self):
            """
            Initializes the byte buffers for both the high and low priority interfaces. And sets up the read and write
            endpoints for each interface.
            """
            self.byte_buffer_high_priority = bytes()
            self.byte_buffer_low_priority = bytes()

            # Find the USB device
            device = None
            i = 0
            while device is None:
                device = usb.core.find(idVendor=1155, idProduct=25432)
                if device is None:
                    print(f"\r", end='')
                    print(f"\r[BasestationV2] Basestation could not be found {chr(33 + i)}", end='')
                    i = (i + 1) % 93
                    time.sleep(0.1)
            print(f"\r[BasestationV2] Connected to basestation")

            # Configure the basestation for high and low priority interfaces
            device.set_configuration()
            device.set_interface_altsetting(interface=0, alternate_setting=1)
            configuration = device.get_active_configuration()

            # Find the high and low priority interfaces
            high_prio_interface = configuration[(0, 1)]
            low_prio_interface = configuration[(1, 0)]

            # Connect to the endpoints (IN, OUT) for each interface (high, low)
            self._high_prio_out = usb.util.find_descriptor(
                high_prio_interface,
                # match the first OUT endpoint
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            )
            self._high_prio_in = usb.util.find_descriptor(
                high_prio_interface,
                # match the first IN endpoint,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
            )
            self._low_prio_out = usb.util.find_descriptor(
                low_prio_interface,
                # match the first OUT endpoint
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            )
            self._low_prio_in = usb.util.find_descriptor(
                low_prio_interface,
                # match the first IN endpoint,
                custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
            )

        @property
        def byte_buffer(self):
            """
            TODO: Still have to figure out whether it is desirable to suddenly return a tuple.
            """
            return self.byte_buffer_high_priority, self.byte_buffer_low_priority

        @byte_buffer.setter
        def byte_buffer(self, value):
            """
            FIXME: This is strange behaviour to have, but I don't really know a good way to use this.
                For now it is only used to reset the byte buffers.
            """
            self.byte_buffer_low_priority = value
            self.byte_buffer_high_priority = value

        def write(self, data, high_priority=False) -> int:
            if high_priority:
                return self._high_prio_out.write(data)
            else:
                return self._low_prio_out.write(data)

        def read(self) -> int:
            byte_count = 0

            # Try to read both the low and high priority interfaces
            try:
                bytes_read = self._high_prio_in.read(self._high_prio_in.wMaxPacketSize, 1)
                self.byte_buffer_high_priority += bytes_read
                byte_count += len(bytes_read)
            except usb.core.USBTimeoutError:
                pass

            try:
                bytes_read = self._low_prio_in.read(self._low_prio_in.wMaxPacketSize, 1)
                self.byte_buffer_low_priority += bytes_read
                byte_count += len(bytes_read)
                print(f'Received {len(bytes_read)} on the low priority connection')
            except usb.core.USBTimeoutError:
                pass

            return byte_count

    def __init__(self, version=None):
        """
        Initializes and connects with the basestation.

        The constructor automatically detects and connects with the first basestation that it finds,
        if you want to override this behaviour, specify the exact basestation version that you want
        to connect to.

        :param version If defined only connect to this specific hardware version.
        """
        i = 0
        while version is None:
            i = (i + 1) % 93
            # If we find a serial connection, we use the basestation v1 connection protocol.
            # If we find an usb connection, we use the basestation v2 connection protocol
            if getBasestationPath() or getSTLinkPath():
                version = ConnectionVersion.BASESTATION_V1
                print("[Basestation] Detected a basestation with hardware version 1")
            elif usb.core.find(idVendor=1155, idProduct=25432):
                version = ConnectionVersion.BASESTATION_V2
                print("[Basestation] Detected a basestation with hardware version 2")
            else:
                print(f"\r[Basestation] Could detect any basestation {chr(33 + i)}", end="")
                time.sleep(0.1)

        self.version = version
        if version == ConnectionVersion.BASESTATION_V1 or version == ConnectionVersion.ST_LINK:
            self.bs = self.BasestationV1()
        elif version == ConnectionVersion.BASESTATION_V2:
            self.bs = self.BasestationV2()

    @property
    def byte_buffer(self):
        return self.bs.byte_buffer

    @byte_buffer.setter
    def byte_buffer(self, value):
        self.bs.byte_buffer = value

    def write(self, data, high_priority=False) -> int:
        """
        Writes data to the basestation.

        :returns The total amount of bytes written
        """
        if self.version == ConnectionVersion.BASESTATION_V2:
            return self.bs.write(data, high_priority)
        else:
            return self.bs.write(data)

    def read(self) -> int:
        """
        Reads data from the basestation and writes it into a byte buffer.

        :returns The total amount of bytes read
        """
        return self.bs.read()


class ConnectionVersion(enum.Enum):
    BASESTATION_V1 = 0
    ST_LINK = 1
    BASESTATION_V2 = 2


def openPort(port=None, suppressError=True, timeout=None):
    ser = None
    if port is None:
        return None

    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
    except serial.serialutil.SerialException as e:
        print("[open][SerialException] Could not open port", port)

    return ser


def openContinuous(*args, **kwargs):
    ser = None
    i = -1

    while ser is None:
        i = (i + 1) % 93

        if "port" not in kwargs or kwargs["port"] is None:
            kwargs["port"] = getBasestationPath()

        if "port" not in kwargs or kwargs["port"] is None:
            kwargs["port"] = getSTLinkPath()

        if kwargs["port"] is None:
            print("\r[openContinuous] Basestation path not found %s " % chr(33 + i), end="")
        else:
            ser = openPort(**kwargs, suppressError=True)
            print("\r[openContinuous] %s " % chr(33 + i), end="")

        time.sleep(0.1)
    print(f"\r[openContinuous] Basestation opened at {ser.port}")
    return ser


def getBasestationPath():
    try:
        usb_devices = os.listdir("/dev/serial/by-id")
        basestations = [device for device in usb_devices if device.startswith("usb-RTT_BaseStation")]
        if 0 < len(basestations):
            return os.path.join("/dev/serial/by-id", basestations[0])
        return None
    except Exception:
        return None


def getSTLinkPath():
    try:
        usb_devices = os.listdir("/dev/serial/by-id")
        stlinks = [device for device in usb_devices if device.startswith("usb-STMicroelectronics_STM32_STLink")]
        if 0 < len(stlinks):
            return os.path.join("/dev/serial/by-id", stlinks[0])
        return None
    except Exception:
        return None


def printCompletePacket(rc):
    types_allowed = [int, str, bool, float]
    maxLength = max([len(k) for k, v in getmembers(rc)])
    title = re.findall(r"_(\w+) ", str(rc))[0]

    lines = [("┌─ %s " % title) + ("─" * 100)[:maxLength * 2 + 2 - len(title)] + "┐"]
    members = [[k, v] for k, v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__")]

    lines += ["│ %s : %s │" % (k.rjust(maxLength), str(v).strip().ljust(maxLength)) for k, v in members]
    lines += ["└" + ("─" * (maxLength * 2 + 5)) + "┘"]
    print("\n".join(lines))


def packetToDict(rc):
    types_allowed = [int, str, bool, float]
    members = {k: v for k, v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__")}
    return members
