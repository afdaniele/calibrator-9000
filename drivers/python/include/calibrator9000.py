import signal
import time
import types
from threading import Thread

import serial

from structures import *


class Calibrator9000:
    _NUM_KNOBS = 6
    _FREQUENCY = 40.0

    _KNOB_VAR_NAME = {
        0: 'x',
        1: 'y',
        2: 'z',
        3: 'r',
        4: 'p',
        5: 'w'
    }

    _KNOB_COLOR_RGB = [
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [255, 255, 0],
        [255, 0, 255],
        [0, 255, 255]
    ]

    class Logger(object):
        def log(self, level, message, origin='drivers'):
            raise NotImplementedError("The method is not implemented")

    class StdOutLogger(Logger):
        def log(self, level, message, origin='drivers'):
            if isinstance(level, LogLevel) or level in LogLevel._value2member_map_:
                print('Calibrator[%s:%s] :: %s' % (origin, LogLevel(level).name, message))

    _device = None
    _device_thread = None
    _knob_status = [KnobStatus.DISABLED] * _NUM_KNOBS
    _device_status = DeviceStatus.UNCONFIGURED
    _shutdown_requested = False
    _logging_level = LogLevel.ERROR
    _logger = StdOutLogger()
    _inbound_packet = data_packet_t()
    _outbound_packet = request_packet_t()
    _payload_to_handler_map = {}
    _outbound_packet_seq = 0
    _listeners = {
        DataFeed.STATUS: {},
        DataFeed.DATA: {}
    }

    def __init__(self, port, baudrate=9600, log_level=LogLevel.ERROR):
        # check arguments
        if not isinstance(port, str):
            self._log(LogLevel.ERROR, "Argument 'port' must be an string indicating the device path")
            return
        if not isinstance(baudrate, int):
            self._log(LogLevel.ERROR, "Argument 'baudrate' must be an integer")
            return
        if baudrate not in serial.Serial.BAUDRATES:
            self._log(LogLevel.WARN, "Non-standard baudrate. Standard %r" % serial.Serial.BAUDRATES)
        self._logging_level = log_level
        # configure the serial connection (the parameters differs on the _device you are connecting to)
        self._device = serial.Serial(port=port, baudrate=baudrate, timeout=None)
        # TODO: handle errors

        # create payload to handler function map
        self._payload_to_handler_map = {
            DataPacketType.EMPTY: lambda _: True,
            DataPacketType.STATUS: self._process_payload_status,
            DataPacketType.DATA: self._process_payload_data,
            DataPacketType.LOG: self._process_payload_log
        }
        # create and launch device thread
        self._device_thread = Thread(target=self._start)
        self._device_thread.start()

    def reset(self):
        self._outbound_packet.header.operation = RequestOperation.RESET
        self._outbound_packet.payload = request_payload_empty_t()
        self._send_packet()

    def initialize(self, **args):
        self._outbound_packet.header.operation = RequestOperation.INITIALIZE
        self._outbound_packet.payload = request_payload_initialize_t()
        for _, attr_name, _ in self._outbound_packet.payload._attributes:
            if attr_name in args:
                self._outbound_packet.payload.__dict__[attr_name] = args[attr_name]
        self._send_packet()

    def recalibrate(self, **args):
        self._outbound_packet.header.operation = RequestOperation.RECALIBRATE
        self._outbound_packet.payload = request_payload_recalibrate_t()
        for _, attr_name, _ in self._outbound_packet.payload._attributes:
            if attr_name in args:
                self._outbound_packet.payload.__dict__[attr_name] = args[attr_name]
        self._send_packet()

    def shutdown(self, halt_device=False):
        self._shutdown_requested = True
        if halt_device:
            self._outbound_packet.header.operation = RequestOperation.SHUTDOWN
            self._outbound_packet.payload = request_payload_empty_t()
            self._send_packet()

    def subscribe(self, callback, datafeed=DataFeed.DATA):
        if isinstance(callback, types.FunctionType):
            callback_id = id(callback)
            if callback_id not in self._listeners[datafeed]:
                self._listeners[datafeed][callback_id] = callback
        else:
            self._log(LogLevel.WARN, "Argument 'callback' passed to function 'subscribe' must be a function.")

    def unsubscribe(self, callback, datafeed=DataFeed.DATA):
        if isinstance(callback, types.FunctionType):
            callback_id = id(callback)
            self._listeners[datafeed].pop(callback_id, None)
        else:
            self._log(LogLevel.WARN, "Argument 'callback' passed to function 'unsubscribe' must be a function.")

    def _process_inbound_packet(self):
        packet = self._inbound_packet
        # get the handler function for this type of message
        payload_handler = self._payload_to_handler_map[DataPacketType(packet.header.type)]
        # process the payload
        payload_handler(packet.payload)

    def _process_payload_status(self, payload):
        if not isinstance(payload, data_payload_status_t):
            self._log(LogLevel.WARN, "Received packet of type STATUS but payload is of type '%s'" % type(payload))
        # process packet
        self._device_status = DeviceStatus(payload.device_status)
        s = [payload.status_x, payload.status_y, payload.status_z, payload.status_r, payload.status_p, payload.status_w]
        for i in range(len(self._knob_status)):
            self._knob_status[i] = KnobStatus(s[i])
        # generate event
        self._generate_event(DataFeed.STATUS, payload)

    def _process_payload_data(self, payload):
        if not isinstance(payload, data_payload_data_t):
            self._log(LogLevel.WARN, "Received packet of type DATA but payload is of type '%s'" % type(payload))
        if self._device_status == DeviceStatus.WORKING:
            # this fixes the problem of empty payload (never found the problem)
            if len([1 for k in data_payload_data_t._attributes if payload.__dict__[k[1]] == 0]) == len(
                    data_payload_data_t._attributes):
                return
            # generate event
            self._generate_event(DataFeed.DATA, payload)

    def _process_payload_log(self, payload):
        if not isinstance(payload, data_payload_log_t):
            self._log(LogLevel.WARN, "Received packet of type LOG but payload is of type '%s'" % type(payload))
        # process packet
        self._log(LogLevel(payload.level), payload.message, origin='firmware')

    def _generate_event(self, datafeed, data):
        for _, fcn in list(self._listeners[datafeed].items()):
            fcn(data)

    def _send_packet(self):
        self._outbound_packet.header.seq = self._outbound_packet_seq
        bytestr = self._outbound_packet.encode()
        self._device.write(bytestr + b'\n')
        self._outbound_packet_seq += 1

    def _start(self):
        while not self._shutdown_requested:
            while self._device.inWaiting() > 0:
                str_in = self._device.readline()
                # remove new line char
                str_in_stripped = str_in.rstrip()
                # process byte string
                success, _ = self._inbound_packet.decode(str_in_stripped)
                if success:  # packet is valid
                    # DEBUGGING
                    self._debug("Packet received")
                    self._debug("Raw data: %d byets %s" % (len(str_in), self._packet_as_string(self._inbound_packet)))
                    # interpret packet
                    self._process_inbound_packet()
            # keep spinning
            time.sleep(1.0 / Calibrator9000._FREQUENCY)

    def _packet_as_string(self, packet):
        packet_str = "\n"
        packet_str += "\tPacket: %d bytes (Header: %d bytes, Payload: %d bytes)\n" % (
            packet.get_size(),
            packet.header.get_size(),
            packet.payload.get_size() if isinstance(packet.payload, Struct) else 0
        )
        packet_str += "\tHeader[%s]\n" % (
            ', '.join(
                ['%s=%r' % (a[1], self._inbound_packet.header.__dict__[a[1]]) for a in
                 self._inbound_packet.header._attributes]
            )
            ,)
        packet_str += "\tPayload[%s]" % (
            ', '.join(
                ['%s=%r' % (a[1], self._inbound_packet.payload.__dict__[a[1]]) for a in
                 self._inbound_packet.payload._attributes]
            )
            ,)
        # return result
        return packet_str

    def _debug(self, message):
        self._log(LogLevel.SW_DEBUG, message)

    def _log(self, level, message, origin='drivers'):
        if level.value <= self._logging_level:
            self._logger.log(origin, level, message)
            # print 'Calibrator[%s:%s] :: %s' % ( origin, LogLevel(level).name, message )


# test sample
if __name__ == '__main__':
    # create device interface
    c = Calibrator9000(
        port='/dev/ttyACM0',
        baudrate=9600,
        log_level=LogLevel.SW_DEBUG
    )

    time.sleep(2)

    c.initialize(
        enable_x=True,
        enable_y=True,
        enable_z=True,
        enable_r=True,
        enable_p=True,
        enable_w=True
    )


    # c.recalibrate(
    #     recalibrate_x=True,
    #     recalibrate_y=True,
    #     recalibrate_z=True,
    #     recalibrate_r=True,
    #     recalibrate_p=True,
    #     recalibrate_w=True
    # )

    # c.initialize( enable_x=True )
    # c.recalibrate( recalibrate_x=1 )

    def c_fcn(config):
        print('Axis: [ x: %+05.2f,  y: %+05.2f,  z: %+05.2f,  r: %+05.2f,  p: %+05.2f,  w: %+05.2f ]' % (
            config.axis_x,
            config.axis_y,
            config.axis_z,
            config.axis_r,
            config.axis_p,
            config.axis_w
        ))


    c.subscribe(c_fcn)


    def s_fcn(status):
        print('Status: [device:%s, x:%s, y:%s, z:%s, r:%s, p:%s, w:%s]' % (
            DeviceStatus(status.device_status).name,
            KnobStatus(status.status_x).name,
            KnobStatus(status.status_y).name,
            KnobStatus(status.status_z).name,
            KnobStatus(status.status_r).name,
            KnobStatus(status.status_p).name,
            KnobStatus(status.status_w).name
        ))


    # c.subscribe( s_fcn, datafeed=DataFeed.STATUS )

    # listen for Ctrl-C
    signal.signal(signal.SIGINT, lambda _a, _b: c.shutdown())
    signal.pause()
