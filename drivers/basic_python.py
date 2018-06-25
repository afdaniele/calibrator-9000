import time
import serial
import struct
from enum import IntEnum


# // device status
# enum STATUS{
#   UNCONFIGURED = -1,
#   FAILURE = -2,
#   WORKING = 0,
#   CALIBRATING = 1
# };
# enum STATUS status = UNCONFIGURED;
#
# // led status
# enum LED_STATUS{
#   OFF = -1,
#   FIX = 0,
#   BLINK = 1,
#   BLINK_FAST = 2
# };
#
# // command request
# enum COMMAND_REQUEST_OP{  // this has size int16_t
#   NOOP = 0,
#   RESET = 1,
#   INITIALIZE = 2,
#   RECALIBRATE = 3,
#   SHUTDOWN = 4
# };
#
# #pragma pack(push, 1)
# typedef struct {
#   uint32_t seq;
#   int16_t operation;
#   int16_t checksum;
# } __attribute__((__packed__)) request_packet_header_t;
#
# typedef struct {
#   request_packet_header_t header;
#   byte payload[16];
# } __attribute__((__packed__)) request_packet_t;
#
# typedef struct {
#   uint8_t enable_x;
#   uint8_t enable_y;
#   uint8_t enable_z;
#   uint8_t enable_r;
#   uint8_t enable_p;
#   uint8_t enable_w;
#   byte unused[10];
# } __attribute__((__packed__)) request_payload_initialize_t;
#
# typedef struct {
#   uint8_t recalibrate_x;
#   uint8_t recalibrate_y;
#   uint8_t recalibrate_z;
#   uint8_t recalibrate_r;
#   uint8_t recalibrate_p;
#   uint8_t recalibrate_w;
#   byte unused[10];
# } __attribute__((__packed__)) request_payload_recalibrate_t;
# #pragma pack(pop)
#
# // data packet
# enum DATA_PACKET_TYPE{  // this has size int16_t
#   EMPTY = 0,
#   STATUS = 1,
#   DATA = 2,
#   LOG = 3
# };
#
# #pragma pack(push, 1)
# typedef struct {
#   uint32_t seq;
#   uint16_t type;
#   int16_t checksum;
# } __attribute__((__packed__)) data_packet_header_t;
#
# typedef struct {
#   data_packet_header_t header;
#   byte payload[24];
# } __attribute__((__packed__)) data_packet_t;
#
# typedef struct {
#   int16_t device_status;
#   uint8_t enabled_x;
#   uint8_t enabled_y;
#   uint8_t enabled_z;
#   uint8_t enabled_r;
#   uint8_t enabled_p;
#   uint8_t enabled_w;
#   uint8_t calibrated_x;
#   uint8_t calibrated_y;
#   uint8_t calibrated_z;
#   uint8_t calibrated_r;
#   uint8_t calibrated_p;
#   uint8_t calibrated_w;
#   byte unused[10];
# } __attribute__((__packed__)) data_payload_status_t;
#
# typedef struct {
#   float axis_x;
#   float axis_y;
#   float axis_z;
#   float axis_r;
#   float axis_p;
#   float axis_w;
# } __attribute__((__packed__)) data_payload_data_t;
#
# typedef struct {
#   uint8_t length;
#   uint8_t level;
#   char message[22];
# } __attribute__((__packed__)) data_payload_log_t;
# #pragma pack(pop)
#
# // log levels
# enum LOG_LEVEL{  // this has size int16_t
#   INFO = 0,
#   WARNING = 1,
#   ERROR = 2
# };




frequency = 40.0

request_packet_t = "<Ihh16s"
request_payload_noop_t = "<16s"
request_payload_reset_t = "<16s"
request_payload_initialize_t = "<BBBBBB10s"
request_payload_recalibrate_t = "<BBBBBB10s"
request_payload_shutdown_t = "<16s"


class RequestOperation(IntEnum):
    NOOP = 0,
    RESET = 1,
    INITIALIZE = 2,
    RECALIBRATE = 3,
    SHUTDOWN = 4

request_payload_map = {
    0 : request_payload_noop_t,
    1 : request_payload_reset_t,
    2 : request_payload_initialize_t,
    3 : request_payload_recalibrate_t,
    4 : request_payload_shutdown_t
}

data_packet_t = "<Ihh24s"
data_payload_empty_t = "<24s"
data_payload_status_t = "<hBBBBBBBBBBBB10s"
data_payload_data_t = "<ffffff"
data_payload_log_t = "<BB22s"

class DataType(IntEnum):
    EMPTY = 0,
    STATUS = 1,
    DATA = 2,
    LOG = 3

data_payload_map = {
    DataType.EMPTY : data_payload_empty_t,
    DataType.STATUS : data_payload_status_t,
    DataType.DATA : data_payload_data_t,
    DataType.LOG : data_payload_log_t
}

class LogLevel(IntEnum):
    INFO = 0,
    WARN = 1,
    ERROR = 2

log_levels_enabled = [
    # LogLevel.INFO,
    LogLevel.WARN,
    LogLevel.ERROR
]

def data_payload_empty_handler( payload ):
    return None

def data_payload_status_handler( payload ):
    return None

def data_payload_data_handler( payload ):
    return None

def data_payload_log_handler( payload ):
    # print payload
    if len(payload) != 3:
        print "[WARN]\t:: Invalid packet of type 'LOG' received"
        return
    _, level, message = payload
    print "[%s]\t:: %s" % ( LogLevel(level).name, message.rstrip() )

data_payload_handler_map = {
    0 : data_payload_empty_handler,
    1 : data_payload_status_handler,
    2 : data_payload_data_handler,
    3 : data_payload_log_handler
}



bytestr = struct.pack(
    request_packet_t,
    0,
    RequestOperation.INITIALIZE,
    -1,
    struct.pack(
        request_payload_initialize_t,
        1, 0, 1, 1, 0, 0,
        str(bytearray([0x02]*10))
    )
    # str(bytearray([0x02]*10))
)


# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/cu.usbmodem1421',
    baudrate=9600,
    timeout=None
)

# print ser.isOpen()


ser.write( bytestr + '\n' )

while True:
    if ser.inWaiting() > 0:
        str_in = ser.readline()
        # check the size of the packet
        if len(str_in) == struct.calcsize( data_packet_t ):
            seq, type, checksum, payload = struct.unpack( data_packet_t, str_in )
            if type in data_payload_map:
                payload_mask = data_payload_map[type]
                payload_tuple = struct.unpack( payload_mask, payload )
                payload_handler = data_payload_handler_map[type]
                payload_handler( payload_tuple )

    time.sleep( 1.0/frequency )
