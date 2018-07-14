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



DEBUG = True
frequency = 40.0

request_packet_t = "<BIBBB16s"
request_payload_noop_t = "<16s"
request_payload_reset_t = "<16s"
request_payload_initialize_t = "<BBBBBB10s"
request_payload_recalibrate_t = "<BBBBBB10s"
request_payload_shutdown_t = "<16s"



request_packet_t = "<BIBBB6s"
request_payload_initialize_t = "<BBBBBB"




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


data_packet_t = "<BIBBB255s"
data_packet_header_t = "<BIBBB"
data_payload_log_t = "<BB%ds"
data_payload_log_metadata_bytes = 2

class DataType(IntEnum):
    EMPTY = 0,
    STATUS = 1,
    DATA = 2,
    LOG = 3

class LogLevel(IntEnum):
    INFO = 0,
    WARN = 1,
    ERROR = 2

log_levels_enabled = [
    # LogLevel.INFO,
    LogLevel.WARN,
    LogLevel.ERROR
]

def data_payload_empty_handler( header, payload_str ):
    return None

def data_payload_status_handler( header, payload_str ):
    return None

def data_payload_data_handler( header, payload_str ):
    return None

def data_payload_log_handler( header, payload_str ):
    payload_mask = data_payload_log_t % (header[3]-data_payload_log_metadata_bytes)

    payload_tuple = struct.unpack( payload_mask, payload_str )

    if DEBUG:
        print "\tPayload[level=%s(%d), message_len=%d, message='%s'(%dB)]" % (
            LogLevel(payload_tuple[0]).name, payload_tuple[0],
            payload_tuple[1],
            payload_tuple[2],
            len(payload_tuple[2])
        )

    # print payload
    if len(payload_tuple) != 3:
        print "[WARN]\t:: Invalid packet of type 'LOG' received"
        return
    level, message_size, message = payload_tuple
    print "[%s]\t:: %s" % ( LogLevel(level).name, message.rstrip() )

data_payload_handler_map = {
    0 : data_payload_empty_handler,
    1 : data_payload_status_handler,
    2 : data_payload_data_handler,
    3 : data_payload_log_handler
}



bytestr = struct.pack(
    request_packet_t,
    89,
    0,
    RequestOperation.INITIALIZE,
    struct.calcsize( request_payload_initialize_t ),
    101,
    struct.pack(
        request_payload_initialize_t,
        0, 1, 1, 1, 0, 0
    )
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
        # if DEBUG: print "LINE[%s]" % str_in.strip()
        # check the size of the packet
        if len(str_in) >= struct.calcsize( data_packet_header_t ):
            str_in_header = str_in[0 : struct.calcsize(data_packet_header_t)]
            str_in_payload = str_in[struct.calcsize(data_packet_header_t) : ]
            header_tuple = struct.unpack( data_packet_header_t, str_in_header )
            package_type = header_tuple[2]
            if package_type in data_payload_handler_map:
                # packet type recognized
                if DEBUG:
                    print "\nDEBUG(Packet received):"
                    print "\tSize: %d bytes (Header: %d bytes, Payload: %d bytes)" % (
                        len(str_in)-1,
                        len(str_in_header),
                        len(str_in_payload)-1
                    )
                    print "\tHeader[BOS=%d, seq=%d, type=%s(%d), payload_size=%d, EOS=%d]" % (
                        header_tuple[0],
                        header_tuple[1],
                        DataType(header_tuple[2]).name, header_tuple[2],
                        header_tuple[3],
                        header_tuple[4]
                    )
                # get the packet handler function
                payload_handler = data_payload_handler_map[ package_type ]
                payload_handler( header_tuple, str_in_payload.rstrip() )

    time.sleep( 1.0/frequency )
