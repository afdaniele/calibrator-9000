from inspect import isclass
from enum import IntEnum
import struct


class KnobStatus(IntEnum):
    DISABLED = -2
    UNUSED = -1
    UNCALIBRATED = 0
    UNITIALIZED = 1
    READY = 2

class DeviceStatus(IntEnum):
    FAILURE = -3
    UNCONFIGURED = -2
    CALIBRATING = -1
    CALIBRATED = 0
    INITIALIZING = 1
    WORKING = 2

class DataFeed(IntEnum):
    STATUS = 0
    DATA = 1

class DataPacketType(IntEnum):
    EMPTY = 0
    STATUS = 1
    DATA = 2
    LOG = 3

class RequestOperation(IntEnum):
    NOOP = 0
    RESET = 1
    INITIALIZE = 2
    RECALIBRATE = 3
    SHUTDOWN = 4

class PacketSignature(IntEnum):
    REQUEST_BOS = 89    # uint8 01 01 10 01
    REQUEST_EOS = 101   # uint8 01 10 01 01
    DATA_BOS = 154      # uint8 10 01 10 10
    DATA_EOS = 166      # uint8 10 10 01 10

class LogLevel(IntEnum):
    OFF = -1  # this log level is not defined in the firmware
    INFO = 0
    WARN = 1
    ERROR = 2
    HW_DEBUG = 3 # this log level is defined in the firmware as DEBUGGER
    SW_DEBUG = 4 # this log level is not defined in the firmware

class Struct(object):
    _device_endianness = '<' # little-endian
    _type_to_format_map = {
        'uint8_t' : lambda _ : 'B',
        'uint16_t' : lambda _ : 'H',
        'uint32_t' : lambda _ : 'I',
        'int8_t' : lambda _ : 'b',
        'float' : lambda _ : 'f',
        'char[]' : lambda n : '%ds' % n,
        'byte*' : lambda n : '%ds' % n
    }
    _attributes = []

    def __init__(self):
        self._min_required_size = 0
        # iterate over the attributes and create placeholders
        for i in range(len(self._attributes)):
            attr_value = 0 # default value (safe value for attr_size, change with caution)
            attr_type, attr_name, attr_size = self._attributes[i]
            # composite attribute (e.g., header)
            if isclass(attr_type) and issubclass(attr_type,Struct):
                attr_value = attr_type()
            self.__dict__[attr_name] = attr_value
            # compute (minimal) required size
            if( not isinstance(attr_size, basestring) ):
                self._min_required_size += attr_size

    @classmethod
    def get_template(clss, skip_endianness=False):
        return '%s%s' % (
            Struct._device_endianness if not skip_endianness else '',
            ''.join([
                t.get_template(skip_endianness=True) if isclass(t) and issubclass(t,Struct)
                else Struct._type_to_format_map[t](s)
                for t,_,s in clss._attributes
            ])
        )

    @classmethod
    def get_template_size(clss):
        return struct.calcsize( clss.get_template() )

    def get_size(self):
        size = 0
        # iterate over the attributes
        for attr_type, attr_name, attr_size in self._attributes:
            attr_value = self.__dict__[attr_name]
            if( isinstance(attr_value, Struct) ):
                size += attr_value.get_size()
            else:
                if isinstance(attr_size, basestring) and attr_type == 'char[]':
                    size += len( str(attr_value) )
                else:
                    size += attr_size
        return size

    def decode(self, data):
        # check size requirements
        if len(data) < self._min_required_size:
            return (False, -1)
        cursor_in = 0
        # iterate over the attributes
        for attr_type, attr_name, attr_size in self._attributes:
            # resolve attr_size based on other parts of the packet
            if( isinstance(attr_size,basestring) ):
                attr_size_tup = attr_size.split('.')
                cur_el = self
                for el in attr_size_tup:
                    cur_el = cur_el.__dict__[el]
                attr_size = cur_el
            # handle composite vs. primitive data type
            if isclass(attr_type) and issubclass(attr_type, Struct):
                # composite attribute (e.g., header)
                attr_data = data[cursor_in:]
                success, size = self.__dict__[attr_name].decode( attr_data )
                if not success: return (False, -1)
                cursor_fin = cursor_in + size
            else:
                # primitive data type (byte array included)
                cursor_fin = cursor_in + attr_size
                attr_data = data[cursor_in:cursor_fin]
                if attr_type == 'byte*':
                    # byte array
                    attr_class = self._bytearr_to_class( attr_name )
                    if attr_class is None: return (False, -1)
                    self.__dict__[attr_name] = attr_class()
                    self.__dict__[attr_name].decode( attr_data )
                else:
                    # primitive data type (byte array excluded)
                    attr_fmt = '%s%s' % (Struct._device_endianness, Struct._type_to_format_map[attr_type](attr_size))
                    try:
                        self.__dict__[attr_name] = struct.unpack( attr_fmt, attr_data )[0]
                    except:
                        return (False, -1)
            # move cursor to next unused byte
            cursor_in = cursor_fin
        success = self._is_valid()
        return ( success, cursor_in )

    def encode(self):
        self._prepare()
        bytestr = ''
        # iterate over the attributes
        for attr_type, attr_name, attr_size in self._attributes:
            attr_value = self.__dict__[attr_name]
            if( isinstance(attr_value, Struct) ):
                bytestr += attr_value.encode()
            else:
                if attr_type == 'byte*': continue
                length = 0
                if isinstance(attr_size, basestring) and attr_type == 'char[]':
                    length = len(attr_value)
                bytestr += struct.pack(
                    '%s%s' % (
                        Struct._device_endianness,
                        Struct._type_to_format_map[attr_type](length)
                    ),
                    attr_value
                )
        return bytestr

    def _is_valid(self):
        raise NotImplementedError( "The method is not implemented" )

    def _prepare(self):
        raise NotImplementedError( "The method is not implemented" )

    def _bytearr_to_class(self, attr_name, attr_data):
        raise NotImplementedError( "The method is not implemented" )


# static payload structure, the payload is valid by default, nothing to prepare
class StaticPayloadStruct(Struct):
    def _is_valid(self):
        return True

    def _prepare(self):
        return


# struct {
#   uint8_t BOS;
#   uint32_t seq;
#   uint8_t operation;
#   uint8_t payload_size;
#   uint8_t EOS;
# } request_packet_header_t
class request_packet_header_t(Struct):
    _attributes = [
        ('uint8_t', 'BOS', 1),
        ('uint32_t', 'seq', 4),
        ('uint8_t', 'operation', 1),
        ('uint8_t', 'payload_size', 1),
        ('uint8_t', 'EOS', 1)
    ]

    def _is_valid(self):
        return (self.BOS == PacketSignature.REQUEST_BOS
        and self.EOS == PacketSignature.REQUEST_EOS
        and self.operation in RequestOperation._value2member_map_)

    def _prepare(self):
        self.BOS = PacketSignature.REQUEST_BOS.value
        if isinstance(self.operation, RequestOperation):
            self.operation = self.operation.value
        self.EOS = PacketSignature.REQUEST_EOS.value


# struct {} request_payload_empty_t;
class request_payload_empty_t(StaticPayloadStruct):
    _attributes = []


# struct {
#   uint8_t enable_x;
#   uint8_t enable_y;
#   uint8_t enable_z;
#   uint8_t enable_r;
#   uint8_t enable_p;
#   uint8_t enable_w;
# } request_payload_initialize_t;
class request_payload_initialize_t(StaticPayloadStruct):
    _attributes = [
        ('uint8_t', 'enable_x', 1),
        ('uint8_t', 'enable_y', 1),
        ('uint8_t', 'enable_z', 1),
        ('uint8_t', 'enable_r', 1),
        ('uint8_t', 'enable_p', 1),
        ('uint8_t', 'enable_w', 1)
    ]


# struct {
#   uint8_t recalibrate_x;
#   uint8_t recalibrate_y;
#   uint8_t recalibrate_z;
#   uint8_t recalibrate_r;
#   uint8_t recalibrate_p;
#   uint8_t recalibrate_w;
# } request_payload_recalibrate_t;
class request_payload_recalibrate_t(StaticPayloadStruct):
    _attributes = [
        ('uint8_t', 'recalibrate_x', 1),
        ('uint8_t', 'recalibrate_y', 1),
        ('uint8_t', 'recalibrate_z', 1),
        ('uint8_t', 'recalibrate_r', 1),
        ('uint8_t', 'recalibrate_p', 1),
        ('uint8_t', 'recalibrate_w', 1)
    ]


# struct {
#   request_packet_header_t header;
#   byte* payload;
# } request_packet_t
class request_packet_t(Struct):
    _attributes = [
        (request_packet_header_t, 'header', request_packet_header_t.get_template_size()),
        ('byte*', 'payload', 'header.payload_size')
    ]

    _payload_bytearr_to_class_map = {
        RequestOperation.NOOP : request_payload_empty_t,
        RequestOperation.RESET : request_payload_empty_t,
        RequestOperation.INITIALIZE : request_payload_initialize_t,
        RequestOperation.RECALIBRATE : request_payload_recalibrate_t,
        RequestOperation.SHUTDOWN : request_payload_empty_t
    }

    def __init__(self):
        super(request_packet_t, self).__init__()
        self.header._prepare()
        self.__dict__['payload'] = request_payload_empty_t()

    def _is_valid(self):
        return self.header._is_valid()

    def _bytearr_to_class(self, attr_name):
        if attr_name == 'payload':
            if self.header.operation in RequestOperation._value2member_map_:
                enum_value = RequestOperation(self.header.operation)
                return request_packet_t._payload_bytearr_to_class_map[enum_value]
            return None

    def _prepare(self):
        if isinstance(self.payload, Struct):
            self.payload._prepare()
            self.header.payload_size = self.payload.get_size()


# struct {
#   uint8_t BOS;
#   uint32_t seq;
#   uint8_t type;
#   uint8_t payload_size;
#   uint8_t EOS;
# } data_packet_header_t
class data_packet_header_t(Struct):
    _attributes = [
        ('uint8_t', 'BOS', 1),
        ('uint32_t', 'seq', 4),
        ('uint8_t', 'type', 1),
        ('uint8_t', 'payload_size', 1),
        ('uint8_t', 'EOS', 1)
    ]

    def _is_valid(self):
        return (self.BOS == PacketSignature.DATA_BOS
        and self.EOS == PacketSignature.DATA_EOS
        and self.type in DataPacketType._value2member_map_)

    def _prepare(self):
        self.BOS = PacketSignature.DATA_BOS.value
        if isinstance(self.type, DataPacketType):
            self.type = self.type.value
        self.EOS = PacketSignature.DATA_EOS.value


# struct {} data_payload_empty_t;
class data_payload_empty_t(StaticPayloadStruct):
    _attributes = []


# struct {
#   int8_t device_status;
#   int8_t status_x;
#   int8_t status_y;
#   int8_t status_z;
#   int8_t status_r;
#   int8_t status_p;
#   int8_t status_w;
# } data_payload_status_t;
class data_payload_status_t(StaticPayloadStruct):
    _attributes = [
        ('int8_t', 'device_status', 1),
        ('int8_t', 'status_x', 1),
        ('int8_t', 'status_y', 1),
        ('int8_t', 'status_z', 1),
        ('int8_t', 'status_r', 1),
        ('int8_t', 'status_p', 1),
        ('int8_t', 'status_w', 1)
    ]

# struct {
#   float axis_x;
#   float axis_y;
#   float axis_z;
#   float axis_r;
#   float axis_p;
#   float axis_w;
# } data_payload_data_t;
class data_payload_data_t(StaticPayloadStruct):
    _attributes = [
        ('float', 'axis_x', 4),
        ('float', 'axis_y', 4),
        ('float', 'axis_z', 4),
        ('float', 'axis_r', 4),
        ('float', 'axis_p', 4),
        ('float', 'axis_w', 4)
    ]


# struct {
#   uint8_t level;
#   uint8_t message_size;
#   char message[256];
# } data_payload_log_t;
class data_payload_log_t(Struct):
    _attributes = [
        ('uint8_t', 'level', 1),
        ('uint8_t', 'message_size', 1),
        ('char[]', 'message', 'message_size')
    ]

    def _is_valid(self):
        return self.level in LogLevel._value2member_map_

    def _prepare(self):
        self.message_size = len(self.message)


# struct {
#   data_packet_header_t header;
#   byte* payload;
# } data_packet_t
class data_packet_t(Struct):
    _attributes = [
        (data_packet_header_t, 'header', data_packet_header_t.get_template_size()),
        ('byte*', 'payload', 'header.payload_size')
    ]

    _payload_bytearr_to_class_map = {
        DataPacketType.EMPTY : data_payload_empty_t,
        DataPacketType.STATUS : data_payload_status_t,
        DataPacketType.DATA : data_payload_data_t,
        DataPacketType.LOG : data_payload_log_t
    }

    def __init__(self):
        super(data_packet_t, self).__init__()
        self.header._prepare()
        self.__dict__['payload'] = data_payload_empty_t()

    def _is_valid(self):
        return self.header._is_valid()

    def _bytearr_to_class(self, attr_name):
        if attr_name == 'payload':
            if self.header.type in DataPacketType._value2member_map_:
                enum_value = DataPacketType(self.header.type)
                return data_packet_t._payload_bytearr_to_class_map[enum_value]
            return None

    def _prepare(self):
        if isinstance(self.payload, Struct):
            self.payload._prepare()
            self.header.payload_size = self.payload.get_size()
