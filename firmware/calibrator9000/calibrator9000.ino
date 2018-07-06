#include<SPIMemory.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// debug variables
const bool DEBUG = true;

SPIFlash flash(39, &SPI1);

// define constants
const float frequency = 100.0;      // the firmware runs at 100Hz
const float data_frequency = 30.0;  // the firmware spits data at 30Hz
const int adc_bits_resolution = 12;
const unsigned int serial_input_max_num_bytes = 256;
const int knob_calibration_time_secs = 2;
const bool potentiometer_inverted = true;
const int RST_port_id = 7;
const int led_pwm_high = 255;
const int num_knobs = 6;
const int smooth_history_size = 10;
int knob_raw_value[num_knobs];
int knob_raw_history[num_knobs][smooth_history_size];
int knob_raw_history_total[num_knobs];
float knob_value[num_knobs];
int knob_zero_value[num_knobs];
int active_knobs[num_knobs];
int num_active_knobs = 0;

// computable parameters
const int data_publisher_spin_range = ((int) frequency / data_frequency );
const int potentiometer_max_value = ((int) pow(2,adc_bits_resolution));
const float knob_calibration_dead_zone_center = ((float)potentiometer_max_value) / 2.0;    // center of range
const float knob_calibration_dead_zone_extension = 0.1 * ((float)potentiometer_max_value); // 10% of range

// ADC ports used to read the potentiometers
//int knob_adc_port[] = {A0, A1, A2, A3, A4, A5};
int knob_adc_port[] = {A3, A2, A1, A0, A4, A5}; //TO BE REMOVED (breadboard only)

// variable name associated to each knob
char knob_var_name[] = {'x', 'y', 'z', 'r', 'p', 'w'};

// RGB colors for each knob
int knob_color_rgb[][3] = {
  {255,   0,   0},
  {0,   255,   0},
  {0,     0, 255},
  {255, 255,   0},
  {255,   0, 255},
  {0,   255, 255}
};

// PWM port for each RGB channel of each knob
int knob_color_port[][3] = {
  {5,  -1, -1},
  {-1,  10, -1},
  {-1, -1, 12},
  {11, -1, -1},
  {9,  -1,  5},
  {-1,  4,  3}
};

// knob status
enum KNOB_STATUS{
  UNUSED = -2,
  UNCALIBRATED = -1,
  READY = 0
};
enum KNOB_STATUS knob_status[num_knobs];

// device status
enum STATUS{
  UNCONFIGURED = -1,
  FAILURE = -2,
  WORKING = 0,
  CALIBRATING = 1
};
enum STATUS status = UNCONFIGURED;

// led status
enum LED_STATUS{
  OFF = -1,
  FIX = 0,
  BLINK = 1,
  BLINK_FAST = 2
};

// command request
enum COMMAND_REQUEST_OP{  // this has size int16_t
  NOOP = 0,
  RESET = 1,
  INITIALIZE = 2,
  RECALIBRATE = 3,
  SHUTDOWN = 4
};

#pragma pack(push, 1)
typedef struct {
  uint8_t BOS;
  uint32_t seq;
  uint8_t operation;
  uint8_t payload_size;
  uint8_t EOS;
} __attribute__((__packed__)) request_packet_header_t;

typedef struct {
  request_packet_header_t header;
  byte* payload;
} __attribute__((__packed__)) request_packet_t;

typedef struct {
  uint8_t enable_x;
  uint8_t enable_y;
  uint8_t enable_z;
  uint8_t enable_r;
  uint8_t enable_p;
  uint8_t enable_w;
} __attribute__((__packed__)) request_payload_initialize_t;

typedef struct {
  uint8_t recalibrate_x;
  uint8_t recalibrate_y;
  uint8_t recalibrate_z;
  uint8_t recalibrate_r;
  uint8_t recalibrate_p;
  uint8_t recalibrate_w;
} __attribute__((__packed__)) request_payload_recalibrate_t;
#pragma pack(pop)

// data packet
enum DATA_PACKET_TYPE{  // this has size int16_t
  EMPTY = 0,
  STATUS = 1,
  DATA = 2,
  LOG = 3
};

#pragma pack(push, 1)
typedef struct {
  uint8_t BOS;
  uint32_t seq;
  uint8_t type;
  uint8_t payload_size;
  uint8_t EOS;
} __attribute__((__packed__)) data_packet_header_t;

typedef struct {
  data_packet_header_t header;
  byte payload[256];
} __attribute__((__packed__)) data_packet_t;

typedef struct {
  int8_t device_status;
  uint8_t enabled_x;
  uint8_t enabled_y;
  uint8_t enabled_z;
  uint8_t enabled_r;
  uint8_t enabled_p;
  uint8_t enabled_w;
  uint8_t calibrated_x;
  uint8_t calibrated_y;
  uint8_t calibrated_z;
  uint8_t calibrated_r;
  uint8_t calibrated_p;
  uint8_t calibrated_w;
} __attribute__((__packed__)) data_payload_status_t;

typedef struct {
  float axis_x;
  float axis_y;
  float axis_z;
  float axis_r;
  float axis_p;
  float axis_w;
} __attribute__((__packed__)) data_payload_data_t;

typedef struct {
  uint8_t level;
  uint8_t message_size;
  char message[256];
} __attribute__((__packed__)) data_payload_log_t;
#pragma pack(pop)

// BOS and EOS for request and data packets
enum PACKET_SIGNATURE{
  REQUEST_BOS = 89,   // uint8 01 01 10 01
  REQUEST_EOS = 101,  // uint8 01 10 01 01
  DATA_BOS = 154,     // uint8 10 01 10 10
  DATA_EOS = 166      // uint8 10 10 01 10
};

// log levels
enum LOG_LEVEL{  // this has size int16_t
  INFO = 0,
  WARN = 1,
  ERROR = 2
};

// compute knob calibration dead zone limits and timer counter
int knob_calibration_lower_limit = knob_calibration_dead_zone_center - knob_calibration_dead_zone_extension;
int knob_calibration_upper_limit = knob_calibration_dead_zone_center + knob_calibration_dead_zone_extension;
const int knob_calibration_time_steps = ((int)knob_calibration_time_secs*frequency);

// spin counter used for animations
int spin_counter = 0;
int spin_counter_range = 2000;
int smooth_history_idx = 0;

// temporary buffers
char strbuf[250];
char sfloat_buf[10];
char input_line_buf[serial_input_max_num_bytes];
int knob_calibration_timer_counters[num_knobs];
uint32_t packet_header_seq = 0;
int inbound_buf_pos = 0;

// temporary structures
// inbound packet and payloads
request_packet_t inbound_packet;
request_payload_initialize_t inbound_init_payload;
request_payload_recalibrate_t inbound_recalib_payload;
// outbound packet and payloads
data_packet_t outbound_packet;
data_payload_status_t outbound_status_payload;
data_payload_log_t outbound_log_payload;
data_payload_data_t outbound_data_payload;

// setup code, runs once
void setup() {
  // initialize serial port 
  Serial.begin(9600);
  // initialize knobs buffer
  for( int i = 0; i < num_knobs; i++ ){
    knob_value[i] = 0.0;
    knob_raw_value[i] = -1;
    knob_raw_history_total[i] = 0;
    knob_zero_value[i] = -1;
    active_knobs[i] = -1;
    knob_status[i] = UNUSED;
    for( int j = 0; j < smooth_history_size; j++ ){
      knob_raw_history[i][j] = 0;
    }
  }
  // configure ADC
  analogReadResolution(adc_bits_resolution);
  // configure RGB PWM ports
  for( int i = 0; i < num_knobs; i++ ){
    for( int j = 0; j < 3; j++ ){
      int port_id = knob_color_port[i][j];
      if( port_id == -1 ) continue;
      // configure port
      pinMode(port_id, OUTPUT);
    }
  }
  // fill in constant data for outbound packet
  outbound_packet.header.BOS = (uint8_t) DATA_BOS;
  outbound_packet.header.EOS = (uint8_t) DATA_EOS;

  flash.begin(MB(1));
}//setup

// this function stops the firmware and halts the device
void(* shutdown_fcn) (void) = 0;

// put your main code here, to run repeatedly:
void loop() {
  // if somebody is talking to the firmware over USBSerial
  while( Serial.available () > 0 )
    process_incoming_byte( Serial.read() );
    
  // FAILURE
  if( status == FAILURE ){
    return;  
  }
  
  // read knob values
  for( int i = 0; i < num_active_knobs; i++ ){
    // get the ID of the current knob and its port
    int knob_id = active_knobs[i];
    int knob_port = knob_adc_port[knob_id];
    // subtract oldest value
    knob_raw_history_total[knob_id] = knob_raw_history_total[knob_id] - knob_raw_history[knob_id][smooth_history_idx];
    // read from ADC
    knob_raw_history[knob_id][smooth_history_idx] = min(analogRead(knob_port), potentiometer_max_value);
    // add reading to the total
    knob_raw_history_total[knob_id] = knob_raw_history_total[knob_id] + knob_raw_history[knob_id][smooth_history_idx];
    // compute new average
    knob_raw_value[knob_id] = ((int) (float) knob_raw_history_total[knob_id] / (float) smooth_history_size );
    // compute next index
    smooth_history_idx = spin_counter % smooth_history_size;
    if( potentiometer_inverted ){
      knob_raw_value[knob_id] = potentiometer_max_value - knob_raw_value[knob_id];
    }
    // if the knob is calibrated, compute the value in the range [-1,1]
    if( knob_status[knob_id] == READY ){
      float knob_relative_value = ((float)knob_raw_value[knob_id] - knob_zero_value[knob_id]);
      knob_value[knob_id] = knob_relative_value / ((float) max(knob_zero_value[knob_id], potentiometer_max_value-knob_zero_value[knob_id]));
    }
  }
  
  // DEBUG only
  if( DEBUG ){
    debug_fcn();

//    char buf[40];
//    int cap = (int)flash.getCapacity();
//    snprintf( buf, sizeof buf, "%s %d", "Flash", -23 );
//    log_to_serial( INFO, buf );
  }
  
  // CALIBRATION
  if( status == CALIBRATING ){
    // check which knob is not calibrated yet
    for( int i = 0; i < num_active_knobs; i++ ){
      int knob_id = active_knobs[i];
      if( knob_status[knob_id] == READY ) continue;
      if( knob_raw_value[knob_id] < knob_calibration_lower_limit || knob_raw_value[knob_id] > knob_calibration_upper_limit ){
        // found uncalibrated knob       
        knob_calibration_timer_counters[knob_id] = 0;
        knob_status[knob_id] = UNCALIBRATED;
        // blink uncalibrated knobs
        animate_led( knob_id, BLINK );
      }else{
        knob_calibration_timer_counters[knob_id] += 1;
        if( knob_calibration_timer_counters[knob_id] >= knob_calibration_time_steps ){
          knob_calibration_timer_counters[knob_id] = 0;
          knob_zero_value[knob_id] = knob_raw_value[knob_id];
          knob_status[knob_id] = READY;
          animate_led( knob_id, FIX );
        }else{
          animate_led( knob_id, BLINK_FAST );
        }
      }
    }
    // switch state
    int uncalibrated_knobs = 0;
    for( int i = 0; i < num_active_knobs; i++ ){
      int knob_id = active_knobs[i];
      if( knob_status[knob_id] == UNCALIBRATED )
        uncalibrated_knobs += 1;
    }
    if( uncalibrated_knobs == 0 )
      status = WORKING;
  }
  
  // WORKING
  if( status == WORKING ){
    if( spin_counter % data_publisher_spin_range == 0 ){ // it is time to publish the configuration of the axis
      
      // fill in header
      outbound_packet.header.seq = packet_header_seq;
      outbound_packet.header.type = (uint8_t) DATA;
      outbound_packet.header.payload_size = sizeof(outbound_data_payload);
      // fill in payload
      outbound_data_payload.axis_x = knob_value[0];
      outbound_data_payload.axis_y = knob_value[1];
      outbound_data_payload.axis_z = knob_value[2];
      outbound_data_payload.axis_r = knob_value[3];
      outbound_data_payload.axis_p = knob_value[4];
      outbound_data_payload.axis_w = knob_value[5];
      // attach payload to packet
      memcpy( outbound_packet.payload, &outbound_data_payload, sizeof(outbound_data_payload) );
      // publish packet
      send_packet_to_serial( &outbound_packet );
      // increate counter
      packet_header_seq += 1;
    }
  }
  // keep spinning
  delay( 1000.0 / frequency );
  // update spin counter
  spin_counter += 1;
  spin_counter = spin_counter % spin_counter_range;
}//loop

void process_incoming_byte( const byte inByte ){
  switch( inByte ){
    case '\n': // terminator reached!
      input_line_buf[inbound_buf_pos] = 0;  // terminating null byte
      process_request( input_line_buf, inbound_buf_pos );
      // reset buffer for next time
      inbound_buf_pos = 0;
      break;
    case '\r': // discard carriage return
      break;
    default:
      // keep adding if not full
      if( inbound_buf_pos < (serial_input_max_num_bytes-1) )
        input_line_buf[inbound_buf_pos] = inByte;
        inbound_buf_pos += 1;
      break;
  }
}//process_incoming_byte

void process_request( const char* request, int request_size ){
  // check minimum size of the packet (at least the header must be present)
  if( request_size < sizeof(inbound_packet.header) ){
    log_to_serial( WARN, "Received an invalid packet. The size of the whole packet is smaller than the header" );
    return; // discard packet
  }
  // interpret the string of bytes as a request packet
  memcpy(&inbound_packet, request, sizeof(inbound_packet.header));
  // check BOS and EOS codes
  if( inbound_packet.header.BOS != REQUEST_BOS || inbound_packet.header.EOS != REQUEST_EOS ){
    log_to_serial( WARN, "Received a packet with invalid signature" );
    return; // discard packet
  }
  int payload_size = request_size - sizeof(inbound_packet.header);
  // check the size of the payload declared in the header
  if( inbound_packet.header.payload_size != payload_size ){
    log_to_serial( WARN, "The declared payload_size does not match the size of the payload received" );
    return; // discard packet
  }
  // analyze id of requested operation
  switch( inbound_packet.header.operation ){
    case NOOP:
      // nothing to do
      break;
      //
    case RESET:
      request_reset();
      break;
      //
    case INITIALIZE:
      // check size of the payload
      if( payload_size != sizeof(inbound_init_payload) ){
        log_to_serial( WARN, "Received a INITIALIZE request packet but the payload has the wrong size" );
        return; // discard packet
      }
      // the packet and its payload are valid, interpret request
      memcpy(&inbound_init_payload, request+sizeof(inbound_packet.header), payload_size);
      request_initialize( &inbound_init_payload );
      break;
      //
    case RECALIBRATE:
      // check size of the payload
      if( payload_size != sizeof(inbound_recalib_payload) ){
        log_to_serial( WARN, "Received a RECALIBRATE request packet but the payload has the wrong size" );
        return; // discard packet
      }
      // the packet and its payload are valid, interpret request
      memcpy(&inbound_recalib_payload, request+sizeof(inbound_packet.header), payload_size);
      request_recalibrate( &inbound_recalib_payload );
      break;
      //
    case SHUTDOWN:
      request_shutdown();
      break;
  }
}//process_request

void request_reset(){
  if( DEBUG )
    log_to_serial( WARN, "Received a RESET request" );
  reboot_fcn();
}

void request_initialize( request_payload_initialize_t* request ){
  //TODO 
  if( DEBUG )
    log_to_serial( WARN, "Received a INITIALIZE request" );
  // initialize device
  int j = 0;
  uint8_t* enable_req = (uint8_t*)request;
  for( int i = 0; i < num_knobs; i++ ){
    if( enable_req[i] == 1 ){
      active_knobs[j] = i;
      j += 1;
    }
  }
  num_active_knobs = j;
  // set next state
  status = CALIBRATING;
}

void request_recalibrate( request_payload_recalibrate_t* request ){
  //TODO
  if( DEBUG )
    log_to_serial( WARN, "Received a RECALIBRATE request" );
}

void request_shutdown(){
  if( DEBUG )
    log_to_serial( WARN, "Received a SHUTDOWN request" );
  // shutdown the device
  shutdown_fcn();
}

void debug_fcn(){
  if( spin_counter % data_publisher_spin_range != 0 ){
    return; // it is not time to publish debug info
  }
  snprintf(strbuf, sizeof strbuf, "%s", "Axes: ");
  for( int i = 0; i < num_active_knobs; i++ ){
    int knob_id = active_knobs[i];
    if( i > 0 ){
      snprintf(strbuf, sizeof strbuf, "%s%s", strbuf, " | ");
    }
    if( knob_status[knob_id] == UNCALIBRATED ){
      snprintf(
        strbuf, sizeof strbuf, 
        "%s%c%s(%d)", 
        strbuf, knob_var_name[knob_id], "=U", knob_calibration_timer_counters[knob_id] 
      );
    }else{
      snprintf(
        strbuf, sizeof strbuf, 
        "%s%c%s%s", 
        strbuf, knob_var_name[knob_id], "=", sfloat( knob_value[knob_id] )
      );
    }
  }
  log_to_serial( INFO, strbuf );
}//debug_fcn

char* sfloat( float floatValue ){
  snprintf(
    sfloat_buf, sizeof sfloat_buf, 
    "%s%d.%03d", 
    ((floatValue<0)? "-":""), (int)fabs(floatValue), (int)(fabs(floatValue) * 1000.0) % 1000
  );
  return sfloat_buf;
}//sfloat

void animate_led( int led_id, enum LED_STATUS led_status ){
  // OFF
  int pwm_value = 0;
  // FIX
  if( led_status == FIX ){
    pwm_value = led_pwm_high;
  }
  // BLINK
  int blink_range = 0;
  if( led_status == BLINK ){
    blink_range = 200;
  }
  // BLINK_FAST
  if( led_status == BLINK_FAST ){
    blink_range = 50;
  }
  // blink LED   
  if( led_status == BLINK || led_status == BLINK_FAST ){
    int animation_step = spin_counter % blink_range;
    int animation_mid = int( ((float)blink_range) / 2.0f );
    if( animation_step <= animation_mid ){
      pwm_value = map( animation_step, 0, animation_mid, 0, led_pwm_high );
    }else{
      pwm_value = map( animation_step, animation_mid, blink_range, led_pwm_high, 0 );
    }
  }
  // write the signal to the PWM ports
  for( int j = 0; j < 3; j++ ){
    int port_id = knob_color_port[led_id][j];
    if( port_id == -1 ) continue;
    // set PWM port
    analogWrite(port_id, pwm_value);
  }
}//animate_led

void log_to_serial( int level, char* message ){
  uint8_t payload_metadata_size = (uint8_t) sizeof(outbound_log_payload)-1; // remove size of pointer to message
  // fill in header
  outbound_packet.header.seq = packet_header_seq;
  outbound_packet.header.type = (uint8_t) LOG;
  // fill in payload
  outbound_log_payload.level = uint8_t(level);
  outbound_log_payload.message_size = uint8_t( min(strlen(message)+1, 255-payload_metadata_size) );
  // complete header
  outbound_packet.header.payload_size = payload_metadata_size + outbound_log_payload.message_size;
  // copy message into the payload object
  strncpy(outbound_log_payload.message, message, outbound_log_payload.message_size);
  // copy the payload into the packet object
  memcpy(outbound_packet.payload, (const unsigned char*)&outbound_log_payload, outbound_packet.header.payload_size);
  // publish packet
  send_packet_to_serial( &outbound_packet );
}//log_to_serial

void send_packet_to_serial( data_packet_t* packet ){
  // convert packet into sequence of bytes
  int packet_size = sizeof(packet->header) + packet->header.payload_size;
  char* byte_buf = (char*) malloc( packet_size+1 );
  memcpy(byte_buf, (const char*)packet, packet_size);
  byte_buf[ packet_size ] = '\n';
  // publish packet
  Serial.write( byte_buf, packet_size+1 );
  // free memory
  free(byte_buf);
  // increase seq counter
  packet_header_seq++;
}//send_packet_to_serial

void reboot_fcn(){
  // configure RST (RESET) port
  delay(1000);
  pinMode(RST_port_id, OUTPUT);
  // activate RST port
  digitalWrite(RST_port_id, HIGH);
  delay(1000);
}//reboot_fcn
