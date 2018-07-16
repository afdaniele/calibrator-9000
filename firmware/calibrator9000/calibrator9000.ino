#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_SPIFlash_FatFs.h>

// configuration of the flash chip pins
#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV  // Flash chip type.
#define FLASH_SS       SS1                    // Flash chip SS pin.
#define FLASH_SPI_PORT SPI1                   // SPI port the Flash is connected to
#define NEOPIN         40

// create SPI Flash memory
Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);     // Use hardware SPI
Adafruit_W25Q16BV_FatFs fatfs(flash);

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// debug variables
const bool _DEBUG = true;
const bool _FORCE_CALIBRATION = false;

// define constants
const float frequency = 100.0;         // the firmware runs at 100Hz
const float data_frequency = 30.0;     // the firmware spits data at 30Hz
const float status_frequency = 5.0;    // the firmware spits status info at 5Hz
const float log_frequency = 2.0;       // the firmware spits logs at 2Hz
const int adc_bits_resolution = 12;
const unsigned int serial_input_max_num_bytes = 256;
const int knob_init_time_secs = 2;
const int knob_calib_time_secs = 2;
const bool potentiometer_inverted = true;
const int RST_port_id = 7;
const int led_pwm_high = 255;
const int num_knobs = 6;
const int smooth_history_size = 10;
int knob_raw_value[num_knobs];
int knob_raw_history[num_knobs][smooth_history_size];
int knob_raw_history_total[num_knobs];
int knob_zero_value[num_knobs];
float knob_value[num_knobs];
int active_knobs[num_knobs];
int num_active_knobs = 0;

// computable parameters
const int data_publisher_spin_range = ((int) frequency / data_frequency );
const int status_publisher_spin_range = ((int) frequency / status_frequency );
const int log_publisher_spin_range = ((int) frequency / log_frequency );
const int potentiometer_max_value = ((int) pow(2,adc_bits_resolution));
const float knob_init_dead_zone_center = ((float)potentiometer_max_value) / 2.0;      // center of range
const float knob_init_dead_zone_extension = 0.1 * ((float)potentiometer_max_value);   // 10% of the range
const float knob_calib_zone_extension = 0.2 * ((float)potentiometer_max_value);       // 20% of the range

// ADC ports used to read the potentiometers
//int knob_adc_port[] = {A0, A1, A2, A3, A4, A5};
int knob_adc_port[] = {A3, A2, A1, A0, A4, A5}; //DEBUG only: TO BE REMOVED (breadboard only)

// variable name associated to each knob
char knob_var_name[] = {'x', 'y', 'z', 'r', 'p', 'w'};

// PWM port for each RGB channel of each knob
int knob_led_port[] = {
  5,
  10,
  12,
  11,
  9,
  4
};

// knob status
enum KNOB_STATUS{
  DISABLED = -2,
  UNUSED = -1,
  UNCALIBRATED = 0,
  UNITIALIZED = 1,
  READY = 2
};
enum KNOB_STATUS knob_status[num_knobs];

// device status
enum STATUS{
  FAILURE = -3,
  UNCONFIGURED = -2,
  CALIBRATING = -1,
  CALIBRATED = 0,
  INITIALIZING = 1,
  WORKING = 2,
};
enum STATUS status = UNCONFIGURED;

// led status
enum LED_STATUS{
  OFF = -1,
  FIX = 0,
  BLINK = 1,
  BLINK_FAST = 2,
  BLINK_CALIB_LOWER_BOUND = 3,
  BLINK_CALIB_UPPER_BOUND = 4
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
  int8_t status_x;
  int8_t status_y;
  int8_t status_z;
  int8_t status_r;
  int8_t status_p;
  int8_t status_w;
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
  ERROR = 2,
  DEBUGGER = 3
};


#pragma pack(push, 1)
// calibration struct
typedef struct {
  uint16_t min_val[num_knobs];
  uint16_t max_val[num_knobs];
} __attribute__((__packed__)) calibration_data_t;
#pragma pack(pop)

// compute knob calibration dead zone limits and timer counter
int knob_init_lower_limit = knob_init_dead_zone_center - knob_init_dead_zone_extension;
int knob_init_upper_limit = knob_init_dead_zone_center + knob_init_dead_zone_extension;
const int knob_init_time_steps = ((int)knob_init_time_secs*frequency);
const int knob_calib_time_steps = ((int)knob_calib_time_secs*frequency);

// spin counter used for animations
int spin_counter = 0;
int spin_counter_range = 2000;
int smooth_history_idx = 0;

// temporary buffers
char strbuf[250];
char sfloat_buf[10];
char last_error_buf[250];
char input_line_buf[serial_input_max_num_bytes];
int knob_init_timer_counters[num_knobs];
int knob_calib_timer_counters[num_knobs];
uint32_t packet_header_seq = 0;
int inbound_buf_pos = 0;
File calibFile;
char* calibFile_path = "/calibration.dat";

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
// device calibration
calibration_data_t device_calib;

// device STATUS to string
const String get_device_status_name(int16_t device_status){
   switch( device_status ){
      case FAILURE: return "FAILURE";
      case UNCONFIGURED: return "UNCONFIGURED";
      case CALIBRATING: return "CALIBRATING";
      case CALIBRATED: return "CALIBRATED";
      case INITIALIZING: return "INITIALIZING";
      case WORKING: return "WORKING";
      default: return "UNKNOWN";
   }
}//get_device_status_name

// KNOB_STATUS to string
const char* get_knob_status_name(int16_t knob_status){
   switch( knob_status ){
      case DISABLED: return "DISABLED";
      case UNUSED: return "UNUSED";
      case UNCALIBRATED: return "UNCALIBRATED";
      case UNITIALIZED: return "UNITIALIZED";
      case READY: return "READY";
      default: return "UNKNOWN";
   }
}//get_knob_status_name

// setup code, runs once
void setup() {
  // initialize serial port 
  Serial.begin(9600);
  // initialize knobs buffer
  for( int i = 0; i < num_knobs; i++ ){
    knob_value[i] = 0.0;
    knob_raw_value[i] = -1;
    knob_raw_history_total[i] = 0;
    knob_init_timer_counters[i] = 0;
    knob_zero_value[i] = -1;
    active_knobs[i] = -1;
    knob_status[i] = UNUSED;
    for( int j = 0; j < smooth_history_size; j++ ){
      knob_raw_history[i][j] = 0;
    }
  }
  // reset calibration
  reset_calibration();

  //DEBUG only (the breadboard has only 4 knobs)
  knob_status[4] = DISABLED;
  knob_status[5] = DISABLED;
  //DEBUG only (the breadboard has only 4 knobs)

  // configure ADC
  analogReadResolution(adc_bits_resolution);
  // configure RGB PWM ports
  for( int i = 0; i < num_knobs; i++ ){
    int port_id = knob_led_port[i];
    // configure port
    pinMode(port_id, OUTPUT);
  }
  // fill in constant data for outbound packet
  outbound_packet.header.BOS = (uint8_t) DATA_BOS;
  outbound_packet.header.EOS = (uint8_t) DATA_EOS;
  // initialize SPI Flash chip
  if( !flash.begin(FLASH_TYPE) ){
    // switch to FAILURE mode
    log_to_serial(ERROR, "Error, failed to initialize flash chip!");
    change_device_status(FAILURE);
    return;
  }
  log_to_serial(INFO, "Flash chip initialized!");
  // mount the FAT filesystem
  if( !fatfs.begin() ){
    // switch to FAILURE mode
    log_to_serial(ERROR, "Error, failed to mount filesystem!");
    change_device_status(FAILURE);
    return;
  }
  log_to_serial(INFO, "FAT filesystem mounted successfully!");

  // calibration
  if( _FORCE_CALIBRATION ){
    log_to_serial(WARN, "Override: _FORCE_CALIBRATION is True.");
    // reset calibration
    reset_calibration();
    // delete calibration file (if it exists)
    if( fatfs.exists(calibFile_path) ){
      fatfs.remove(calibFile_path);
      log_to_serial(WARN, "Calibration file deleted.");  
    }
    // switch to CALIBRATION mode
    change_device_status(CALIBRATING);
  }else{
    // check if the calibration file exists
    if( !fatfs.exists(calibFile_path) ){
      log_to_serial(WARN, "No calibration file found, recalibrating now!");
      // reset calibration
      reset_calibration();
      // switch to CALIBRATION mode
      change_device_status(CALIBRATING);
    }else{
      log_to_serial(INFO, "Calibration file found, accessing now!");
      calibFile = fatfs.open(calibFile_path, FILE_READ);
      if (!calibFile) {
        // switch to FAILURE mode
        log_to_serial(ERROR, "Error, failed to open calibration file for reading!");
        change_device_status(FAILURE);
        return;
      }
      log_to_serial(INFO, "Opened calibration file, reading...");
      // read calibration struct
      calibFile.read( &device_calib, sizeof(device_calib) );
      calibFile.close();
      log_to_serial(INFO, "Calibration loaded!");
      // switch to CALIBRATED mode
      change_device_status(CALIBRATED);
    }
  }
  if( status == CALIBRATING ){
    log_to_serial( INFO, "Device uncalibrated! Waiting for the knobs to be calibrated" );
    for( int i = 0; i < num_knobs; i++ ){
      if( knob_status[i] != DISABLED )
        knob_status[i] = UNCALIBRATED;
    }
  }
}//setup

// this function stops the firmware and halts the device
void(* shutdown_fcn) (void) = 0;

// put your main code here, to run repeatedly:
void loop() {
  // if somebody is talking to the firmware over USBSerial
  while( Serial.available() > 0 )
    process_incoming_byte( Serial.read() );
    
  // STATUS packets
  if( spin_counter % status_publisher_spin_range == 0 ){
    // fill in header
    outbound_packet.header.seq = (uint32_t) packet_header_seq;
    outbound_packet.header.type = (uint8_t) STATUS;
    outbound_packet.header.payload_size = (uint8_t) sizeof(outbound_status_payload);
    // fill in payload
    outbound_status_payload.device_status = (int8_t) status;
    for( int i=0; i < num_knobs; i++ ){
      ((int8_t*)&outbound_status_payload)[i+1] = (int8_t) knob_status[i];
    }
    // attach payload to packet
    memcpy( outbound_packet.payload, &outbound_status_payload, sizeof(outbound_status_payload) );
    // publish packet
    send_packet_to_serial( &outbound_packet );
  }
    
  // FAILURE mode
  if( status == FAILURE ){
    if( spin_counter % log_publisher_spin_range == 0 ){
      // TODO: maybe also blink the red knob?
      log_to_serial( ERROR, last_error_buf );
      return;
    }
  }
  
  // read knob values
  for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
    // get the port of the current knob
    int knob_port = knob_adc_port[knob_id];
    // subtract oldest value
    knob_raw_history_total[knob_id] = knob_raw_history_total[knob_id] - knob_raw_history[knob_id][smooth_history_idx];
    // read from ADC
    knob_raw_history[knob_id][smooth_history_idx] = min(analogRead(knob_port), potentiometer_max_value);
    if( potentiometer_inverted ){
      knob_raw_history[knob_id][smooth_history_idx] = potentiometer_max_value - knob_raw_history[knob_id][smooth_history_idx];
    }
    // add reading to the total
    knob_raw_history_total[knob_id] = knob_raw_history_total[knob_id] + knob_raw_history[knob_id][smooth_history_idx];
    // compute new average
    knob_raw_value[knob_id] = ((int) (float) knob_raw_history_total[knob_id] / (float) smooth_history_size );
    // compute next index
    smooth_history_idx = spin_counter % smooth_history_size;
    // if the knob is initialized, compute the value in the range [-1,1]
    if( knob_status[knob_id] == READY ){
      float knob_raw_calibrated_value = max( (float)device_calib.min_val[knob_id], min( (float)knob_raw_value[knob_id], (float)device_calib.max_val[knob_id] ) );
      float knob_relative_value = float(knob_raw_calibrated_value - knob_zero_value[knob_id]);
      float range;
      if( knob_relative_value < 0 ){ // negative value
        range = float(knob_zero_value[knob_id]) - (float)device_calib.min_val[knob_id];
      }else{ // positive value
        range = (float)device_calib.max_val[knob_id] - float(knob_zero_value[knob_id]);
      }
      knob_value[knob_id] = knob_relative_value / range; // ((float) max(knob_zero_value[knob_id], potentiometer_max_value-knob_zero_value[knob_id]));
    }
  }
  
  // DEBUG
  debug_fcn();

  // CALIBRATION
  if( status == CALIBRATING ){
    // check which knob is not calibrated yet
    for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
      if( knob_status[knob_id] != UNCALIBRATED ) continue;
      // get min and max for the current knob in the current history
      int knob_raw_min_in_history = min_in_array( knob_raw_history[knob_id], smooth_history_size );
      int knob_raw_max_in_history = max_in_array( knob_raw_history[knob_id], smooth_history_size );
      // MIN/MAX calibration
      if( device_calib.min_val[knob_id] != potentiometer_max_value ){
        // lower bound calibrated, now calibrate upper bound
        if( knob_raw_value[knob_id] < potentiometer_max_value - knob_calib_zone_extension ){
          // found uncalibrated upper bound
          knob_calib_timer_counters[knob_id] = 0;
          // blink uncalibrated knob
          animate_led( knob_id, BLINK_CALIB_UPPER_BOUND );
        }else{
          knob_calib_timer_counters[knob_id] += 1;
          if( knob_calib_timer_counters[knob_id] >= knob_calib_time_steps ){
            knob_calib_timer_counters[knob_id] = 0;
            device_calib.max_val[knob_id] = uint16_t( knob_raw_min_in_history );
            knob_status[knob_id] = UNUSED;
            animate_led( knob_id, OFF );  // show that the lower bound was calibrated
          }else{
            animate_led( knob_id, FIX ); // show that the knob is being calibrated
          }
        }
      }else{
        // calibrating lower bound
        if( knob_raw_value[knob_id] > knob_calib_zone_extension ){
          // found uncalibrated lower bound
          knob_calib_timer_counters[knob_id] = 0;
          // blink uncalibrated knob
          animate_led( knob_id, BLINK_CALIB_LOWER_BOUND );
        }else{
          knob_calib_timer_counters[knob_id] += 1;
          if( knob_calib_timer_counters[knob_id] >= knob_calib_time_steps ){
            knob_calib_timer_counters[knob_id] = 0;
            device_calib.min_val[knob_id] = uint16_t( knob_raw_max_in_history );
            animate_led( knob_id, OFF );  // show that the lower bound was calibrated
          }else{
            animate_led( knob_id, FIX ); // show that the knob is being calibrated
          }
        } 
      }
    }
    // save calibration to file
    int uncalibrated_knobs = 0;
    for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
      if( knob_status[knob_id] == UNCALIBRATED )
        uncalibrated_knobs += 1;
    }
    if( uncalibrated_knobs == 0 ){
      // store calibration
      calibFile = fatfs.open(calibFile_path, FILE_WRITE);
      if (!calibFile) {
        log_to_serial(ERROR, "Error, failed to open calibration file for writing!");
        change_device_status(FAILURE);
        return;
      }
      log_to_serial(INFO, "Opened calibration file, writing...");
      calibFile.seek( 0 );
      // write struct to disk
      int calib_size = sizeof(device_calib);
      // dump bytes
      calibFile.write( (const byte*)&device_calib, calib_size );
      calibFile.flush();
      calibFile.close();
      log_to_serial(INFO, "Calibration file wrote!");
      // switch state to CALIBRATED
      change_device_status(CALIBRATED);
    }
  }
  
  // INITIALIZATION
  if( status == INITIALIZING ){
    // check which knob is not initialized yet
    for( int i = 0; i < num_active_knobs; i++ ){
      int knob_id = active_knobs[i];
      if( knob_status[knob_id] == READY ) continue;
      if( knob_raw_value[knob_id] < knob_init_lower_limit || knob_raw_value[knob_id] > knob_init_upper_limit ){
        // found uninitialized knob       
        knob_init_timer_counters[knob_id] = 0;
        knob_status[knob_id] = UNITIALIZED;
        // blink uninitialized knobs
        animate_led( knob_id, BLINK );
      }else{
        knob_init_timer_counters[knob_id] += 1;
        if( knob_init_timer_counters[knob_id] >= knob_init_time_steps ){
          knob_init_timer_counters[knob_id] = 0;
          knob_zero_value[knob_id] = knob_raw_value[knob_id];
          knob_status[knob_id] = READY;
          animate_led( knob_id, FIX );
        }else{
          animate_led( knob_id, BLINK_FAST );
        }
      }
    }
    // switch state
    int uninitialized_knobs = 0;
    for( int i = 0; i < num_active_knobs; i++ ){
      int knob_id = active_knobs[i];
      if( knob_status[knob_id] == UNITIALIZED )
        uninitialized_knobs += 1;
    }
    if( uninitialized_knobs == 0 ){
      change_device_status(WORKING);
    }
  }
  
  // WORKING
  if( status == WORKING ){
    if( spin_counter % data_publisher_spin_range == 0 ){ // it is time to publish the configuration of the axis
      // fill in header
      outbound_packet.header.seq = (uint32_t) packet_header_seq;
      outbound_packet.header.type = (uint8_t) DATA;
      outbound_packet.header.payload_size = (uint8_t) sizeof(outbound_data_payload);
      // fill in payload
      for( int i=0; i < num_knobs; i++ ){
        ((float*)&outbound_data_payload)[i] = knob_value[i];
      }
      // attach payload to packet
      memcpy( outbound_packet.payload, &outbound_data_payload, sizeof(outbound_data_payload) );
      // publish packet
      send_packet_to_serial( &outbound_packet );
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
  log_to_serial( DEBUGGER, "Received a RESET request" );
  reboot_fcn();
}//request_reset

void request_initialize( request_payload_initialize_t* request ){
  log_to_serial( DEBUGGER, "Received a INITIALIZE request" );
  // initialize device
  bool update_active_knobs = false;
  bool update_status = false;
  enum STATUS next_status = status;
  switch( status ){
    case FAILURE:       // device in failure mode, it is not possible to initialize it
    case UNCONFIGURED:  // this should never happen, the device should have been configured in setup()
    case CALIBRATING:   // the device is still calibrating, it is not possible to initialize it
      // nothing to do
      break;
    case CALIBRATED:
      // the device is fully calibrated, it is possible to activate other knobs
      update_active_knobs = true;
      update_status = true;
      next_status = INITIALIZING;
      break;
    case INITIALIZING:
      // device already initialized but the initialization procedure is incomplete, it is possible to activate other knobs
      update_active_knobs = true;
      break;
    case WORKING:
      // device already initialized and working, it is not possible to activate other knobs; nothing to do
      break;
  }
  // update list of active knobs
  if( update_active_knobs ){
    // initialize knobs
    int j = 0;
    uint8_t* enable_req = (uint8_t*)request;
    for( int i = 0; i < num_knobs; i++ ){
      if( enable_req[i] == 1 ){
        active_knobs[j] = i;
        knob_status[i] = UNITIALIZED;
        j += 1;
      }else{
        knob_status[i] = UNUSED;  
      }
    }
    num_active_knobs = j;
  }
  // update device status
  if( update_status ){
    change_device_status(next_status);
  }
}//request_initialize

void request_recalibrate( request_payload_recalibrate_t* request ){
  log_to_serial( DEBUGGER, "Received a RECALIBRATE request" );
  if( status >= INITIALIZING ){
    log_to_serial( WARN, "Device already initialized, it is not possible to calibrate the knobs now." );
    return;
  }
  // set all (indicated) knobs as uncalibrated
  int num_recalib_knobs = 0;
  uint8_t* recalib_req = (uint8_t*)request;
  for( int i = 0; i < num_knobs; i++ ){
    if( recalib_req[i] == 1 ){
      knob_status[i] = UNCALIBRATED;
      reset_knob_calibration( i );
      num_recalib_knobs += 1;
    }
  }
  if( num_recalib_knobs > 0 )
    change_device_status( CALIBRATING );
}//request_recalibrate

void change_device_status( enum STATUS new_status ){
  log_to_serial(WARN, String("Status change: " + get_device_status_name(status) + " -> " + get_device_status_name(new_status)).c_str() );
  status = new_status;
}//change_device_status

void request_shutdown(){
  log_to_serial( DEBUGGER, "Received a SHUTDOWN request" );
  // shutdown the device
  shutdown_fcn();
}//request_shutdown

void debug_fcn(){
  if( !_DEBUG ) return; // debug disabled
  if( spin_counter % log_publisher_spin_range != 0 ) return; // it is not time to publish debug info
  // 1. send a packet with the status of the device and the status of each axis
  snprintf( strbuf, sizeof strbuf, "STATUS: Device(%s); Axes: ", get_device_status_name(status).c_str() );
  for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
    if( knob_id > 0 ){
      snprintf( strbuf, sizeof strbuf, "%s%s", strbuf, " | " );
    }
    snprintf(
      strbuf, sizeof strbuf,
      "%s%c(%s)", 
      strbuf, knob_var_name[knob_id], get_knob_status_name( knob_status[knob_id] )
    );
  }
  log_to_serial( DEBUGGER, strbuf );
  // 2. send a packet with the configuration of the axis
  snprintf(strbuf, sizeof strbuf, "%s", "AXES: ");
  for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
    if( knob_id > 0 ){
      snprintf(strbuf, sizeof strbuf, "%s%s", strbuf, " | ");
    }
    if( knob_status[knob_id] == UNITIALIZED || knob_status[knob_id] == UNCALIBRATED ){
      int knob_counter = (knob_status[knob_id] == UNITIALIZED)? knob_init_timer_counters[knob_id] : knob_calib_timer_counters[knob_id];
      snprintf(
        strbuf, sizeof strbuf,
        "%s%c%s(%d):%d",
        strbuf, knob_var_name[knob_id], "=U", knob_raw_value[knob_id], knob_counter
      );
    }else{
      snprintf(
        strbuf, sizeof strbuf, 
        "%s%c%s%s", 
        strbuf, knob_var_name[knob_id], "=", sfloat( knob_value[knob_id] )
      );
    }
  }
  log_to_serial( DEBUGGER, strbuf );
  // 3. send a packet with the axis calibration
  snprintf( strbuf, sizeof strbuf, "%s", "CALIBRATION: Axes: " );
  for( int knob_id = 0; knob_id < num_knobs; knob_id++ ){
    if( knob_id > 0 ){
      snprintf( strbuf, sizeof strbuf, "%s%s", strbuf, " | " );
    }
    snprintf(
      strbuf, sizeof strbuf,
      "%s%c(%d-%d)",
      strbuf, knob_var_name[knob_id], (int)device_calib.min_val[knob_id], (int)device_calib.max_val[knob_id]
    );
  }
  log_to_serial( DEBUGGER, strbuf );
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
  // blink LED
  if( led_status == BLINK || led_status == BLINK_FAST ){
    int blink_range = (led_status == BLINK_FAST)? 50 : 200;
    int animation_step = spin_counter % blink_range;
    int animation_mid = int( ((float)blink_range) / 2.0f );
    if( animation_step <= animation_mid ){
      pwm_value = map( animation_step, 0, animation_mid, 0, led_pwm_high );
    }else{
      pwm_value = map( animation_step, animation_mid, blink_range, led_pwm_high, 0 );
    }
  }
  // blink LED in calibration mode (lower bound)
  if( led_status == BLINK_CALIB_LOWER_BOUND ){
    int animation_step = spin_counter % 200;
    if( animation_step < 30 )
      pwm_value = led_pwm_high;
  }
  // blink LED in calibration mode (upper bound)
  if( led_status == BLINK_CALIB_UPPER_BOUND ){
    int animation_step = spin_counter % 200;
    if( animation_step < 30 || (animation_step > 50 && animation_step < 80) )
      pwm_value = led_pwm_high;
  }
  // write the signal to the PWM port
  int port_id = knob_led_port[led_id];
  // set PWM port
  analogWrite(port_id, pwm_value);
}//animate_led

void reset_calibration(){
  // reset calibration for all knobs
  for( int i = 0; i < num_knobs; i++ ){
    reset_knob_calibration( i );
  }
}//reset_calibration

void reset_knob_calibration( int knob_id ){
  uint16_t* calib_values = (uint16_t*) &device_calib;
  // reset knob calibration
  calib_values[knob_id] = potentiometer_max_value;
  calib_values[knob_id+num_knobs] = 0;
}//reset_knob_calibration

void log_to_serial( int level, const char* message ){
  if( level == DEBUGGER && !_DEBUG ) return;
  // trim whitespace from message
  String msg = String(message);
  msg.trim();
  message = msg.c_str();
  uint8_t payload_metadata_size = (uint8_t) sizeof(outbound_log_payload);
  // fill in header
  outbound_packet.header.seq = packet_header_seq;
  outbound_packet.header.type = (uint8_t) LOG;
  // fill in payload
  outbound_log_payload.level = (uint8_t) level;
  outbound_log_payload.message_size = uint8_t( min(strlen(message), 255-payload_metadata_size) );
  // complete header
  outbound_packet.header.payload_size = payload_metadata_size + outbound_log_payload.message_size;
  // copy message into the payload object
  strncpy(outbound_log_payload.message, message, outbound_log_payload.message_size);
  // copy the payload into the packet object
  memcpy(outbound_packet.payload, (const unsigned char*)&outbound_log_payload, outbound_packet.header.payload_size);
  // publish packet
  send_packet_to_serial( &outbound_packet );
  // store latest error message
  if( level == ERROR )
    strncpy(last_error_buf, message, outbound_log_payload.message_size);
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

int min_in_array( int* arr, int arr_len ){
  if( arr_len < 1 ) return -1;
  int arr_min = arr[0];
  for( int i=0; i < arr_len; i++ ){
    if( arr[i] < arr_min )
      arr_min = arr[i];
  }
  return arr_min;
}//min_in_array

int max_in_array( int* arr, int arr_len ){
  if( arr_len < 1 ) return -1;
  int arr_max = arr[0];
  for( int i=0; i < arr_len; i++ ){
    if( arr[i] > arr_max )
      arr_max = arr[i];
  }
  return arr_max;
}//max_in_array
