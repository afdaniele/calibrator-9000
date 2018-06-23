#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// debug variables
const bool DEBUG = true;

// define constants
const float frequency = 100.0;      // the firmware runs at 100Hz
const float data_frequency = 30.0;  // the firmware spits data at 30Hz
const int adc_bits_resolution = 12;
const unsigned int serial_input_max_num_bytes = 50;
const int knob_calibration_time_secs = 2;
const bool potentiometer_inverted = true;
const int num_knobs = 6;
int knob_raw_value[num_knobs];
float knob_value[num_knobs];
int knob_zero_value[num_knobs];
int active_knobs[num_knobs];
int num_active_knobs = 0;

// computable parameters
const int data_publisher_spin_range = ((int) frequency / data_frequency );
const int potentiometer_max_value = ((int) pow(2,adc_bits_resolution) );
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
  RESET_COMM = 1,
  INITIALIZE = 2,
  RECALIBRATE = 3,
  SHUTDOWN = 4
};

typedef struct {
 uint32_t seq;
 int16_t operation;
 int16_t checksum;
} __attribute__((__packed__)) request_packet_header_t;

typedef struct {
 request_packet_header_t header;
 byte payload[16];
} __attribute__((__packed__)) request_packet_t;

typedef struct {
 uint8_t enable_x;
 uint8_t enable_y;
 uint8_t enable_z;
 uint8_t enable_r;
 uint8_t enable_p;
 uint8_t enable_w;
 byte unused[10];
} __attribute__((__packed__)) request_payload_initialize_t;

typedef struct {
 uint8_t recalibrate_x;
 uint8_t recalibrate_y;
 uint8_t recalibrate_z;
 uint8_t recalibrate_r;
 uint8_t recalibrate_p;
 uint8_t recalibrate_w;
 byte unused[10];
} __attribute__((__packed__)) request_payload_recalibrate_t;

// data packet
enum DATA_PACKET_TYPE{  // this has size int16_t
  EMPTY = 0,
  STATUS = 1,
  DATA = 2
};

typedef struct {
 uint32_t seq;
 uint16_t type;
 int16_t checksum;
} __attribute__((__packed__)) data_packet_header_t;

typedef struct {
 data_packet_header_t header;
 byte payload[16];
} __attribute__((__packed__)) data_packet_t;

typedef struct {
 uint8_t enable_x;
 uint8_t enable_y;
 uint8_t enable_z;
 uint8_t enable_r;
 uint8_t enable_p;
 uint8_t enable_w;
 byte unused[10];
} __attribute__((__packed__)) data_payload_t;

// compute knob calibration dead zone limits and timer counter
int knob_calibration_lower_limit = knob_calibration_dead_zone_center - knob_calibration_dead_zone_extension;
int knob_calibration_upper_limit = knob_calibration_dead_zone_center + knob_calibration_dead_zone_extension;
const int knob_calibration_time_steps = ((int)knob_calibration_time_secs*frequency);

// spin counter used for animations
int spin_counter = 0;
int spin_counter_range = 2000;

//temporary buffers
char strbuf[512];
int knob_calibration_timer_counters[num_knobs];
uint32_t packet_header_seq = 0;

// setup code, runs once
void setup() {
  // initialize serial port 
  Serial.begin(9600);
  // initialize knobs buffer
  for( int i = 0; i < num_knobs; i++ ){
    knob_value[i] = -1.0;
    knob_raw_value[i] = -1;
    knob_zero_value[i] = -1;
    active_knobs[i] = -1;
    knob_status[i] = UNUSED;
  }
  // configure ADC
  analogReadResolution(adc_bits_resolution);
  // TODO: read this from serial
  active_knobs[0] = 0;
  active_knobs[1] = 1;
  active_knobs[2] = 2;
  active_knobs[3] = 3;
  num_active_knobs = 4;
  // configure RGB PWM ports
  for( int i = 0; i < num_knobs; i++ ){
    for( int j = 0; j < 3; j++ ){
      int port_id = knob_color_port[i][j];
      if( port_id == -1 ) continue;
      // configure port
      pinMode(port_id, OUTPUT);
    }
  }
  // set next state
  status = CALIBRATING;
}//setup

// this function stops the firmware and halts the device
void(* shutdown_fcn) (void) = 0;

void reboot_fcn(){
  //TODO: connect the RST port of the board to a GPIO port (e.g., 7) and set it to HIGH to reboot the board
}

// put your main code here, to run repeatedly:
void loop() {
  // if somebody is talking to the firmware over USBSerial
  while (Serial.available () > 0)
    process_incoming_byte( Serial.read() );
  // FAILURE
  if( status == FAILURE ){
    return;  
  }
  // read knob values
  for( int i = 0; i < num_active_knobs; i++ ){
    int knob_id = active_knobs[i];
    int knob_port = knob_adc_port[knob_id];
    knob_raw_value[knob_id] = analogRead( knob_port );
    if( potentiometer_inverted ){
      knob_raw_value[knob_id] = potentiometer_max_value - knob_raw_value[knob_id];
    }
    // if the knob is calibrated, compute the value in the range [0,1]   
    if( knob_status[knob_id] == READY ){
      float knob_relative_value = ((float)knob_raw_value[knob_id] - knob_zero_value[knob_id]);
      if( knob_raw_value[knob_id] <= knob_zero_value[knob_id] ){
        knob_value[knob_id] = knob_relative_value / ((float)knob_zero_value[knob_id]);
      }else{
        knob_value[knob_id] = knob_relative_value / ((float)potentiometer_max_value - knob_zero_value[knob_id]);
      }
    }
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
          knob_zero_value[knob_id] = knob_raw_value[i];
          knob_status[knob_id] = READY;
          animate_led( knob_id, FIX );
        }else{
          animate_led( knob_id, BLINK_FAST );
        }
      }
    }
    // debug only
    if( DEBUG ){
      debug_fcn();
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
  
  // WORKING
  if( status == WORKING ){
    if( spin_counter % data_publisher_spin_range == 0 ){
      // it is time to publish the configuration of the axis
      request_packet_t pk;
      pk.header.seq = packet_header_seq;
//      pk.header.stamp = ((uint32_t) millis());
//      pk.
      debug_fcn();

      

      packet_header_seq += 1;
    }

    
  }



  
  
  // TODO
  
  // keep spinning
  delay( 1000.0 / frequency );
  // update spin counter
  spin_counter += 1;
  spin_counter = spin_counter % spin_counter_range;
}//loop

void process_incoming_byte( const byte inByte ){
  static char input_line[serial_input_max_num_bytes];
  static unsigned int input_pos = 0;
  switch (inByte){
    case '\n':   // end of text
      input_line[input_pos] = 0;  // terminating null byte
      // terminator reached! process input_line here ...
      process_request( input_line );
      // reset buffer for next time
      input_pos = 0;  
      break;
    case '\r':   // discard carriage return
      break;
    default:
      // keep adding if not full ... allow for terminating null byte
      if( input_pos < (serial_input_max_num_bytes - 1) )
        input_line[input_pos] = inByte;
        input_pos += 1;
      break;
  }
}//process_incoming_byte

void process_request( const char request[] ){
  char buf[128];
  snprintf( buf, sizeof buf, "%s%s", "RECEIVED: ", request );
  Serial.println(buf);
}//process_request

void debug_fcn(){
  snprintf(strbuf, sizeof strbuf, "%s", "Calibrator9000 :: ");
  for( int i = 0; i < num_active_knobs; i++ ){
    int knob_id = active_knobs[i];
    if( i > 0 ){
      snprintf(strbuf, sizeof strbuf, "%s%s", strbuf, " | ");
    }
    if( knob_status[knob_id] == UNCALIBRATED ){
      snprintf(
        strbuf, sizeof strbuf, 
        "%s%s%c%s(%d)", 
        strbuf, "AXIS_", knob_var_name[knob_id], "=UNCAL", knob_calibration_timer_counters[knob_id] 
      );
    }else{
      snprintf(
        strbuf, sizeof strbuf, 
        "%s%s%c%s%s", 
        strbuf, "AXIS_", knob_var_name[knob_id], "=", sfloat(knob_value[knob_id])
      );
    }
  }
  snprintf( strbuf, sizeof strbuf, "%s%c", strbuf, '\n' );
  Serial.print(strbuf);
}//debug_fcn

char* sfloat( float floatValue ){
  char buf[10];
  snprintf(
    buf, sizeof buf, 
    "%s%d.%03d", 
    ((floatValue<0)? "-":""), (int)fabs(floatValue), (int)(fabs(floatValue) * 1000.0) % 1000
  );
  return buf;
}//sfloat

void animate_led( int led_id, enum LED_STATUS led_status ){
  int led_pwm_high = 255;
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
      pwm_value = map( animation_step, 0, animation_mid, 0, 255 );
    }else{
      pwm_value = map( animation_step, animation_mid, blink_range, 255, 0 );
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
