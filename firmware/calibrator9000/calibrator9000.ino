#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// put your setup code here, to run once:
void setup() {
  Serial.begin(9600);
}

// put your main code here, to run repeatedly:
void loop() {
  delay(10);
  int analogValue = analogRead(A5);
  Serial.print( "ADC: " );
  Serial.println( analogValue );
}
