/*
This device acts as controller for the CGO3+ with SR24 and ST16.
The processor reads the channel data from ST16 and send following 
data via control message to CGO3+:
- pan
- tilt
- pan mode
- tilt mode.
Camera settings and video stream still using WiFi connection.

You need power source for the camera 12-16V (VBAT, GND) and a step-down 
converter to 3.3V (or 5V depending on ESP32 module type) for the ESP32.
3.3V for SR24 comes from ESP32 board. 
CGO3_TXD2 to cam mRx/PWM
CGO3_RXD2 to cam mTx
Wiring depends on HW port definition below.
*/

#define BINDPIN 14        // Bind button to GND
#define LEDlightPIN 22    // LED illumination
#define SR24_RXD1 19      // serial1 SR24  yellow
#define SR24_TXD1 18      // gray          GND black, 3.3v white
#define CGO3_RXD2 16      // mTx
#define CGO3_TXD2 17      // mRx/PWM

#define UART_speed 115200
#define cgo3buffer_size 36
#define bindarray_size 11
#define panmode_F 683
const uint16_t X25_INIT_CRC = 0xFFFF;
const byte sr24header = 0x55;
const uint16_t CRC_EXTRA = 0;


const byte BINDARR[bindarray_size] = {0x55, 0x55, 0x08, 0x04, 0x00, 0x00, 0x42, 0x49, 0x4E, 0x44, 0xB0};
//                                    hd1   hd2   len     1    2      3    4     5      6    7    CRC8 (for payload 0..7 = 8 bytes)
//                                    SR24buffer
// byte FC_heartbeat[15]= {0xFE, 5, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0};

byte sr24buffer[44];
uint16_t crc16 = 0xFFFF;
byte cgo3sequno = 0;


//                                                                                      pan------ tilt-----       pan mode- tilt mode          CRC_l CRC_h
byte cgo3buffer[cgo3buffer_size] = {0xFE,26,0,1,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0x37,0,8,0, 0x00,0x08,0xAB,0x02, 0,8, 0xAB,0x02,0x88,0x08, 0xF4,1, 0x00, 0x00};

static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
  // Accumulate one byte of data into the CRC
  uint8_t tmp;
  tmp = data ^ (uint8_t)(*crcAccum &0xFF);
  tmp ^= (tmp<<4);
  *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

uint16_t UpscaleTo150 (int val, bool convert = true) {                 // Scale 683 to 3412 (100%)
  if (convert) {
    val -= panmode_F;
    if (val < 0) {val = 0;} 
    float val_asfloat = val * 4095.0 / 2729.0;      // Scale 0 to 4095 (150%)
    val = round(val_asfloat);
  }  
  return val & 0x0FFF;
}

void setup() {
  btStop();

  Serial.begin(UART_speed);
  Serial1.begin(UART_speed, SERIAL_8N1, SR24_RXD1, SR24_TXD1);       // SR24
  Serial2.begin(UART_speed, SERIAL_8N1, CGO3_RXD2, CGO3_TXD2);       // Camera
  
// Set the receiver into bind mode during ESP32 boot by button on GPIO14  
  pinMode(BINDPIN, INPUT_PULLUP);
  delay(100);
  if (digitalRead(BINDPIN) == LOW) {
    delay(500);
    Serial1.write(BINDARR, bindarray_size);
    delay(200);
    Serial1.write(BINDARR, bindarray_size);
    Serial.println("BIND");
  }

  pinMode(LEDlightPIN, OUTPUT);
  delay(100);
  digitalWrite(LEDlightPIN, LOW); 

}

void loop() {
  uint16_t aux;            
  byte sr24len = 0;
  byte incomingByte; 
  int numreadbytes;

  if (Serial1.available() > 1) {
    incomingByte = Serial1.read();
    if ((incomingByte == sr24header) && (Serial1.peek() == sr24header)) {   

/*
SR24 like message header found. Header 1 and 2 = 0x55
We peek the length of the message and load the whole message in the sr24buffer.
Buffer contains the message inclucsive msg length and CRC8, but not the 2 header bytes.

If not we send simply the received byte.
*/

      incomingByte = Serial1.read();     // header2
      sr24len = Serial1.peek();          // Save message length
      numreadbytes = Serial1.readBytes(sr24buffer, sr24len+1);    

      if ((sr24len > 0) && (numreadbytes == sr24len+1)) { 

/* 
Complete message received (buffer read matches msg length). Now we need to check
the message type in sr24buffer[1]. Channel messages can be used to create a
control message for a Yuneec camera (here the CGO3+).

If messaage is broken the buffer will send anyway.
*/

        if ((sr24buffer[1] == 0) || (sr24buffer[1] == 1) || (sr24buffer[1] == 3)) {   // Those message ID's contain channel data

/* 
Create a control message for CGO3+ with default values in unused parameters.
Pan, tilt, pan mode, and tilt mode will be taken from SR24 message and overwritten in
CGO3+ default message.
After that the CRC16 will be recreated and the message will be sent to CGO3+ port.
CRC16 calculation from https://github.com/mavlink/c_library_v2/blob/master/checksum.h
*/


          cgo3buffer[22] = sr24buffer[17];                            // Camera pan 
          cgo3buffer[23] = (sr24buffer[16] & 0x0F);                   // 12 bytes per channel

          aux = (sr24buffer[15] << 4) + (sr24buffer[16] >> 4);        // Camera  tilt
          cgo3buffer[24] = lowByte(aux);
          cgo3buffer[25] = highByte(aux);

//          Serial.println("Tilt="+String(aux));                      // Debug
             
          cgo3buffer[28] = sr24buffer[20];                            // Pan mode
          cgo3buffer[29] = sr24buffer[19] & 0x0F;

          aux = (sr24buffer[18] << 4) + (sr24buffer[19] >> 4);        // Camera  tilt mode
          cgo3buffer[30] = lowByte(aux);
          cgo3buffer[31] = highByte(aux);

          aux = (sr24buffer[21] << 4) + (sr24buffer[22] >> 4);        // Switch on/off light: LG switch channel 10
//          aux = ((sr24buffer[22] & 0x0F) << 8) + sr24buffer[23];      // Switch on/off light: AUX button channel 11
          if (aux > 2048) {
            digitalWrite(LEDlightPIN, HIGH);
          } else {
            digitalWrite(LEDlightPIN, LOW);
          }

          cgo3buffer[2] = cgo3sequno;
          crc16 = 0xFFFF;
          for (uint8_t k=1; k<(cgo3buffer_size-2); k++) {
            crc_accumulate(cgo3buffer[k], &crc16);
          }
          crc_accumulate(CRC_EXTRA, &crc16);
          cgo3buffer[34] = lowByte(crc16);
          cgo3buffer[35] = highByte(crc16);

          Serial2.write(cgo3buffer, cgo3buffer_size);                 //  To CGO3+
          cgo3sequno++;
        }                                                             

      } 

    }

  }

}

