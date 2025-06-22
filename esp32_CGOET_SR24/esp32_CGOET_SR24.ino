/*
This device acts as controller for the E90+ with SR24 and ST16.
The processor reads the channel data from ST16 and send following 
data via control message to E90:
- pan
- tilt
- pan mode
- tilt mode.
Camera settings and video stream still using WiFi connection.

Pan value conversion is for ST16.
If ST16S is used the conversion should be skipped.

You need power source for the camera 12-16V (VBAT, GND) and a step-down 
converter to 3.3V (or 5V depending on ESP32 module type) for the ESP32.
3.3V for SR24 comes from ESP32 board. 
E90_TXD2 to cam mRx/PWM
E90_RXD2 to cam mTx
Wiring depends on HW port definition below.
*/

#define LEDlightPIN 22    // LED illumination or something else to switch on/off
#define SR24_RXD1 19      // serial1 SR24  yellow
#define SR24_TXD1 18      // gray          GND black, 3.3v white
#define E90_RXD2 16       // mTx
#define E90_TXD2 17       // mRx/PWM

#define UART_speed 115200
#define UART_speed_E90 500000
#define sr24buffer_size 44
#define E90buffer_size 38
#define panmode_F 683

const uint16_t X25_INIT_CRC = 0xFFFF;
const byte sr24header = 0x55;
const byte FDheader = 0xFD;
const uint16_t CRC_EXTRA = 252;
const byte nibblemask = 0x0F;
const uint16_t channelmask = 0x0FFF; // 12 bytes per channel 


// const byte E90heartbeat[21] ={FDheader, 9, 0,	0, 0, 1, 0, 0, 0, 0, 0, 0, 4, 3, 0x0D, 0x0C, 0x1D, 3, 3, 0x4E, 0x84};
//                                                                                                        CRC   CRC
//                                                                                                 pan------ tilt-----       pan mode- tilt mode      CRC_l CRC_h
byte E90buffer[E90buffer_size] = {FDheader,0x1A,0,0,0,1,1,0x88,0x13,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0x00,0x08,0x20,0x03, 0,8, 0x3E,0x03,0x34,0x08,0,8, 0x00, 0x00};	

byte sr24buffer[sr24buffer_size];
byte E90sequno = 0;

uint16_t UpscaleTo150 (int val, bool convert = true) {   // Scale 683 to 3412 (100%)
  if (convert) {
    val -= panmode_F;
    if (val < 0) {val = 0;} 
    float val_asfloat = val * 4095.0 / 2729.0;           // Scale 0 to 4095 (150%)
    val = round(val_asfloat);
  }  
  return val & channelmask;
}

static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
  // Accumulate one byte of data into the CRC
  uint8_t tmp;
  tmp = data ^ (uint8_t)(*crcAccum &0xFF);
  tmp ^= (tmp<<4);
  *crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}

void setup() {
  btStop();

  Serial.begin(UART_speed);
  Serial1.begin(UART_speed, SERIAL_8N1, SR24_RXD1, SR24_TXD1);         // SR24
  Serial2.begin(UART_speed_E90, SERIAL_8N1, E90_RXD2, E90_TXD2);       // E90 family cameras
  
  pinMode(LEDlightPIN, OUTPUT);
  delay(100);
  digitalWrite(LEDlightPIN, LOW); 
}

void loop() {
  uint16_t crc16;
  uint16_t aux;            
  byte incomingByte; 
  byte sr24len = 0;
  byte numreadbytes;

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
      sr24len = Serial1.peek();          // Msg length
      numreadbytes = Serial1.readBytes(sr24buffer, sr24len+1);    

      if ((sr24len > 0) && (numreadbytes == sr24len+1)) {                             
/* 
Complete message received (buffer read matches msg length). Now we need to check
the message type in sr24buffer[1]. Channel messages can be used to create a
control message for a Yuneec camera (here the E90+).

If messaage is broken the buffer will send anyway.
*/
        if ((sr24buffer[1] == 0) || (sr24buffer[1] == 1) || (sr24buffer[1] == 3)) {
/* 
Create a control message for E90+ with default values in unused parameters.
Pan, tilt, pan mode, and tilt mode will be taken from SR24 message and overwritten in
E90+ default message.
After that the CRC16 will be recreated and the message will be sent to E90+ port.
CRC16 calculation from https://github.com/mavlink/c_library_v2/blob/master/checksum.h
*/
          E90buffer[4] = E90sequno;
          
          aux = UpscaleTo150(((sr24buffer[16] & nibblemask) << 8) + sr24buffer[17]);
          E90buffer[24] = lowByte(aux);                              // Camera pan 
          E90buffer[25] = highByte(aux);
          Serial.println("Pan="+String(aux));                        // Debug

          aux = (sr24buffer[15] << 4) + (sr24buffer[16] >> 4);       // Camera  tilt
          E90buffer[26] = lowByte(aux);
          E90buffer[27] = highByte(aux);
             
          aux = ((sr24buffer[19] & nibblemask) << 8) + sr24buffer[20];      // Pan mode
          E90buffer[30] = 0x3E;                                       // Reset to default "P"
          E90buffer[31] = 3;
          if (aux < 1400) {                                           // Overwrite pan knob to center camera, pan mode "F" 
            E90buffer[24] = 0;
            E90buffer[25] = 8;
          } else {
            if (aux > 1600) {                                         // Global mode "G"
              E90buffer[30] = 0xB8;
              E90buffer[31] = 0x0B;
            }
          }

          aux = (sr24buffer[18] << 4) + (sr24buffer[19] >> 4);        // Camera  tilt mode
          if (aux > 2200) {                                           // Tilt mode "V"
            E90buffer[32] = 0xB8;
            E90buffer[33] = 0x0B;
          } else {                                                    // Default tilt mode "A"  
            E90buffer[32] = 0x34;
            E90buffer[33] = 8;
          }

//          aux = (sr24buffer[21] << 4) + (sr24buffer[22] >> 4);        // Switch on/off light: LG switch, channel 10
          aux = ((sr24buffer[22] & nibblemask) << 8) + sr24buffer[23];      // Switch on/off light: AUX button, channel 11
          if (aux > 2048) {
            digitalWrite(LEDlightPIN, HIGH);
          } else {
            digitalWrite(LEDlightPIN, LOW);
          }

          crc16 = X25_INIT_CRC;
          for (uint8_t k=1; k<(E90buffer_size-2); k=k+1) {
            crc_accumulate(E90buffer[k], &crc16);
          }
          crc_accumulate(CRC_EXTRA, &crc16);
          E90buffer[36] = lowByte(crc16);
          E90buffer[37] = highByte(crc16);

          Serial2.write(E90buffer, E90buffer_size);                 //  To E90
          E90sequno++;
        }                                                             
      } 
    }
  }
}
