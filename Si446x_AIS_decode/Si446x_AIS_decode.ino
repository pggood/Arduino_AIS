/*
 * Library to control Silicon Laboratories SI6342 radio
 * Author: Peter Goodricke
 * based on dAISy project Adrian Studer ported to arduino
 * License: CC BY-NC-SA Creative Commons Attribution-NonCommercial-ShareAlike
 *       http://creativecommons.org/licenses/by-nc-sa/4.0/
 */
#include <SPI.h> 
#include "radio_config.h"
//vcc on radio to 3.3volt
//gnd on radio to gnd
//gpio2 on radio to pin d2
//irq on radio to pin d3
//nsel on radio to pin d4
//gpio1 on radio to pin d5
//gpio3 on radio to pin d6
//sdn on radio to pin d8 
//mosi on radio to pin d11
//miso on radio to pin d12
//clock on radio to pin d13
static const char config[] = RADIO_CONFIGURATION_DATA_ARRAY;
const int ClockPin =2; // 
const int IRQPin =3;
const int chipSelectPin = 4;
const int CLSpin=5;
const int SDNPin = 8;
const int DATA_PIN = 6;
const int DEBUG_PIN = 7;
// paramters for package detection
#define DEBUG
#define PH_PREAMBLE_LENGTH  8   // minimum number of alternating bits we need for a valid preamble
#define PH_SYNC_TIMEOUT 16      // number of bits we wait for a preamble to start before changing channel
#define FIFO_BUFFER_SIZE    128         // size of FIFO in bytes (must be 2^x)
#define FIFO_PACKETS      8         // max number of individual packets in FIFO (must be 2^x, should be approx. FIFO_BUFFER_SIZE/avg message size)
#define FIFO_PTR_TYPE uint8_t           // 8 bit for FIFO smaller than 256 bytes
#define FIFO_BUFFER_MASK (FIFO_BUFFER_SIZE - 1)   // mask for easy warping of buffer
#define FIFO_PACKET_MASK (FIFO_PACKETS - 1)     // mask for easy warping of packet table
#define NMEA_MAX_AIS_PAYLOAD 42    // number of AIS bytes per NMEA sentence, to keep total NMEA sentence always below 82 characters
#define NMEA_AIS_BITS (NMEA_MAX_AIS_PAYLOAD * 8)
#define NMEA_AIS_BITS_ENCODED ((NMEA_AIS_BITS + 5) / 6)
const char nmea_lead[] = "!AIVDM,";   // static start of NMEA sentence
char nmea_buffer[8+NMEA_AIS_BITS_ENCODED+5+1];  // buffer for dynamic part of NMEA sentence
                                                // fragment and channel info, AIS payload, stuff-bit and crc, 0-termination
uint8_t nmea_buffer_index;        // current buffer position
#define NMEA_LEAD_CRC 'A' ^ 'I' ^ 'V' ^ 'D' ^ 'M' ^ ','   // CRC for static start of sentence
uint8_t nmea_crc;           // calculated CRC
uint8_t nmea_message_id = 0;      // sequential message id for multi-sentence message
const char nmea_hex[] = { '0', '1', '2', '3','4', '5', '6', '7','8', '9', 'A', 'B','C', 'D', 'E', 'F' };    // lookup table for hex conversion of CRC
enum PH_STATE{
  PH_STATE_OFF = 0,
  PH_STATE_RESET,         // reset/restart packet handler
  PH_STATE_WAIT_FOR_SYNC,     // wait for preamble (010101..) and start flag (0x7e)
  PH_STATE_PREFETCH,        // receive first 8 bits of packet
  PH_STATE_RECEIVE_PACKET     // receive packet payload
};
// packet handler errors
enum PH_ERROR {
  PH_ERROR_NONE = 0,
  PH_ERROR_STUFFBIT,    // invalid stuff-bit
  PH_ERROR_NOEND,     // no end flag after more than 1020 bits, message too long
  PH_ERROR_CRC,     // CRC error
  PH_ERROR_RSSI_DROP    // signal strength fell below threshold
};
// preamble and start flag detection states
enum PH_SYNC_STATE {
  PH_SYNC_RESET = 0,      // reset/restart sync detection
  PH_SYNC_0,          // last bit was a 0
  PH_SYNC_1,          // last bit was a 1
  PH_SYNC_FLAG        // detecting start flag
};
unsigned char results[9];
unsigned char interrupt[9];
volatile boolean interuptwaiting;
unsigned char sync;
unsigned int bitstream;
volatile unsigned char  ph_state = PH_STATE_OFF;
volatile unsigned char ph_last_error = PH_ERROR_NONE;
volatile unsigned char  ph_radio_channel = 0;
volatile unsigned char  ph_message_type = 0;
unsigned char fifo_buffer[128];        // buffer to hold packet data
unsigned int fifo_packets[8];     // table with start offsets of received packets

unsigned int fifo_bytes_in;            // counter for bytes written into current packet
unsigned int  fifo_bytes_out;           // counter for bytes read from current packet
volatile unsigned char fifo_packet_in;          // table index of incoming packet
unsigned char fifo_packet_out;              // table index of outgoing packet
unsigned int chan_timer;
void timer(){
  for (int timer = 0; timer < 2595; timer++) {
    asm("nop");
  }
}
void waitforreply(){
  unsigned char reply;
    reply=0;
    int counter =0;
    while (reply != 0xFF)
    {
      digitalWrite(chipSelectPin, LOW);
      timer();
      SPI.transfer(0x44);
      timer();
      reply = SPI.transfer(0x00);
      timer();
      digitalWrite(chipSelectPin, HIGH);
      timer();
      counter++;
      if (counter == 2000)reply = 0xFF;      
    }
    timer();
}

static void configure(void)
{
  unsigned char buff[17];
  unsigned char buffpos=0;
  unsigned char configvalue;
  unsigned char configlength;
  unsigned int configpos=0;
  unsigned int configendpos=0;
  unsigned int configcurrentpos=0;
  timer();
  digitalWrite(chipSelectPin, HIGH);
  timer();
  digitalWrite(SDNPin, LOW);
  while (!(digitalRead(CLSpin)));
  while(configpos < sizeof(config)){
    configlength=config[configpos];
    configpos++;
    configcurrentpos=configpos;
    configendpos=configpos+configlength;
    buffpos=0;
    for(unsigned int i=configcurrentpos;i<configendpos;i++)// load each value from config
    { 
      buff[buffpos++]=config[i];
      configpos++;
    }
    Si4464_write(buff,configlength);
  }
 
}
void Si4464_write(const byte pData[],int byteCountTx) {
byte wordb;
  digitalWrite(chipSelectPin, LOW);
   //timer();
    for (int j = 0; j < byteCountTx; j++) // Loop through the bytes of the pData
    {
      wordb = pData[j];
      SPI.transfer(wordb);
    }
   //timer();
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  while (!(digitalRead(CLSpin)));
}
void fifo_write_byte(unsigned char data)
{
  // add byte to the incoming packet
  unsigned int position = (fifo_packets[fifo_packet_in] + fifo_bytes_in) & 127;   // calculate position in buffer
  fifo_buffer[position] = data;         // store byte at position
  fifo_bytes_in++;                // increase byte counter
}
void fifo_new_packet(void)
{
  // reset offset to (re)start packet
  fifo_bytes_in = 0;
}
void fifo_commit_packet(void)
{
  // complete incoming packet by advancing to next slot in FIFO
  FIFO_PTR_TYPE new_position = (fifo_packets[fifo_packet_in] + fifo_bytes_in) & FIFO_BUFFER_MASK; // calculate position in buffer for next packet
  fifo_packet_in = (fifo_packet_in + 1) & FIFO_PACKET_MASK;
  fifo_packets[fifo_packet_in] = new_position;  // store new position in packet table
  fifo_bytes_in = 0;                // reset offset to be ready to store data
}
void si4464clock() {
  digitalWrite(DEBUG_PIN, LOW);
  static uint16_t rx_bitstream;       // shift register with incoming data
  static uint16_t rx_bit_count;       // bit counter for various purposes
  static uint16_t rx_crc;           // word for AIS payload CRC calculation
  static uint8_t rx_one_count;        // counter of 1's to identify stuff bits
  static uint8_t rx_data_byte;        // byte to receive actual package data
  uint8_t rx_this_bit_NRZI;         // current bit for NRZI decoding
  static uint8_t rx_prev_bit_NRZI;      // previous bit for NRZI decoding
  uint8_t rx_bit;               // current decoded bit
  static uint8_t rx_sync_state;       // state of preamble and start flag detection
  static uint8_t rx_sync_count;       // length of valid bits in current sync sequence

  uint8_t wake_up = 0;            // if set, LPM bits will be cleared
  
  if(digitalRead(CLSpin)){
    rx_this_bit_NRZI=digitalRead(DATA_PIN);
    // packet handler state machine
    rx_bit = !(rx_prev_bit_NRZI ^ rx_this_bit_NRZI);  // NRZI decoding: change = 0-bit, no change = 1-bit, i.e. 00,11=>1, 01,10=>0, i.e. NOT(A XOR B)
    rx_prev_bit_NRZI = rx_this_bit_NRZI;        // store encoded bit for next round of decoding

    // add decoded bit to bit-stream (receiving LSB first)
    rx_bitstream >>= 1;
    if (rx_bit){
      rx_bitstream |= 0x8000;
    }
    
    // packet handler state machine
    switch (ph_state) {

  // STATE: OFF
    case PH_STATE_OFF:                  // state: off, do nothing
      break;

  // STATE: RESET
    case PH_STATE_RESET:                // state: reset, prepare state machine for next packet

      rx_bitstream &= 8000;             // reset bit-stream (but don't throw away incoming bit)
      rx_bit_count = 0;               // reset bit counter
      fifo_new_packet();                // reset fifo packet
      fifo_write_byte(ph_radio_channel);        // indicate channel for this packet
      ph_state = PH_STATE_WAIT_FOR_SYNC;        // next state: wait for training sequence
      rx_sync_state = PH_SYNC_RESET;
      break;

  // STATE: WAIT FOR PREAMBLE AND START FLAG
    case PH_STATE_WAIT_FOR_SYNC:            // state: waiting for preamble and start flag
      rx_bit_count++;                 // count processed bits since reset

    // START OF SYNC STATE MACHINE
      switch (rx_sync_state) {

    // SYNC STATE: RESET
      case PH_SYNC_RESET:               // sub-state: (re)start sync process
        if (rx_bit_count > PH_SYNC_TIMEOUT)     // if we exceeded sync time out
          ph_state = PH_STATE_RESET;        // reset state machine, will trigger channel hop
        else {                    // else
          rx_sync_count = 0;            // start new preamble
          if (rx_bit)
            rx_sync_state = PH_SYNC_1;      // we started with a 1
          else
            rx_sync_state = PH_SYNC_0;      // we started with a 0
        }
        break;

      // SYNC STATE: 0-BIT
      case PH_SYNC_0:                 // sub-state: last bit was a 0
        if (rx_bit) {               // if we get a 1
          rx_sync_count++;              // valid preamble bit
          rx_sync_state = PH_SYNC_1;          // next state
        } else {                  // if we get another 0
          if (rx_sync_count > PH_PREAMBLE_LENGTH) { // if we have a sufficient preamble length
            rx_sync_count = 7;              // treat this as part of start flag, we already have 1 out of 8 bits (0.......)
            rx_sync_state = PH_SYNC_FLAG;       // next state flag detection
          }
          else                    // if not
            rx_sync_state = PH_SYNC_RESET;        // invalid preamble bit, restart preamble detection
        }
        break;

      // SYNC STATE: 1-BIT
      case PH_SYNC_1:                 // sub-state: last bit was a 1
        if (!rx_bit) {                // if we get a 0
          rx_sync_count++;              // valid preamble bit
          rx_sync_state = PH_SYNC_0;          // next state
        } else {                  // if we get another 1
          if (rx_sync_count > PH_PREAMBLE_LENGTH) { // if we have a sufficient preamble length
            rx_sync_count = 5;              // treat this as part of start flag, we already have 3 out of 8 bits (011.....)
            rx_sync_state = PH_SYNC_FLAG;       // next state flag detection
          }
          else                    // if not
            rx_sync_state = PH_SYNC_RESET;        // treat this as invalid preamble bit
        }
        break;

      // SYNC STATE: START FLAG
      case PH_SYNC_FLAG:                // sub-state: start flag detection
        rx_sync_count--;              // count down bits
        if (rx_sync_count != 0) {         // if this is not the last bit of start flag
          if (!rx_bit)                // we expect a 1, 0 is an error
            rx_sync_state = PH_SYNC_RESET;      // restart preamble detection
        } else {                  // if this is the last bit of start flag
          if (!rx_bit) {                // we expect a 0
            rx_bit_count = 0;             // reset bit counter
            ph_state = PH_STATE_PREFETCH;       // next state: start receiving packet
            wake_up = 1;                // main thread might want to do something on sync detect
          } else                    // 1 is an error
            rx_sync_state = PH_SYNC_RESET;        // restart preamble detection
        }
        break;
      }
    // END OF SYNC STATE MACHINE - preamble and start flag detected
      break;

  // STATE: PREFETCH FIRST PACKET BYTE
    case PH_STATE_PREFETCH:               // state: pre-fill receive buffer with 8 bits
      rx_bit_count++;                 // increase bit counter
      if (rx_bit_count == 8) {            // after 8 bits arrived
        rx_bit_count = 0;             // reset bit counter
        rx_one_count = 0;             // reset counter for stuff bits
        rx_data_byte = 0;             // reset buffer for data byte
        rx_crc = 0xffff;              // init CRC calculation
        ph_state = PH_STATE_RECEIVE_PACKET;     // next state: receive and process packet
        ph_message_type = rx_bitstream >> 10;   // store AIS message type for debugging
        break;
      }

      break;                      // do nothing for the first 8 bits to fill buffer

  // STATE: RECEIVE PACKET
    case PH_STATE_RECEIVE_PACKET:           // state: receiving packet data
    digitalWrite(DEBUG_PIN, HIGH);
      rx_bit = rx_bitstream & 0x80;         // extract data bit for processing

      if (rx_one_count == 5) {            // if we expect a stuff-bit..
        if (rx_bit) {               // if stuff bit is not zero the packet is invalid
          ph_last_error = PH_ERROR_STUFFBIT;    // report invalid stuff-bit error
          ph_state = PH_STATE_RESET;        // reset state machine
        } else
          rx_one_count = 0;           // else ignore bit and reset stuff-bit counter
        break;
      }

      rx_data_byte = rx_data_byte >> 1 | rx_bit;    // shift bit into current data byte

      if (rx_bit) {                 // if current bit is a 1
        rx_one_count++;               // count 1's to identify stuff bit
        rx_bit = 1;                 // avoid shifting for CRC
      } else
        rx_one_count = 0;             // or reset stuff-bit counter

      if (rx_bit ^ (rx_crc & 0x0001))         // CCITT CRC calculation (according to Dr. Dobbs)
        rx_crc = (rx_crc >> 1) ^ 0x8408;
      else
        rx_crc >>= 1;

      if ((rx_bit_count & 0x07)==0x07) {        // every 8th bit.. (counter started at 0)
        fifo_write_byte(rx_data_byte);        // add buffered byte to FIFO
        rx_data_byte = 0;             // reset buffer
      }

      rx_bit_count++;                 // count valid, de-stuffed data bits

      if ((rx_bitstream & 0xff00) == 0x7e00) {    // if we found the end flag 0x7e we're done
        if (rx_crc != 0xf0b8)           // if CRC verification failed
          ph_last_error = PH_ERROR_CRC;     // report CRC error
        else {
          fifo_commit_packet();         // else commit packet in FIFO
          //digitalWrite(DEBUG_PIN, HIGH);
        }
        ph_state = PH_STATE_RESET;          // reset state machine
        break;
      }

      if (rx_bit_count > 1020) {            // if packet is too long, it's probably invalid
        ph_last_error = PH_ERROR_NOEND;       // report error
        ph_state = PH_STATE_RESET;          // reset state machine
        break;
      }

      break;
    }
// END OF PACKET HANDLER STATE MACHINE

    if (ph_state == PH_STATE_RESET) {         // if next state is reset
      ph_radio_channel ^= 1;              // toggle radio channel between 0 and 1
      Si4464_write((const byte[]){0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00},8);
      wake_up = 1;                  // wake up main thread for packet processing and error reporting
    }
  }

  //LED1_OFF;


}
void si4464IRQ() {
  /*
  unsigned char reply;
  reply=0;
  //void waitforreply();
  Si4464_write((const byte[]){0x20,0x00,0x00,0x00 },4); //reply 0x04,0x04,0x00,0x02,0x00,0x00,0x14,0x14,0x00
  timer();
  while (reply != 0xFF)
    {
      digitalWrite(chipSelectPin, LOW);
      timer();
      SPI.transfer(0x44);
      timer();
      reply = SPI.transfer(0x00);
      timer();
      if (reply != 0xFF){
        digitalWrite(chipSelectPin, HIGH);
        timer();
      }
    }
    timer();
  for (int j = 0; j < 9; j++) // Loop through the bytes of the pData
     {
         interrupt[j]=SPI.transfer(0x00);
     }
    timer();
   digitalWrite(chipSelectPin, HIGH);
   interuptwaiting =1;
   */
}
uint16_t fifo_get_packet(void)
{
  // if available, initiate reading from packet from FIFO
  if (fifo_packet_in == fifo_packet_out)      // if no packets are in FIFO
    return 0;                 // return 0

  fifo_bytes_out = 0;               // reset read offset within current packet

  // calculate and size of available packet
  FIFO_PTR_TYPE next_packet = (fifo_packet_out + 1) & FIFO_PACKET_MASK;
  return (FIFO_BUFFER_SIZE -  fifo_packets[fifo_packet_out] + fifo_packets[next_packet]) & FIFO_BUFFER_MASK;
}
uint8_t fifo_read_byte(void)
{
  // retrieve byte from current packet
  FIFO_PTR_TYPE position = (fifo_packets[fifo_packet_out] + fifo_bytes_out) & FIFO_BUFFER_MASK; // calculate current read position
  fifo_bytes_out++;               // increase current read offset
  return fifo_buffer[position];         // return byte from calculated position
}
uint8_t nmea_push_packet(uint8_t packet_size)
{
  uint8_t raw_byte;
  uint8_t raw_bit;

  uint8_t nmea_byte;
  uint8_t nmea_bit;

  nmea_byte = 0;
  nmea_bit = 6;

  while (packet_size != 0) {
    raw_byte = fifo_read_byte();
    raw_bit = 8;

    while (raw_bit > 0) {
      nmea_byte <<= 1;
      if (raw_byte & 0x80)
        nmea_byte |= 1;
      nmea_bit--;

      if (nmea_bit == 0) {
        if (nmea_byte > 39)
          nmea_byte += 8;
        nmea_byte += 48;
        nmea_push_char(nmea_byte);
        nmea_byte = 0;
        nmea_bit = 6;
      }

      raw_byte <<= 1;
      raw_bit--;
    }

    packet_size--;
  }

  // stuff unfinished NMEA character
  uint8_t stuff_bits = 0;
  if (nmea_bit != 6)
  {
    // stuff with 0 bits as needed
    while (nmea_bit != 0) {
      nmea_byte <<= 1;
      nmea_bit--;
      stuff_bits++;
    }

    // .. and convert and store last byte
    if (nmea_byte > 39)
      nmea_byte += 8;
    nmea_byte += 48;
    nmea_push_char(nmea_byte);
  }

  return stuff_bits;
}
void nmea_process_packet(void)
{
  uint16_t packet_size = fifo_get_packet();

  if (packet_size == 0 || packet_size < 4)  // check for empty packet
    return;                 // no (valid) packet available in FIFO, nothing to send

  uint8_t radio_channel = fifo_read_byte() + 'A'; // retrieve radio channel (0=A, 1=B)

  // calculate number of fragments, NMEA allows 82 characters per sentence
  //      -> max 62 6-bit characters payload
  //      -> max 46 AIS bytes (368 bits) per sentence
  packet_size -= 3;             // Ignore channel information and AIS CRC
  uint8_t curr_fragment = 1;
  uint8_t total_fragments = 1;
  uint16_t packet_bits = packet_size * 8;
  while (packet_bits > NMEA_AIS_BITS) {
    packet_bits -= NMEA_AIS_BITS;
    total_fragments++;
  }

  // avoid sending garbage if fragment count does not make sense
  if (total_fragments > 9)
  {
    return;
  }

  // maintain message id if this is a multi-sentence message
  if (total_fragments > 1)
  {
    nmea_message_id++;
    if (nmea_message_id > 9)
      nmea_message_id = 1;    // keep message id < 10
  }

  // create fragments
  while (packet_size > 0) {
    // reset buffer and CRC
    nmea_buffer_index = 0;
    nmea_crc = NMEA_LEAD_CRC;

    // write fragment information, I assume total fragments always < 10
    nmea_push_char(total_fragments + '0');
    nmea_push_char(',');
    nmea_push_char(curr_fragment + '0');
    nmea_push_char(',');
    curr_fragment++;

    // write message id if there are multiple fragments
    if (total_fragments > 1)
      nmea_push_char(nmea_message_id + '0');
    nmea_push_char(',');

    // write channel information
    nmea_push_char(radio_channel);
    nmea_push_char(',');

    // encode and write next NMEA_MAX_AIS_PAYLOAD bytes from AIS packet
    uint8_t fragment_size = packet_size;
    if (fragment_size > NMEA_MAX_AIS_PAYLOAD)
    {
      fragment_size = NMEA_MAX_AIS_PAYLOAD;
    }
    uint8_t stuff_bits = nmea_push_packet(fragment_size);
    packet_size -= fragment_size;

    // write stuff bit
    nmea_push_char(',');
    nmea_push_char(stuff_bits + '0');

    // write CRC
    uint8_t final_crc = nmea_crc;   // copy CRC as push_char will modify it
    nmea_push_char('*');
    nmea_push_char(nmea_hex[final_crc >> 4]);
    nmea_push_char(nmea_hex[final_crc & 0x0f]);

    // terminate message with 0
    nmea_push_char(0);

    // send NMEA sentence over UART
     Serial.print(nmea_lead);
     Serial.println(nmea_buffer);
  }
}

void setup() {
        // put your setup code here, to run once:
        SPI.begin();
        Serial.begin(9600);
        pinMode(ClockPin, INPUT);    
        pinMode(IRQPin, INPUT); 
        pinMode(chipSelectPin, OUTPUT);
        pinMode(DEBUG_PIN, OUTPUT);
        pinMode(CLSpin, INPUT); 
        pinMode(SDNPin, OUTPUT);
        pinMode(DATA_PIN , INPUT);    
        digitalWrite(SDNPin, HIGH);
        digitalWrite(DEBUG_PIN, HIGH);
        pinMode(chipSelectPin, OUTPUT);
        #ifdef DEBUG
        Serial.println("Setup Start");
        #endif 
        configure(); 
        Si4464_write((const byte[]){0x23,0x00},2);
        Si4464_write((const byte[]){0x44,0x00,0x00,0x00,0x00},5);
      
      attachInterrupt(digitalPinToInterrupt(ClockPin),si4464clock, RISING );
      
      //attachInterrupt(digitalPinToInterrupt(IRQPin),si4464IRQ, RISING );
      ph_state = PH_STATE_RESET;
      #ifdef DEBUG
      Serial.println("end");
      #endif
      Si4464_write((const byte[]){0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00},8);
      chan_timer=0;
 }
 void fifo_remove_packet(void)
{
 // remove packet from FIFO, advance to next slot
  if(fifo_packet_in != fifo_packet_out)     // but only do so, if there's actually a packet available
    fifo_packet_out = (fifo_packet_out + 1) & FIFO_PACKET_MASK;
}
 void nmea_push_char(char c)
{
 nmea_crc ^= c;
  nmea_buffer[nmea_buffer_index++] = c;
}
void loop() {
  unsigned int packet_size;
  char radio_channel;
  unsigned char curr_fragment = 1;
  unsigned char total_fragments = 1;
  unsigned int packet_bits;
  unsigned int packet_pos;
  unsigned char nmea_message_id = 0; 
  unsigned char raw_byte;
  unsigned char raw_bit;
  unsigned char nmea_byte=0;
  unsigned char nmea_bit=6;
  unsigned char stuff_bits;
  /*
  chan_timer++;          
  if (chan_timer == 343)Si4464_write((const byte[]){0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00},8);
  for (int timer = 0; timer < 6082; timer++) {
    asm("nop");
  }
  if (chan_timer == 686){
    Si4464_write((const byte[]){0x32,0x01,0x00,0x00,0x00,0x00,0x00,0x00},8);
    chan_timer=0;
  }
  for (int timer = 0; timer < 6082; timer++) {
    asm("nop");
  }
*/
  #ifdef DEBUG
   switch (ph_last_error) {
    case PH_ERROR_NONE:
    break; 
    case PH_ERROR_STUFFBIT:
      Serial.println("invalid stuff-bit");
      ph_last_error = PH_ERROR_NONE;
      break;
    case PH_ERROR_NOEND:
      Serial.println("no end flag after more than 1020 bits, message too long");
      ph_last_error = PH_ERROR_NONE;
      break;
    case PH_ERROR_CRC:
      Serial.println("CRC error");
      ph_last_error = PH_ERROR_NONE;
      break;
    case PH_ERROR_RSSI_DROP:
      Serial.println("signal strength fell below threshold");
      ph_last_error = PH_ERROR_NONE;
      break;
    }
  #endif
  packet_size = fifo_get_packet();
  if (packet_size>0){
      nmea_process_packet();          // process packet (NMEA message will be sent over UART)
      fifo_remove_packet();         // remove processed packet from FIFO
  }    

}
