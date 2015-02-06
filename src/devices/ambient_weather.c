#include "rtl_433.h"

static float
get_os_temperature (unsigned char * message, unsigned int sensor_id)
{
  // sensor ID included  to support sensors with temp in different position
  float temp_c = 0;
  temp_c = (((message[5]>>4)*100)+((message[4]&0x0f)*10) + ((message[4]>>4)&0x0f)) /10.0F;
  if (message[5] & 0x0f)
    temp_c = -temp_c;
  return temp_c;
}

static unsigned int
get_os_humidity (unsigned char * message, unsigned int sensor_id)
{
  // sensor ID included to support sensors with temp in different position
  int humidity = 0;
  humidity = ((message[6]&0x0f)*10)+(message[6]>>4);
  return humidity;
}

static int
validate_os_checksum (unsigned char * msg, int checksum_nibble_idx)
{
  // Oregon Scientific v2.1 and v3 checksum is a  1 byte  'sum of nibbles' checksum.
  // with the 2 nibbles of the checksum byte  swapped.
  int i;
  unsigned int checksum, sum_of_nibbles=0;
  for (i=0; i<(checksum_nibble_idx-1);i+=2) {
    unsigned char val=msg[i>>1];
    sum_of_nibbles += ((val>>4) + (val &0x0f));
  }
  if (checksum_nibble_idx & 1) {
    sum_of_nibbles += (msg[checksum_nibble_idx>>1]>>4);
    checksum = (msg[checksum_nibble_idx>>1] & 0x0f) | (msg[(checksum_nibble_idx+1)>>1]&0xf0);
  } else
    checksum = (msg[checksum_nibble_idx>>1]>>4) | ((msg[checksum_nibble_idx>>1]&0x0f)<<4);
  sum_of_nibbles &= 0xff;

  if (sum_of_nibbles == checksum)
    return 0;
  else {
    fprintf(stderr, "Checksum error in Oregon Scientific message.  Expected: %02x  Calculated: %02x\n", checksum, sum_of_nibbles);
    fprintf(stderr, "Message: "); int i; for (i=0 ;i<((checksum_nibble_idx+4)>>1) ; i++) fprintf(stderr, "%02x ", msg[i]); fprintf(stderr, "\n\n");
    return 1;
  }
}

static uint8_t 
validate_checksum(uint8_t *buff, int length)
{
  uint8_t mask = 0x7C;
  uint8_t checksum = 0x64;
  uint8_t data;
  int byteCnt;	

  for (byteCnt=0; byteCnt < length; byteCnt++)
  {
    int bitCnt;
    data = buff[byteCnt];

    for ( bitCnt= 7; bitCnt >= 0 ; bitCnt-- )
    {
      uint8_t bit;

      // Rotate mask right
      bit = mask & 1;
      mask =  (mask >> 1 ) | (mask << 7);
      if ( bit )
      {
	mask ^= 0x18;
      }

      // XOR mask into checksum if data bit is 1	
      if ( data & 0x80 )
      {
	checksum ^= mask;
      }
      data <<= 1; 
    }
  }
  return checksum;
}


static int
ambient_weather_parser (uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS])
{
  /* shift all the bits left 1 to align the fields */
  int i;
  for (i = 0; i < BITBUF_COLS-1; i++) {
    uint8_t bits1 = bb[0][i] << 1;
    uint8_t bits2 = (bb[0][i+1] & 0x80) >> 7;
    bits1 |= bits2;
    bb[0][i] = bits1;
  }
  fprintf(stderr, "\n! ");
  for (i = 0 ; i < BITBUF_COLS ; i++) {
    fprintf (stderr, "%02x ", bb[0][i]);
  }
  fprintf (stderr,"\n\n");



  /*
  00 14 50 60 49
  */
  if ( (bb[0][0] == 0x00) && (bb[0][1] == 0x14) && (bb[0][2] & 0x50) ) {

    uint16_t deviceID = ( (bb[0][2] & 0x0f) << 4) | ((bb[0][3] & 0xf0)  >> 4);
    fprintf (stderr, "DeviceID: %d\n", deviceID);

    uint16_t channel = (bb[0][3] & 0x07);
    fprintf (stderr, "Channel: %d\n", channel);

    /* does not yet take into account the sign bit */
    uint16_t temp_raw = bb[0][4];
    temp_raw <<= 4;
    temp_raw |= ((bb[0][5] & 0xf0) >> 4);
    temp_raw -= 400;
    float temperature = temp_raw / 10.0f;
    fprintf (stderr, "Temperature: %.1f\n", temperature);

    uint8_t humidity = ( (bb[0][5] & 0x0f) << 4) | ((bb[0][6] & 0xf0) >> 4);
    fprintf (stderr, "Humidity: %d\n", humidity);

    uint8_t checksum = ( (bb[0][6] & 0x0f) << 4) | ((bb[0][7] & 0xf0) >> 4);
    fprintf (stderr, "Checksum: %d\n", checksum);
  
  
    uint8_t pkt[5];
    pkt[0] = ((bb[0][1] & 0x0f) << 4) | ((bb[0][2] & 0xf0) >> 4);
    pkt[1] = deviceID;
    pkt[2] = ((bb[0][3] & 0x0f) << 4) | ((bb[0][4] & 0xf0) >> 4);
    pkt[3] = (temp_raw + 400) & 0x0ff;
    pkt[4] = humidity;
    checksum = validate_checksum (pkt, 5);
    fprintf (stderr, "MyChecksum: %d\n", checksum);
  } 

  return 0;
}

static int
ambient_weather_callback (uint8_t bb[BITBUF_ROWS][BITBUF_COLS], int16_t bits_per_row[BITBUF_ROWS])
{
  return ambient_weather_parser (bb, bits_per_row);
}

r_device ambient_weather = {
    /* .id             = */ 14,
    /* .name           = */ "Ambient Weather Temperature Sensor",
    /* .modulation     = */ OOK_MANCHESTER,
    /* .short_limit    = */ 125,
    /* .long_limit     = */ 0, // not used
    /* .reset_limit    = */ 600,
    /* .json_callback  = */ &ambient_weather_callback,
};
