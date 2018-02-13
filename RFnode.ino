#include <rflib.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

const uint64_t pipes[1] = { 0xF0F0F0F0E1LL };

void setup(void)
{
  Serial.begin(57600);
  printf_begin();
  
  if (rflib_sensor_init(9, 10, 0, pipes[0], 15, 15) < 0) {
    printf("Init failed :(\n\r");
    abort();
  }
}

static struct rflib_msg_t msg;
static struct rflib_msg_t ackmsg;

void loop(void)
{
  unsigned long time = millis();
  int msg_size = snprintf((char *) msg.data, RFLIB_MAX_MSGSIZE, "%d", time);
  msg.size = msg_size + 1;
  if (msg_size < 0 || msg_size >= RFLIB_MAX_MSGSIZE) {
    printf("String too long :(\n\r");
    msg.size = 0;
  }
  
  rflib_sensor_tx_pre();
  if (rflib_sensor_tx(&msg, &ackmsg) < 0) {
    printf("Send failed :(\n\r");
  } else {
    printf("Sent :), acklen = %d, ackmsg = \"%s\"\n\r", ackmsg.size, ackmsg.data);
  }
  rflib_sensor_tx_post();

  delay(1000);
}
