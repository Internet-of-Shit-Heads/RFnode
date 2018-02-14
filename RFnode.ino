#include <sensordata.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <rflib.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Wire.h>
#include "RTClib.h"
#include <avr/sleep.h>

static struct {
  uint64_t address = 0xF0F0F0F0E1LL;
  uint32_t room_number = 7;
  unsigned int wakeup_interval = 1;
  at_ac_tuwien_iot1718_N2C_SensorType sensor_type = at_ac_tuwien_iot1718_N2C_SensorType_TEMPERATURE;
  unsigned int baud_rate = 57600;
  uint16_t cepin = 9;
  uint16_t cspin = 10;
  uint8_t channel = 0;
  uint8_t delay = 15;
  uint8_t retransmits = 15;
} node_config;

void rtc_interrupt(void) {}

static void do_sleep(unsigned int seconds_to_sleep)
{
  unsigned int seconds_slept;
  for (seconds_slept = 0; seconds_slept < seconds_to_sleep; seconds_slept++) {
    sleep_mode();
  }
}

static RTC_DS3231 rtc;

static bool pre_send(rflib_msg_t *msg, int32_t data)
{
  static at_ac_tuwien_iot1718_N2C msg_to_send = at_ac_tuwien_iot1718_N2C_init_zero;
  msg_to_send.roomNo = node_config.room_number;
  msg_to_send.type = node_config.sensor_type;
  msg_to_send.data = data;

  static pb_ostream_t stream = pb_ostream_from_buffer(msg->data, RFLIB_MAX_MSGSIZE);
  bool enc_res = pb_encode(&stream, at_ac_tuwien_iot1718_N2C_fields, &msg_to_send);
  msg->size = enc_res ? stream.bytes_written : 0;
  return enc_res;
}

static bool post_recv(rflib_msg_t *msg, uint32_t *timestamp)
{
  static at_ac_tuwien_iot1718_C2N msg_to_recv;
  static pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->size);
  bool dec_res = pb_decode(&stream, at_ac_tuwien_iot1718_C2N_fields, &msg_to_recv);
  *timestamp = dec_res ? msg_to_recv.timestamp : 0;
  return dec_res;
}

void setup(void)
{
  Serial.begin(node_config.baud_rate);
  printf_begin();
  
  if (rflib_sensor_init(node_config.cepin, node_config.cspin,
                        node_config.channel, node_config.address,
                        node_config.delay, node_config.retransmits) < 0) {
    printf("Init failed :(\n\r");
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, resetting time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_bod_disable();
  // enable the 1 Hz output
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, rtc_interrupt, FALLING);
}

void loop(void)
{
  static struct rflib_msg_t msg;
  static struct rflib_msg_t ackmsg;

  /* TODO */
  if (!pre_send(&msg, millis())) {
    printf("Encoding failed :(\n\r");
  }
  
  rflib_sensor_tx_pre();
  if (rflib_sensor_tx(&msg, &ackmsg) < 0) {
    printf("Send failed :(\n\r");
  } else {
    printf("Sent :), acklen = %d, ackmsg = \"%s\"\n\r", ackmsg.size, ackmsg.data);
  }
  rflib_sensor_tx_post();

  uint32_t updated_ts;
  if (!post_recv(&ackmsg, &updated_ts)) {
    printf("Decoding failed :(\n\r");
  } else {
    rtc.adjust(DateTime(updated_ts));
  }

  Serial.flush();

  do_sleep(node_config.wakeup_interval);
}
