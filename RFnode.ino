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

struct sensor_info {
  uint16_t id;
  at_ac_tuwien_iot1718_N2C_SensorType type;
  uint8_t pin;
  uint8_t degree;
  float *poly;
  bool emulated;
  float min;
  float max;
};

static struct sensor_info emulated_temperature = {
  .id = 0,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_TEMPERATURE,
  .pin = 255,
  .degree = 0,
  .poly = NULL,
  .emulated = true,
  .min = -20,
  .max = 40
};

static struct sensor_info emulated_humidity = {
  .id = 1,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_HUMIDITY,
  .pin = 255,
  .degree = 0,
  .poly = NULL,
  .emulated = true,
  .min = 0,
  .max = 100
};

// linear, needs calibration
static float humidity_poly[] = { 0, 1 };
static struct sensor_info real_humidity = {
  .id = 2,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_HUMIDITY,
  .pin = 2,
  .degree = 1,
  .poly = humidity_poly,
};

#define N_SENSORS  3

static struct {
  uint64_t address = 0xF0F0F0F0E1LL;
  uint32_t room_number = 7;
  uint32_t node_id = 1;
  unsigned int wakeup_interval = 1;
  unsigned int baud_rate = 57600;
  uint16_t cepin = 9;
  uint16_t cspin = 10;
  uint8_t channel = 0;
  uint8_t delay = 15;
  uint8_t retransmits = 15;
  struct sensor_info sensors[N_SENSORS] = { emulated_temperature, emulated_humidity, real_humidity };
} node_config;

static float calc_poly(float *coeff, uint8_t degree, float x)
{
  float y = 0;
  float xi = 1;
  
  int i;
  for (i = 0; i < degree + 1; i++) {
    y += xi * coeff[i];
    xi *= x;
  }

  return y;
}

void rtc_interrupt(void) {}

static void do_sleep(unsigned int seconds_to_sleep)
{
  unsigned int seconds_slept;
  for (seconds_slept = 0; seconds_slept < seconds_to_sleep; seconds_slept++) {
    sleep_mode();
  }
}

static RTC_DS3231 rtc;

static bool pre_send(rflib_msg_t *msg, uint16_t id, at_ac_tuwien_iot1718_N2C_SensorType type,
                     float data, uint32_t timestamp)
{
  static at_ac_tuwien_iot1718_N2C msg_to_send = at_ac_tuwien_iot1718_N2C_init_zero;
  msg_to_send.timestamp = timestamp;
  msg_to_send.roomNo = node_config.room_number;
  msg_to_send.nodeId = node_config.node_id;
  msg_to_send.sensorId = id;
  msg_to_send.type = type;
  msg_to_send.data = data;

  pb_ostream_t stream = pb_ostream_from_buffer(msg->data, RFLIB_MAX_MSGSIZE);
  bool enc_res = pb_encode(&stream, at_ac_tuwien_iot1718_N2C_fields, &msg_to_send);
  msg->size = enc_res ? stream.bytes_written : 0;
  return enc_res;
}

static bool post_recv(rflib_msg_t *msg, uint32_t *timestamp)
{
  static at_ac_tuwien_iot1718_C2N msg_to_recv;
  pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->size);
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

  rtc.begin();
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, resetting time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime startup = rtc.now();
  randomSeed(startup.unixtime());

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_bod_disable();
  // enable the 1 Hz output
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, rtc_interrupt, FALLING);
}

void loop(void)
{
  static struct rflib_msg_t msgs[N_SENSORS];
  static struct rflib_msg_t emptymsg = { .size = 0 };
  static struct rflib_msg_t ackmsgs[N_SENSORS + 1];
  static uint32_t last_ts;
  DateTime now = rtc.now();

  int i;
  int encoded = 0;
  for (i = 0; i < N_SENSORS; i++) {
    float sensor_value;
    struct sensor_info *sensor = &node_config.sensors[i];
    if (sensor->emulated) {
      sensor_value = random(sensor->min, sensor->max);
    } else {
      sensor_value = analogRead(sensor->pin);
    }

    float real_value;
    if (sensor->poly == NULL) {
      real_value = sensor_value;
    } else {
      real_value = calc_poly(sensor->poly, sensor->degree, sensor_value);
    }

    /* printf doesn't support floats, so we cast to int */
    printf("Sensor %d: value = %d\n\r", sensor->id, (int) real_value);

    if (!pre_send(&msgs[encoded], sensor->id, sensor->type, real_value, now.unixtime())) {
      printf("Sensor %d: Encoding failed :(\n\r", sensor->id);
      continue;
    }

    encoded++;
  }

  rflib_sensor_tx_pre();
  int acks = 0;
  for (i = 0; i < encoded; i++) {
    if (rflib_sensor_tx(&msgs[i], &ackmsgs[acks]) < 0) {
      printf("Message %d: Send failed :(\n\r", i);
      continue;
    }
    printf("Message %d: Sent :)\n\r", i);
    
    if (ackmsgs[acks].size == 0) {
      printf("Message %d: No ACK :(\n\r", i);
      continue;
    }
    
    printf("Message %d: Got ACK with %d bytes.\n\r", i, ackmsgs[acks].size);
    acks++;
  }

  /* no message sent (or none got through), try empty msg to get ACK */
  if (acks == 0) {
    if (rflib_sensor_tx(&emptymsg, &ackmsgs[acks]) < 0) {
      printf("No messages got through :(, don't have an ACK.\n\r");
    } else {
      acks++;
    }
  }
  rflib_sensor_tx_post();

  uint32_t updated_ts = last_ts;
  for (i = 0; i < acks; i++) {
    uint32_t ack_ts;
    if (!post_recv(&ackmsgs[i], &ack_ts)) {
      printf("ACK %d: Decoding failed :(\n\r", i);
      continue;
    }

    if (ack_ts <= last_ts) {
      printf("ACK %d: Stale timestamp :(\n\r", i);
      continue;
    }

    if (ack_ts > updated_ts) {
      updated_ts = ack_ts;
      
      /* TODO: apply command */
    }
  }

  if (updated_ts != last_ts) {
    now = DateTime(updated_ts);
    rtc.adjust(now);
    printf("New time: %04d/%02d/%02d %02d:%02d:%02d\n",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  }

  Serial.flush();

  do_sleep(node_config.wakeup_interval);
}
