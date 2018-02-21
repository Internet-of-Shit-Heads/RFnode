#include <sensordata.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <rflib.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include "RTClib.h"
#include <avr/sleep.h>
#include <cryptlib.h>
#include <cfglib.h>

const struct sensor_info emulated_temperature = {
  .id = 0,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_TEMPERATURE,
  .pin = 255,
  .degree = 0,
  .poly = NULL,
};

const struct sensor_info emulated_humidity = {
  .id = 1,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_HUMIDITY,
  .pin = 255,
  .degree = 0,
  .poly = NULL,
};

// linear, needs calibration
const float humidity_poly[] = { 0, 1 };
const struct sensor_info real_humidity = {
  .id = 2,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_HUMIDITY,
  .pin = 3,
  .degree = 1,
  .poly = humidity_poly,
};

const struct sensor_info *const sensors[] = { &emulated_temperature , &emulated_humidity, &real_humidity };
const struct rfnode_config node_config = {
  .address = 0xF0F0F0F0E1LL,
  .room_number = 7,
  .node_id = 1,
  .wakeup_interval = 1,
  .baud_rate = 57600,
  .cepin = 9,
  .cspin = 10,
  .channel = 0,
  .delay = 15,
  .retransmits = 15,
  .auth_key = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  .sensors = sensors,
};

#define N_SENSORS (sizeof(sensors) / sizeof(struct sensor_info *))

static float calc_poly(const float *coeff, uint8_t degree, float x)
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

static bool pre_send(rflib_msg_t *msg, uint16_t id, at_ac_tuwien_iot1718_N2C_SensorType type,
                     float data, uint32_t timestamp)
{
  at_ac_tuwien_iot1718_N2C msg_to_send = at_ac_tuwien_iot1718_N2C_init_zero;
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
  at_ac_tuwien_iot1718_C2N msg_to_recv;
  pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->size);
  bool dec_res = pb_decode(&stream, at_ac_tuwien_iot1718_C2N_fields, &msg_to_recv);
  *timestamp = dec_res ? msg_to_recv.timestamp : 0;
  return dec_res;
}

static RTC_DS3231 rtc;

void setup(void)
{
  Serial.begin(node_config.baud_rate);
  
  if (rflib_sensor_init(node_config.cepin, node_config.cspin,
                        node_config.channel, node_config.address,
                        node_config.delay, node_config.retransmits) < 0) {
    Serial.println(F("Init failed :("));
    abort();
  }

  rtc.begin();
  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, resetting time!"));
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
  
  Serial.flush();
}

void loop(void)
{
  static struct rflib_msg_t msgs[N_SENSORS];
  static uint32_t last_ts;
  DateTime now = rtc.now();

  int i;
  int encoded = 0;
  for (i = 0; i < N_SENSORS; i++) {
    float sensor_value;
    const struct sensor_info *sensor = node_config.sensors[i];
    if (sensor->pin == 255) {
      sensor_value = random(0, 1023);
    } else {
      sensor_value = analogRead(sensor->pin);
    }

    float real_value;
    if (sensor->poly == NULL) {
      real_value = sensor_value;
    } else {
      real_value = calc_poly(sensor->poly, sensor->degree, sensor_value);
    }

    Serial.print(F("Sensor "));
    Serial.print(sensor->id);
    Serial.println(F(":"));

    Serial.print(F("  value = "));
    Serial.println((int)real_value);

    struct rflib_msg_t *msg = &msgs[encoded];
    if (!pre_send(msg, sensor->id, sensor->type, real_value, now.unixtime())) {
      Serial.println(F("  Encoding failed :("));
      continue;
    }

    int authed_size = cryptlib_auth(msg->data, msg->size,
                                    RFLIB_MAX_MSGSIZE, node_config.auth_key);
    if (authed_size < 0) {
      Serial.println(F("  Auth failed :("));
      continue;
    }
    msg->size = authed_size;

    encoded++;
  }

  rflib_sensor_tx_pre();
  int acks = 0;
  for (i = 0; i < encoded; i++) {
    Serial.print(F("Message "));
    Serial.print(i);
    Serial.println(F(":"));
    
    if (rflib_sensor_tx(&msgs[i], &msgs[acks]) < 0) {
      Serial.println(F("  Send failed :("));
      continue;
    }
    Serial.println(F("  Sent :)"));
    
    if (msgs[acks].size == 0) {
      Serial.println(F("  No ACK :("));
      continue;
    }
    
    Serial.print(F("  Got ACK with "));
    Serial.print(msgs[acks].size);
    Serial.println(F(" bytes."));
    acks++;
  }

  /* no message sent (or none got through), try empty msg to get ACK */
  if (acks == 0) {
    memset(&msgs[0], 0, sizeof(msgs[0]));
    if (rflib_sensor_tx(&msgs[0], &msgs[0]) < 0) {
      Serial.println(F("No messages got through :(, don't have an ACK."));
    } else {
      acks++;
    }
  }
  rflib_sensor_tx_post();

  uint32_t updated_ts = last_ts;
  for (i = 0; i < acks; i++) {
    Serial.print(F("ACK "));
    Serial.print(i);
    Serial.println(F(":"));

    struct rflib_msg_t *msg = &msgs[i];

    int verified_size = cryptlib_verify(msg->data, msg->size, node_config.auth_key);
    if (verified_size < 0) {
      Serial.println(F("  Verify failed :("));
      continue;
    }
    msg->size = verified_size;

    uint32_t ack_ts;
    if (!post_recv(msg, &ack_ts)) {
      Serial.println(F("  Decoding failed :("));
      continue;
    }

    if (ack_ts <= last_ts) {
      Serial.println(F("  Stale timestamp :("));
      continue;
    }

    if (ack_ts > updated_ts) {
      Serial.println(F("  accepted :)"));
      updated_ts = ack_ts;
      
      /* TODO: apply command */
    }
  }

  if (updated_ts != last_ts) {
    now = DateTime(updated_ts);
    rtc.adjust(now);
    Serial.print(F("New time: "));
    Serial.print(now.year());
    Serial.print(F("/"));
    Serial.print(now.month());
    Serial.print(F("/"));
    Serial.print(now.day());
    Serial.print(F(" "));
    Serial.print(now.hour());
    Serial.print(F(":"));
    Serial.print(now.minute());
    Serial.print(F(":"));
    Serial.println(now.second());
  }

  Serial.flush();

  do_sleep(node_config.wakeup_interval);
}
