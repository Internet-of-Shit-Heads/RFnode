//#define DEBUG_HARD_OFF
#define CFG_SENSORS sensors_node2
#define CFG_CURRENT cfg_node2

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

// linear, roughly percent (0 = dry, 100 = in water)
const float humidity_poly[] = { 7500.0/39.0, -10.0/39.0 };
const struct sensor_info real_humidity = {
  .id = 2,
  .type = at_ac_tuwien_iot1718_N2C_SensorType_HUMIDITY,
  .pin = 3,
  .degree = 1,
  .poly = humidity_poly,
};

const struct sensor_info *const sensors_node0[] = { &emulated_humidity };
const struct sensor_info *const sensors_node1[] = { &emulated_temperature, &emulated_humidity };
const struct sensor_info *const sensors_node2[] = { &emulated_temperature, &real_humidity };
const struct sensor_info *const sensors_node3[] = { &emulated_temperature };

struct rfnode_config cfg_node0 = {
  /* operations */
  .sensors = sensors_node0,
  .room_number = 1,
  .node_id = 0,
  .wakeup_interval = 30,
  /* communications */
  .address = 0xF0F0F0F0E1LL,
  .channel = 0,
  .delay = 15,
  .retransmits = 15,
  /* security */
  .auth_key = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* hardware */
  .cepin = 9,
  .cspin = 10,
  .rtcpin = 2,
  .rtcint = 0,
  /* debug */
  .baud_rate = 57600,
  .debug = DEBUG_ALL,
};

struct rfnode_config cfg_node1 = {
  /* operations */
  .sensors = sensors_node1,
  .room_number = 1,
  .node_id = 1,
  .wakeup_interval = 30,
  /* communications */
  .address = 0xF0F0F0F0E1LL,
  .channel = 0,
  .delay = 15,
  .retransmits = 15,
  /* security */
  .auth_key = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* hardware */
  .cepin = 9,
  .cspin = 10,
  .rtcpin = 2,
  .rtcint = 0,
  /* debug */
  .baud_rate = 57600,
  .debug = DEBUG_ALL,
};

struct rfnode_config cfg_node2 = {
  /* operations */
  .sensors = sensors_node2,
  .room_number = 1,
  .node_id = 2,
  .wakeup_interval = 30,
  /* communications */
  .address = 0xF0F0F0F0E1LL,
  .channel = 0,
  .delay = 15,
  .retransmits = 15,
  /* security */
  .auth_key = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* hardware */
  .cepin = 9,
  .cspin = 10,
  .rtcpin = 2,
  .rtcint = 0,
  /* debug */
  .baud_rate = 57600,
  .debug = DEBUG_ALL,
};

struct rfnode_config cfg_node3 = {
  /* operations */
  .sensors = sensors_node3,
  .room_number = 1,
  .node_id = 3,
  .wakeup_interval = 30,
  /* communications */
  .address = 0xF0F0F0F0E1LL,
  .channel = 0,
  .delay = 15,
  .retransmits = 15,
  /* security */
  .auth_key = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  /* hardware */
  .cepin = 9,
  .cspin = 10,
  .rtcpin = 2,
  .rtcint = 0,
  /* debug */
  .baud_rate = 57600,
  .debug = DEBUG_ALL,
};

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
    rflib_sensor_tx_post();
    sleep_mode();
  }
}

static bool pre_send(rflib_msg_t *msg, uint16_t id, at_ac_tuwien_iot1718_N2C_SensorType type,
                     float data, uint32_t timestamp)
{
  at_ac_tuwien_iot1718_N2C msg_to_send = at_ac_tuwien_iot1718_N2C_init_zero;
  msg_to_send.timestamp = timestamp;
  msg_to_send.roomNo = CFG_CURRENT.room_number;
  msg_to_send.nodeId = CFG_CURRENT.node_id;
  msg_to_send.sensorId = id;
  msg_to_send.type = type;
  msg_to_send.data = data;

  pb_ostream_t stream = pb_ostream_from_buffer(msg->data, RFLIB_MAX_MSGSIZE);
  bool enc_res = pb_encode(&stream, at_ac_tuwien_iot1718_N2C_fields, &msg_to_send);
  msg->size = enc_res ? stream.bytes_written : 0;
  return enc_res;
}

static bool post_recv(rflib_msg_t *msg, at_ac_tuwien_iot1718_C2N *msg_to_recv)
{
  pb_istream_t stream = pb_istream_from_buffer(msg->data, msg->size);
  return pb_decode(&stream, at_ac_tuwien_iot1718_C2N_fields, msg_to_recv);
}

static RTC_DS3231 rtc;

void printHeader(const __FlashStringHelper *name, int number, bool *already_done)
{
  if (! *already_done) {
    Serial.print(name);
    Serial.print(F(" "));
    Serial.print(number);
    Serial.println(F(":"));
  }

  *already_done = true;
}

void setup(void)
{
  DO_DEBUG_AVAIL {
    Serial.begin(CFG_CURRENT.baud_rate);
  }
  
  if (rflib_sensor_init(CFG_CURRENT.cepin, CFG_CURRENT.cspin,
                        CFG_CURRENT.channel, CFG_CURRENT.address,
                        CFG_CURRENT.delay, CFG_CURRENT.retransmits) < 0) {
    DO_DEBUG(DEBUG_ERRORS) Serial.println(F("Init failed :("));
    abort();
  }

  rtc.begin();
  if (rtc.lostPower()) {
    DO_DEBUG(DEBUG_NOTICE) Serial.println(F("RTC lost power, resetting time!"));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime startup = rtc.now();
  randomSeed(startup.unixtime());

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_bod_disable();
  // enable the 1 Hz output
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  pinMode(CFG_CURRENT.rtcpin, INPUT_PULLUP);
  attachInterrupt(CFG_CURRENT.rtcint, rtc_interrupt, FALLING);

  DO_DEBUG_AVAIL {
    Serial.flush();
  }
}

void loop(void)
{
  static struct rflib_msg_t msgs[CFG_N_SENSORS];
  static uint32_t last_ts;
  DateTime now = rtc.now();

  int i;
  int encoded = 0;
  for (i = 0; i < CFG_N_SENSORS; i++) {
    bool sensor_hdr = false;
    float sensor_value;
    const struct sensor_info *sensor = CFG_CURRENT.sensors[i];
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

    DO_DEBUG(DEBUG_ALL) printHeader(F("Sensor"), sensor->id, &sensor_hdr);

    DO_DEBUG(DEBUG_ALL) {
      Serial.print(F("  value = "));
      Serial.println((int)real_value);
    }

    struct rflib_msg_t *msg = &msgs[encoded];
    if (!pre_send(msg, sensor->id, sensor->type, real_value, now.unixtime())) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("Sensor"), sensor->id, &sensor_hdr);
        Serial.println(F("  Encoding failed :("));
      }
      continue;
    }

    int authed_size = cryptlib_auth(msg->data, msg->size,
                                    RFLIB_MAX_MSGSIZE, CFG_CURRENT.auth_key);
    if (authed_size < 0) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("Sensor"), sensor->id, &sensor_hdr);
        Serial.println(F("  Auth failed :("));
      }
      continue;
    }
    msg->size = authed_size;

    encoded++;
  }

  rflib_sensor_tx_pre();
  int acks = 0;
  for (i = 0; i < encoded; i++) {
    bool msg_hdr = false;
    DO_DEBUG(DEBUG_ALL) printHeader(F("Message"), i, &msg_hdr);

    if (rflib_sensor_tx(&msgs[i], &msgs[acks]) < 0) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("Message"), i, &msg_hdr);
        Serial.println(F("  Send failed :("));
      }
      continue;
    }
    DO_DEBUG(DEBUG_ALL) Serial.println(F("  Sent :)"));
    
    if (msgs[acks].size == 0) {
      DO_DEBUG(DEBUG_NOTICE) {
        printHeader(F("Message"), i, &msg_hdr);
        Serial.println(F("  Empty ACK :/"));
      }
      continue;
    }
    
    DO_DEBUG(DEBUG_ALL) {
      Serial.print(F("  Got ACK with "));
      Serial.print(msgs[acks].size);
      Serial.println(F(" bytes :)"));
    }
    acks++;
  }

  /* no message sent (or none got through), try empty msg to get ACK */
  if (acks == 0) {
    memset(&msgs[0], 0, sizeof(msgs[0]));
    if (rflib_sensor_tx(&msgs[0], &msgs[0]) < 0) {
      DO_DEBUG(DEBUG_ERRORS) Serial.println(F("No messages got through, don't have an ACK. :("));
    } else {
      acks++;
    }
  }
  rflib_sensor_tx_post();

  uint32_t updated_ts = last_ts;
  for (i = 0; i < acks; i++) {
    bool ack_hdr = false;
    DO_DEBUG(DEBUG_ALL) printHeader(F("ACK"), i, &ack_hdr);

    struct rflib_msg_t *msg = &msgs[i];

    int verified_size = cryptlib_verify(msg->data, msg->size, CFG_CURRENT.auth_key);
    if (verified_size < 0) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("ACK"), i, &ack_hdr);
        Serial.println(F("  Verify failed :("));
      }
      continue;
    }
    msg->size = verified_size;

    at_ac_tuwien_iot1718_C2N decoded_ack;
    if (!post_recv(msg, &decoded_ack)) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("ACK"), i, &ack_hdr);
        Serial.println(F("  Decoding failed :("));
      }
      continue;
    }

    if (decoded_ack.timestamp <= last_ts) {
      DO_DEBUG(DEBUG_ERRORS) {
        printHeader(F("ACK"), i, &ack_hdr);
        Serial.println(F("  Stale timestamp :("));
      }
      continue;
    }

    if (decoded_ack.timestamp <= updated_ts) {
      DO_DEBUG(DEBUG_NOTICE) {
        printHeader(F("ACK"), i, &ack_hdr);
        Serial.println(F("  Duplicate ACK :/"));
      }
      continue;
    }

    updated_ts = decoded_ack.timestamp;
    DO_DEBUG(DEBUG_ALL) Serial.println(F("  accepted :)"));

    if (decoded_ack.has_command) {
      at_ac_tuwien_iot1718_Command *cmd = &decoded_ack.command;
      DO_DEBUG(DEBUG_NOTICE) {
        printHeader(F("ACK"), i, &ack_hdr);
        Serial.print(F("  Got "));
      }
      switch (cmd->type) {
      case at_ac_tuwien_iot1718_Command_CommandType_NEW_UPDATE_INTERVAL:
        DO_DEBUG(DEBUG_NOTICE) Serial.println(F("NEW_UPDATE_INTERVAL:"));
        if (!cmd->has_param1) {
          DO_DEBUG(DEBUG_ERRORS) {
            printHeader(F("ACK"), i, &ack_hdr);
            Serial.println(F("    Invalid command, no update interval specified :("));
          }
          break;
        }
        DO_DEBUG(DEBUG_NOTICE) {
          Serial.print(F("    New update interval "));
          Serial.print(cmd->param1);
          Serial.println(F(" :)"));
        }
        CFG_CURRENT.wakeup_interval = cmd->param1;
        break;
      default:
        DO_DEBUG(DEBUG_ERRORS) {
          printHeader(F("ACK"), i, &ack_hdr);
          Serial.print(F("unsupported command "));
          Serial.print(cmd->type);
          Serial.println(F(" :("));
        }
        break;
      }
    }
  }

  if (updated_ts != last_ts) {
    last_ts = updated_ts;
    now = DateTime(updated_ts);
    rtc.adjust(now);
    DO_DEBUG(DEBUG_ALL) {
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
  }

  DO_DEBUG_AVAIL {
    Serial.flush();
  }

  do_sleep(CFG_CURRENT.wakeup_interval);
}
