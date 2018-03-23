RFnode
======

A low power node that wakes up, reads values from a set of sensors, sends those
values to a coordinator node, receives commands from the coordinator node and
finally goes back to sleep for a predetermined interval.

Configuration
-------------

The RFnode's configurations is specified by the following struct:

	struct rfnode_config {
		/* operations */
		const struct sensor_info *const *sensors; 
		uint32_t room_number;
		uint32_t node_id;
		unsigned int wakeup_interval;
		/* communications */
		uint64_t address;
		uint8_t channel;
		uint8_t delay;
		uint8_t retransmits;
		/* security */
		const uint8_t auth_key[CRYPTLIB_KEY_SIZE];
		/* hardware */
		uint16_t cepin;
		uint16_t cspin;
		uint16_t rtcpin;
		uint16_t rtcint;
		/* debug */
		unsigned int baud_rate;
		debug_level debug;
	} __attribute__((packed));

The `sensors` array specifies the sensors connected to the node (see below).
`room_number and `node_id` are sent as information to the coordinator along with
each sensor value. The `wakeup_interval` determines the time (in seconds) a node
goes to sleep between reading the sensor values. This can be changed at run-time
via a command from the coordinator.

The `address`, `channel`, `delay` and `retransmits` values configure the
communication between RFnode and coordinator and should be identical on both.

The `auth_key` array sets the shared secret key the RFnode and coordinator use
for message authentication.

The `cepin` and `cspin` specify how the RF module is connected to the Arduino,
while the `rtcpin` and `rtcint` specify the pin the RTC clock is connected to
and the interrupt it triggers respectively.

Lastly, `baud_rate` and `debug` determine the configuration and verbosity of the
serial debugging facilities.

Sensors
-------

A sensor is specified by the following struct:

	struct sensor_info {
		uint16_t id;
		at_ac_tuwien_iot1718_N2C_SensorType type;
		uint8_t pin;
		uint8_t degree;
		const float *poly;
	} __attribute__((packed));

The `id` value identifies the sensor, while the `type` specifies what kind of
data the sensor reads. The `pin` specifies which analog pin is read to obtain
the raw sensor value. A value of 255 will instead cause the node to use a random
value between 0 and 1023.

The `degree` and `poly` values specify a polynomial with degree `degree` that is
applied to the raw sensor value before sending it to the coordinator. The `poly`
array contains the coefficients of the polynomial from lowest to highest degree.

Commands
--------

Right now the RFnode only supports a command to set a new time interval between
the bursts of activity described above.

Bugs
----

The DS3221 RTC clock has a time of day alarm but the RTC library we used does
not support it. Hence, currently the RFnode will wake up every second and then
immediately go back to sleep if the interval has not elapsed yet.
