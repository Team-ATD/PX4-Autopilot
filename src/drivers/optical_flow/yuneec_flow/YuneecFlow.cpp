/****************************************************************************
 *
 *   Copyright (c) 2016-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "YuneecFlow.hpp"

int static cmp(const void *a, const void *b)
{
	return (*(const float *)a > *(const float *)b);
}

FLOW::FLOW(const char *port):
	CDev("FLOW", FLOW_DEVICE_PATH),
	_initialized(false),
	_taskShouldExit(false),
	_taskIsRunning(false),
	_gnd_dist_valid(false),
	_measure_ticks(USEC2TICK(CONVERSION_INTERVAL_FLOW)),
	_work{},
	_min_valid_range(0.f),
	_max_valid_range(0.f),
	_mf_cycle_counter(0),
	_var_cycle_couter(0),
	_rng_noise_var_threshold(0.015f),
	_filter_value(0.f),
	_last_value(0.f),
	_valid_orign_distance(0.f),
	_data_abnormal_time(0),
	_keep_valid_time(0),
	_flow_quality_cycle_counter{0},
	_last_flow_quality_value{0},
	_last_flow_quality_sample_timestamp{0},
	_flow_pub(nullptr),
	_flow_distance_sensor_pub(nullptr),
#ifdef BOARD_OF_USE_FLTCTL_GYRO
	_sensor_combined_sub(-1),
	_rb_gyro(nullptr),
#endif
	_flow_delay_ms(0.f)
{
	/* store port name */
	strncpy(_device_flow, port, sizeof(_device_flow));

	/* enforce null termination */
	_device_flow[sizeof(_device_flow) - 1] = '\0';

	/* get min and max valid range for the range finder */
	param_t min_valid_range_p = param_find("EKF2_MIN_RNG");
	param_t max_valid_range_p = param_find("EKF2_RNG_A_HMAX");

	if (min_valid_range_p == PARAM_INVALID || param_get(min_valid_range_p, &_min_valid_range) != PX4_OK) {
		_min_valid_range = MIN_VALID_RANGE_DEFAULT;
	}

	if (max_valid_range_p == PARAM_INVALID || param_get(max_valid_range_p, &_max_valid_range) != PX4_OK) {
		_max_valid_range = MAX_VALID_RANGE_DEFAULT;
	}

	param_get(param_find("EKF2_OF_DELAY"), &_flow_delay_ms);
}

FLOW::~FLOW()
{
	if (_initialized) {
		/* tell the task we want it to go away */
		_initialized = false;
	}

#ifdef BOARD_OF_USE_FLTCTL_GYRO

	if (_rb_gyro != nullptr) {
		delete _rb_gyro;
	}

#endif

	work_cancel(HPWORK, &_work);

	_taskShouldExit = true;
	::close(_uart_fd);
}

void FLOW::start()
{
	/* reset the report ring and state machine */
#ifdef BOARD_OF_USE_FLTCTL_GYRO
	_rb_gyro->flush();
#endif

	_taskShouldExit = false;
	usleep(1000000);

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&FLOW::_cycle_trampoline, this, 0);
}

int FLOW::init_dev()
{
	int ret = CDev::init();

	if (ret != OK) {
		PX4_ERR("CDev init failed");
		goto out;
	}

#ifdef BOARD_OF_USE_FLTCTL_GYRO

	_rb_gyro = new ringbuffer::RingBuffer(12, sizeof(vector3f_timestamped));

	if (_rb_gyro == nullptr) {
		PX4_ERR("could not allocate gyro ringbuffer");
		return PX4_ERROR;
	}

#endif

	ret = OK;

out:
	return ret;
}

void FLOW::stop()
{
	_taskShouldExit = true;
}

void FLOW::_cycle_trampoline(void *arg)
{
	FLOW *dev = reinterpret_cast<FLOW *>(arg);
	dev->_cycle_flow();
}

void FLOW::poll_subscriptions()
{

	_measure_ticks = USEC2TICK(CONVERSION_INTERVAL_FLOW);

#ifdef BOARD_OF_USE_FLTCTL_GYRO
	bool updated = false;
	orb_check(_sensor_combined_sub, &updated);

	if (updated) {
		struct sensor_combined_s sensor_combined;
		vector3f_timestamped gyro_stamped;
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_combined);
		// write gyro values into ring buffer for optical flow
		gyro_stamped.time_usec = sensor_combined.timestamp;
		gyro_stamped.x = sensor_combined.gyro_rad[0];
		gyro_stamped.y = sensor_combined.gyro_rad[1];
		gyro_stamped.z = sensor_combined.gyro_rad[2];

		_rb_gyro->force(&gyro_stamped);
	}

#endif
}

void FLOW::_read_flow_data()
{
	static uint8_t rcs_buf[64] = {};
	mavlink_message_t msg = {};
	mavlink_status_t status = {};

	/* read all available data from the serial port */
	int newBytes = ::read(_uart_fd, &rcs_buf[0], sizeof(rcs_buf));

	for (int i = 0; i < newBytes; i++) {
		if (mavlink_parse_char(FLOW_CHAN, rcs_buf[i], &msg, &status)) {
			if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD) {
				_handle_message_optical_flow_rad(&msg);
			}
		}
	}
}

void FLOW::_handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	mavlink_optical_flow_rad_t flow = {};
	struct optical_flow_s optical_flow = {};

	/* optical flow */
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

#ifdef BOARD_OF_USE_FLTCTL_GYRO
	/* integrate gyro values */
	vector3f gyro_integrated{};
	vector3f_timestamped gyro;
	hrt_abstime time_now_us = hrt_absolute_time();
	static hrt_abstime last_imu_time = 0;
	float dt = 0.0f;

	/* integrate and account for flow delay */
	do {
		if (!_rb_gyro->get(&gyro)) {
			break;
		}

		dt = gyro.time_usec - last_imu_time;
		last_imu_time = gyro.time_usec;

		if (last_imu_time == 0) {
			PX4_WARN("optical flow: gyro time stamp equals 0");

		} else {
			gyro_integrated.x += gyro.x * dt;
			gyro_integrated.y += gyro.y * dt;
			gyro_integrated.z += gyro.z * dt;
		}
	} while (gyro.time_usec < (time_now_us - (hrt_abstime)(_flow_delay_ms * 1000.f)));

	optical_flow.timestamp = time_now_us;
#else
	optical_flow.timestamp = hrt_absolute_time();
#endif

	if (flow.distance < FLT_EPSILON) {
		flow.distance =	OUT_OF_RANGE;
	}

	optical_flow.integration_timespan = flow.integration_time_us;
	optical_flow.pixel_flow_x_integral = -flow.integrated_x;
	optical_flow.pixel_flow_y_integral = -flow.integrated_y;

#ifdef BOARD_OF_USE_FLTCTL_GYRO
	optical_flow.gyro_x_rate_integral = gyro_integrated.x * (float)1e-6;
	optical_flow.gyro_y_rate_integral = gyro_integrated.y * (float)1e-6;
	optical_flow.gyro_z_rate_integral = gyro_integrated.z * (float)1e-6;
#else
	optical_flow.gyro_x_rate_integral = -flow.integrated_xgyro;
	optical_flow.gyro_y_rate_integral = -flow.integrated_ygyro;
	optical_flow.gyro_z_rate_integral = flow.integrated_zgyro;
#endif

	optical_flow.time_since_last_sonar_update = flow.time_delta_distance_us;
	optical_flow.ground_distance_m = flow.distance;
	optical_flow.quality = _flow_quality_filter(flow.quality, FLOW_QUALITY_SAMPLE_INTERVAL_TIME_MS);
	optical_flow.sensor_id = flow.sensor_id;
	optical_flow.gyro_temperature = flow.temperature;
	optical_flow.max_flow_rate = MAX_FLOW_RATE;
	optical_flow.min_ground_distance = _min_valid_range;
	optical_flow.max_ground_distance = _max_valid_range;

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &optical_flow);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &optical_flow);
	}

	/* use distance value for distance sensor topic */
	struct distance_sensor_s ground_distance = {};

	if (flow.distance >= -FLT_EPSILON) {	// negative values signal invalid data
		ground_distance.timestamp = hrt_absolute_time();
		ground_distance.min_distance = _min_valid_range;	// according to the sensor to determine
		ground_distance.max_distance = _max_valid_range;
		ground_distance.variance = _calc_variance(flow.distance);	// units are m^2
		ground_distance.current_distance = _handle_abnormal_data(_valid_orign_distance,
						   ground_distance.variance);	// units are m and m^2
		ground_distance.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		ground_distance.id = 0;
		ground_distance.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		ground_distance.signal_quality = -1;
		// ground_distance.valid = _gnd_dist_valid;

		/* there are 2 sonars -> use multi */
		int _orb_class_instance = -1;

		if (_flow_distance_sensor_pub == nullptr) {
			_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &ground_distance, &_orb_class_instance,
						    ORB_PRIO_LOW);

		} else {
			orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &ground_distance);
		}
	}
}

float FLOW::_handle_abnormal_data(float value, float noise_variance)
{
	/* the noise variance is too large and the data is abnormal */
	if (noise_variance > _rng_noise_var_threshold || value < _min_valid_range || value > _max_valid_range) {
		_filter_value = OUT_OF_RANGE;

		/* setting ranger finder data accuracy is invalid */
		_gnd_dist_valid = false;
		_data_abnormal_time = hrt_absolute_time();

	} else {
		_filter_value = _median_filter(value);

		/* after the data is abnormal, must wait a few seconds(RNG_RECOVERY_TIMEOUT) to recover */
		if (hrt_elapsed_time(&_data_abnormal_time) > RNG_RECOVERY_TIMEOUT) {
			_gnd_dist_valid = true;
		}
	}

	return _filter_value;
}

float FLOW::_calc_variance(float value)
{
	float accum = 0.f;
	float variance_sum = 0.f;

	/* this is an unusual special case handling,
	 * because ultrasound does not receive echoes in many cases, so we ignore the data
	 */
	if (value >= OUT_OF_RANGE && ((value - _last_value) > 1.f) &&
	    (hrt_elapsed_time(&_keep_valid_time) < RNG_DATA_ABNORMAL_TIMEOUT)) { // add abnormal timeout handle
		value = _last_value;
		_valid_orign_distance = _last_value;

	} else {
		_valid_orign_distance = value;
		_keep_valid_time = hrt_absolute_time();
	}

	_last_value = value;

	_variance_window[(_var_cycle_couter + 1) % _VAR_WINDOW_SIZE] = value;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		variance_sum += _variance_window[i];
	}

	_var_cycle_couter++;

	float average = variance_sum / _VAR_WINDOW_SIZE;

	for (int i = 0; i < _VAR_WINDOW_SIZE; i++) {
		accum  += (_variance_window[i] - average) * (_variance_window[i] - average);
	}

	return (accum / (_VAR_WINDOW_SIZE - 1));
}

float FLOW::_median_filter(float value)
{
	/* TODO: replace with ring buffer */
	_mf_window[(_mf_cycle_counter + 1) % _MF_WINDOW_SIZE] = value;

	for (int i = 0; i < _MF_WINDOW_SIZE; ++i) {
		_mf_window_sorted[i] = _mf_window[i];
	}

	qsort(_mf_window_sorted, _MF_WINDOW_SIZE, sizeof(float), cmp);

	_mf_cycle_counter++;

	return _mf_window_sorted[_MF_WINDOW_SIZE / 2];
}

uint8_t FLOW::_flow_quality_filter(uint8_t value, uint16_t interval_time_ms)
{
	uint8_t filter_value = _last_flow_quality_value;
	uint8_t window_size = FLOW_QUALITY_WINDOW_SIZE;

	if (hrt_elapsed_time(&_last_flow_quality_sample_timestamp) >= interval_time_ms * 1000) {
		_flow_quality_window[_flow_quality_cycle_counter % window_size] = value;

		uint16_t window_sum = 0;

		for (int i = 0; i < window_size; ++i) {
			window_sum += _flow_quality_window[i];
		}

		filter_value = window_sum / window_size;
		_last_flow_quality_value = filter_value;
		_last_flow_quality_sample_timestamp = hrt_absolute_time();
		_flow_quality_cycle_counter++;
	}

	return filter_value;
}

bool FLOW::_init_flow()	// init - initialise the sensor
{
	if (!_initialized) {

#ifdef BOARD_OF_USE_FLTCTL_GYRO
		_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
#endif

		_initialized = true;
	}

	return true;
}

void FLOW::update()	// update - check input and send out data
{
	if (_uart_fd < 0) {
		/* we need to initialise the UART here because the file descriptor needs to be initialized in the correct task context */
		int ret = _initialise_uart(_device_flow);

		if (ret < 0) {
			PX4_ERR("initialise uart failed");
			return;
		}
	}

	_read_flow_data();
}

void FLOW::_cycle_flow()
{
	if (!_initialized) {
		_init_flow();
		_taskIsRunning = true;
	}

	poll_subscriptions();
	update();

	if (!_taskShouldExit) {
		/* schedule next cycle */
		work_queue(HPWORK, &_work, (worker_t)&FLOW::_cycle_trampoline, this,
			   _measure_ticks);

	} else {
		_taskIsRunning = false;
	}
}

int FLOW::_initialise_uart(const char *device)
{
	/* open uart */
	_uart_fd = ::open(device, O_RDWR | O_NONBLOCK | O_NOCTTY);

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return PX4_ERROR;
	}

	struct termios uart_config;

	int termios_state = -1;

	/* fill the struct for the new configuration */
	if (tcgetattr(_uart_fd, &uart_config)) {
		PX4_ERR("error getting uart configuration");
		return PX4_ERROR;
	}

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* clear CRTSCTS flag (which enables flow-control)
	 * otherwise, it may cause failure to write to the optical flow board
	 */
	uart_config.c_cflag &= ~CRTSCTS;

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, B500000)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	if ((termios_state = cfsetospeed(&uart_config, B500000)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		::close(_uart_fd);
		return PX4_ERROR;
	}

	return PX4_OK;
}

namespace flow
{
// driver 'main' command
/*
 * flow start / stop handling function
 * This makes the flow drivers accessible from the nuttx shell
 * @ingroup apps
 */
extern "C" __EXPORT int yuneec_flow_main(int argc, char *argv[]);

// Private variables
static FLOW *flow_drv_task = nullptr;
static char _device[20] {};
void usage();
static void flow_stop(void);

static void flow_stop()
{
	if (flow_drv_task == nullptr) {
		PX4_WARN("not running");
		return;
	}

	flow_drv_task->stop();

	/* wait for task to die */
	int i = 0;

	do {
		/* wait 2000ms at a time */
		usleep(2000000);
	} while (flow_drv_task->is_running() && ++i < 50);


	delete flow_drv_task;
	flow_drv_task = nullptr;

	PX4_WARN("stopped");
}

void usage()
{
	PX4_INFO("usage: yuneec_flow start -d /dev/ttyS2");
	PX4_INFO("       yuneec_flow stop");
	PX4_INFO("       yuneec_flow status");
}

int yuneec_flow_main(int argc, char *argv[])
{
	const char *device = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];
	}

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			strncpy(_device, device, strlen(device));
			break;
		}
	}

	/* start/load the driver */
	if (!strcmp(verb, "start")) {
		if (flow_drv_task != nullptr) {
			PX4_WARN("flow already running");
			return 1;
		}

		/* check on required arguments */
		if (device == nullptr || strlen(device) == 0) {
			usage();
			return 1;
		}

		if (flow_drv_task == nullptr) {
			flow_drv_task = new FLOW(_device);

			if (flow_drv_task == nullptr) {
				PX4_ERR("failed to start flow.");
				return -1;
			}
		}

		if (flow_drv_task->init_dev() != OK) {
			PX4_ERR("failed to initialise flow.");
			return -1;
		}

		flow_drv_task->start();

	} else if (!strcmp(verb, "stop")) {
		if (flow_drv_task == nullptr) {
			PX4_WARN("flow is not running");
			return 1;
		}

		flow_stop();

	} else if (!strcmp(verb, "status")) {
		PX4_WARN("flow is %s", flow_drv_task != nullptr ? "running" : "not running");
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}

} // namespace flow
