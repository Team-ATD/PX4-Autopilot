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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/workqueue.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>

#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>	// NAN
#include <math.h>
#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/device/device.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/systemlib/mavlink_log.h>

#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_combined.h>

#include <v2.0/common/mavlink.h>
#include <v2.0/mavlink_types.h>

#define BOARD_OF_USE_FLTCTL_GYRO

#ifdef BOARD_OF_USE_FLTCTL_GYRO
#define CONVERSION_INTERVAL_FLOW (1000000 / 1000)	/* microseconds, the measurement rate is 1kHz, it should be faster than the gyro update(3~10ms) */
#else
#define CONVERSION_INTERVAL_FLOW (1000000 / 100)	/* microseconds, the measurement rate is 100Hz */
#endif

#define UPGRADE_INTERVAL_FLOW (1000000 / 1000)		/* microseconds, the image upgrade rate is 1KHz */
#define FLOW_DEVICE_PATH "/dev/flow"
#define FLOW_CHAN 1
#define _MF_WINDOW_SIZE 4		/* window size for median filter on sonar range */
#define _VAR_WINDOW_SIZE 10		/* window size for calc variance on sonar range */

#define MIN_VALID_RANGE_DEFAULT 0.3f	/* minimum valid distance sensor reading in m according to sonar dead zone  */
#define MAX_VALID_RANGE_DEFAULT 5.0f	/* maximum valid distance sensor reading in m */
#define OUT_OF_RANGE 7.0f				/* set distance sensor reading to this value when out of range or invalid */
#define RNG_RECOVERY_TIMEOUT (1 * 1000 * 1000)			/**< wait range finder accuracy recover*/
#define RNG_DATA_ABNORMAL_TIMEOUT (0.5 * 1000 * 1000)	/**< range finder data abnormal timeout */
#define FLOW_QUALITY_WINDOW_SIZE 10						/**<  window size for optical flow quality filter */
#define FLOW_QUALITY_SAMPLE_INTERVAL_TIME_MS 100		/**< sample interval time for optical flow quality filter */
#define MAX_FLOW_RATE 7.4f

#ifdef BOARD_OF_USE_FLTCTL_GYRO
// Single precision floating point 3-element vector.
typedef union __attribute__((packed))
{
	struct {
		float x, y, z;
	};
	float elem[3];
} vector3f;

// Single precision floating point 3-element vector with time stamp.
typedef union __attribute__((packed))
{
	struct {
		float x, y, z;
		hrt_abstime time_usec;
	};
	float elem[4];
} vector3f_timestamped;
#endif

class FLOW: public device::CDev
{
public:
	FLOW(const char *port);
	~FLOW();

	void start();
	void stop();
	virtual int init_dev();

	/**
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _taskIsRunning;
	}

private:
	bool _initialized;
	bool _taskShouldExit;
	bool _taskIsRunning;
	bool _check_optical_upgrade_enable;
	bool _gnd_dist_valid;
	char _device_flow[20];
	unsigned int _measure_ticks;

	struct work_s	_work;
	int _uart_fd = -1;

	float _min_valid_range;
	float _max_valid_range;

	int _mf_cycle_counter;
	int _var_cycle_couter;
	float _rng_noise_var_threshold;	/**< Ultrasonic range noise variance threshold */
	float _filter_value;
	float _last_value;
	float _valid_orign_distance;
	float _mf_window[_MF_WINDOW_SIZE] = {};
	float _mf_window_sorted[_MF_WINDOW_SIZE] = {};
	float _variance_window[_VAR_WINDOW_SIZE] = {};
	hrt_abstime _data_abnormal_time;
	hrt_abstime _keep_valid_time;

	uint8_t _flow_quality_window[FLOW_QUALITY_WINDOW_SIZE] = {};
	uint8_t _flow_quality_cycle_counter;
	uint8_t _last_flow_quality_value;
	hrt_abstime _last_flow_quality_sample_timestamp;

	orb_advert_t _flow_pub;
	orb_advert_t _flow_distance_sensor_pub;

#ifdef BOARD_OF_USE_FLTCTL_GYRO
	int _sensor_combined_sub;
	ringbuffer::RingBuffer  *_rb_gyro;
#endif
	float _flow_delay_ms;

	static void _cycle_trampoline(void *arg);
	bool _init_flow();	// init - initialise the sensor
	void update();	// update - check input and send out data
	void poll_subscriptions();	// update all msg
	void _cycle_flow();
	int _initialise_uart(const char *device);

	void _read_flow_data();
	void _handle_message_optical_flow_rad(mavlink_message_t *msg);
	float _median_filter(float value);
	float _handle_abnormal_data(float value, float noise_variance);
	float _calc_variance(float value);
	uint8_t _flow_quality_filter(uint8_t value, uint16_t interval_time);
};
