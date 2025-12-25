/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#include "special_serial.hpp"

#include <inttypes.h>
#include <fcntl.h>
#include <termios.h>

#include <string.h>
#include <stdlib.h>





SpecialSerial::SpecialSerial(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, 25),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_battery(2, nullptr ,100000, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	/* store port name */
	strncpy(_port, port, sizeof(_port) - 1);

	/* get device name */
	param_get(param_find("SPECIAL_DEVICE"), &device_name);

	/* get MFlow2 volume */
	param_get(param_find("MFLOW2_VOL"), &MFlow2_Volume);


	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	if (bus_num < 10) {
		device_id.devid_s.bus = bus_num;
	}

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_LIGHTWARE_LASER);

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_battery.setConnected(false);
	_battery.updateVoltage(0.f);
	_battery.updateCurrent(0.f);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());


}

SpecialSerial::~SpecialSerial()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
SpecialSerial::init()
{

	switch (device_name) {
	case 1:
		_interval = 100000; // 1,000,000 / (10 Hz) = 100,000 us
		break;

	case 2:
		_px4_rangefinder.set_min_distance(4.0f);
		_px4_rangefinder.set_max_distance(600.0f);
		_interval = 500000; // 1,000,000 / (2 Hz) = 500,000 us
		break;

	case 3:
		_interval = 20000;
		break;

	case 4:
		_interval = 20000;
		break;

	case 5:
		_interval = 20000;
		break;

	case 6:
		_interval = 20000;
		break;

	case 7:
		_interval = 20000;
		break;

	case 8:
		_interval = 20000;
		break;

	default:
		PX4_ERR("invalid device %" PRIi32 ".", device_name);
		return -1;
	}

	start();

	return PX4_OK;
}

int SpecialSerial::measure()
{
	// Send the command to begin a measurement.
	/*
	uint8_t cmd[8] = {0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE};
	int ret = ::write(_fd, &cmd, 8);
	if (ret != sizeof(cmd)) {
			perf_count(_comms_errors);
			PX4_DEBUG("write fail %d", ret);
			return ret;
		}*/

	switch (device_name) {
	case 1:{
		//uint8_t cmd[8] = {0x3D, 0x01, 0x08, 0x4F, 0x3A, 0x00, 0xF0, 0x1B};  // 8 B Telemetry request
		//uint8_t cmd[8] = {0x3D, 0x01, 0x08, 0x03, 0x40, 0x00, 0x91, 0xB5}; // 8 B X request


		if(MFlow2_flag){

			if(MFlow2_flag == 4){ // get size
				uint8_t cmd[58] = {0x3E, 0x01, 0x3A, 0xCE, 0x31, 0x20, 0xDC, 0x2E, 0xC9, 0x2E, 0x58, 0x1B, 0xEB, 0x2E, 0x68, 0x42, 0xE0, 0x2E, 0xE0, 0x2E, 0x40, 0x1F, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0x57, 0x10, 0xC5, 0xAA, 0x33, 0x54, 0x02, 0x00, 0x01, 0x4A, 0x00, 0x01, 0x4E, 0x00, 0x02, 0x54, 0x00, 0x04, 0x4E, 0x2E}; // 58 B set size command
				int ret = ::write(_fd,&cmd,sizeof(cmd));
				if (ret != sizeof(cmd)) {
				perf_count(_comms_errors);
				PX4_DEBUG("write fail %d", ret);
				return ret;}
			}else if(MFlow2_flag == 3){  // set size


				uint8_t cmd[54] = {0x3E, 0x01, 0x36, 0x4D, 0x31, 0x20, 0xDC, 0x2E, 0xF9, 0x2E, 0x58, 0x1B, 0xEE, 0x2E, 0x68, 0x42, 0xE0, 0x2E, 0xE0, 0x2E, 0x80, 0x3E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0x54, 0x0C, 0xC5, 0xAA, 0x33, 0x54, 0x02, 0x54, 0x00, 0x04, 0x62, 0x27, 0x00, 0x00, 0xB4, 0x80}; // 54 B set size command

				uint16_t volume = (uint16_t)(MFlow2_Volume * 1000);
				uint8_t volume1 = volume & 0xFF;
				uint8_t volume2 = (volume >> 8) & 0xFF;
				cmd[48] = {volume1};
				cmd[49] = {volume2};


				  	uint16_t crc = 0;
					for (int i=0;i<52;i++) {
						uint8_t data = cmd[i];

						data ^= (uint8_t)(crc & 0xFF);
						data ^= data << 4;

						crc = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
						^ (uint8_t)(data >> 4)
						^ ((uint16_t)data << 3));
					}

					uint8_t crc1 = crc & 0xFF;
					uint8_t crc2 = (crc >> 8) & 0xFF;
					cmd[sizeof(cmd)-2] = {crc1};
					cmd[sizeof(cmd)-1] = {crc2};


				int ret = ::write(_fd,&cmd,sizeof(cmd));
				if (ret != sizeof(cmd)) {
				perf_count(_comms_errors);
				PX4_DEBUG("write fail %d", ret);
				return ret;}else{MFlow2_flag =4;}
			}else{  // data reset
				uint8_t cmd[51] = {0x3E, 0x01, 0x33, 0xE3, 0x31, 0x20, 0xDD, 0x2E, 0xCD, 0x2E, 0x58, 0x1B, 0xEF, 0x2E, 0x68, 0x42, 0xE0, 0x2E, 0xE0, 0x2E, 0x40, 0x1F, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0x54, 0x09, 0xC5, 0xAA, 0x33, 0x54, 0x01, 0x99, 0x08, 0x01, 0x01, 0xA0, 0x62}; // 51 B set size command
				int ret = ::write(_fd,&cmd,sizeof(cmd));
				if (ret != sizeof(cmd)) {
				perf_count(_comms_errors);
				PX4_DEBUG("write fail %d", ret);
				return ret;}else{MFlow2_flag =0;}
			}
		}else{ // get data
			uint8_t cmd[55] = {0x3E, 0x01, 0x37, 0x0A, 0x31, 0x20, 0xDD, 0x2E, 0xFB, 0x2E, 0x58, 0x1B, 0xF1, 0x2E, 0x68, 0x42, 0xE0, 0x2E, 0xE0, 0x2E, 0x80, 0x3E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0xE0, 0x2E, 0x57, 0x0D, 0xC5, 0xAA, 0x33, 0x54, 0x46, 0x00, 0x02, 0x04, 0x08, 0x04, 0x0E, 0x08, 0x04, 0x61, 0x52}; // 55 B data request
			int ret = ::write(_fd, &cmd, 55);
			if (ret != sizeof(cmd)) {
			perf_count(_comms_errors);
			PX4_DEBUG("write fail %d", ret);
			return ret;}
		}

		}
		break;
	case 2:{
		uint8_t cmd[8] = {0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE};
		int ret = ::write(_fd, &cmd, 8);
		if (ret != sizeof(cmd)) {
			perf_count(_comms_errors);
			PX4_DEBUG("write fail %d", ret);
			return ret;
		}}
		break;;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	default:
		return -1;
	}



	return PX4_OK;
}

int SpecialSerial::collect()
{
	/* advertise attitude topic */
	struct special_serial_data_s ssd;
	memset(&ssd, 0, sizeof(ssd));
	orb_advert_t ssd_pub_fd = orb_advertise(ORB_ID(special_serial_data), &ssd);

	perf_begin(_sample_perf);

	/* clear buffer if last read was too long ago */
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	/* the buffer for read chars is buflen minus null termination */
	uint8_t readbuf[sizeof(_linebuf)];
	unsigned readlen = sizeof(readbuf) - 1;

	// Get timestamp
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	// PX4_INFO("timestamp_sample: %llu",timestamp_sample);


	/* read from the device (uart buffer) */
	int ret = ::read(_fd, &readbuf[0], readlen);
	//std::string s(reinterpret_cast<char*>(readbuf), ret);
	//readbuf.print();
	//PX4_INFO("TXT: %s", reinterpret_cast<char*>(readbuf));
	//PX4_INFO("ret: %d bytes",ret);
	//PX4_INFO("Buffer: [ %d %d %d ]",readbuf[0],readbuf[1],readbuf[2]);

	ssd.timestamp = timestamp_sample;



	if (ret < 0) {
		PX4_DEBUG("read err: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		/* only throw an error if we time out */
		if (read_elapsed > (_interval * 2)) {
			return ret;

		} else {
			return -EAGAIN;
		}

	} else if (ret == 0) {
		return -EAGAIN;
	}


	_last_read = hrt_absolute_time();



	switch (device_name) {
	case 1:{
		uint16_t flow;
		uint16_t volume;
		//m_flow2_parser(&readbuf[0],ret);
		ssd.a = m_flow2_parser(&readbuf[0],ret,&volume,&flow,&MFlow2_flag,MFlow2_Volume*1000);
		ssd.b = volume;
		ssd.c = flow;
		_battery.setConnected(true);
		_battery.updateVoltage(static_cast<float>(volume/1000.0));
		_battery.updateCurrent(static_cast<float>(flow/1000.0));
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
	}
		break;
	case 2:{
		int distance_m = lrf600_parser(&readbuf[0],ret);
		if (distance_m != -1){
			_px4_rangefinder.update(timestamp_sample, distance_m * .1f); // valid
		}
		}
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	default:
		return -1;
	}

		orb_publish(ORB_ID(special_serial_data), ssd_pub_fd, &ssd);

	perf_end(_sample_perf);

	return PX4_OK;
}

void SpecialSerial::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

void SpecialSerial::stop()
{
	ScheduleClear();
}

void SpecialSerial::Run()
{
	/* fds initialized? */
	if (_fd < 0) {
		/* open fd */
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open failed (%i)", errno);
			return;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(_fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);

		unsigned speed;

		switch (device_name) {
		case 1:{uart_config.c_speed = 250000;
			//uart_config.c_ospeed = 250000;
			speed = B115200;
			}
			break;

		case 2:{
			if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
				PX4_ERR("CFG: %d ISPD", termios_state);
			}

			if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
				PX4_ERR("CFG: %d OSPD", termios_state);
			}
			speed = B9600;}
			break;

		case 3:
			speed = B57600;
			break;

		case 4:
			speed = B57600;
			break;

		case 5:
			speed = B57600;
			break;

		case 6:
			speed = B57600;
			break;

		case 7:
			speed = B57600;
			break;

		case 8:
			speed = B57600;
			break;

		default:
			speed = B57600;
		}


		/* set baud rate */
		/*
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD", termios_state);
		}*/

		if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
		}

		// Enable serial mode by sending some characters
		if (device_name == 8) {
			const char *data = "www\r\n";
			(void)!::write(_fd, &data, strlen(data));
		}
	}

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */

		int collect_ret = collect();
		PX4_INFO("collection");
		//measure();

		//if (collect_ret == -EAGAIN) {
			/* reschedule to grab the missing bits, time to transmit 8 bytes @ 9600 bps */
		//	ScheduleDelayed(1042 * 8);
		//	return;
		//}

		if (OK != collect_ret) {

			/* we know the sensor needs about four seconds to initialize */
			if (hrt_absolute_time() > 5 * 1000 * 1000LL && _consecutive_fail_count < 5) {
				PX4_ERR("collection error #%u", _consecutive_fail_count);
			}

			_consecutive_fail_count++;

			/* restart the measurement state machine */
			start();
			return;

		} else {
			/* apparently success */
			_consecutive_fail_count = 0;
			PX4_INFO("collection success");
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	//tcflush(_fd, TCIFLUSH);

	/* measurement phase */
	if (OK != measure()) {
		PX4_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(_interval);
}

void SpecialSerial::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	collect();
}


void SpecialSerial::reset_MFlow()
{
		PX4_INFO("Reset data command sent");
		MFlow2_flag= 4;

}
