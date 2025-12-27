/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "digital_read.hpp"

digital_read::digital_read() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

digital_read::~digital_read()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool digital_read::init()
{

	// Run on fixed interval
	//ScheduleOnInterval(20000_us); // 20000 us interval, 50 Hz rate

	// kkkk
	ScheduleDelayed(1000000);

	px4_arch_configgpio(GPIO_DIGITAL_WRITE);
	px4_arch_gpiowrite(GPIO_DIGITAL_WRITE, 1);

	px4_arch_configgpio(GPIO_DIGITAL_READ);
	px4_arch_gpioread(GPIO_DIGITAL_READ);




	return true;
}

void digital_read::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	uint8_t val = px4_arch_gpioread(GPIO_DIGITAL_READ);

	//  publish some data
	orb_test_s data{};
	data.val = val;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);

	ScheduleDelayed(20000);
	perf_end(_loop_perf);
}

int digital_read::task_spawn(int argc, char *argv[])
{
	digital_read *instance = new digital_read();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int digital_read::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int digital_read::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int digital_read::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Digital read.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("digital_read", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int digital_read_main(int argc, char *argv[])
{
	return digital_read::main(argc, argv);
}
