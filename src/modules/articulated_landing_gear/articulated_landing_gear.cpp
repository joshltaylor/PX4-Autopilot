/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "articulated_landing_gear.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#include <drivers/drv_pwm_output.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include "systemlib/err.h"

#include <uORB/uORB.h>
#include "uORB/topics/actuator_controls.h"
#include <poll.h>

#include <math.h>


static const double PI = 3.1415926535;






int ArticulatedLandingGear::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}



int ArticulatedLandingGear::custom_command(int argc, char* argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}


int ArticulatedLandingGear::task_spawn(int argc, char* argv[])
{
	_task_id = px4_task_spawn_cmd("module",
		SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT,
		1024,
		(px4_main_t)&run_trampoline,
		(char* const*)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}


ArticulatedLandingGear* ArticulatedLandingGear::instantiate(int argc, char* argv[])
{

	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char* myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			//example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			//example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	ArticulatedLandingGear* instance = new ArticulatedLandingGear();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ArticulatedLandingGear::ArticulatedLandingGear()
	: ModuleParams(nullptr)
{
}

void ArticulatedLandingGear::run()
{


	//_dyn_angles.x = 0;
	//_debug_vect_pub.publish(_dyn_angles);

	state = LANDING_GEAR;

	int current_pos;


	parameters_update(true);


	




	while (!should_exit()) {

		px4_usleep(1000);


		if (_manual_control_switches_sub.updated()) {

			_manual_control_switches_sub.copy(&_manual);
		}



		if (state == LANDING_GEAR) {
			if (_manual.mode_slot == manual_control_switches_s::MODE_SLOT_4) {
				state = GRIPPER_OPEN;
			}
		}
		else if (state == GRIPPER_OPEN) {
			if (_manual.mode_slot == manual_control_switches_s::MODE_SLOT_1) {
				state = GRIPPER_CLOSED;
			}
			else if (_manual.mode_slot == manual_control_switches_s::MODE_SLOT_6) {
				state = LANDING_GEAR;
			}
		}
		else {
			if (_manual.mode_slot == manual_control_switches_s::MODE_SLOT_6) {
				state = LANDING_GEAR;
			}
			else if (_manual.mode_slot == manual_control_switches_s::MODE_SLOT_4) {
				state = GRIPPER_OPEN;
			}
		}

		if (state == LANDING_GEAR) {
			current_pos = _landing_gear_pos.get();
		}
		else if (state == GRIPPER_OPEN) {
			current_pos = _grip_open_pos.get();
		}
		else {
			current_pos = _grip_closed_pos.get();
		}

		_alg_setpoint.setpoint = current_pos;
		_alg_setpoint_pub.publish(_alg_setpoint);
		PX4_INFO("setpoint: %i", current_pos);

		parameters_update();
	}


}

void ArticulatedLandingGear::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int ArticulatedLandingGear::print_usage(const char* reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module runs a bench-test for a variable-tilt drone.
Before using the module run:
$ benchtest stop
$ mc_rate_control stop
$ benchtest start



### Examples
Start the benchtest module
$ benchtest start

Start a full benchtest
$ benchtest full 30 10 8 1604

Start a single benchtest
$ benchtest single 30 45 1604

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("benchtest", "Variable-tilt drone");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND("full");
	PRINT_MODULE_USAGE_ARG("MAX_TILT, NUM_TILT, NUM_HEADING, PWM_HOVER", "maximum tilt angle, number of tilt angles, number of headings, PWM at hover", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("MAX_TILT|NUM_TILT|NUM_HEADING|PWM_HOVER", "Start full benchtest sequence");



	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int articulated_landing_gear_main(int argc, char* argv[])
{
	return ArticulatedLandingGear::main(argc, argv);
}
