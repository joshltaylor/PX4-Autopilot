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

#include "tilt_rotors.h"

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




int TiltRotors::angle2counts(double angle)
{
	int counts = (angle / 360) * 4095;
	return (_home_pos.get() + counts);
}




int TiltRotors::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}



int TiltRotors::custom_command(int argc, char* argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	return print_usage("unknown command");
}


int TiltRotors::task_spawn(int argc, char* argv[])
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


TiltRotors* TiltRotors::instantiate(int argc, char* argv[])
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

	TiltRotors* instance = new TiltRotors();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

TiltRotors::TiltRotors()
	: ModuleParams(nullptr)
{
}

void TiltRotors::run()
{

	//px4_pollfd_struct_t fds[1];
	//fds[0].fd = _manual_control_switches_sub.get_instance();
	//fds[0].events = POLLIN;

	_dyn_angles.x = _home_pos.get();
	_debug_vect_pub.publish(_dyn_angles);


	double tilt_angle = 0;
	double thrust_command = -1;

	state = DISARMED;

	float dt = 0.001f; // prevent division with 0
	hrt_abstime start = 0;
	start = hrt_absolute_time();

	parameters_update(true);


	




	while (!should_exit()) {

		px4_usleep(1000);

		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_actuator_armed_sub.update(&_actuator_armed);


		if (_manual_control_switches_sub.updated()) {

			_manual_control_switches_sub.copy(&_manual);
		}

		// if using transmitter knob method - replace gear switch if statement with...
		//if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
		//	_dyn_angles.x = angle2counts(_tilt_knob.get());
		//	PX4_INFO("Tilt angle: %f", (double)_tilt_knob.get());
		//	_debug_vect_pub.publish(_dyn_angles);
		//}
		//else if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
		//	_dyn_angles.x = angle2counts((double)_manual_control_setpoint.x * 45.00);
		//	_debug_vect_pub.publish(_dyn_angles);
		//	PX4_INFO("Vertical");
		//}
		// and make required to edits to mc_att_control_main.cpp

		if (state == DISARMED) {
			if (_actuator_armed.armed) {
				state = RAMP;
				start = hrt_absolute_time();
			}
		}
		else if (state == RAMP) {
			PX4_INFO("RAMP");
			PX4_INFO("dt: %f", (double)dt);


			if (dt >= 2) {
				state = ARMED;
				dt = 0.001f; // prevent division with 0
				start = 0;
			}
		}
		else {
			if (!_actuator_armed.armed) {
				state = DISARMED;
			}
		}

		if (state == DISARMED) {
			thrust_command = -1;
		}
		else if (state == RAMP) {
			dt = hrt_elapsed_time(&start) * 1e-6;

			thrust_command = ((double)_tilt_thrtl.get() * ((double)dt / 2)) * 2.0 - 1;
		}
		else {
			if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
				tilt_angle = (double)_manual_control_setpoint.x * (double)_tilt_max_angle.get();
				_dyn_angles.x = angle2counts(tilt_angle);
				_debug_vect_pub.publish(_dyn_angles);

				// for dual transmitter flight mode
				//thrust_command = (((double)_manual_control_setpoint.z * 0.9) / cos(tilt_angle * PI / 180)) - 0.9;
				thrust_command = ((double)_tilt_thrtl.get() / cos(tilt_angle * PI / 180)) * 2.0 - 1; // nominal 40% throttle
				/*if (thrust_command > 0.5) { thrust_command = 0.5; }
				else if (thrust_command < -1) { thrust_command = -1; }
				PX4_INFO("Thrust command: %f", (double)thrust_command);
				actuator_servos.control[0] = thrust_command;
				actuator_servos.control[1] = thrust_command;

				for (int i = 2; i < actuator_servos_s::NUM_CONTROLS; i++) {
					actuator_servos.control[i] = NAN;
				}

				_actuator_servos_pub.publish(actuator_servos);*/
			}
			else if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
				_dyn_angles.x = _home_pos.get();
				_debug_vect_pub.publish(_dyn_angles);

				// for dual trasmitter flight mode
				/*for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; i++) {
					actuator_servos.control[i] = NAN;
				}

				_actuator_servos_pub.publish(actuator_servos);*/
				thrust_command = (double)_tilt_thrtl.get() * 2.0 - 1;
			}
		}

		if (thrust_command > (double)_tilt_thrtl_lim.get() * 2 - 1) { thrust_command = (double)_tilt_thrtl_lim.get() * 2 - 1; }
		else if (thrust_command < -1) { thrust_command = -1; }
		PX4_INFO("Thrust command: %f", (double)thrust_command);
		actuator_servos.control[0] = thrust_command;
		actuator_servos.control[1] = thrust_command;

		for (int i = 2; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
		


		//int poll_ret = px4_poll(fds, 1, 1000);

		//if (fds[0].revents & POLLIN) {

		//	if (_manual_control_switches_sub.updated()) {

		//		_manual_control_switches_sub.copy(&_manual);
		//	}

		//	if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_ON) {
		//		//_dyn_angles.x = _tilt_knob.get();
		//		PX4_INFO("Tilt angle: %f", (double)_tilt_knob.get());
		//		_debug_vect_pub.publish(_dyn_angles);
		//	}
		//	else if (_manual.gear_switch == manual_control_switches_s::SWITCH_POS_OFF) {
		//		_dyn_angles.x = HOME;
		//		_debug_vect_pub.publish(_dyn_angles);
		//	}

		//}

		parameters_update();
	}


}

void TiltRotors::parameters_update(bool force)
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

int TiltRotors::print_usage(const char* reason)
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

int tilt_rotors_main(int argc, char* argv[])
{
	return TiltRotors::main(argc, argv);
}
