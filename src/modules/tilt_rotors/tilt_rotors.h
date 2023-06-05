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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
//#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/debug_vect.h>

#include <drivers/drv_hrt.h>
//#include <drivers/pwm_out.h>

#include <uORB/topics/manual_control_switches.h>

#include <uORB/topics/manual_control_setpoint.h>

#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
//#include <uORB/topics/tilt_command.h>

using namespace time_literals;

extern "C" __EXPORT int tilt_rotors_main(int argc, char *argv[]);

enum system_state { DISARMED, RAMP, ARMED };


class TiltRotors : public ModuleBase<TiltRotors>, public ModuleParams
{
public:
	TiltRotors();// : ModuleParams(nullptr) {};

	virtual ~TiltRotors() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TiltRotors *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	system_state state = DISARMED;

	debug_vect_s			_dyn_angles{};
	struct manual_control_setpoint_s	_manual_control_setpoint {};	/**< manual control setpoint */
	actuator_servos_s actuator_servos;
	actuator_armed_s _actuator_armed;
	struct manual_control_switches_s _manual;

	// convert servo setpoint in degrees to encoder counts
	int angle2counts(double angle);


	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);




	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TILT_KNOB>) _tilt_knob,
		(ParamInt<px4::params::HOME_POS>) _home_pos,
		(ParamFloat<px4::params::TILT_THRTL>) _tilt_thrtl,
		(ParamFloat<px4::params::TILT_MAX_ANGLE>) _tilt_max_angle,
		(ParamFloat<px4::params::TILT_THRTL_LIM>) _tilt_thrtl_lim
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches) };
	uORB::Subscription _manual_control_setpoint_sub{ ORB_ID(manual_control_setpoint) };	/**< manual control setpoint subscription */
	uORB::Subscription _actuator_armed_sub{ ORB_ID(actuator_armed) };
//	uORB::Subscription _tilt_command_sub{ ORB_ID(tilt_command) };



	// Publications
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication<actuator_servos_s>	_actuator_servos_pub{ ORB_ID(actuator_servos) };


};

