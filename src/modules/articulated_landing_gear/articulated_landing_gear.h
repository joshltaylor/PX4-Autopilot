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
#include <uORB/topics/alg_setpoint.h>

#include <drivers/drv_hrt.h>
//#include <drivers/pwm_out.h>

#include <uORB/topics/manual_control_switches.h>


using namespace time_literals;

extern "C" __EXPORT int articulated_landing_gear_main(int argc, char *argv[]);

enum ALG_state { LANDING_GEAR, GRIPPER_OPEN, GRIPPER_CLOSED };


class ArticulatedLandingGear : public ModuleBase<ArticulatedLandingGear>, public ModuleParams
{
public:
	ArticulatedLandingGear();// : ModuleParams(nullptr) {};

	virtual ~ArticulatedLandingGear() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ArticulatedLandingGear*instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	ALG_state state = LANDING_GEAR;
	
	struct manual_control_switches_s _manual;



	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);




	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LANDING_GEAR_POS>) _landing_gear_pos,
		(ParamInt<px4::params::GRIP_OPEN_POS>) _grip_open_pos,
		(ParamInt<px4::params::GRIP_CLOSED_POS>) _grip_closed_pos
	)	

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches) };
	

	
	// Publications
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
	uORB::Publication<alg_setpoint_s> _alg_setpoint_pub{ ORB_ID(alg_setpoint) };


	debug_vect_s			_dyn_angles{};
	alg_setpoint_s			_alg_setpoint{};
	

};

