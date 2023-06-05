/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file tilt_rotors_params.c
 * Parameter definition for tilt_rotors.
 *
 * @author Joshua Taylor <joshualeo2000@gmail.com>
 *
 */



/**
 * tilt_angle
 *
 * @group TiltRotors
 * @decimal 5
 * @min -45.0
 * @max 45.0
 */
PARAM_DEFINE_FLOAT(TILT_KNOB, 0.f);


/**
 * Home Position (Vertical Tilt Rotors)
 *
 * 
 *
 * @group TiltRotors
 * @min 0
 * @max 4095
 */
PARAM_DEFINE_INT32(HOME_POS, 2048);


/**
 * Tilt Rotor Throttle (Corresponds to the level of upward thrust produced by the tilt-rotors - this remains constant throughout flight)
 *
 * @group TiltRotors
 * @decimal 5
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(TILT_THRTL, 0.4f);


/**
 * Tilt Rotor Max Angle
 *
 * @group TiltRotors
 * @decimal 5
 * @min 0.0
 * @max 89.0
 */
PARAM_DEFINE_FLOAT(TILT_MAX_ANGLE, 45.0f);


/**
 * Tilt Rotor Throttle Limit
 *
 * @group TiltRotors
 * @decimal 5
 * @min 0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(TILT_THRTL_LIM, 0.5f);