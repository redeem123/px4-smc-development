/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
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

#include "systemCheck.hpp"

#include "../../Arming/ArmAuthorization/ArmAuthorization.h"
#include <drivers/drv_hrt.h>
#include <lib/circuit_breaker/circuit_breaker.h>
#include <uORB/topics/vehicle_command_ack.h>

void SystemChecks::checkAndReport(const Context &context, Report &reporter)
{
	actuator_armed_s actuator_armed;

	if (!context.isArmed() && _param_mc_rate_ctrl_t.get() == 2) {
		if (_param_mc_msmc_cfg.get() != 1) {
			/* EVENT
			 * @description
			 * Review the physical inertia, torque effectiveness, SMC gains, and limits for this vehicle.
			 * Then set <param>MC_MSMC_CFG</param> to 1 as the last model-card step.
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system,
						    events::ID("check_system_smc_config"), events::Log::Error,
						    "SMC model card not acknowledged");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: SMC model card not acknowledged");
			}

		} else {
			rate_ctrl_status_s controller_status{};
			parameter_update_s parameter_update{};
			const bool status_available = _rate_ctrl_status_sub.copy(&controller_status);
			const bool update_available = _parameter_update_sub.copy(&parameter_update);
			const bool status_stale = !status_available || hrt_elapsed_time(&controller_status.timestamp) > 2_s;
			const bool update_pending = update_available && controller_status.timestamp < parameter_update.timestamp;
			const bool controller_ready = !status_stale && !update_pending
						      && controller_status.controller_type == 2 && controller_status.model_valid;

			if (!controller_ready) {
				/* EVENT
				 * @description
				 * The rate controller rejected the SMC card, has not applied the latest parameter update,
				 * or is not publishing a current valid SMC status.
				 */
				reporter.armingCheckFailure(NavModes::All, health_component_t::system,
							    events::ID("check_system_smc_not_ready"), events::Log::Error,
							    "SMC controller not ready");

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: SMC controller not ready");
				}
			}
		}
	}

	if (_actuator_armed_sub.copy(&actuator_armed)) {
		if (actuator_armed.termination || actuator_armed.kill) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_flight_term_active"),
						    events::Log::Critical, "Flight termination active");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Flight termination active");
			}
		}
	}

	// USB not connected
	if (!circuit_breaker_enabled_by_val(_param_cbrk_usb_chk.get(), CBRK_USB_CHK_KEY) && context.status().usb_connected) {
		/* EVENT
		 * @description
		 * Flying with USB is not safe. Disconnect it and reboot the FMU.
		 *
		 * <profile name="dev">
		 * This check can be configured via <param>CBRK_USB_CHK</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_usb_connected"),
					    events::Log::Error, "USB connected");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Flying with USB is not safe");
		}
	}

	// Global position required
	if (!_param_com_arm_wo_gps.get() && !context.isArmed()) {
		if (reporter.failsafeFlags().global_position_invalid) {
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_WO_GPS</param> parameter.
			 * </profile>
			 * The available positioning data is not sufficient to determine the vehicle's global position.
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_no_global_pos"),
						    events::Log::Error, "Global position estimate required");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Global position estimate required");
			}

		} else if (reporter.failsafeFlags().home_position_invalid) {
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>COM_ARM_WO_GPS</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_no_home_pos"),
						    events::Log::Error, "Home position required");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Home position required");
			}
		}
	}

	// safety button
	if (context.status().safety_button_available && !context.status().safety_off && !context.isArmed()) {
		/* EVENT
		 * @description
		 * <profile name="dev">
		 * This check can be configured via <param>CBRK_IO_SAFETY</param> parameter.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_safety_button"),
					    events::Log::Info, "Press safety button first");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Press safety button first");
		}
	}

	// VTOL in transition
	if (context.status().is_vtol && !context.isArmed()) {
		if (context.status().in_transition_mode) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_vtol_in_transition"),
						    events::Log::Warning, "Vehicle is in transition state");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Vehicle is in transition state");
			}
		}

		if (!circuit_breaker_enabled_by_val(_param_cbrk_vtolarming.get(), CBRK_VTOLARMING_KEY)
		    && context.status().vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			/* EVENT
			 * @description
			 * <profile name="dev">
			 * This check can be configured via <param>CBRK_VTOLARMING</param> parameter.
			 * </profile>
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_vtol_in_fw_mode"),
						    events::Log::Warning, "Vehicle is not in multicopter mode");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Vehicle is not in multicopter mode");
			}
		}
	}

	// Arm Requirements: authorization
	if (_param_com_arm_auth_req.get() != 0 && !context.isArmed() && context.isArmingRequest()) {
		if (arm_auth_check() != vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
			/* EVENT
			 */
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_system_arm_auth_failed"),
						    events::Log::Error, "Arm authorization denied");
		}
	}
}
