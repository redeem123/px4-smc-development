/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include <lib/rate_control/rate_control.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class MulticopterRateControl : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	static Descriptor desc;

	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class RateSetpointSource {
		Unknown,
		Disabled,
		ManualRate,
		Position,
		Velocity,
		Altitude,
		Attitude,
		Topic
	};

	void Run() override;
	RateSetpointSource getRateSetpointSource() const;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< class for rate control calculations
	RateSetpointSource _rate_setpoint_source{RateSetpointSource::Unknown};
	uint8_t _rate_setpoint_nav_state{0};
	bool _rate_setpoint_context_valid{false};

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;

	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};
	bool _smc_configuration_valid{false};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	AlphaFilter<float> _output_lpf_yaw;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>) _param_mc_yaw_tq_cutoff,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		(ParamInt<px4::params::MC_RATE_CTRL_T>) _param_mc_rate_ctrl_t,
		(ParamInt<px4::params::MC_MSMC_CFG>) _param_mc_msmc_cfg,

		(ParamFloat<px4::params::MC_MPC_J_R>) _param_mc_mpc_j_roll,
		(ParamFloat<px4::params::MC_MPC_J_P>) _param_mc_mpc_j_pitch,
		(ParamFloat<px4::params::MC_MPC_J_Y>) _param_mc_mpc_j_yaw,

		(ParamFloat<px4::params::MC_MPC_EFF_R>) _param_mc_mpc_eff_roll,
		(ParamFloat<px4::params::MC_MPC_EFF_P>) _param_mc_mpc_eff_pitch,
		(ParamFloat<px4::params::MC_MPC_EFF_Y>) _param_mc_mpc_eff_yaw,

		(ParamFloat<px4::params::MC_MPC_Q_R>) _param_mc_mpc_q_roll,
		(ParamFloat<px4::params::MC_MPC_Q_P>) _param_mc_mpc_q_pitch,
		(ParamFloat<px4::params::MC_MPC_Q_Y>) _param_mc_mpc_q_yaw,

		(ParamFloat<px4::params::MC_MPC_R_R>) _param_mc_mpc_r_roll,
		(ParamFloat<px4::params::MC_MPC_R_P>) _param_mc_mpc_r_pitch,
		(ParamFloat<px4::params::MC_MPC_R_Y>) _param_mc_mpc_r_yaw,

		(ParamFloat<px4::params::MC_MPC_DU_R>) _param_mc_mpc_du_roll,
		(ParamFloat<px4::params::MC_MPC_DU_P>) _param_mc_mpc_du_pitch,
		(ParamFloat<px4::params::MC_MPC_DU_Y>) _param_mc_mpc_du_yaw,

		(ParamFloat<px4::params::MC_MPC_I_R>) _param_mc_mpc_i_roll,
		(ParamFloat<px4::params::MC_MPC_I_P>) _param_mc_mpc_i_pitch,
		(ParamFloat<px4::params::MC_MPC_I_Y>) _param_mc_mpc_i_yaw,
		(ParamFloat<px4::params::MC_MPC_ILIM_R>) _param_mc_mpc_ilim_roll,
		(ParamFloat<px4::params::MC_MPC_ILIM_P>) _param_mc_mpc_ilim_pitch,
		(ParamFloat<px4::params::MC_MPC_ILIM_Y>) _param_mc_mpc_ilim_yaw,

		(ParamFloat<px4::params::MC_MPC_TMAX_R>) _param_mc_mpc_tmax_roll,
		(ParamFloat<px4::params::MC_MPC_TMAX_P>) _param_mc_mpc_tmax_pitch,
		(ParamFloat<px4::params::MC_MPC_TMAX_Y>) _param_mc_mpc_tmax_yaw,

		(ParamInt<px4::params::MC_MPC_HORIZON>) _param_mc_mpc_horizon,
		(ParamFloat<px4::params::MC_MPC_SLEW>) _param_mc_mpc_slew,
		(ParamFloat<px4::params::MC_MPC_RSPD_L>) _param_mc_mpc_rate_sp_deriv_lim,
		(ParamFloat<px4::params::MC_MPC_GYRO>) _param_mc_mpc_gyro,
		(ParamFloat<px4::params::MC_MPC_TAU>) _param_mc_mpc_tau,

		(ParamFloat<px4::params::MC_MSMC_J_R>) _param_mc_msmc_j_roll,
		(ParamFloat<px4::params::MC_MSMC_J_P>) _param_mc_msmc_j_pitch,
		(ParamFloat<px4::params::MC_MSMC_J_Y>) _param_mc_msmc_j_yaw,
		(ParamFloat<px4::params::MC_MSMC_EFF_R>) _param_mc_msmc_eff_roll,
		(ParamFloat<px4::params::MC_MSMC_EFF_P>) _param_mc_msmc_eff_pitch,
		(ParamFloat<px4::params::MC_MSMC_EFF_Y>) _param_mc_msmc_eff_yaw,

		(ParamFloat<px4::params::MC_MSMC_C_R>) _param_mc_msmc_c_roll,
		(ParamFloat<px4::params::MC_MSMC_C_P>) _param_mc_msmc_c_pitch,
		(ParamFloat<px4::params::MC_MSMC_C_Y>) _param_mc_msmc_c_yaw,

		(ParamFloat<px4::params::MC_MSMC_ETA_R>) _param_mc_msmc_eta_roll,
		(ParamFloat<px4::params::MC_MSMC_ETA_P>) _param_mc_msmc_eta_pitch,
		(ParamFloat<px4::params::MC_MSMC_ETA_Y>) _param_mc_msmc_eta_yaw,

		(ParamFloat<px4::params::MC_MSMC_BND_R>) _param_mc_msmc_bnd_roll,
		(ParamFloat<px4::params::MC_MSMC_BND_P>) _param_mc_msmc_bnd_pitch,
		(ParamFloat<px4::params::MC_MSMC_BND_Y>) _param_mc_msmc_bnd_yaw,

		(ParamFloat<px4::params::MC_MSMC_KS_R>) _param_mc_msmc_ks_roll,
		(ParamFloat<px4::params::MC_MSMC_KS_P>) _param_mc_msmc_ks_pitch,
		(ParamFloat<px4::params::MC_MSMC_KS_Y>) _param_mc_msmc_ks_yaw,
		(ParamFloat<px4::params::MC_MSMC_RSPD_L>) _param_mc_msmc_rate_sp_deriv_lim,
		(ParamFloat<px4::params::MC_MSMC_ILIM_R>) _param_mc_msmc_ilim_roll,
		(ParamFloat<px4::params::MC_MSMC_ILIM_P>) _param_mc_msmc_ilim_pitch,
		(ParamFloat<px4::params::MC_MSMC_ILIM_Y>) _param_mc_msmc_ilim_yaw,
		(ParamFloat<px4::params::MC_MSMC_TMAX_R>) _param_mc_msmc_tmax_roll,
		(ParamFloat<px4::params::MC_MSMC_TMAX_P>) _param_mc_msmc_tmax_pitch,
		(ParamFloat<px4::params::MC_MSMC_TMAX_Y>) _param_mc_msmc_tmax_yaw,

		(ParamFloat<px4::params::MC_SMC_LPF>) _param_mc_smc_lpf,
		(ParamFloat<px4::params::MC_SMC_SLEW>) _param_mc_smc_slew,
		(ParamFloat<px4::params::MC_SMC_WD_ERR>) _param_mc_smc_wd_err,
		(ParamFloat<px4::params::MC_SMC_WD_TOUT>) _param_mc_smc_wd_tout
	)
};
