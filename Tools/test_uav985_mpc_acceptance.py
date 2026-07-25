#!/usr/bin/env python3
"""Tests for UAV985 controller-3 SITL acceptance helpers."""

import argparse
import importlib.util
import json
import sys
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path
from unittest import mock

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TMP_MPC_LOGS = ROOT / "tmp_mpc_logs"
sys.path.insert(0, str(TMP_MPC_LOGS))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ANALYZER = load_module("analyze_offboard_square", TMP_MPC_LOGS / "analyze_offboard_square.py")
RUNNER = load_module("run_uav985_mpc_acceptance", TMP_MPC_LOGS / "run_uav985_mpc_acceptance.py")
GRID = load_module("run_uav985_trajectory_grid", TMP_MPC_LOGS / "run_uav985_trajectory_grid.py")
FLIGHT = load_module("offboard_square_smooth", TMP_MPC_LOGS / "offboard_square_smooth.py")


def acceptance_args(controller):
    return Namespace(
        controller=controller,
        expected_inertia=RUNNER.DEFAULT_INERTIA,
        expected_effectiveness=RUNNER.DEFAULT_EFFECTIVENESS,
        expected_torque_limit=None,
        expected_mpc_q=(1.0, 1.0, 0.7),
        expected_mpc_r=(1.0, 1.0, 1.0),
        expected_mpc_du=(0.05, 0.05, 0.05),
        expected_actuator_tau=0.025,
        expected_horizon=8,
        expected_mpc_slew=10.0,
        expected_mpc_gyro=1.0,
        expected_smc_c=(2.0, 2.0, 2.0),
        expected_smc_eta=(7.0, 8.2, 1.0),
        expected_smc_boundary=(0.5, 0.5, 0.2),
        expected_smc_ks=(1.0, 1.0, 1.0),
        expected_smc_rate_sp_derivative=0.0,
        expected_smc_lpf=20.0,
        expected_smc_slew=15.0,
        expected_smc_integral_limit=(0.3, 0.3, 0.3),
        expected_ast_k1=(3.0, 3.0, 1.5),
        expected_ast_k2=(4.5, 4.5, 1.5),
        expected_ast_reference_acceleration=(20.0, 20.0, 5.0),
        expected_ast_reference_jerk=(100.0, 100.0, 40.0),
        expected_ast_residual_torque=(0.10, 0.10, 0.05),
        expected_ast_actuator_tau=(0.0, 0.0, 0.0),
        expected_ast_slew=(0.0, 0.0, 0.0),
        expected_ast_variation_weight=(0.0, 0.0, 0.0),
        expected_ast_sliding_boundary=(0.0, 0.0, 0.0),
        expected_ast_tracking_blend=0.0,
        expected_ast_reference_feedforward=1.0,
        expected_ast_reference_feedforward_rp=1.0,
        expected_ast_gyro=1.0,
        expected_ast_dt_min=0.0005,
        expected_ast_dt_max=0.005,
        expected_ast_recovery_error=1.0,
        min_ast_samples=100,
        min_ast_rate_hz=100.0,
        max_ast_invalid_dt_fraction=0.0,
        min_ast_allocator_feedback_valid_fraction=1.0,
        max_ast_allocator_stale_fraction=0.0,
        max_ast_internal_saturation_fraction=0.05,
        max_ast_allocation_residual=0.005,
        min_ast_torque_timestamp_match_fraction=0.99,
        torque_limit_tolerance=0.002,
    )


def math_check_passes(check):
    _label, value, operator, threshold = check
    return np.isfinite(value) and (value <= threshold if operator == "<=" else value >= threshold)


def synthetic_astsmc_data(sample_count=400, rate_hz=200.0):
    timestamps = 1_000_000 + np.arange(sample_count, dtype=np.uint64) * int(1_000_000 / rate_hz)
    ast = {
        "timestamp": timestamps + 500,
        "timestamp_sample": timestamps.copy(),
        "configuration_valid": np.ones(sample_count, dtype=bool),
        "dt_valid": np.ones(sample_count, dtype=bool),
        "total_bound_valid": np.ones(sample_count, dtype=bool),
        "raw_dt": np.full(sample_count, 1.0 / rate_hz),
        "accepted_dt": np.full(sample_count, 1.0 / rate_hz),
        "dt_min": np.full(sample_count, 0.0005),
        "dt_max": np.full(sample_count, 0.005),
        "max_airborne_dt": np.full(sample_count, 1.0 / rate_hz),
        "valid_update_count": np.arange(1, sample_count + 1, dtype=np.uint32),
        "invalid_dt_hold_count": np.zeros(sample_count, dtype=np.uint32),
        "consecutive_invalid_dt_hold_count": np.zeros(sample_count, dtype=np.uint32),
        "reference_reset_count": np.ones(sample_count, dtype=np.uint32),
        "total_bound_violation_count": np.zeros(sample_count, dtype=np.uint32),
        "allocator_feedback_valid": np.ones(sample_count, dtype=bool),
        "allocator_feedback_stale": np.zeros(sample_count, dtype=bool),
        "allocator_torque_setpoint_achieved": np.ones(sample_count, dtype=bool),
        "allocator_timestamp_sample": timestamps.copy(),
        "allocator_feedback_age_s": np.full(sample_count, 0.001),
    }
    safety = {
        "timestamp": timestamps + 500,
        "timestamp_sample": timestamps.copy(),
        "runtime_fault_latched": np.zeros(sample_count, dtype=bool),
        "runtime_fault_reason": np.zeros(sample_count, dtype=np.uint32),
        "ground_containment_active": np.zeros(sample_count, dtype=bool),
        "ground_containment_count": np.zeros(sample_count, dtype=np.uint32),
    }
    k1 = (3.0, 3.0, 1.5)
    k2 = (4.5, 4.5, 1.5)

    for axis in range(3):
        torque = 0.01 * np.sin(np.arange(sample_count) * 0.02 + axis)
        ast[f"internal_saturation[{axis}]"] = np.zeros(sample_count, dtype=bool)
        ast[f"nominal_saturation[{axis}]"] = np.zeros(sample_count, dtype=bool)
        ast[f"nominal_limit_count[{axis}]"] = np.zeros(sample_count, dtype=np.uint32)
        ast[f"residual_limit_count[{axis}]"] = np.zeros(sample_count, dtype=np.uint32)
        safety[f"state_recovery_active[{axis}]"] = np.zeros(sample_count, dtype=bool)
        safety[f"state_recovery_count[{axis}]"] = np.zeros(sample_count, dtype=np.uint32)
        safety[f"recovery_trigger_sliding[{axis}]"] = np.zeros(sample_count)
        safety[f"recovery_trigger_state[{axis}]"] = np.zeros(sample_count)
        safety[f"recovery_trigger_reaching[{axis}]"] = np.zeros(sample_count)
        ast[f"rate_setpoint_raw[{axis}]"] = np.zeros(sample_count)
        ast[f"rate_setpoint_shaped[{axis}]"] = np.zeros(sample_count)
        ast[f"reference_acceleration[{axis}]"] = np.zeros(sample_count)
        ast[f"reference_jerk[{axis}]"] = np.zeros(sample_count)
        ast[f"sliding_variable_raw[{axis}]"] = 0.02 * np.sin(np.arange(sample_count) * 0.01 + axis)
        ast[f"sliding_variable[{axis}]"] = ast[f"sliding_variable_raw[{axis}]"].copy()
        ast[f"sliding_boundary[{axis}]"] = np.zeros(sample_count)
        ast[f"integral_state[{axis}]"] = np.zeros(sample_count)
        ast[f"k1[{axis}]"] = np.full(sample_count, k1[axis])
        ast[f"k2[{axis}]"] = np.full(sample_count, k2[axis])
        ast[f"reaching_input_raw[{axis}]"] = torque * 100.0
        ast[f"reaching_input_limited[{axis}]"] = torque * 100.0
        ast[f"nominal_torque_raw[{axis}]"] = np.zeros(sample_count)
        ast[f"nominal_torque_limited[{axis}]"] = np.zeros(sample_count)
        ast[f"residual_torque_raw[{axis}]"] = torque
        ast[f"residual_torque_limited[{axis}]"] = torque
        ast[f"residual_authority[{axis}]"] = np.full(sample_count, (0.10, 0.10, 0.05)[axis])
        ast[f"variation_weight[{axis}]"] = np.zeros(sample_count)
        ast[f"variation_regularization[{axis}]"] = np.zeros(sample_count)
        ast[f"torque_raw[{axis}]"] = torque
        ast[f"torque_limited[{axis}]"] = torque
        ast[f"allocated_torque[{axis}]"] = torque
        ast[f"allocation_residual[{axis}]"] = np.zeros(sample_count)

    # Shared array aliases preserve the existing mutation-focused test setup.
    ast.update(safety)
    rate_status = {
        "timestamp": timestamps.copy(),
        "controller_type": np.full(sample_count, 3, dtype=np.uint8),
        "astsmc_valid": np.ones(sample_count, dtype=bool),
    }
    land_detected = {
        "timestamp": np.array([900_000], dtype=np.uint64),
        "landed": np.array([False]),
        "maybe_landed": np.array([False]),
    }
    torque_setpoint = {"timestamp_sample": timestamps.copy()}

    for axis in range(3):
        torque_setpoint[f"xyz[{axis}]"] = ast[f"torque_limited[{axis}]"].copy()

    parameters = {
        "MC_AST_K1_R": 3.0,
        "MC_AST_K1_P": 3.0,
        "MC_AST_K1_Y": 1.5,
        "MC_AST_K2_R": 4.5,
        "MC_AST_K2_P": 4.5,
        "MC_AST_K2_Y": 1.5,
        "MC_AST_REC_ERR": 1.0,
    }
    end_s = timestamps[-1] / 1e6
    result = ANALYZER.analyze_astsmc_diagnostics(
        ast,
        safety,
        rate_status,
        land_detected,
        torque_setpoint,
        parameters,
        offboard_start_s=1.0,
        offboard_end_s=end_s,
        planned_maneuver_end_s=end_s,
        torque_limits=np.array([0.2, 0.2, 0.15]),
    )
    return result, ast


class ControllerParameterTest(unittest.TestCase):
    def test_controller_specific_parameter_branches(self):
        mpc = RUNNER.expected_controller_parameters(acceptance_args(1))
        smc = RUNNER.expected_controller_parameters(acceptance_args(2))
        ast = RUNNER.expected_controller_parameters(acceptance_args(3))

        self.assertIn("MC_MPC_J_R", mpc)
        self.assertNotIn("MC_MSMC_J_R", mpc)
        self.assertEqual(mpc["MC_MPC_TMAX_Y"], 0.10)
        self.assertIn("MC_MSMC_J_R", smc)
        self.assertNotIn("MC_AST_J_R", smc)
        self.assertEqual(smc["MC_MSMC_ILIM_R"], 0.3)
        self.assertEqual(smc["MC_MSMC_ILIM_P"], 0.3)
        self.assertEqual(smc["MC_MSMC_ILIM_Y"], 0.3)
        self.assertEqual(smc["MC_MSMC_TMAX_Y"], 0.15)
        self.assertEqual(ast["MC_AST_CFG"], 1)
        self.assertEqual(ast["MC_BAT_SCALE_EN"], 0)
        self.assertEqual(ast["MC_AST_TMAX_Y"], 0.15)
        self.assertEqual(ast["MC_AST_K1_Y"], 1.5)
        self.assertEqual(ast["MC_AST_RACC_Y"], 5.0)
        self.assertEqual(ast["MC_AST_RJERK_Y"], 40.0)
        self.assertEqual(ast["MC_AST_TRES_Y"], 0.05)
        self.assertEqual(ast["MC_AST_SBD_R"], 0.0)
        self.assertEqual(ast["MC_AST_SBD_P"], 0.0)
        self.assertEqual(ast["MC_AST_SBD_Y"], 0.0)
        self.assertEqual(ast["MC_AST_TRK_B"], 0.0)
        self.assertEqual(ast["MC_AST_RFF"], 1.0)
        self.assertEqual(ast["MC_AST_RFF_RP"], 1.0)
        self.assertEqual(ast["MC_AST_GYRO"], 1.0)
        self.assertEqual(ast["MC_AST_DT_MIN"], 0.0005)
        self.assertEqual(ast["MC_AST_DT_MAX"], 0.005)
        self.assertEqual(ast["MC_AST_REC_ERR"], 1.0)

    def test_torque_limit_prefix_is_explicit(self):
        self.assertEqual(ANALYZER.torque_limit_prefix(1), "MC_MPC_TMAX")
        self.assertEqual(ANALYZER.torque_limit_prefix(2), "MC_MSMC_TMAX")
        self.assertEqual(ANALYZER.torque_limit_prefix(3), "MC_AST_TMAX")

        with self.assertRaises(RuntimeError):
            ANALYZER.torque_limit_prefix(4)

    def test_type_three_launch_defaults_and_override_precedence(self):
        defaults = GRID.default_sitl_parameter_overrides(3)
        overridden = GRID.effective_parameter_overrides(3, {"MC_AST_K1_R": 6.0})

        self.assertEqual(defaults["SDLOG_PROFILE"], 25)
        self.assertEqual(defaults["MC_AST_CFG"], 1)
        self.assertEqual(defaults["MC_AST_J_R"], 0.040461)
        self.assertEqual(defaults["MC_AST_TRK_B"], 0.0)
        self.assertEqual(defaults["MC_AST_RFF_RP"], 1.0)
        self.assertEqual(defaults["MC_AST_SBD_R"], 0.0)
        self.assertEqual(defaults["MC_AST_REC_ERR"], 1.0)
        self.assertEqual(overridden["MC_AST_K1_R"], 6.0)

    def test_counterbalanced_schedule(self):
        self.assertEqual(
            RUNNER.comparison_schedule(2, 3, 3),
            [(0, 2), (0, 3), (1, 3), (1, 2), (2, 2), (2, 3)],
        )

    def test_type_three_real_flight_is_rejected(self):
        args = Namespace(
            sitl_ok=False,
            real_flight_ok=True,
            connection="serial:/dev/ttyUSB0",
            expected_controller=3,
            altitude=1.0,
            side=0.5,
            move=3.0,
            hold=2.0,
            yaw_step_deg=0.0,
            yaw_hold=3.0,
        )

        with self.assertRaisesRegex(RuntimeError, "restricted to SITL"):
            FLIGHT.validate_authorization(args)


class ComparisonAndArtifactTest(unittest.TestCase):
    def test_metric_margin_uses_absolute_floor_near_zero(self):
        decision = RUNNER.paired_metric_decision(
            0.0025,
            0.001,
            "track_rms",
        )

        self.assertAlmostEqual(decision["margin"], 0.002)
        self.assertAlmostEqual(decision["delta"], 0.0015)
        self.assertTrue(decision["passed"])
        self.assertEqual(decision["status"], "pass")

        failed = RUNNER.paired_metric_decision(
            0.0031,
            0.001,
            "track_rms",
        )
        self.assertFalse(failed["passed"])
        self.assertEqual(failed["status"], "fail")

    def test_invariant_and_descriptive_metrics_are_not_comparative_wins(self):
        invariant = RUNNER.paired_metric_decision(
            0.0,
            0.0,
            "unallocated_torque_max",
        )
        descriptive = RUNNER.paired_metric_decision(
            1.0,
            2.0,
            "motor_max",
        )

        self.assertIsNone(invariant["passed"])
        self.assertEqual(invariant["status"], "invariant")
        self.assertIsNone(descriptive["passed"])
        self.assertEqual(descriptive["status"], "descriptive")

    def test_paired_decisions_preserve_round_pairing(self):
        records = [
            {"round": 0, "controller": 2, "metrics": {"track_rms": 0.04}},
            {"round": 0, "controller": 3, "metrics": {"track_rms": 0.041}},
            {"round": 1, "controller": 3, "metrics": {"track_rms": 0.050}},
            {"round": 1, "controller": 2, "metrics": {"track_rms": 0.04}},
        ]
        decisions = RUNNER.comparison_decisions(records, 3, 2)

        self.assertTrue(decisions["0"]["track_rms"]["passed"])
        self.assertFalse(decisions["1"]["track_rms"]["passed"])

    def test_comparative_failures_are_detectable(self):
        records = [
            {"round": 0, "controller": 3, "metrics": {"track_rms": 0.05}},
            {"round": 0, "controller": 2, "metrics": {"track_rms": 0.04}},
        ]
        decisions = RUNNER.comparison_decisions(records, 3, 2)
        failures = [
            (round_index, metric_name)
            for round_index, round_decisions in decisions.items()
            for metric_name, decision in round_decisions.items()
            if decision["passed"] is False
        ]

        self.assertEqual(failures, [("0", "track_rms")])

    def test_main_returns_two_for_comparative_failure(self):
        analyzed = ({"final_square": {"mean": 0.0}}, {}, [])
        arguments = [
            "run_uav985_mpc_acceptance.py",
            "--controller",
            "3",
            "--compare-controller",
            "2",
            "--campaign-log",
            "1,3,candidate.ulg",
            "--campaign-log",
            "1,2,reference.ulg",
        ]

        with mock.patch.object(sys, "argv", arguments), mock.patch.object(
            RUNNER, "analyze_acceptance_run", side_effect=[analyzed, analyzed]
        ), mock.patch.object(
            RUNNER,
            "comparison_metrics",
            side_effect=[{"track_rms": 0.05}, {"track_rms": 0.04}],
        ), mock.patch("builtins.print"):
            self.assertEqual(RUNNER.main(), 2)

    def test_campaign_artifact_is_json_serializable_and_complete(self):
        args = acceptance_args(2)
        args.sitl_target = "gz_uav985"
        args.side = 0.6
        args.altitude = 2.0
        args.yaw_step_deg = 30.0
        args.yaw_hold = 3.0
        config = GRID.GridConfig(3.2, 2.0, 0.75)
        result = {
            "parameters": {"MC_RATE_CTRL_T": 2, "MC_MSMC_ILIM_R": 0.3},
            "rate_timestamp_source": "timestamp_sample",
        }
        records = [
            {
                "round": 0,
                "controller": 2,
                "log": Path("baseline.ulg"),
                "metrics": {"track_rms": np.float64(0.04)},
                "failures": [],
                "result": result,
            }
        ]
        artifact = RUNNER.campaign_artifact(args, config, records, {})

        self.assertIn("MC_MSMC_ILIM_R", artifact["cards"]["2"])
        self.assertIn("metric_registry", artifact)
        self.assertEqual(artifact["records"][0]["log"], Path("baseline.ulg"))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "campaign.json"
            RUNNER.write_campaign_artifact(path, artifact)
            loaded = json.loads(path.read_text(encoding="utf-8"))

        self.assertEqual(loaded["records"][0]["log"], "baseline.ulg")
        self.assertIsInstance(loaded["git"]["dirty"], bool)

    def test_campaign_log_parser(self):
        self.assertEqual(
            RUNNER.parse_campaign_log("2,3,flight.ulg"),
            (1, 3, Path("flight.ulg")),
        )

        with self.assertRaises(argparse.ArgumentTypeError):
            RUNNER.parse_campaign_log("0,3,flight.ulg")


class AstsmcDiagnosticTest(unittest.TestCase):
    def test_valid_diagnostics(self):
        result, _ = synthetic_astsmc_data()

        self.assertTrue(result["runtime_valid"])
        self.assertTrue(result["timestamps_monotonic"])
        self.assertGreaterEqual(result["sample_rate_hz"], 199.0)
        self.assertEqual(result["dt_invalid_count"], 0)
        self.assertEqual(result["nonfinite_rows"], 0)
        self.assertEqual(result["allocator"]["stale_count"], 0)
        self.assertAlmostEqual(result["torque_timestamp_match_fraction"], 1.0)
        self.assertTrue(result["gains_match_parameters"])

    def test_valid_diagnostics_pass_acceptance_gates(self):
        result, _ = synthetic_astsmc_data()
        numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertTrue(all(math_check_passes(check) for check in numeric_checks))
        self.assertTrue(all(passed for _label, passed in boolean_checks))

    def test_runtime_recovery_is_visible_and_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["runtime_fault_latched"][20:] = True
        ast["runtime_fault_reason"][20:] = 1
        ast["state_recovery_active[1]"][20] = True
        ast["state_recovery_count[1]"][20:] = 1
        ast["recovery_trigger_sliding[1]"][20:] = -1.5
        ast["recovery_trigger_state[1]"][20:] = 4.7
        ast["recovery_trigger_reaching[1]"][20:] = 2.3
        ast["sliding_variable[1]"][20] = -1.5
        ast["reaching_input_limited[1]"][20] = -1.0
        result, _ = synthetic_astsmc_data_from_existing(ast)
        numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }

        self.assertTrue(result["runtime_fault_latched"])
        self.assertEqual(result["runtime_fault_reason"], 1)
        self.assertEqual(result["state_recovery_count_delta"], 1)
        self.assertEqual(result["axes"]["pitch"]["state_recovery_count_delta"], 1)
        self.assertIn("ASTSMC state-recovery interventions", failed_numeric)
        self.assertIn(("ASTSMC runtime fault clear", False), boolean_checks)

    def test_wrong_direction_large_error_escape_is_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["sliding_variable[0]"][30] = -1.5
        ast["reaching_input_limited[0]"][30] = 2.0
        result, _ = synthetic_astsmc_data_from_existing(ast)
        numeric_checks, _ = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }

        self.assertEqual(result["wrong_direction_escape_count"], 1)
        self.assertEqual(result["axes"]["roll"]["wrong_direction_escape_count"], 1)
        self.assertIn("ASTSMC wrong-direction large-error escapes", failed_numeric)

    def test_ground_containment_is_visible_and_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["ground_containment_active"][40:42] = True
        ast["ground_containment_count"][40:] = 1
        result, _ = synthetic_astsmc_data_from_existing(ast)
        numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }

        self.assertEqual(result["ground_containment_active_count"], 2)
        self.assertEqual(result["ground_containment_count_delta"], 1)
        self.assertIn("ASTSMC ground-containment interventions", failed_numeric)
        self.assertIn(("ASTSMC ground containment inactive", False), boolean_checks)

    def test_variation_regularization_is_reported_separately_from_saturation(self):
        _result, ast = synthetic_astsmc_data()
        ast["variation_weight[0]"][:] = 0.35
        ast["variation_regularization[0]"][10:30] = -0.002
        result, _ = synthetic_astsmc_data_from_existing(ast)
        roll = result["axes"]["roll"]

        self.assertEqual(roll["internal_saturation_fraction"], 0.0)
        self.assertEqual(roll["variation_regularization_active_count"], 20)
        self.assertAlmostEqual(
            roll["variation_regularization_active_fraction"],
            20 / len(ast["timestamp"]),
        )
        self.assertAlmostEqual(roll["variation_regularization"]["max"], 0.002)
        self.assertAlmostEqual(roll["variation_weight"]["max"], 0.35)

    def test_maybe_landed_samples_remain_airborne_for_type_three(self):
        _result, ast = synthetic_astsmc_data()
        sample_count = len(ast["timestamp"])
        timestamps = ast["timestamp_sample"].copy()
        rate_status = {
            "timestamp": timestamps.copy(),
            "controller_type": np.full(sample_count, 3, dtype=np.uint8),
            "astsmc_valid": np.ones(sample_count, dtype=bool),
        }
        land_detected = {
            "timestamp": np.array(
                [timestamps[0] - 100_000, timestamps[sample_count // 2]],
                dtype=np.uint64,
            ),
            "landed": np.array([False, False]),
            "maybe_landed": np.array([False, True]),
        }
        torque_setpoint = {"timestamp_sample": timestamps.copy()}

        for axis in range(3):
            torque_setpoint[f"xyz[{axis}]"] = ast[f"torque_limited[{axis}]"].copy()

        result = ANALYZER.analyze_astsmc_diagnostics(
            ast,
            ast,
            rate_status,
            land_detected,
            torque_setpoint,
            {
                "MC_AST_K1_R": 3.0,
                "MC_AST_K1_P": 3.0,
                "MC_AST_K1_Y": 1.5,
                "MC_AST_K2_R": 4.5,
                "MC_AST_K2_P": 4.5,
                "MC_AST_K2_Y": 1.5,
            },
            timestamps[0] / 1e6,
            timestamps[-1] / 1e6,
            timestamps[-1] / 1e6,
            np.array([0.2, 0.2, 0.15]),
        )

        self.assertEqual(result["airborne_samples"], sample_count)

    def test_first_airborne_sample_is_checked(self):
        _result, ast = synthetic_astsmc_data()
        ast["dt_valid"][0] = False
        ast["accepted_dt"][0] = np.nan
        ast["invalid_dt_hold_count"][:] += 1
        ast["valid_update_count"][:] -= 1
        result, _ = synthetic_astsmc_data_from_existing(ast)

        self.assertEqual(result["dt_invalid_count"], 0)
        self.assertTrue(result["accepted_dt_consistent"])
        self.assertEqual(result["airborne_samples"], len(ast["timestamp"]))

    def test_invalid_dt_is_counted_and_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["dt_valid"][10] = False
        ast["accepted_dt"][10] = np.nan
        ast["invalid_dt_hold_count"][10:] += 1
        ast["valid_update_count"][10:] -= 1
        result, _ = synthetic_astsmc_data_from_existing(ast)
        numeric_checks, _ = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertEqual(result["dt_invalid_count"], 1)
        self.assertGreater(result["dt_invalid_fraction"], 0.0)
        self.assertIn(
            "ASTSMC invalid-dt fraction",
            [label for label, value, operator, threshold in numeric_checks
             if not math_check_passes((label, value, operator, threshold))],
        )

    def test_nonfinite_field_is_not_hidden(self):
        _result, ast = synthetic_astsmc_data()
        ast["torque_raw[0]"][20] = np.nan
        result, _ = synthetic_astsmc_data_from_existing(ast)
        _numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertEqual(result["nonfinite_rows"], 1)
        self.assertIn("torque_raw[0]", result["nonfinite_fields"])
        self.assertIn(
            ("ASTSMC diagnostics finite", False),
            boolean_checks,
        )

    def test_first_airborne_raw_dt_may_be_uninitialized(self):
        _result, ast = synthetic_astsmc_data()
        ast["raw_dt"][0] = np.nan
        result, _ = synthetic_astsmc_data_from_existing(ast)

        self.assertEqual(result["nonfinite_rows"], 0)
        self.assertNotIn("raw_dt", result["nonfinite_fields"])

    def test_later_nonfinite_raw_dt_is_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["raw_dt"][20] = np.nan
        result, _ = synthetic_astsmc_data_from_existing(ast)
        _numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertEqual(result["nonfinite_rows"], 1)
        self.assertIn("raw_dt", result["nonfinite_fields"])
        self.assertIn(("ASTSMC diagnostics finite", False), boolean_checks)

    def test_accepted_dt_inconsistency_is_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["accepted_dt"][20] = np.nan
        result, _ = synthetic_astsmc_data_from_existing(ast)
        _numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertFalse(result["accepted_dt_consistent"])
        self.assertIn(
            ("ASTSMC accepted dt matches validity and bounds", False),
            boolean_checks,
        )

    def test_consecutive_invalid_hold_counter_survives_downsampling(self):
        _result, ast = synthetic_astsmc_data()
        ast["invalid_dt_hold_count"][10:] += 2
        ast["consecutive_invalid_dt_hold_count"][10:] += 1
        ast["valid_update_count"][10:] -= 2
        result, _ = synthetic_astsmc_data_from_existing(ast)

        self.assertEqual(result["dt_invalid_count"], 2)
        self.assertEqual(result["dt_invalid_consecutive_count"], 1)

    def test_counter_reset_is_rejected_without_unsigned_overflow(self):
        _result, ast = synthetic_astsmc_data()
        ast["valid_update_count"][200:] -= 150
        result, _ = synthetic_astsmc_data_from_existing(ast)
        _numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )

        self.assertGreater(result["counter_reset_count"], 0)
        self.assertLess(result["valid_update_count_delta"], 1000)
        self.assertIn(
            ("ASTSMC counters monotonic or wrapped", False),
            boolean_checks,
        )

    def test_saturation_and_allocator_failures_are_reported(self):
        _result, ast = synthetic_astsmc_data()
        ast["internal_saturation[2]"][:80] = True
        ast["torque_raw[2]"][:80] = 0.3
        ast["reaching_input_raw[2]"][:80] = 30.0
        ast["allocator_feedback_stale"][5] = True
        ast["allocator_feedback_valid"][6] = False
        ast["allocation_residual[0]"][7] = 0.02
        ast["allocator_timestamp_sample"][8] += 10_000
        result, _ = synthetic_astsmc_data_from_existing(ast)

        self.assertGreater(result["axes"]["yaw"]["internal_saturation_fraction"], 0.05)
        self.assertGreater(result["allocator"]["stale_fraction"], 0.0)
        self.assertLess(result["allocator"]["feedback_valid_fraction"], 1.0)
        self.assertGreater(result["allocator"]["residual_norm"]["max"], 0.005)
        self.assertEqual(result["allocator"]["future_timestamp_count"], 1)
        numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }
        self.assertIn("yaw ASTSMC internal saturation", failed_numeric)
        self.assertIn("ASTSMC stale allocator-feedback fraction", failed_numeric)
        self.assertIn("ASTSMC allocator residual norm", failed_numeric)
        self.assertIn(
            ("ASTSMC allocator timestamps not in future", False),
            boolean_checks,
        )

    def test_gain_mismatch_and_torque_bound_are_rejected(self):
        _result, ast = synthetic_astsmc_data()
        ast["k1[0]"][:] = 4.0
        ast["torque_limited[1]"][30] = 0.25
        ast["torque_raw[1]"][30] = 0.25
        result, _ = synthetic_astsmc_data_from_existing(ast)
        numeric_checks, boolean_checks = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }
        self.assertIn("pitch ASTSMC torque bound excess", failed_numeric)
        self.assertIn(
            ("ASTSMC active gains match logged card", False),
            boolean_checks,
        )

    def test_rate_error_decomposition_uses_exact_sample_timestamps(self):
        _result, ast = synthetic_astsmc_data()
        sample_count = len(ast["timestamp"])

        for axis in range(3):
            shaped = 0.03 * np.sin(np.arange(sample_count) * 0.02 + axis)
            raw = shaped + 0.01 * np.cos(np.arange(sample_count) * 0.03 + axis)
            measured = shaped - 0.005 * np.sin(np.arange(sample_count) * 0.05 + axis)
            ast[f"rate_setpoint_raw[{axis}]"] = raw
            ast[f"rate_setpoint_shaped[{axis}]"] = shaped
            ast[f"sliding_variable_raw[{axis}]"] = raw - measured
            ast[f"sliding_variable[{axis}]"] = shaped - measured

        result, _ = synthetic_astsmc_data_from_existing(
            ast, include_angular_velocity=True
        )

        self.assertEqual(result["rate_timestamp_source"], "timestamp_sample")
        self.assertAlmostEqual(result["rate_timestamp_match_fraction"], 1.0)
        self.assertLess(result["rate_error_identity_max"], 1e-12)

        for axis in ANALYZER.AXES:
            tracking = result["tracking"][axis]
            self.assertGreater(tracking["raw_error"]["rms"], 0.0)
            self.assertGreater(tracking["reference_shaping_lag"]["rms"], 0.0)
            self.assertGreater(tracking["conditioned_error"]["rms"], 0.0)
            self.assertLess(tracking["identity_error"]["max"], 1e-12)

    def test_rate_error_decomposition_rejects_publication_only_match(self):
        _result, ast = synthetic_astsmc_data()
        result, _ = synthetic_astsmc_data_from_existing(
            ast,
            include_angular_velocity=True,
            angular_timestamp_offset=1,
        )

        self.assertEqual(result["rate_timestamp_match_count"], 0)
        self.assertEqual(result["rate_timestamp_match_fraction"], 0.0)
        self.assertFalse(result["tracking"])

    def test_torque_component_dynamics_reconcile_with_total(self):
        result, _ = synthetic_astsmc_data()

        for axis in ANALYZER.AXES:
            components = result["axes"][axis]["torque_components"]
            self.assertEqual(components["nominal"]["rms"], 0.0)
            self.assertAlmostEqual(
                components["residual"]["rms"],
                components["total"]["rms"],
            )
            self.assertAlmostEqual(
                components["residual"]["variation_per_s"],
                components["total"]["variation_per_s"],
            )
            self.assertEqual(components["nominal"]["saturation_fraction"], 0.0)
            self.assertEqual(components["residual"]["saturation_fraction"], 0.0)

    def test_low_rate_and_command_mismatch_are_reported(self):
        result, _ = synthetic_astsmc_data(sample_count=20, rate_hz=5.0)
        numeric_checks, _ = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }
        self.assertLess(result["sample_rate_hz"], 100.0)
        self.assertIn("ASTSMC diagnostic rate", failed_numeric)

        _result, ast = synthetic_astsmc_data()
        result, torque = synthetic_astsmc_data_from_existing(ast, torque_offset=0.01)
        numeric_checks, _ = RUNNER.astsmc_threshold_checks(
            {"astsmc_diagnostics": result}, acceptance_args(3)
        )
        failed_numeric = {
            label for label, value, operator, threshold in numeric_checks
            if not math_check_passes((label, value, operator, threshold))
        }
        self.assertGreater(result["command_mismatch"]["roll"]["max"], 0.009)
        self.assertIn("roll ASTSMC published command mismatch", failed_numeric)
        self.assertIsNotNone(torque)


def synthetic_astsmc_data_from_existing(
    ast,
    torque_offset=0.0,
    include_angular_velocity=False,
    angular_timestamp_offset=0,
):
    sample_count = len(ast["timestamp"])
    timestamps = ast["timestamp_sample"].copy()
    rate_status = {
        "timestamp": timestamps.copy(),
        "controller_type": np.full(sample_count, 3, dtype=np.uint8),
        "astsmc_valid": np.ones(sample_count, dtype=bool),
    }
    land_detected = {
        "timestamp": np.array([timestamps[0] - 100_000], dtype=np.uint64),
        "landed": np.array([False]),
        "maybe_landed": np.array([False]),
    }
    torque_setpoint = {"timestamp_sample": timestamps.copy()}

    for axis in range(3):
        torque_setpoint[f"xyz[{axis}]"] = ast[f"torque_limited[{axis}]"].copy()

    torque_setpoint["xyz[0]"] += torque_offset
    parameters = {
        "MC_AST_K1_R": 3.0,
        "MC_AST_K1_P": 3.0,
        "MC_AST_K1_Y": 1.5,
        "MC_AST_K2_R": 4.5,
        "MC_AST_K2_P": 4.5,
        "MC_AST_K2_Y": 1.5,
        "MC_AST_REC_ERR": 1.0,
    }
    start_s = timestamps[0] / 1e6
    end_s = timestamps[-1] / 1e6
    angular_velocity = None

    if include_angular_velocity:
        angular_velocity = {
            "timestamp": timestamps + 500,
            "timestamp_sample": timestamps + angular_timestamp_offset,
        }

        for axis in range(3):
            raw_setpoint = ast[f"rate_setpoint_raw[{axis}]"].astype(float)
            raw_error = ast[f"sliding_variable_raw[{axis}]"].astype(float)
            angular_velocity[f"xyz[{axis}]"] = raw_setpoint - raw_error

    result = ANALYZER.analyze_astsmc_diagnostics(
        ast,
        ast,
        rate_status,
        land_detected,
        torque_setpoint,
        parameters,
        start_s,
        end_s,
        end_s,
        np.array([0.2, 0.2, 0.15]),
        angular_velocity=angular_velocity,
    )
    return result, torque_setpoint


if __name__ == "__main__":
    unittest.main()
