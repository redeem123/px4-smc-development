#!/usr/bin/env python3
import importlib.util
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("analyze_smc_flight.py")
SPEC = importlib.util.spec_from_file_location("analyze_smc_flight", MODULE_PATH)
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


class AnalyzeSmcFlightTest(unittest.TestCase):
    def test_true_segments_filters_short_windows(self):
        timestamps = np.arange(0.0, 2.0, 0.1)
        mask = np.zeros(len(timestamps), dtype=bool)
        mask[1:3] = True
        mask[5:15] = True

        self.assertEqual(ANALYZER.true_segments(mask, timestamps, 0.5), [(5, 14)])

    def test_reset_safe_counter_delta_handles_counter_reset(self):
        values = np.array([10, 12, 15, 1, 4], dtype=np.uint32)

        self.assertEqual(
            ANALYZER.reset_safe_counter_delta(values, np.arange(len(values))),
            9,
        )

    def test_exclude_astsmc_ground_containment_removes_contained_samples(self):
        timestamps = np.arange(0.0, 5.0)
        safety = {
            "timestamp_sample": (timestamps * 1e6).astype(np.uint64),
            "ground_containment_active": np.array([True, True, False, False, True]),
        }

        active = ANALYZER.exclude_astsmc_ground_containment(
            timestamps,
            np.ones(5, dtype=bool),
            safety,
        )

        np.testing.assert_array_equal(active, [False, False, True, True, False])

    def test_command_release_metrics_reports_safe_settling(self):
        timestamps = np.arange(0.0, 2.0, 0.02)
        setpoint = np.where(timestamps < 0.5, 1.0, 0.0)
        rate = np.where(timestamps < 0.5, 1.0, np.exp(-(timestamps - 0.5) * 12.0))
        torque = np.where(timestamps < 0.5, 0.03, -0.02)

        releases = ANALYZER.command_release_metrics(timestamps, setpoint, rate, torque)

        self.assertEqual(len(releases), 1)
        self.assertLess(releases[0]["peak_opposite_rate"], 1e-9)
        self.assertLessEqual(releases[0]["settling_time_s"], 1.0)
        self.assertEqual(releases[0]["wrong_direction_torque_fraction"], 0.0)

    def test_command_release_metrics_detects_rebound_and_wrong_torque(self):
        timestamps = np.arange(0.0, 2.0, 0.02)
        setpoint = np.where(timestamps < 0.5, 1.0, 0.0)
        rate = np.where(timestamps < 0.5, 1.0, -0.4 * np.exp(-(timestamps - 0.5) * 0.5))
        torque = np.where(timestamps < 0.5, 0.03, 0.04)

        releases = ANALYZER.command_release_metrics(timestamps, setpoint, rate, torque)

        self.assertEqual(len(releases), 1)
        self.assertGreater(releases[0]["peak_opposite_rate"], 0.3)
        self.assertGreater(releases[0]["wrong_direction_torque_fraction"], 0.9)
        self.assertFalse(np.isfinite(releases[0]["settling_time_s"]))

    def test_command_release_metrics_accepts_no_maneuver(self):
        timestamps = np.arange(0.0, 2.0, 0.02)
        releases = ANALYZER.command_release_metrics(
            timestamps,
            np.zeros_like(timestamps),
            np.zeros_like(timestamps),
            np.zeros_like(timestamps),
        )

        self.assertEqual(releases, [])

    def test_high_frequency_metrics_detects_four_hz_oscillation(self):
        timestamps = np.arange(0.0, 6.0, 1.0 / 250.0)
        signal = 0.2 * np.sin(2.0 * np.pi * 4.5 * timestamps)

        result = ANALYZER.high_frequency_metrics(timestamps, signal, cutoff_hz=4.0)

        self.assertGreater(result["rms"], 0.08)
        self.assertAlmostEqual(result["peak_hz"], 4.5, delta=0.2)
        self.assertGreater(result["peak_amplitude"], 0.08)

    def test_high_frequency_metrics_rejects_slow_motion(self):
        timestamps = np.arange(0.0, 6.0, 1.0 / 250.0)
        signal = 0.2 * np.sin(2.0 * np.pi * 1.0 * timestamps)

        result = ANALYZER.high_frequency_metrics(timestamps, signal, cutoff_hz=4.0)

        self.assertLess(result["rms"], 0.08)

    def test_astsmc_safety_summary_reports_interventions(self):
        timestamps = (1_000_000 + np.arange(10) * 100_000).astype(np.uint64)
        status = {
            "timestamp_sample": timestamps,
            "runtime_fault_latched": np.array([False] * 3 + [True] * 7),
            "runtime_fault_reason": np.array([0] * 3 + [1] * 7, dtype=np.uint32),
            "ground_containment_count": np.array([0] * 7 + [1] * 3, dtype=np.uint32),
        }

        for axis in range(3):
            status[f"state_recovery_count[{axis}]"] = np.zeros(10, dtype=np.uint32)
            status[f"state_recovery_active[{axis}]"] = np.zeros(10, dtype=bool)
            status[f"sliding_variable[{axis}]"] = np.zeros(10)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(10)

        status["state_recovery_count[1]"][3:] = 1
        status["state_recovery_active[1]"][3] = True
        summary = ANALYZER.astsmc_safety_summary(status, None, 1.0, 1.9)

        self.assertTrue(summary["runtime_fault_latched"])
        self.assertEqual(summary["runtime_fault_reason"], 1)
        self.assertEqual(summary["state_recovery_count_delta"], 1)
        self.assertEqual(summary["ground_containment_count_delta"], 1)
        self.assertIsNone(summary["residual_limit_occupancy"])

    def test_astsmc_safety_summary_reports_reset_safe_limit_occupancy(self):
        timestamps = (1_000_000 + np.arange(6) * 100_000).astype(np.uint64)
        status = {
            "timestamp_sample": timestamps,
            "valid_update_count": np.array([10, 20, 30, 2, 12, 20], dtype=np.uint32),
            "runtime_fault_latched": np.zeros(6, dtype=bool),
            "runtime_fault_reason": np.zeros(6, dtype=np.uint32),
            "ground_containment_count": np.zeros(6, dtype=np.uint32),
        }

        for axis in range(3):
            status[f"state_recovery_count[{axis}]"] = np.zeros(6, dtype=np.uint32)
            status[f"state_recovery_active[{axis}]"] = np.zeros(6, dtype=bool)
            status[f"sliding_variable[{axis}]"] = np.zeros(6)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(6)
            status[f"nominal_limit_count[{axis}]"] = np.zeros(6, dtype=np.uint32)
            status[f"residual_limit_count[{axis}]"] = np.zeros(6, dtype=np.uint32)

        status["residual_limit_count[2]"] = np.array(
            [5, 15, 25, 1, 11, 19], dtype=np.uint32
        )
        summary = ANALYZER.astsmc_safety_summary(status, None, 1.0, 1.5)

        self.assertEqual(summary["valid_update_count_delta"], 40)
        self.assertEqual(summary["residual_limit_count_delta"], [0, 0, 39])
        self.assertAlmostEqual(summary["residual_limit_occupancy"][2], 39 / 40)

    def test_astsmc_safety_summary_reports_roll_pitch_authority_release(self):
        timestamps = (1_000_000 + np.arange(6) * 100_000).astype(np.uint64)
        status = {"timestamp_sample": timestamps}

        for axis in range(3):
            status[f"sliding_variable[{axis}]"] = np.zeros(6)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(6)

        status["residual_authority[0]"] = np.array(
            [0.1, 0.1, 0.15, 0.2, 0.2, 0.1]
        )
        status["residual_authority[1]"] = np.array(
            [0.1, 0.125, 0.15, 0.175, 0.2, 0.1]
        )
        summary = ANALYZER.astsmc_safety_summary(
            status,
            None,
            1.0,
            1.5,
            roll_pitch_residual_card=[(0.1, 0.2), (0.1, 0.2)],
        )

        self.assertEqual(summary["roll_pitch_residual_authority_base"], [0.1, 0.1])
        self.assertEqual(summary["roll_pitch_residual_authority_max"], [0.2, 0.2])
        self.assertAlmostEqual(
            summary["roll_pitch_residual_authority_extension_fraction"][0], 2.5 / 6
        )
        self.assertAlmostEqual(
            summary["roll_pitch_residual_authority_extension_fraction"][1], 2.5 / 6
        )
        self.assertAlmostEqual(
            summary["roll_pitch_residual_authority_full_fraction"][0], 2 / 6
        )
        self.assertAlmostEqual(
            summary["roll_pitch_residual_authority_full_fraction"][1], 1 / 6
        )

    def test_astsmc_safety_summary_reports_selective_release(self):
        timestamps = (1_000_000 + np.arange(5) * 100_000).astype(np.uint64)
        status = {"timestamp_sample": timestamps}
        safety = {
            "timestamp_sample": timestamps,
            "runtime_fault_latched": np.zeros(5, dtype=bool),
            "runtime_fault_reason": np.zeros(5, dtype=np.uint32),
            "ground_containment_count": np.zeros(5, dtype=np.uint32),
            "selective_release_count[0]": np.array([0, 0, 1, 1, 2], dtype=np.uint32),
            "selective_release_count[1]": np.array([0, 0, 0, 1, 1], dtype=np.uint32),
            "quiet_anchor_count[0]": np.array([0, 0, 1, 1, 1], dtype=np.uint32),
            "quiet_anchor_count[1]": np.array([0, 0, 0, 1, 1], dtype=np.uint32),
            "quiet_anchor_active[0]": np.array([False, False, True, True, False]),
            "quiet_anchor_active[1]": np.array([False, False, False, True, True]),
            "deep_quiet_active[0]": np.array([False, False, True, True, True]),
            "deep_quiet_active[1]": np.array([False, False, False, True, False]),
            "trim_valid[0]": np.array([False, True, True, True, True]),
            "trim_valid[1]": np.array([False, False, True, True, True]),
            "trim_confidence[0]": np.array([0.1, 0.5, 0.5, 0.5, 0.5]),
            "trim_confidence[1]": np.array([0.0, 0.2, 0.5, 0.5, 0.5]),
        }

        for axis in range(3):
            status[f"sliding_variable[{axis}]"] = np.zeros(5)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(5)
            safety[f"state_recovery_count[{axis}]"] = np.zeros(5, dtype=np.uint32)
            safety[f"state_recovery_active[{axis}]"] = np.zeros(5, dtype=bool)

        summary = ANALYZER.astsmc_safety_summary(status, safety, 1.0, 1.4)

        self.assertEqual(summary["selective_release_count_delta"], [2, 1])
        self.assertEqual(summary["quiet_anchor_count_delta"], [1, 1])
        self.assertAlmostEqual(summary["quiet_anchor_active_fraction"][0], 0.4)
        self.assertAlmostEqual(summary["quiet_anchor_active_fraction"][1], 0.4)
        self.assertAlmostEqual(summary["deep_quiet_active_fraction"][0], 0.6)
        self.assertAlmostEqual(summary["deep_quiet_active_fraction"][1], 0.2)
        self.assertEqual(summary["trim_valid"], [True, True])
        self.assertEqual(summary["trim_confidence_max"], [0.5, 0.5])
        self.assertFalse(summary["runtime_fault_latched"])

    def test_astsmc_safety_summary_accepts_separate_safety_topic(self):
        timestamps = (1_000_000 + np.arange(5) * 100_000).astype(np.uint64)
        status = {"timestamp_sample": timestamps}
        safety = {
            "timestamp_sample": timestamps,
            "runtime_fault_latched": np.array([False, False, True, True, True]),
            "runtime_fault_reason": np.array([0, 0, 1, 1, 1], dtype=np.uint32),
            "ground_containment_count": np.zeros(5, dtype=np.uint32),
        }

        for axis in range(3):
            status[f"sliding_variable[{axis}]"] = np.zeros(5)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(5)
            safety[f"state_recovery_count[{axis}]"] = np.zeros(5, dtype=np.uint32)
            safety[f"state_recovery_active[{axis}]"] = np.zeros(5, dtype=bool)

        safety["state_recovery_count[0]"][2:] = 1
        safety["state_recovery_active[0]"][2] = True
        summary = ANALYZER.astsmc_safety_summary(status, safety, 1.0, 1.4)

        self.assertTrue(summary["schema_complete"])
        self.assertTrue(summary["runtime_fault_latched"])
        self.assertEqual(summary["state_recovery_count_delta"], 1)

    def test_astsmc_safety_summary_detects_wrong_direction_escape(self):
        timestamps = (1_000_000 + np.arange(5) * 100_000).astype(np.uint64)
        status = {
            "timestamp_sample": timestamps,
            "runtime_fault_latched": np.zeros(5, dtype=bool),
            "runtime_fault_reason": np.zeros(5, dtype=np.uint32),
            "ground_containment_count": np.zeros(5, dtype=np.uint32),
        }

        for axis in range(3):
            status[f"state_recovery_count[{axis}]"] = np.zeros(5, dtype=np.uint32)
            status[f"state_recovery_active[{axis}]"] = np.zeros(5, dtype=bool)
            status[f"sliding_variable[{axis}]"] = np.zeros(5)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(5)

        status["sliding_variable[0]"][2] = -1.5
        status["reaching_input_limited[0]"][2] = 2.0
        summary = ANALYZER.astsmc_safety_summary(status, None, 1.0, 1.4)

        self.assertEqual(summary["wrong_direction_escape_count"], 1)

    def test_legacy_astsmc_schema_still_exposes_wrong_direction_escape(self):
        timestamps = (1_000_000 + np.arange(5) * 100_000).astype(np.uint64)
        status = {"timestamp_sample": timestamps}

        for axis in range(3):
            status[f"sliding_variable[{axis}]"] = np.zeros(5)
            status[f"reaching_input_limited[{axis}]"] = np.zeros(5)

        status["sliding_variable[1]"][2] = -1.5
        status["reaching_input_limited[1]"][2] = 2.0
        summary = ANALYZER.astsmc_safety_summary(status, None, 1.0, 1.4)

        self.assertFalse(summary["schema_complete"])
        self.assertIsNone(summary["runtime_fault_latched"])
        self.assertEqual(summary["wrong_direction_escape_count"], 1)

    def test_astsmc_allocator_summary_reports_governor_activity(self):
        timestamps = (1_000_000 + np.arange(6) * 100_000).astype(np.uint64)
        allocator = {
            "timestamp_sample": timestamps,
            "publication_age_s": np.array([0.002, 0.003, 0.004, 0.006, 0.008, 0.01]),
            "sample_age_s": np.array([0.004, 0.005, 0.006, 0.008, 0.01, 0.012]),
            "allocation_residual_norm": np.array([0.0, 0.0, 0.0, 0.0012, 0.0, 0.0]),
            "yaw_residual_authority[0]": np.full(6, 0.1),
            "yaw_residual_authority[1]": np.full(6, 0.15),
            "yaw_residual_authority[2]": np.array([0.1, 0.11, 0.12, 0.1, 0.1, 0.11]),
            "event_count[2]": np.array([0, 0, 0, 1, 1, 1], dtype=np.uint32),
            "event_count[3]": np.array([0, 0, 0, 0, 1, 1], dtype=np.uint32),
            "state_flags": np.array([2, 2 | 64, 2 | 64, 2 | 32, 0, 2 | 64], dtype=np.uint16),
        }

        summary = ANALYZER.astsmc_allocator_summary(allocator, 1.0, 1.5)

        self.assertTrue(summary["schema_complete"])
        self.assertAlmostEqual(summary["usable_feedback_fraction"], 5 / 6)
        self.assertAlmostEqual(summary["publication_age_max_s"], 0.01)
        self.assertAlmostEqual(summary["sample_age_max_s"], 0.012)
        self.assertAlmostEqual(summary["effective_authority_fraction"], 0.8 / 6)
        self.assertAlmostEqual(summary["effective_authority_max"], 0.12)
        self.assertEqual(summary["release_count_delta"], 2)
        self.assertEqual(summary["backoff_count_delta"], 1)
        self.assertEqual(summary["fallback_count_delta"], 1)
        self.assertAlmostEqual(summary["residual_at_backoff_max"], 0.0012)

    def test_astsmc_allocator_summary_reports_headroom_and_conditioning(self):
        timestamps = (1_000_000 + np.arange(6) * 100_000).astype(np.uint64)
        allocator = {
            "timestamp_sample": timestamps,
            "publication_age_s": np.full(6, 0.002),
            "sample_age_s": np.full(6, 0.004),
            "allocation_residual_norm": np.array([0.0, 0.02, 0.03, 0.0, 0.0, 0.0]),
            "yaw_residual_authority[0]": np.full(6, 0.1),
            "yaw_residual_authority[1]": np.full(6, 0.1),
            "yaw_residual_authority[2]": np.full(6, 0.1),
            "event_count[2]": np.zeros(6, dtype=np.uint32),
            "event_count[3]": np.zeros(6, dtype=np.uint32),
            "event_count[4]": np.arange(6, dtype=np.uint32),
            "event_count[5]": np.array([0, 1, 1, 1, 1, 1], dtype=np.uint32),
            "requested_allocation_policy": np.array([1, 1, 1, 1, 0, 0], dtype=np.uint8),
            "applied_allocation_policy": np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8),
            "roll_pitch_achieved_fraction": np.array([1.0, 0.8, 0.7, 1.0, 1.0, 1.0]),
            "state_flags": np.array([2, 2 | 512, 2 | 512 | 1024, 2, 2, 2], dtype=np.uint16),
        }

        summary = ANALYZER.astsmc_allocator_summary(allocator, 1.0, 1.5)

        self.assertTrue(summary["headroom_schema_complete"])
        self.assertAlmostEqual(summary["headroom_request_fraction"], 4 / 6)
        self.assertAlmostEqual(summary["headroom_honor_fraction"], 3 / 4)
        self.assertEqual(summary["headroom_request_count_delta"], 5)
        self.assertEqual(summary["roll_pitch_miss_count_delta"], 1)
        self.assertAlmostEqual(summary["roll_pitch_achieved_fraction_mean"], 5.5 / 6)
        self.assertAlmostEqual(summary["roll_pitch_achieved_fraction_min"], 0.7)
        self.assertAlmostEqual(summary["roll_pitch_allocation_loss_longest_s"], 0.1)
        self.assertAlmostEqual(summary["roll_conditioning_fraction"], 2 / 6)
        self.assertAlmostEqual(summary["pitch_conditioning_fraction"], 1 / 6)

    def test_astsmc_allocator_summary_accepts_historical_schema_without_headroom_fields(self):
        timestamps = (1_000_000 + np.arange(2) * 100_000).astype(np.uint64)
        allocator = {
            "timestamp_sample": timestamps,
            "publication_age_s": np.full(2, 0.002),
            "sample_age_s": np.full(2, 0.004),
            "allocation_residual_norm": np.zeros(2),
            "yaw_residual_authority[0]": np.full(2, 0.1),
            "yaw_residual_authority[1]": np.full(2, 0.1),
            "yaw_residual_authority[2]": np.full(2, 0.1),
            "event_count[2]": np.zeros(2, dtype=np.uint32),
            "event_count[3]": np.zeros(2, dtype=np.uint32),
            "state_flags": np.full(2, 2, dtype=np.uint16),
        }

        summary = ANALYZER.astsmc_allocator_summary(allocator, 1.0, 1.1)

        self.assertTrue(summary["schema_complete"])
        self.assertFalse(summary["headroom_schema_complete"])
        self.assertIsNone(summary["headroom_request_fraction"])

    def test_astsmc_allocator_summary_accepts_historical_log_without_topic(self):
        summary = ANALYZER.astsmc_allocator_summary(None, 1.0, 2.0)

        self.assertFalse(summary["present"])
        self.assertFalse(summary["schema_complete"])
        self.assertIsNone(summary["usable_feedback_fraction"])

    def test_check_result_allows_observed_ground_containment(self):
        result = {
            "segments": [
                {
                    "duration_s": 2.0,
                    "terminal_duration_s": 2.0,
                    "astsmc_safety": {
                        "present": True,
                        "schema_complete": True,
                        "runtime_fault_latched": False,
                        "runtime_fault_reason": 0,
                        "state_recovery_count_delta": 0,
                        "ground_containment_count_delta": 3,
                        "residual_limit_occupancy": [0.0, 0.0, 0.0],
                        "wrong_direction_escape_count": 0,
                    },
                    "yaw_attitude": {
                        "available": True,
                        "error_rms_deg": 0.5,
                        "error_abs_max_deg": 1.0,
                        "final_error_abs_deg": 0.25,
                    },
                    "axes": {
                        axis: {
                            "rate_error_rms": 0.0,
                            "rate_abs_max": 0.0,
                            "torque_abs_max": 0.0,
                            "oscillation_rms": 0.0,
                            "peak_hz": 0.0,
                            "peak_amplitude": 0.0,
                            "first_half_oscillation_rms": 0.0,
                            "second_half_oscillation_rms": 0.0,
                            "growth_ratio": 0.0,
                            "command_releases": [],
                        }
                        for axis in ANALYZER.AXES
                    },
                }
            ]
        }

        failures = ANALYZER.check_result(result, 1.0, 1.0, 10.0)

        self.assertEqual(failures, [])

    def test_check_result_rejects_log_810_like_yaw_failure(self):
        result = {
            "segments": [
                {
                    "duration_s": 50.0,
                    "terminal_duration_s": 6.0,
                    "astsmc_safety": {
                        "present": True,
                        "schema_complete": True,
                        "runtime_fault_latched": False,
                        "runtime_fault_reason": 0,
                        "state_recovery_count_delta": 0,
                        "ground_containment_count_delta": 0,
                        "residual_limit_occupancy": [0.0, 0.0, 0.87],
                        "wrong_direction_escape_count": 0,
                    },
                    "yaw_attitude": {
                        "available": True,
                        "error_rms_deg": 22.7,
                        "error_abs_max_deg": 37.5,
                        "final_error_abs_deg": 30.0,
                    },
                    "axes": {
                        "roll": {
                            "rate_error_rms": 0.05,
                            "rate_abs_max": 0.1,
                            "torque_abs_max": 0.03,
                            "oscillation_rms": 0.02,
                            "peak_hz": 12.0,
                            "peak_amplitude": 0.01,
                            "first_half_oscillation_rms": 0.02,
                            "second_half_oscillation_rms": 0.02,
                            "growth_ratio": 1.0,
                            "command_releases": [],
                        },
                        "pitch": {
                            "rate_error_rms": 0.05,
                            "rate_abs_max": 0.1,
                            "torque_abs_max": 0.04,
                            "oscillation_rms": 0.02,
                            "peak_hz": 12.0,
                            "peak_amplitude": 0.01,
                            "first_half_oscillation_rms": 0.02,
                            "second_half_oscillation_rms": 0.02,
                            "growth_ratio": 1.0,
                            "command_releases": [],
                        },
                        "yaw": {
                            "rate_error_rms": 0.27,
                            "rate_abs_max": 0.3,
                            "torque_abs_max": 0.06,
                            "oscillation_rms": 0.02,
                            "peak_hz": 12.0,
                            "peak_amplitude": 0.01,
                            "first_half_oscillation_rms": 0.02,
                            "second_half_oscillation_rms": 0.02,
                            "growth_ratio": 1.0,
                            "command_releases": [],
                        },
                    },
                }
            ]
        }

        failures = ANALYZER.check_result(result, 0.08, 0.12, 2.5)

        self.assertIn("segment 1 ASTSMC yaw residual-limit occupancy", failures)
        self.assertIn("segment 1 yaw terminal rate-error RMS", failures)
        self.assertIn("segment 1 yaw final attitude error", failures)
        self.assertFalse(any("roll" in failure for failure in failures))
        self.assertFalse(any("pitch" in failure for failure in failures))

    def test_zero_order_hold_supports_offboard_window_selection(self):
        sample_timestamps = np.arange(0.0, 5.0, 0.5)
        status_timestamps = np.array([0.0, 1.0, 4.0])
        offboard_values = np.array([0.0, 1.0, 0.0])
        offboard = ANALYZER.zero_order_hold(
            sample_timestamps,
            status_timestamps,
            offboard_values,
            0.0,
        ) > 0.5

        self.assertEqual(
            ANALYZER.true_segments(offboard, sample_timestamps, 0.5),
            [(2, 7)],
        )


if __name__ == "__main__":
    unittest.main()
