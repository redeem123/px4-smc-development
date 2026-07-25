#!/usr/bin/env python3
"""Tests for ASTSMC residual-bound identification helpers."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("identify_astsmc_residual_bounds.py")
SPEC = importlib.util.spec_from_file_location("identify_astsmc_residual_bounds", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VelocityReferenceTest(unittest.TestCase):
    def test_reference_respects_acceleration_and_jerk_limits(self):
        reference = MODULE.VelocityReference(2.0, 10.0)
        previous_acceleration = 0.0

        for sample in range(300):
            dt = 0.004 if sample % 2 == 0 else 0.008
            velocity, acceleration, jerk = reference.update(5.0, dt)
            self.assertLessEqual(abs(acceleration), 2.0 + 1e-9)
            self.assertLessEqual(abs(jerk), 10.0 + 1e-9)
            self.assertLessEqual(abs(acceleration - previous_acceleration), 10.0 * dt + 1e-9)
            self.assertTrue(np.isfinite(velocity))
            previous_acceleration = acceleration

    def test_invalid_reference_interval_is_held_by_wrapper(self):
        timestamps = np.array([0.0, 0.004, 0.012, 0.016])
        targets = np.tile(np.array([1.0, -1.0, 0.5]), (4, 1))
        rates = np.zeros((4, 3))
        shaped, acceleration, jerk, valid = MODULE.simulate_references(
            timestamps,
            targets,
            rates,
            np.array([2.0, 2.0, 2.0]),
            np.array([10.0, 10.0, 10.0]),
            0.001,
            0.005,
        )

        self.assertFalse(valid[2])
        np.testing.assert_array_equal(shaped[2], shaped[1])
        np.testing.assert_array_equal(acceleration[2], acceleration[1])
        np.testing.assert_array_equal(jerk[2], jerk[1])

    def test_too_short_reference_interval_is_held_by_wrapper(self):
        timestamps = np.array([0.0, 0.0005, 0.004])
        targets = np.tile(np.array([1.0, -1.0, 0.5]), (3, 1))
        rates = np.zeros((3, 3))
        shaped, acceleration, jerk, valid = MODULE.simulate_references(
            timestamps,
            targets,
            rates,
            np.array([2.0, 2.0, 2.0]),
            np.array([10.0, 10.0, 10.0]),
            0.001,
            0.005,
        )

        self.assertFalse(valid[1])
        np.testing.assert_array_equal(shaped[1], shaped[0])
        np.testing.assert_array_equal(acceleration[1], acceleration[0])
        np.testing.assert_array_equal(jerk[1], jerk[0])


class ResidualBoundTest(unittest.TestCase):
    def test_missing_card_parameter_requires_explicit_override(self):
        class EmptyLog:
            initial_parameters = {}
            changed_parameters = []

        with self.assertRaisesRegex(RuntimeError, "MC_AST_J_R"):
            MODULE.card_vector(EmptyLog(), "MC_AST_J")

    def test_changed_card_parameters_include_controller_and_astsmc_card(self):
        class ChangedLog:
            changed_parameters = [
                (1, "MC_AST_K1_R", 2.5),
                (2, "MC_RATE_CTRL_T", 3),
                (3, "MC_ROLLRATE_P", 0.2),
            ]

        self.assertEqual(
            MODULE.changed_card_parameters(ChangedLog()),
            ["MC_AST_K1_R", "MC_RATE_CTRL_T"],
        )

    def test_exact_timestamp_indexes_do_not_use_publication_nearest_neighbor(self):
        reference, sampled = MODULE.exact_timestamp_indexes(
            np.array([1000, 2000, 3000], dtype=np.uint64),
            np.array([1000, 2500, 3000], dtype=np.uint64),
        )
        np.testing.assert_array_equal(reference, np.array([0, 2]))
        np.testing.assert_array_equal(sampled, np.array([0, 2]))

    def test_tracking_residual_matches_residual_control_demand(self):
        inertia = np.array([2.0, 3.0, 4.0])
        effectiveness = np.array([4.0, 6.0, 8.0])
        nominal = np.array([[0.1, -0.2, 0.05]])
        residual = np.array([[0.03, -0.04, 0.02]])
        total = nominal + residual
        gyro_rate = np.array([[0.2, -0.1, 0.05]])
        disturbance = np.array([[0.01, -0.02, 0.03]])
        acceleration = effectiveness / inertia * total - gyro_rate + disturbance
        identified = MODULE.model_disturbance(
            acceleration,
            gyro_rate,
            total,
            effectiveness,
            inertia,
        )
        tracking = MODULE.tracking_residual_disturbance(
            np.zeros((1, 3)),
            gyro_rate,
            nominal,
            identified,
            effectiveness,
            inertia,
        )
        expected = gyro_rate - (effectiveness / inertia) * nominal - disturbance
        np.testing.assert_allclose(identified, disturbance, atol=1e-12)
        np.testing.assert_allclose(tracking, expected, atol=1e-12)

    def test_robust_derivative_tracks_linear_signal(self):
        timestamps = np.linspace(0.0, 1.0, 101)
        values = np.column_stack(
            [2.0 * timestamps, -3.0 * timestamps, 0.5 * timestamps]
        )
        derivative = MODULE.robust_derivative(timestamps, values, 0.1)
        np.testing.assert_allclose(
            np.median(derivative[5:-5], axis=0),
            np.array([2.0, -3.0, 0.5]),
            atol=1e-10,
        )

    def test_distribution_uses_requested_robust_percentile(self):
        metrics = MODULE.distribution(np.arange(1.0, 101.0), 99.0)
        self.assertAlmostEqual(metrics["p50"], 50.5)
        self.assertAlmostEqual(metrics["p95"], 95.05)
        self.assertAlmostEqual(metrics["robust"], 99.01)
        self.assertEqual(metrics["max"], 100.0)

    def test_sample_accounting_reports_overlapping_and_exclusive_reasons(self):
        operational, accounting = MODULE.sample_accounting(
            {
                "invalid_timing": np.array([False, True, True, False]),
                "reset": np.array([False, False, True, False]),
                "saturation": np.array([False, False, False, True]),
            },
            retained={"maybe_landed": np.array([False, True, False, False])},
        )

        np.testing.assert_array_equal(operational, np.array([True, False, False, False]))
        self.assertEqual(accounting["overlapping_exclusions"]["reset"], 1)
        self.assertEqual(accounting["exclusive_exclusions"]["reset"], 0)
        self.assertEqual(accounting["exclusive_exclusions"]["saturation"], 1)
        self.assertEqual(accounting["retained_diagnostics"]["maybe_landed"], 1)
        self.assertEqual(accounting["operational_samples"], 1)

    def test_vector_distribution_uses_euclidean_norm(self):
        metrics = MODULE.vector_distribution(
            np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 10.0]]),
            99.0,
        )

        self.assertAlmostEqual(metrics["p50"], 7.5)
        self.assertEqual(metrics["max"], 10.0)

    def test_regularized_samples_can_be_retained_and_reported(self):
        regularized = np.array([False, True, True, False])
        operational, accounting = MODULE.sample_accounting(
            {"invalid_timing": np.array([False, False, False, True])},
            retained={"variation_regularized": regularized},
        )

        np.testing.assert_array_equal(
            operational,
            np.array([True, True, True, False]),
        )
        self.assertEqual(
            accounting["retained_diagnostics"]["variation_regularized"],
            2,
        )

    def test_robustness_margin_scales_operational_bound(self):
        robust = MODULE.distribution(np.array([1.0, 2.0, 3.0]), 99.0)["robust"]
        self.assertAlmostEqual(robust * 1.25, 3.725)

    def test_infeasible_recommendation_is_suppressed(self):
        recommendation = MODULE.screened_recommendation(
            0.2,
            0.15,
            {
                "k2_gt_L": False,
                "authority_gt_W_plus_k2_dt": True,
                "k1_sufficient": True,
            },
        )

        self.assertFalse(recommendation["recommendation_supported"])
        self.assertIsNone(recommendation["recommended_residual_authority"])
        self.assertIn("k2_gt_L", recommendation["blocking_conditions"])
        self.assertIn(
            "screened_authority_within_total",
            recommendation["blocking_conditions"],
        )


if __name__ == "__main__":
    unittest.main()
