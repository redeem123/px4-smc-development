#!/usr/bin/env python3
"""Tests for the UAV985 ULog rate-model identification helpers."""

import importlib.util
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("uav985_rate_model_identification.py")
SPEC = importlib.util.spec_from_file_location("uav985_rate_model_identification", MODULE_PATH)
IDENTIFICATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IDENTIFICATION)


class RateModelIdentificationTest(unittest.TestCase):
    def test_sample_timestamps_prefers_measurement_time(self):
        data = {
            "timestamp": np.array([2_000_000, 3_000_000]),
            "timestamp_sample": np.array([1_000_000, 2_000_000]),
        }

        np.testing.assert_allclose(IDENTIFICATION.sample_timestamps(data), [1.0, 2.0])

    def test_fit_axis_recovers_positive_effectiveness(self):
        timestamps = np.arange(0.0, 20.0, 0.01)
        command = 0.2 * np.sin(2.0 * np.pi * 0.7 * timestamps)
        rate = 0.1 * np.sin(2.0 * np.pi * 0.2 * timestamps)
        actuator = IDENTIFICATION.simulate_actuator(timestamps, command, 0.03)
        physical_torque = 0.8 * actuator - 0.02 * rate + 0.01

        estimate = IDENTIFICATION.fit_axis(
            timestamps, command, rate, physical_torque, np.linspace(0.005, 0.08, 151)
        )

        self.assertAlmostEqual(estimate["effectiveness"], 0.8, places=5)
        self.assertAlmostEqual(estimate["tau"], 0.03, places=5)
        self.assertAlmostEqual(estimate["damping"], 0.02, places=5)
        self.assertGreater(estimate["r_squared"], 0.9999)

    def test_fit_axis_rejects_negative_effectiveness(self):
        timestamps = np.arange(0.0, 5.0, 0.01)
        command = np.sin(timestamps)
        physical_torque = -IDENTIFICATION.simulate_actuator(timestamps, command, 0.02)

        with self.assertRaisesRegex(RuntimeError, "no positive-effectiveness fit"):
            IDENTIFICATION.fit_axis(
                timestamps,
                command,
                np.zeros_like(command),
                physical_torque,
                np.linspace(0.005, 0.08, 151),
            )


if __name__ == "__main__":
    unittest.main()
