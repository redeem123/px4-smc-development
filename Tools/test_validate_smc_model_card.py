#!/usr/bin/env python3
"""Tests for model-based SMC parameter-card validation."""

import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("validate_smc_model_card.py")
SPEC = importlib.util.spec_from_file_location("validate_smc_model_card", MODULE_PATH)
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class SmcModelCardValidationTest(unittest.TestCase):
    def test_axis_gain_matches_pid_baseline(self):
        parameters = {
            "MC_MSMC_J_R": 0.0135,
            "MC_MSMC_EFF_R": 4.03,
            "MC_MSMC_C_R": 3.0,
            "MC_MSMC_ETA_R": 6.0,
            "MC_MSMC_BND_R": 0.15,
            "MC_MSMC_KS_R": 1.5,
            "MC_ROLLRATE_P": 0.15,
            "MC_ROLLRATE_K": 1.0,
        }

        smc_gain, pid_gain, ratio = VALIDATOR.axis_gain(parameters, "R", "ROLL")

        self.assertAlmostEqual(smc_gain, 0.149069, places=6)
        self.assertAlmostEqual(pid_gain, 0.15, places=6)
        self.assertAlmostEqual(ratio, 0.993797, places=6)

    def test_axis_gain_exposes_underpowered_card(self):
        parameters = {
            "MC_MSMC_J_R": 0.01,
            "MC_MSMC_EFF_R": 1.0,
            "MC_MSMC_C_R": 0.85,
            "MC_MSMC_ETA_R": 0.45,
            "MC_MSMC_BND_R": 0.30,
            "MC_MSMC_KS_R": 0.045,
            "MC_ROLLRATE_P": 0.15,
            "MC_ROLLRATE_K": 1.0,
        }

        smc_gain, _, ratio = VALIDATOR.axis_gain(parameters, "R", "ROLL")

        self.assertAlmostEqual(smc_gain, 0.02395, places=6)
        self.assertLess(ratio, 0.2)


if __name__ == "__main__":
    unittest.main()
