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


if __name__ == "__main__":
    unittest.main()
