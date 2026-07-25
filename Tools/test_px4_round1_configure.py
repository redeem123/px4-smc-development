#!/usr/bin/env python3
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tmp_mpc_logs" / "px4_round1_configure.py"
SPEC = importlib.util.spec_from_file_location("px4_round1_configure", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Px4Round1ConfigureTest(unittest.TestCase):
    def test_real_yaw_card_has_expected_bounded_authority(self):
        card = MODULE.validate_card(MODULE.PARAMETERS)
        authority = MODULE.card_authority(card)

        self.assertEqual(card["MC_AST_EFF_Y"], 0.8)
        self.assertEqual(card["MC_AST_TRES_Y"], 0.1)
        self.assertEqual(card["MC_AST_TMAX_Y"], 0.15)
        self.assertEqual(card["MC_AST_YAW_EXT"], 0.0)
        self.assertAlmostEqual(
            authority["yaw"]["residual_reaching_authority"],
            0.8 / 0.050951 * 0.1,
        )
        self.assertAlmostEqual(
            authority["yaw"]["maximum_governed_residual_reaching_authority"],
            authority["yaw"]["residual_reaching_authority"],
        )
        self.assertAlmostEqual(authority["yaw"]["nominal_torque_partition"], 0.05)

    def test_yaw_extension_trial_reports_governed_maximum(self):
        card = MODULE.validate_card(MODULE.card_with_yaw_extension(0.05))
        authority = MODULE.card_authority(card)

        self.assertEqual(card["MC_AST_TRES_Y"], 0.1)
        self.assertEqual(card["MC_AST_YAW_EXT"], 0.05)
        self.assertAlmostEqual(
            authority["yaw"]["residual_reaching_authority"],
            0.8 / 0.050951 * 0.1,
        )
        self.assertAlmostEqual(
            authority["yaw"]["maximum_governed_residual_reaching_authority"],
            0.8 / 0.050951 * 0.15,
        )

    def test_invalid_yaw_extension_is_rejected(self):
        for invalid in (-0.001, 0.051, math.nan):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    MODULE.validate_card(MODULE.card_with_yaw_extension(invalid))

    def test_yaw_extension_must_fit_total_torque_bound(self):
        parameters = tuple(
            (name, 0.14 if name == "MC_AST_TMAX_Y" else value)
            for name, value in MODULE.card_with_yaw_extension(0.05)
        )

        with self.assertRaisesRegex(ValueError, "governed yaw reserve"):
            MODULE.validate_card(parameters)

    def test_roll_pitch_authority_matches_pid_identification(self):
        card = MODULE.validate_card(MODULE.PARAMETERS)
        authority = MODULE.card_authority(card)

        self.assertEqual(card["MC_AST_EFF_R"], 0.56)
        self.assertEqual(card["MC_AST_EFF_P"], 0.53)
        self.assertEqual(card["MC_AST_K1_R"], 4.5)
        self.assertEqual(card["MC_AST_K1_P"], 4.5)
        self.assertEqual(card["MC_AST_QK1_ERR"], 0.5)
        self.assertEqual(card["MC_AST_DU_R"], 1.0)
        self.assertEqual(card["MC_AST_DU_P"], 1.0)
        self.assertEqual(card["MC_AST_RFF_RP"], 0.5)
        self.assertEqual(card["MC_AST_RP_EXT"], 0.1)
        self.assertEqual(card["MC_AST_RP_K1_B"], 1.5)
        self.assertEqual(card["MC_AST_RP_AIR"], 1.0)
        self.assertEqual(card["MC_AIRMODE"], 0.0)
        self.assertAlmostEqual(authority["roll"]["control_gain"], 56.0)
        self.assertAlmostEqual(authority["pitch"]["control_gain"], 53.0)
        self.assertAlmostEqual(authority["roll"]["residual_reaching_authority"], 5.6)
        self.assertAlmostEqual(authority["pitch"]["residual_reaching_authority"], 5.3)
        self.assertAlmostEqual(
            authority["roll"]["maximum_governed_residual_reaching_authority"], 11.2
        )
        self.assertAlmostEqual(
            authority["pitch"]["maximum_governed_residual_reaching_authority"], 10.6
        )
        self.assertAlmostEqual(
            authority["roll"]["nominal_acceleration_demand"], 5.0 / 56.0
        )
        self.assertAlmostEqual(
            authority["pitch"]["nominal_acceleration_demand"], 5.0 / 53.0
        )
        self.assertAlmostEqual(authority["roll"]["nominal_torque_partition"], 0.1)
        self.assertAlmostEqual(authority["pitch"]["nominal_torque_partition"], 0.1)

    def test_roll_pitch_headroom_feature_must_be_binary(self):
        for invalid in (-1.0, 0.5, 2.0, math.nan):
            with self.subTest(invalid=invalid):
                parameters = tuple(
                    (name, invalid if name == "MC_AST_RP_AIR" else value)
                    for name, value in MODULE.PARAMETERS
                )

                with self.assertRaisesRegex(ValueError, "MC_AST_RP_AIR"):
                    MODULE.validate_card(parameters)

    def test_global_airmode_must_remain_disabled(self):
        parameters = tuple(
            (name, 1.0 if name == "MC_AIRMODE" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "MC_AIRMODE=0"):
            MODULE.validate_card(parameters)

    def test_invalid_roll_pitch_extension_is_rejected(self):
        for invalid in (-0.001, 0.101, math.nan):
            with self.subTest(invalid=invalid):
                parameters = tuple(
                    (name, invalid if name == "MC_AST_RP_EXT" else value)
                    for name, value in MODULE.PARAMETERS
                )

                with self.assertRaises(ValueError):
                    MODULE.validate_card(parameters)

    def test_roll_pitch_extension_must_fit_total_torque_bound(self):
        parameters = tuple(
            (name, 0.19 if name == "MC_AST_TMAX_R" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "recovery reserve"):
            MODULE.validate_card(parameters)

    def test_invalid_roll_pitch_k1_recovery_boost_is_rejected(self):
        for invalid in (-0.001, 1.501, math.nan):
            with self.subTest(invalid=invalid):
                parameters = tuple(
                    (name, invalid if name == "MC_AST_RP_K1_B" else value)
                    for name, value in MODULE.PARAMETERS
                )

                with self.assertRaises(ValueError):
                    MODULE.validate_card(parameters)

    def test_roll_pitch_k1_boost_requires_distinct_recovery_envelope(self):
        parameters = tuple(
            (name, 0.5 if name == "MC_AST_REC_ERR" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "requires MC_AST_QK1_ERR below"):
            MODULE.validate_card(parameters)

    def test_old_roll_pitch_k1_card_is_rejected(self):
        parameters = tuple(
            (
                name,
                3.0 if name in ("MC_AST_K1_R", "MC_AST_K1_P") else value,
            )
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "roll/pitch K1"):
            MODULE.validate_card(parameters)

    def test_old_roll_pitch_effectiveness_card_is_rejected(self):
        parameters = tuple(
            (
                name,
                {"MC_AST_EFF_R": 0.8, "MC_AST_EFF_P": 0.75}.get(name, value),
            )
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "roll/pitch authority model"):
            MODULE.validate_card(parameters)

    def test_excessive_roll_pitch_feedforward_is_rejected(self):
        parameters = tuple(
            (name, 0.75 if name == "MC_AST_RFF_RP" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "reference feedforward demand"):
            MODULE.validate_card(parameters)

    def test_selective_release_card_is_enabled_and_ordered(self):
        card = MODULE.validate_card(MODULE.PARAMETERS)

        self.assertEqual(card["MC_AST_REL_ERR"], 0.2)
        self.assertEqual(card["MC_AST_REL_RATE"], 20.0)
        self.assertEqual(card["MC_AST_TRIM_CMD"], 0.15)
        self.assertEqual(card["MC_AST_TRIM_ERR"], 0.2)
        self.assertEqual(card["MC_AST_TRIM_ACC"], 1.0)
        self.assertEqual(card["MC_AST_TRIM_DWL"], 0.5)
        self.assertEqual(card["MC_AST_TRIM_TC"], 20.0)
        self.assertLess(card["MC_AST_REL_ERR"], card["MC_AST_REC_ERR"])

    def test_quiet_k1_threshold_above_hard_recovery_is_rejected(self):
        parameters = tuple(
            (name, 1.1 if name == "MC_AST_QK1_ERR" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "MC_AST_QK1_ERR"):
            MODULE.validate_card(parameters)

    def test_selective_release_above_hard_recovery_is_rejected(self):
        parameters = tuple(
            (name, 1.1 if name == "MC_AST_REL_ERR" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "MC_AST_REL_ERR"):
            MODULE.validate_card(parameters)

    def test_trim_threshold_units_are_validated_independently(self):
        for parameter, invalid in (
            ("MC_AST_TRIM_CMD", 0.01),
            ("MC_AST_TRIM_ERR", 0.51),
            ("MC_AST_TRIM_ACC", 10.1),
        ):
            parameters = tuple(
                (name, invalid if name == parameter else value)
                for name, value in MODULE.PARAMETERS
            )

            with self.assertRaisesRegex(ValueError, parameter):
                MODULE.validate_card(parameters)

    def test_duplicate_parameter_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            MODULE.validate_card(MODULE.PARAMETERS + (("MC_AST_EFF_Y", 0.8),))

    def test_log_834_unsafe_yaw_reserve_is_rejected(self):
        parameters = tuple(
            (name, 0.15 if name == "MC_AST_TRES_Y" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "log-834 allocation-safe bound"):
            MODULE.validate_card(parameters)

    def test_invalid_authority_partition_is_rejected(self):
        parameters = tuple(
            (name, 0.16 if name == "MC_AST_TRES_Y" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "log-834 allocation-safe bound"):
            MODULE.validate_card(parameters)

    def test_nonfinite_card_value_is_rejected(self):
        parameters = tuple(
            (name, math.nan if name == "MC_AST_EFF_Y" else value)
            for name, value in MODULE.PARAMETERS
        )

        with self.assertRaisesRegex(ValueError, "non-finite"):
            MODULE.validate_card(parameters)

    def test_snapshot_preserves_before_target_and_verified_values(self):
        card = MODULE.validate_card(MODULE.PARAMETERS)
        before = {name: (value, 9) for name, value in MODULE.PARAMETERS}
        verified = {name: {"value": value, "type": 9} for name, value in card.items()}

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot.json"
            MODULE.write_snapshot(path, before, card, verified)
            contents = path.read_text(encoding="utf-8")

        self.assertIn('"before"', contents)
        self.assertIn('"target"', contents)
        self.assertIn('"verified"', contents)
        self.assertIn('"MC_AST_EFF_Y": 0.8', contents)


if __name__ == "__main__":
    unittest.main()
