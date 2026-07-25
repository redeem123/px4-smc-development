#!/usr/bin/env python3
import importlib.util
import math
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tmp_mpc_logs" / "px4_mode2_configure.py"
SPEC = importlib.util.spec_from_file_location("px4_mode2_configure", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Px4Mode2ConfigureTest(unittest.TestCase):
    def test_card_installs_the_identified_physical_model(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))

        self.assertEqual(card["MC_MSMC_J_R"], 0.01)
        self.assertEqual(card["MC_MSMC_J_P"], 0.01)
        self.assertEqual(card["MC_MSMC_J_Y"], 0.050951)
        self.assertEqual(card["MC_MSMC_EFF_R"], 0.56)
        self.assertEqual(card["MC_MSMC_EFF_P"], 0.53)
        self.assertEqual(card["MC_MSMC_EFF_Y"], 0.80)

    def test_model_matches_the_installed_type_3_card(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        type3_path = Path(__file__).parents[1] / "tmp_mpc_logs" / "px4_round1_configure.py"
        spec = importlib.util.spec_from_file_location("px4_round1_configure", type3_path)
        type3 = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(type3)
        type3_card = dict(type3.PARAMETERS)

        for suffix in ("R", "P", "Y"):
            self.assertEqual(card[f"MC_MSMC_J_{suffix}"], type3_card[f"MC_AST_J_{suffix}"])
            self.assertEqual(card[f"MC_MSMC_EFF_{suffix}"], type3_card[f"MC_AST_EFF_{suffix}"])

    def test_model_correction_changes_the_command_scale(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        authority = MODULE.card_authority(card)

        self.assertAlmostEqual(authority["roll"]["model_gain_change"], 1.0 / 0.56, places=6)
        self.assertAlmostEqual(authority["pitch"]["model_gain_change"], 1.0 / 0.53, places=6)
        self.assertAlmostEqual(
            authority["yaw"]["model_gain_change"], 0.050951 / 0.02, places=6
        )

    def test_local_torque_slope_is_preserved_per_axis(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        authority = MODULE.card_authority(card)

        self.assertAlmostEqual(authority["roll"]["legacy_local_torque_slope"], 0.12)
        self.assertAlmostEqual(authority["pitch"]["legacy_local_torque_slope"], 0.12)
        self.assertAlmostEqual(authority["yaw"]["legacy_local_torque_slope"], 0.275)

        for axis, values in authority.items():
            with self.subTest(axis=axis):
                self.assertAlmostEqual(
                    values["local_torque_slope"],
                    values["legacy_local_torque_slope"],
                    places=7,
                )

    def test_integral_surface_clamp_is_not_increased(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        authority = MODULE.card_authority(card)

        for axis, values in authority.items():
            with self.subTest(axis=axis):
                self.assertLessEqual(
                    values["integral_surface_clamp"],
                    values["legacy_integral_surface_clamp"] * (1.0 + 1e-6),
                )

        # Yaw is the only axis truncated by the MC_MSMC_ILIM_* parameter maximum.
        self.assertEqual(card["MC_MSMC_ILIM_Y"], MODULE.INTEGRAL_LIMIT_MAXIMUM)
        self.assertGreater(authority["yaw"]["integral_surface_clamp"], 4.9)
        self.assertAlmostEqual(authority["roll"]["integral_surface_clamp"], 0.4, places=5)
        self.assertAlmostEqual(authority["pitch"]["integral_surface_clamp"], 0.6, places=5)

    def test_integral_action_is_slower_by_the_model_correction(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        authority = MODULE.card_authority(card)

        self.assertAlmostEqual(authority["roll"]["integral_time_constant_change"], 0.56, places=6)
        self.assertAlmostEqual(authority["pitch"]["integral_time_constant_change"], 0.53, places=6)
        self.assertAlmostEqual(
            authority["yaw"]["integral_time_constant_change"], 0.02 / 0.050951, places=6
        )

    def test_boundary_layer_and_torque_limits_are_unchanged(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))

        for suffix in ("R", "P", "Y"):
            for name in (f"MC_MSMC_BND_{suffix}", f"MC_MSMC_TMAX_{suffix}"):
                self.assertEqual(card[name], MODULE.LEGACY_PARAMETERS[name])

        self.assertEqual(card["MC_SMC_LPF"], 40.0)
        self.assertEqual(card["MC_SMC_SLEW"], 15.0)
        self.assertEqual(card["MC_MSMC_RSPD_L"], 0.0)

    def test_stale_model_card_is_rejected(self):
        for name, stale in (
            ("MC_MSMC_EFF_R", 1.0),
            ("MC_MSMC_EFF_P", 1.0),
            ("MC_MSMC_J_Y", 0.02),
        ):
            with self.subTest(name=name):
                parameters = tuple(
                    (candidate, stale if candidate == name else value)
                    for candidate, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
                )

                with self.assertRaises(ValueError):
                    MODULE.validate_card(parameters)

    def test_uncompensated_model_correction_is_rejected(self):
        parameters = tuple(
            (name, MODULE.LEGACY_PARAMETERS.get(name, value) if name.startswith(
                ("MC_MSMC_C_", "MC_MSMC_ETA_", "MC_MSMC_KS_")) else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "local torque slope"):
            MODULE.validate_card(parameters)

    def test_boundary_layer_retune_is_rejected(self):
        parameters = tuple(
            (name, 0.25 if name == "MC_MSMC_BND_R" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_BND_R"):
            MODULE.validate_card(parameters)

    def test_torque_limit_retune_is_rejected(self):
        parameters = tuple(
            (name, 0.30 if name == "MC_MSMC_TMAX_R" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_TMAX_R"):
            MODULE.validate_card(parameters)

    def test_integral_limit_above_parameter_maximum_is_rejected(self):
        parameters = tuple(
            (name, 5.0951 if name == "MC_MSMC_ILIM_Y" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "parameter maximum"):
            MODULE.validate_card(parameters)

    def test_rate_setpoint_derivative_must_stay_disabled(self):
        parameters = tuple(
            (name, 100.0 if name == "MC_MSMC_RSPD_L" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_RSPD_L"):
            MODULE.validate_card(parameters)

    def test_acknowledgment_and_controller_selection_are_ordered(self):
        for name, invalid, message in (
            ("MC_MSMC_CFG", 1.0, "MC_MSMC_CFG"),
            ("MC_RATE_CTRL_T", 3.0, "MC_RATE_CTRL_T"),
        ):
            with self.subTest(name=name):
                parameters = tuple(
                    (candidate, invalid if candidate == name else value)
                    for candidate, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
                )

                with self.assertRaisesRegex(ValueError, message):
                    MODULE.validate_card(parameters)

    def test_duplicate_parameter_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED) + (("MC_MSMC_EFF_Y", 0.8),))

    def test_nonfinite_card_value_is_rejected(self):
        parameters = tuple(
            (name, math.nan if name == "MC_MSMC_EFF_Y" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)
        )

        with self.assertRaisesRegex(ValueError, "non-finite"):
            MODULE.validate_card(parameters)

    def test_snapshot_preserves_before_target_and_verified_values(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        before = {name: (value, 9) for name, value in MODULE.card_parameters(MODULE.CARD_IDENTIFIED)}
        verified = {name: {"value": value, "type": 9} for name, value in card.items()}

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "snapshot.json"
            MODULE.write_snapshot(path, before, card, verified)
            contents = path.read_text(encoding="utf-8")

        self.assertIn('"before"', contents)
        self.assertIn('"target"', contents)
        self.assertIn('"verified"', contents)
        self.assertIn('"MC_MSMC_J_Y": 0.050951', contents)
        self.assertIn('"MC_MSMC_CFG": 1.0', contents)

    def test_paper_terms_are_cleared_by_default(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))

        self.assertFalse(MODULE.paper_terms_enabled(card))

        for name, _ in MODULE.PAPER_DISABLED:
            with self.subTest(name=name):
                # Written explicitly so a previously installed paper card is
                # cleared rather than silently inherited.
                self.assertEqual(card[name], 0.0)

    def test_paper_card_installs_both_added_terms(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_PAPER))

        self.assertTrue(MODULE.paper_terms_enabled(card))
        self.assertEqual(card["MC_MSMC_A1_R"], MODULE.PAPER_ATTITUDE_SLOPE[0])
        self.assertEqual(card["MC_MSMC_A1_P"], MODULE.PAPER_ATTITUDE_SLOPE[1])
        self.assertEqual(card["MC_MSMC_A1_Y"], MODULE.PAPER_ATTITUDE_SLOPE[2])
        self.assertEqual(card["MC_MSMC_JR"], MODULE.PAPER_ROTOR_INERTIA)
        self.assertEqual(card["MC_MSMC_ROTOR_K"], MODULE.PAPER_ROTOR_SPEED_GAIN)
        self.assertEqual(card["MC_MSMC_ROTOR_D"], float(MODULE.PAPER_ROTOR_DIRECTIONS))

    def test_paper_card_keeps_the_flight_validated_torque_slope(self):
        plain = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        paper = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_PAPER))

        for _, suffix in MODULE.AXES:
            with self.subTest(suffix=suffix):
                self.assertEqual(
                    MODULE.local_torque_slope(paper, suffix),
                    MODULE.local_torque_slope(plain, suffix),
                )

    def test_rotor_term_without_a_usable_speed_map_is_rejected(self):
        for name, value, message in (
            ("MC_MSMC_ROTOR_K", 0.0, "MC_MSMC_ROTOR_K"),
            ("MC_MSMC_ROTOR_D", 0.0, "MC_MSMC_ROTOR_D"),
        ):
            with self.subTest(name=name):
                parameters = tuple(
                    (candidate, value if candidate == name else current)
                    for candidate, current in MODULE.card_parameters(MODULE.CARD_PAPER)
                )

                with self.assertRaisesRegex(ValueError, message):
                    MODULE.validate_card(parameters)

    def test_speed_map_without_rotor_inertia_is_rejected(self):
        parameters = tuple(
            (name, 0.0 if name == "MC_MSMC_JR" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_PAPER)
        )

        with self.assertRaisesRegex(ValueError, "without MC_MSMC_JR"):
            MODULE.validate_card(parameters)

    def test_fractional_rotor_direction_mask_is_rejected(self):
        parameters = tuple(
            (name, 12.5 if name == "MC_MSMC_ROTOR_D" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_PAPER)
        )

        with self.assertRaisesRegex(ValueError, "integer bitmask"):
            MODULE.validate_card(parameters)

    def test_strict_card_reproduces_the_paper_surface_and_reaching_law(self):
        card = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_STRICT))

        self.assertTrue(MODULE.strict_terms_enabled(card))
        self.assertTrue(MODULE.paper_terms_enabled(card))

        for suffix in ("R", "P", "Y"):
            with self.subTest(suffix=suffix):
                # ILIM=0 clamps the error integral to zero, leaving s = e.
                self.assertEqual(card[f"MC_MSMC_ILIM_{suffix}"], 0.0)
                self.assertEqual(
                    card[f"MC_MSMC_BND_{suffix}"], MODULE.STRICT_BOUNDARY_LAYER
                )

        self.assertEqual(card["MC_SMC_LPF"], 0.0)
        self.assertEqual(card["MC_SMC_SLEW"], 0.0)

    def test_strict_card_keeps_the_identified_model_and_gains(self):
        plain = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_IDENTIFIED))
        strict = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_STRICT))

        for suffix in ("R", "P", "Y"):
            for gain in ("J", "EFF", "C", "ETA", "KS"):
                name = f"MC_MSMC_{gain}_{suffix}"

                with self.subTest(name=name):
                    self.assertEqual(strict[name], plain[name])

    def test_strict_card_removes_the_software_torque_limit(self):
        strict = MODULE.validate_card(MODULE.card_parameters(MODULE.CARD_STRICT))

        for suffix in ("R", "P", "Y"):
            with self.subTest(suffix=suffix):
                # Only the allocator's own normalized-torque saturation remains.
                self.assertEqual(
                    strict[f"MC_MSMC_TMAX_{suffix}"], MODULE.STRICT_TORQUE_LIMIT
                )

    def test_strict_card_rejects_a_retuned_reaching_gain(self):
        parameters = tuple(
            (name, 5.0 if name == "MC_MSMC_ETA_R" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_STRICT)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_ETA_R"):
            MODULE.validate_card(parameters)

    def test_strict_card_rejects_a_reinstated_torque_limit(self):
        parameters = tuple(
            (name, 0.2 if name == "MC_MSMC_TMAX_R" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_STRICT)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_TMAX_R"):
            MODULE.validate_card(parameters)

    def test_flight_cards_keep_the_flight_validated_torque_limit(self):
        for mode in (MODULE.CARD_IDENTIFIED, MODULE.CARD_PAPER):
            with self.subTest(mode=mode):
                card = MODULE.validate_card(MODULE.card_parameters(mode))

                for suffix in ("R", "P", "Y"):
                    name = f"MC_MSMC_TMAX_{suffix}"
                    self.assertEqual(card[name], MODULE.LEGACY_PARAMETERS[name])

    def test_strict_card_rejects_a_reinstated_output_filter(self):
        for name in ("MC_SMC_LPF", "MC_SMC_SLEW"):
            with self.subTest(name=name):
                parameters = tuple(
                    (candidate, 40.0 if candidate == name else value)
                    for candidate, value in MODULE.card_parameters(MODULE.CARD_STRICT)
                )

                with self.assertRaisesRegex(ValueError, name):
                    MODULE.validate_card(parameters)

    def test_partial_strict_card_is_still_detected_as_strict(self):
        # A card carrying the paper surface must be judged against the strict
        # rules, not demoted to the flight branch where BND would report first.
        for name, value in (("MC_MSMC_TMAX_R", 0.2), ("MC_SMC_SLEW", 15.0)):
            with self.subTest(name=name):
                parameters = tuple(
                    (candidate, value if candidate == name else current)
                    for candidate, current in MODULE.card_parameters(MODULE.CARD_STRICT)
                )
                card = dict(parameters)

                self.assertTrue(MODULE.strict_terms_enabled(card))

                with self.assertRaisesRegex(ValueError, name):
                    MODULE.validate_card(parameters)

    def test_strict_card_requires_the_paper_terms(self):
        cleared = dict(MODULE.PAPER_DISABLED)
        parameters = tuple(
            (name, cleared.get(name, value))
            for name, value in MODULE.card_parameters(MODULE.CARD_STRICT)
        )

        with self.assertRaisesRegex(ValueError, "strict card requires"):
            MODULE.validate_card(parameters)

    def test_unknown_card_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown card mode"):
            MODULE.card_parameters("paper-ish")

    def test_negative_attitude_surface_slope_is_rejected(self):
        parameters = tuple(
            (name, -6.5 if name == "MC_MSMC_A1_R" else value)
            for name, value in MODULE.card_parameters(MODULE.CARD_PAPER)
        )

        with self.assertRaisesRegex(ValueError, "MC_MSMC_A1_R"):
            MODULE.validate_card(parameters)


if __name__ == "__main__":
    unittest.main()
