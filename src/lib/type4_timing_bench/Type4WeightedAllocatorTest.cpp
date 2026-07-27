/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "Type4WeightedAllocator.hpp"

#include <cmath>

using type4_timing_bench::Type4WeightedAllocator;

namespace
{

Type4WeightedAllocator::Problem baseProblem(uint8_t num_axes = 4)
{
	Type4WeightedAllocator::Problem problem{};
	problem.num_axes = num_axes;
	problem.regularization = 0.1f;

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		problem.lower_bound(actuator) = -1.f;
		problem.upper_bound(actuator) = 1.f;
	}

	for (int axis = 0; axis < num_axes; ++axis) {
		problem.axis_weight(axis) = 1.f;
	}

	return problem;
}

void expectValidResult(const Type4WeightedAllocator::Result &result, double kkt_tolerance = 1e-4)
{
	EXPECT_EQ(result.status, Type4WeightedAllocator::Status::Success);
	EXPECT_TRUE(result.solution.isAllFinite());
	EXPECT_TRUE(std::isfinite(result.objective));
	EXPECT_TRUE(std::isfinite(result.objective_scale));
	EXPECT_GE(result.objective_scale, 1.f);
	EXPECT_LE(result.primal_residual_inf, kkt_tolerance);
	EXPECT_TRUE(std::isfinite(result.kkt_residual_inf));
	EXPECT_LE(result.normalized_kkt_residual_inf, kkt_tolerance);
	EXPECT_EQ(result.faces_evaluated, Type4WeightedAllocator::NumFaces);
	EXPECT_EQ(result.linear_solves, 65);
}

} // namespace

TEST(Type4WeightedAllocatorTest, InteriorIdentitySolution)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem();

	for (int axis = 0; axis < 4; ++axis) {
		problem.effectiveness(axis, axis) = 1.f;
		problem.target(axis) = 0.5f * static_cast<float>(axis + 1) / 4.f;
	}

	const auto result = allocator.solve(problem);
	expectValidResult(result);

	for (int actuator = 0; actuator < 4; ++actuator) {
		const float expected = problem.target(actuator) / 1.1f;
		EXPECT_NEAR(result.solution(actuator), expected, 1e-5f);
	}

	EXPECT_EQ(result.lower_active_mask, 0);
	EXPECT_EQ(result.upper_active_mask, 0);
}

TEST(Type4WeightedAllocatorTest, SimultaneousLowerAndUpperSaturation)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem();

	for (int axis = 0; axis < 4; ++axis) {
		problem.effectiveness(axis, axis) = 1.f;
	}

	problem.target(0) = -4.f;
	problem.target(1) = 4.f;
	problem.target(2) = 0.25f;
	problem.target(3) = -0.25f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), -1.f, 1e-6f);
	EXPECT_NEAR(result.solution(1), 1.f, 1e-6f);
	EXPECT_NEAR(result.solution(2), 0.25f / 1.1f, 1e-5f);
	EXPECT_NEAR(result.solution(3), -0.25f / 1.1f, 1e-5f);
	EXPECT_EQ(result.lower_active_mask, 1u << 0);
	EXPECT_EQ(result.upper_active_mask, 1u << 1);
}

TEST(Type4WeightedAllocatorTest, ResolvesFreeVariableAfterSaturation)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(2);
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(0, 1) = 1.f;
	problem.effectiveness(1, 0) = 1.f;
	problem.effectiveness(1, 1) = -1.f;
	problem.axis_weight(0) = 2.f;
	problem.target(0) = 1.85f;
	problem.target(1) = 1.f;
	problem.lower_bound(0) = 0.f;
	problem.lower_bound(1) = 0.f;
	problem.upper_bound(0) = 0.6f;
	problem.upper_bound(1) = 1.f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.6f, 1e-5f);
	EXPECT_NEAR(result.solution(1), 0.901961f, 1e-5f);
	EXPECT_EQ(result.upper_active_mask, 1u << 0);
}

TEST(Type4WeightedAllocatorTest, ZeroWeightAxisIsIgnored)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(2);
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(1, 1) = 1.f;
	problem.target(0) = 0.55f;
	problem.target(1) = 100.f;
	problem.axis_weight(1) = 0.f;
	problem.reference(1) = -0.3f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.5f, 1e-5f);
	EXPECT_NEAR(result.solution(1), -0.3f, 1e-5f);
}

TEST(Type4WeightedAllocatorTest, RankDeficientEffectivenessUsesRegularization)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(6);
	problem.reference(0) = -0.8f;
	problem.reference(1) = -0.2f;
	problem.reference(2) = 0.3f;
	problem.reference(3) = 0.9f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);

	for (int actuator = 0; actuator < 4; ++actuator) {
		EXPECT_NEAR(result.solution(actuator), problem.reference(actuator), 1e-6f);
	}
}

TEST(Type4WeightedAllocatorTest, CancellationRoundoffDoesNotRejectClippedReference)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.target(0) = 0.00018803686543833464f;
	problem.axis_weight(0) = 0.48869529366493225f;
	problem.regularization = 0.00507131265476346f;
	problem.lower_bound(0) = -0.2542306483f;
	problem.lower_bound(1) = -0.4128435850f;
	problem.lower_bound(2) = -1.1416567564f;
	problem.lower_bound(3) = -1.8330838680f;
	problem.upper_bound(0) = 2.1518623829f;
	problem.upper_bound(1) = 2.3704779148f;
	problem.upper_bound(2) = 0.9956646562f;
	problem.upper_bound(3) = -1.7458642721f;
	problem.reference(0) = -0.05848208815f;
	problem.reference(1) = 1.3269037008f;
	problem.reference(2) = 1.4747344255f;
	problem.reference(3) = 1.8769636154f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), problem.reference(0), 1e-6f);
	EXPECT_NEAR(result.solution(1), problem.reference(1), 1e-6f);
	EXPECT_NEAR(result.solution(2), problem.upper_bound(2), 1e-6f);
	EXPECT_NEAR(result.solution(3), problem.upper_bound(3), 1e-6f);
	EXPECT_EQ(result.upper_active_mask, (1u << 2) | (1u << 3));
}

TEST(Type4WeightedAllocatorTest, FreeFaceOutsideBoundsIsResolvedOnExactFace)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 0.0013118692440912127f;
	problem.effectiveness(0, 0) = -1.6380681991577148f;
	problem.effectiveness(0, 1) = -0.8323767781257629f;
	problem.effectiveness(0, 2) = -1.5097219944000244f;
	problem.effectiveness(0, 3) = 0.7443081140518188f;
	problem.target(0) = -1.6441737413406372f;
	problem.axis_weight(0) = 2.8277857303619385f;
	problem.lower_bound(0) = -0.3476029634475708f;
	problem.lower_bound(1) = -0.4780748784542084f;
	problem.lower_bound(2) = -1.3708940744400024f;
	problem.lower_bound(3) = -1.2725428342819214f;
	problem.upper_bound(0) = 1.0171610116958618f;
	problem.upper_bound(1) = 0.1983037143945694f;
	problem.upper_bound(2) = 1.9857813119888306f;
	problem.upper_bound(3) = -0.13882598280906677f;
	problem.reference(0) = 0.6026015281677246f;
	problem.reference(1) = 0.08888170123100281f;
	problem.reference(2) = -0.17467011511325836f;
	problem.reference(3) = -0.04098125919699669f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.8179375f, 1e-6f);
	EXPECT_NEAR(result.solution(1), problem.upper_bound(1), 1e-6f);
	EXPECT_NEAR(result.solution(2), 0.02379382f, 1e-6f);
	EXPECT_NEAR(result.solution(3), problem.upper_bound(3), 1e-6f);
	EXPECT_EQ(result.upper_active_mask, 1u << 3);
}

TEST(Type4WeightedAllocatorTest, EqualityBoundHasNoStationarityViolation)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.effectiveness(0, 0) = 1.f;
	problem.target(0) = 1.f;
	problem.lower_bound(0) = 0.25f;
	problem.upper_bound(0) = 0.25f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.25f, 1e-6f);
	EXPECT_EQ(result.lower_active_mask & 1u, 1u);
	EXPECT_EQ(result.upper_active_mask & 1u, 1u);
}

TEST(Type4WeightedAllocatorTest, NarrowIntervalInteriorOptimumRemainsInterior)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 1.f;
	problem.effectiveness(0, 0) = 1.f;
	problem.target(0) = 1e-6f;
	problem.lower_bound(0) = 0.f;
	problem.upper_bound(0) = 1e-6f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 5e-7f, 1e-12f);
	EXPECT_EQ(result.lower_active_mask & 1u, 0u);
	EXPECT_EQ(result.upper_active_mask & 1u, 0u);
}

TEST(Type4WeightedAllocatorTest, RejectsInvalidInput)
{
	Type4WeightedAllocator allocator;
	auto expectInvalid = [&allocator](const Type4WeightedAllocator::Problem &problem) {
		EXPECT_EQ(allocator.solve(problem).status, Type4WeightedAllocator::Status::InvalidInput);
	};

	auto problem = baseProblem();
	problem.num_axes = 0;
	expectInvalid(problem);

	problem = baseProblem();
	problem.num_axes = Type4WeightedAllocator::MaxWrenchAxes + 1;
	expectInvalid(problem);

	problem = baseProblem();
	problem.regularization = 0.f;
	expectInvalid(problem);

	problem = baseProblem();
	problem.regularization = -1.f;
	expectInvalid(problem);

	problem = baseProblem();
	problem.regularization = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.lower_bound(2) = 2.f;
	problem.upper_bound(2) = 1.f;
	expectInvalid(problem);

	problem = baseProblem();
	problem.lower_bound(0) = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.upper_bound(0) = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.reference(0) = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.axis_weight(1) = -1.f;
	expectInvalid(problem);

	problem = baseProblem();
	problem.axis_weight(1) = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.target(0) = NAN;
	expectInvalid(problem);

	problem = baseProblem();
	problem.effectiveness(0, 0) = NAN;
	expectInvalid(problem);
}

TEST(Type4WeightedAllocatorTest, FloatQuantizedOptimumPassesKktValidation)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 1e-4f;
	problem.effectiveness(0, 0) = 1.f;
	problem.target(0) = 0.5f;
	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		problem.lower_bound(actuator) = -2.f;
		problem.upper_bound(actuator) = 2.f;
	}

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.5f / 1.0001f, 1e-7f);

	const double solution = static_cast<double>(result.solution(0));
	const double regularization = static_cast<double>(problem.regularization);
	const double expected_kkt_residual = fabs((solution - 0.5) + regularization * solution);
	const double expected_kkt_scale = fabs(solution) + 0.5 + regularization * fabs(solution);
	EXPECT_DOUBLE_EQ(result.kkt_residual_inf, expected_kkt_residual);
	EXPECT_DOUBLE_EQ(result.normalized_kkt_residual_inf, expected_kkt_residual / expected_kkt_scale);
	EXPECT_GT(result.kkt_residual_inf, 0.0);
}

TEST(Type4WeightedAllocatorTest, LargeConstantResidualDoesNotChangeOptimalFace)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(3);
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(1, 1) = 1.f;
	problem.target(0) = 2.f;
	problem.target(1) = -2.f;
	problem.target(2) = 1e15f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 1.f, 1e-6f);
	EXPECT_NEAR(result.solution(1), -1.f, 1e-6f);
}

TEST(Type4WeightedAllocatorTest, RejectsUnresolvableRegularizationScale)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.axis_weight(0) = 1e10f;

	for (int actuator = 0; actuator < 4; ++actuator) {
		problem.effectiveness(0, actuator) = 1.f;
	}

	const auto result = allocator.solve(problem);
	EXPECT_EQ(result.status, Type4WeightedAllocator::Status::NumericalFailure);
}

TEST(Type4WeightedAllocatorTest, DynamicRangeSolutionRemainsValid)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(2);
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(1, 1) = 1.f;
	problem.target(0) = 1e20f;
	problem.target(1) = 2.f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 1.f, 1e-6f);
	EXPECT_NEAR(result.solution(1), 1.f, 1e-6f);

	problem.axis_weight(0) = 1e-4f;
	problem.axis_weight(1) = 1e-4f;
	problem.regularization = 1e-9f;
	const auto scaled_result = allocator.solve(problem);
	expectValidResult(scaled_result);
	EXPECT_NEAR(scaled_result.solution(0), 1.f, 1e-6f);
	EXPECT_NEAR(scaled_result.solution(1), 1.f, 1e-6f);
}

TEST(Type4WeightedAllocatorTest, ResolvesSmallLinearComponentAcrossScaledAxes)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(2);
	problem.effectiveness(0, 0) = 1e4f;
	problem.effectiveness(1, 1) = 1.f;
	problem.target(0) = 1.f;
	problem.target(1) = 1.f;
	problem.reference(0) = 1.f;
	problem.lower_bound(0) = -2.f;
	problem.lower_bound(1) = -2.f;
	problem.upper_bound(0) = 2.f;
	problem.upper_bound(1) = 2.f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 1.00001e-4f, 1e-9f);
	EXPECT_NEAR(result.solution(1), 1.f / 1.1f, 1e-6f);
}

TEST(Type4WeightedAllocatorTest, FullRankProblemDoesNotRequireResolvedRegularization)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem();

	for (int axis = 0; axis < 4; ++axis) {
		problem.effectiveness(axis, axis) = 1.f;
		problem.axis_weight(axis) = 1e10f;
		problem.target(axis) = (axis & 1) ? -0.4f : 0.4f;
	}

	const auto result = allocator.solve(problem);
	expectValidResult(result);

	for (int actuator = 0; actuator < 4; ++actuator) {
		EXPECT_NEAR(result.solution(actuator), problem.target(actuator), 1e-6f);
	}
}

TEST(Type4WeightedAllocatorTest, NearDependentEffectivenessRemainsSolvable)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem();
	problem.regularization = 1e-4f;
	problem.axis_weight(0) = 8.f;
	problem.axis_weight(1) = 8.f;
	problem.axis_weight(2) = 1.f;
	problem.axis_weight(3) = 4.f;
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(0, 1) = 1.f;
	problem.effectiveness(0, 2) = 1.f;
	problem.effectiveness(0, 3) = 1.f;
	problem.effectiveness(1, 0) = 1.f;
	problem.effectiveness(1, 1) = 1.01f;
	problem.effectiveness(1, 2) = 0.99f;
	problem.effectiveness(1, 3) = 1.02f;
	problem.effectiveness(2, 0) = 1.f;
	problem.effectiveness(2, 1) = 0.99f;
	problem.effectiveness(2, 2) = 1.02f;
	problem.effectiveness(2, 3) = 0.98f;
	problem.effectiveness(3, 0) = 1.f;
	problem.effectiveness(3, 1) = 1.02f;
	problem.effectiveness(3, 2) = 0.98f;
	problem.effectiveness(3, 3) = 1.01f;
	problem.target(0) = 2.f;
	problem.target(1) = 2.01f;
	problem.target(2) = 1.99f;
	problem.target(3) = 2.02f;

	const auto result = allocator.solve(problem);
	expectValidResult(result, 1e-3);
}

TEST(Type4WeightedAllocatorTest, IntermediateRoundoffDoesNotRejectValidOptimum)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem();
	problem.regularization = 4.87677994e-7f;
	problem.reference(0) = 0.033492744f;
	problem.reference(1) = -0.0658297837f;
	problem.reference(2) = -0.124986976f;
	problem.reference(3) = 0.0358937979f;
	problem.axis_weight(0) = 0.00104811718f;
	problem.axis_weight(1) = 0.000167179096f;
	problem.axis_weight(2) = 0.000186025092f;
	problem.axis_weight(3) = 0.233942255f;
	problem.target(0) = 0.0494296551f;
	problem.target(1) = -0.881421864f;
	problem.target(2) = 0.955625534f;
	problem.target(3) = -0.870416045f;
	problem.effectiveness(0, 0) = 463.673798f;
	problem.effectiveness(0, 1) = -748.536072f;
	problem.effectiveness(0, 2) = 0.000678226352f;
	problem.effectiveness(0, 3) = 0.0197233111f;
	problem.effectiveness(1, 0) = -0.45036605f;
	problem.effectiveness(1, 1) = 4.17735672f;
	problem.effectiveness(1, 2) = 1.89499223f;
	problem.effectiveness(1, 3) = 0.0139824841f;
	problem.effectiveness(2, 0) = 0.0163427554f;
	problem.effectiveness(2, 1) = 575.985779f;
	problem.effectiveness(2, 2) = -0.0200531203f;
	problem.effectiveness(2, 3) = 22.9614143f;
	problem.effectiveness(3, 0) = -6.51008558f;
	problem.effectiveness(3, 1) = -0.000505808275f;
	problem.effectiveness(3, 2) = -1774.35339f;
	problem.effectiveness(3, 3) = 0.00240915385f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 3.17699e-5f, 1e-7f);
	EXPECT_NEAR(result.solution(1), -4.53067e-5f, 1e-7f);
	EXPECT_NEAR(result.solution(2), 4.90495e-4f, 1e-7f);
	EXPECT_NEAR(result.solution(3), 4.25586e-2f, 1e-6f);
}

TEST(Type4WeightedAllocatorTest, CholeskyRoundoffDoesNotRejectInteriorOptimum)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(3);
	problem.regularization = 0.00119571888f;
	problem.effectiveness(0, 0) = -1074.17151f;
	problem.effectiveness(0, 1) = 2.36630356e-7f;
	problem.effectiveness(0, 2) = -49190.293f;
	problem.effectiveness(0, 3) = 23696.6523f;
	problem.effectiveness(1, 0) = 1.12418661e-6f;
	problem.effectiveness(1, 1) = 600.385559f;
	problem.effectiveness(1, 2) = 8.57223034f;
	problem.effectiveness(1, 3) = -1.27126086e-5f;
	problem.effectiveness(2, 0) = 0.0111448774f;
	problem.effectiveness(2, 1) = -118.169182f;
	problem.effectiveness(2, 2) = -5.5473032e-7f;
	problem.effectiveness(2, 3) = 1.85512101e-6f;
	problem.target(0) = 1.77920424e-6f;
	problem.target(1) = -3.17686317e-5f;
	problem.target(2) = 2.57272745e-7f;
	problem.axis_weight(0) = 0.248366609f;
	problem.axis_weight(1) = 0.000648992078f;
	problem.axis_weight(2) = 14.3015814f;
	problem.lower_bound(0) = -541.329041f;
	problem.lower_bound(1) = -0.646021307f;
	problem.lower_bound(2) = -0.733087897f;
	problem.lower_bound(3) = -4648.69385f;
	problem.upper_bound(0) = 541.329163f;
	problem.upper_bound(1) = 0.64370209f;
	problem.upper_bound(2) = 0.720535159f;
	problem.upper_bound(3) = 4648.81201f;
	problem.reference(0) = 0.000694400631f;
	problem.reference(1) = -0.000733314606f;
	problem.reference(2) = -0.0287903473f;
	problem.reference(3) = 0.0216650646f;

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.001389924f, 1e-7f);
	EXPECT_NEAR(result.solution(1), 1.289929e-7f, 1e-9f);
	EXPECT_NEAR(result.solution(2), 0.003008740f, 1e-7f);
	EXPECT_NEAR(result.solution(3), 0.006308648f, 1e-7f);
}

TEST(Type4WeightedAllocatorTest, UnresolvedCurvatureFailsClosedInsteadOfReturningWrongFace)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 2.90375992e-5f;
	problem.axis_weight(0) = 640.109192f;
	problem.target(0) = -4.18402433f;
	problem.effectiveness(0, 0) = -896.139648f;
	problem.effectiveness(0, 1) = -33.1465073f;
	problem.effectiveness(0, 2) = 841.942627f;
	problem.effectiveness(0, 3) = -672.408264f;
	problem.lower_bound(0) = -13.0707226f;
	problem.lower_bound(1) = -24.1316814f;
	problem.lower_bound(2) = -14.9340496f;
	problem.lower_bound(3) = -58.8121109f;
	problem.upper_bound(0) = 13.2704144f;
	problem.upper_bound(1) = 24.1817951f;
	problem.upper_bound(2) = -14.9257584f;
	problem.upper_bound(3) = 63.7196884f;
	problem.reference(0) = 50.3181839f;
	problem.reference(1) = 0.0225331448f;
	problem.reference(2) = -3.14205813f;
	problem.reference(3) = 0.00473246351f;

	const auto result = allocator.solve(problem);
	EXPECT_EQ(result.status, Type4WeightedAllocator::Status::NumericalFailure);
	EXPECT_EQ(result.solution, Type4WeightedAllocator::ActuatorVector{});
	EXPECT_EQ(result.faces_evaluated, Type4WeightedAllocator::NumFaces);
	EXPECT_EQ(result.linear_solves, 65);
}

TEST(Type4WeightedAllocatorTest, EqualityFixedDirectionsDoNotRequireResolvedRegularization)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 0.1f;
	problem.effectiveness(0, 0) = 1e8f;
	problem.target(0) = 5e7f;
	problem.reference(0) = 0.1f;
	problem.reference(1) = 0.2f;
	problem.reference(2) = 0.3f;
	problem.reference(3) = 0.4f;
	problem.lower_bound(0) = 0.f;
	problem.upper_bound(0) = 1.f;

	for (int actuator = 1; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		problem.lower_bound(actuator) = 0.f;
		problem.upper_bound(actuator) = 0.f;
	}

	const auto result = allocator.solve(problem);
	expectValidResult(result);
	EXPECT_NEAR(result.solution(0), 0.5f, 1e-6f);

	for (int actuator = 1; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		EXPECT_EQ(result.solution(actuator), 0.f);
		EXPECT_NE(result.lower_active_mask & (1u << actuator), 0u);
		EXPECT_NE(result.upper_active_mask & (1u << actuator), 0u);
	}
}

TEST(Type4WeightedAllocatorTest, FullyEqualityFixedProblemNeedsNoResolvedRegularization)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(1);
	problem.regularization = 0.1f;
	problem.effectiveness(0, 0) = 1e8f;
	problem.target(0) = 5e7f;

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		const float fixed_value = 0.1f * static_cast<float>(actuator + 1);
		problem.lower_bound(actuator) = fixed_value;
		problem.upper_bound(actuator) = fixed_value;
	}

	const auto result = allocator.solve(problem);
	expectValidResult(result);

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		EXPECT_EQ(result.solution(actuator), problem.lower_bound(actuator));
		EXPECT_NE(result.lower_active_mask & (1u << actuator), 0u);
		EXPECT_NE(result.upper_active_mask & (1u << actuator), 0u);
	}
}

TEST(Type4WeightedAllocatorTest, RepeatedSolveIsDeterministic)
{
	Type4WeightedAllocator allocator;
	auto problem = baseProblem(3);
	problem.effectiveness(0, 0) = 1.f;
	problem.effectiveness(0, 1) = -1.f;
	problem.effectiveness(1, 1) = 1.f;
	problem.effectiveness(1, 2) = -1.f;
	problem.effectiveness(2, 2) = 1.f;
	problem.effectiveness(2, 3) = -1.f;
	problem.target(0) = 0.7f;
	problem.target(1) = -0.4f;
	problem.target(2) = 0.9f;

	const auto first = allocator.solve(problem);
	expectValidResult(first);

	for (int iteration = 0; iteration < 20; ++iteration) {
		const auto repeated = allocator.solve(problem);
		expectValidResult(repeated);
		EXPECT_EQ(repeated.solution, first.solution);
		EXPECT_EQ(repeated.objective, first.objective);
		EXPECT_EQ(repeated.lower_active_mask, first.lower_active_mask);
		EXPECT_EQ(repeated.upper_active_mask, first.upper_active_mask);
	}
}
