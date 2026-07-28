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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

using type4_timing_bench::Type4WeightedAllocator;

namespace
{

class DeterministicRandom
{
public:
	explicit DeterministicRandom(uint32_t seed) :
		_state(seed)
	{
	}

	float uniform(float lower, float upper)
	{
		_state = 1664525u * _state + 1013904223u;
		const float unit = static_cast<float>(_state >> 8) * (1.f / 16777216.f);
		return lower + (upper - lower) * unit;
	}

private:
	uint32_t _state;
};

struct OracleResult {
	Type4WeightedAllocator::ActuatorVector solution{};
	uint8_t lower_active_mask{0};
	uint8_t upper_active_mask{0};
	bool valid{false};
};

bool solveLinearSystem(const double input_matrix[Type4WeightedAllocator::NumActuators]
		       [Type4WeightedAllocator::NumActuators],
		       const double input_rhs[Type4WeightedAllocator::NumActuators],
		       double solution[Type4WeightedAllocator::NumActuators], int size)
{
	double augmented[Type4WeightedAllocator::NumActuators][Type4WeightedAllocator::NumActuators + 1] {};

	for (int row = 0; row < size; ++row) {
		for (int column = 0; column < size; ++column) {
			augmented[row][column] = input_matrix[row][column];
		}

		augmented[row][size] = input_rhs[row];
	}

	for (int column = 0; column < size; ++column) {
		int pivot_row = column;

		for (int row = column + 1; row < size; ++row) {
			if (fabs(augmented[row][column]) > fabs(augmented[pivot_row][column])) {
				pivot_row = row;
			}
		}

		if (!std::isfinite(augmented[pivot_row][column]) || (fabs(augmented[pivot_row][column]) < 1e-14)) {
			return false;
		}

		if (pivot_row != column) {
			for (int entry = column; entry <= size; ++entry) {
				std::swap(augmented[column][entry], augmented[pivot_row][entry]);
			}
		}

		for (int row = column + 1; row < size; ++row) {
			const double factor = augmented[row][column] / augmented[column][column];

			for (int entry = column; entry <= size; ++entry) {
				augmented[row][entry] -= factor * augmented[column][entry];
			}
		}
	}

	for (int row = size - 1; row >= 0; --row) {
		double value = augmented[row][size];

		for (int column = row + 1; column < size; ++column) {
			value -= augmented[row][column] * solution[column];
		}

		solution[row] = value / augmented[row][row];

		if (!std::isfinite(solution[row])) {
			return false;
		}
	}

	return true;
}

long double objective(const Type4WeightedAllocator::Problem &problem,
		      const double solution[Type4WeightedAllocator::NumActuators])
{
	long double value = 0.0L;

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		long double modeled_wrench = 0.0L;

		for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
			modeled_wrench += static_cast<long double>(problem.effectiveness(axis, actuator))
					  * static_cast<long double>(solution[actuator]);
		}

		const long double residual = static_cast<long double>(problem.axis_weight(axis))
					     * (modeled_wrench - static_cast<long double>(problem.target(axis)));
		value += 0.5L * residual * residual;
	}

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		const long double error = static_cast<long double>(solution[actuator])
					  - static_cast<long double>(problem.reference(actuator));
		value += 0.5L * static_cast<long double>(problem.regularization) * error * error;
	}

	return value;
}

OracleResult solveExhaustiveOracle(const Type4WeightedAllocator::Problem &problem)
{
	double hessian[Type4WeightedAllocator::NumActuators][Type4WeightedAllocator::NumActuators] {};
	double linear_term[Type4WeightedAllocator::NumActuators] {};

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		const double weight = static_cast<double>(problem.axis_weight(axis));
		const double weight_squared = weight * weight;

		for (int row = 0; row < Type4WeightedAllocator::NumActuators; ++row) {
			const double effectiveness = static_cast<double>(problem.effectiveness(axis, row));
			linear_term[row] += weight_squared * effectiveness * static_cast<double>(problem.target(axis));

			for (int column = 0; column < Type4WeightedAllocator::NumActuators; ++column) {
				hessian[row][column] += weight_squared * effectiveness
							* static_cast<double>(problem.effectiveness(axis, column));
			}
		}
	}

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		hessian[actuator][actuator] += static_cast<double>(problem.regularization);
		linear_term[actuator] += static_cast<double>(problem.regularization)
					 * static_cast<double>(problem.reference(actuator));
	}

	long double best_objective = std::numeric_limits<long double>::infinity();
	double best_solution[Type4WeightedAllocator::NumActuators] {};
	bool solution_found = false;

	for (int face = 0; face < Type4WeightedAllocator::NumFaces; ++face) {
		double candidate[Type4WeightedAllocator::NumActuators] {};
		int free_indices[Type4WeightedAllocator::NumActuators] {};
		int free_count = 0;
		int encoded_face = face;

		for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
			const int state = encoded_face % 3;
			encoded_face /= 3;

			if (state == 0) {
				free_indices[free_count++] = actuator;

			} else {
				candidate[actuator] = state == 1 ? static_cast<double>(problem.lower_bound(actuator))
						      : static_cast<double>(problem.upper_bound(actuator));
			}
		}

		if (free_count > 0) {
			double reduced_matrix[Type4WeightedAllocator::NumActuators][Type4WeightedAllocator::NumActuators] {};
			double reduced_rhs[Type4WeightedAllocator::NumActuators] {};
			double reduced_solution[Type4WeightedAllocator::NumActuators] {};

			for (int free_row = 0; free_row < free_count; ++free_row) {
				const int row = free_indices[free_row];
				reduced_rhs[free_row] = linear_term[row];

				for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
					bool free_actuator = false;

					for (int free_column = 0; free_column < free_count; ++free_column) {
						free_actuator |= free_indices[free_column] == actuator;
					}

					if (!free_actuator) {
						reduced_rhs[free_row] -= hessian[row][actuator] * candidate[actuator];
					}
				}

				for (int free_column = 0; free_column < free_count; ++free_column) {
					reduced_matrix[free_row][free_column] = hessian[row][free_indices[free_column]];
				}
			}

			if (!solveLinearSystem(reduced_matrix, reduced_rhs, reduced_solution, free_count)) {
				continue;
			}

			for (int free_index = 0; free_index < free_count; ++free_index) {
				candidate[free_indices[free_index]] = reduced_solution[free_index];
			}
		}

		bool feasible = true;

		for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
			const double lower = static_cast<double>(problem.lower_bound(actuator));
			const double upper = static_cast<double>(problem.upper_bound(actuator));
			const double tolerance = 2e-12 * (1.0 + fmax(fabs(lower), fabs(upper)));

			if (!std::isfinite(candidate[actuator]) || (candidate[actuator] < lower - tolerance)
			    || (candidate[actuator] > upper + tolerance)) {
				feasible = false;
				break;
			}

			candidate[actuator] = fmin(fmax(candidate[actuator], lower), upper);
		}

		if (!feasible) {
			continue;
		}

		const long double candidate_objective = objective(problem, candidate);

		if (!solution_found || (candidate_objective < best_objective)) {
			std::copy(candidate, candidate + Type4WeightedAllocator::NumActuators, best_solution);
			best_objective = candidate_objective;
			solution_found = true;
		}
	}

	OracleResult result{};
	result.valid = solution_found;

	if (!solution_found) {
		return result;
	}

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		result.solution(actuator) = static_cast<float>(best_solution[actuator]);

		if (result.solution(actuator) <= problem.lower_bound(actuator)) {
			result.lower_active_mask |= 1u << actuator;
		}

		if (result.solution(actuator) >= problem.upper_bound(actuator)) {
			result.upper_active_mask |= 1u << actuator;
		}
	}

	return result;
}

Type4WeightedAllocator::Problem makeSequenceProblem(int sample, DeterministicRandom &random,
		const Type4WeightedAllocator::ActuatorVector &previous_command)
{
	Type4WeightedAllocator::Problem problem{};
	problem.num_axes = 4;
	problem.regularization = 0.02f;
	const int kind = sample % 5;
	const float direction = ((sample / 4) & 1) ? 1.f : -1.f;

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		problem.axis_weight(actuator) = random.uniform(0.5f, 8.f);
		problem.reference(actuator) = previous_command(actuator);
		const float delta = 0.015f + 0.01f * static_cast<float>((sample + actuator) % 5);
		problem.lower_bound(actuator) = fmaxf(0.f, previous_command(actuator) - delta);
		problem.upper_bound(actuator) = fminf(1.f, previous_command(actuator) + delta);
		problem.target(actuator) = direction * random.uniform(0.4f, 3.f);
	}

	if (kind == 0) {
		for (int axis = 0; axis < problem.num_axes; ++axis) {
			for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
				problem.effectiveness(axis, actuator) = random.uniform(-2.f, 2.f);
			}
		}

	} else if (kind == 1) {
		for (int axis = 0; axis < problem.num_axes; ++axis) {
			problem.effectiveness(axis, axis) = 1.f;
			problem.target(axis) = direction * (3.f + 0.1f * static_cast<float>(axis));
		}

	} else if (kind == 2) {
		for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
			const float entry = random.uniform(-1.5f, 1.5f);
			problem.effectiveness(0, actuator) = entry;
			problem.effectiveness(1, actuator) = entry;
			problem.effectiveness(2, actuator) = 0.f;
			problem.effectiveness(3, actuator) = 0.5f * entry;
		}

		problem.axis_weight(2) = 0.f;
		problem.regularization = 0.05f;

	} else if (kind == 3) {
		constexpr float Effectiveness[Type4WeightedAllocator::NumActuators] {50.f, 0.02f, 5.f, 0.2f};
		constexpr float Weight[Type4WeightedAllocator::NumActuators] {0.02f, 20.f, 0.2f, 5.f};

		for (int axis = 0; axis < problem.num_axes; ++axis) {
			problem.effectiveness(axis, axis) = Effectiveness[axis];
			problem.axis_weight(axis) = Weight[axis];
		}

		problem.regularization = 0.005f;

	} else {
		for (int axis = 0; axis < problem.num_axes; ++axis) {
			for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
				problem.effectiveness(axis, actuator) = 1.f + 0.005f
									* static_cast<float>((axis + 2 * actuator) % 5 - 2);
			}
		}

		problem.regularization = 0.001f;
	}

	if ((sample % 37) == 0) {
		const int actuator = (sample / 37) % Type4WeightedAllocator::NumActuators;
		problem.lower_bound(actuator) = previous_command(actuator);
		problem.upper_bound(actuator) = previous_command(actuator);
	}

	return problem;
}

void expectOracleMatch(const Type4WeightedAllocator::Result &actual, const OracleResult &oracle)
{
	ASSERT_TRUE(oracle.valid);
	ASSERT_EQ(actual.status, Type4WeightedAllocator::Status::Success);
	EXPECT_EQ(actual.lower_active_mask, oracle.lower_active_mask);
	EXPECT_EQ(actual.upper_active_mask, oracle.upper_active_mask);

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		EXPECT_NEAR(actual.solution(actuator), oracle.solution(actuator), 2e-5f);
	}
}

} // namespace

TEST(Type4WeightedAllocatorOracleTest, DeterministicMovingSequenceMatchesExhaustiveOracle)
{
	constexpr int SampleCount = 256;
	DeterministicRandom random{0x5a17beefu};
	Type4WeightedAllocator allocator;
	Type4WeightedAllocator::ActuatorVector previous_command{};
	std::vector<Type4WeightedAllocator::Problem> problems;
	std::vector<Type4WeightedAllocator::Result> first_results;
	problems.reserve(SampleCount);
	first_results.reserve(SampleCount);

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		previous_command(actuator) = 0.5f;
	}

	int warm_hits = 0;
	int warm_transitions = 0;

	for (int sample = 0; sample < SampleCount; ++sample) {
		const auto problem = makeSequenceProblem(sample, random, previous_command);
		const OracleResult oracle = solveExhaustiveOracle(problem);
		const auto actual = allocator.solve(problem);
		expectOracleMatch(actual, oracle);
		EXPECT_GE(actual.iterations, 1);
		EXPECT_LE(actual.iterations, Type4WeightedAllocator::MaxIterations);
		EXPECT_EQ(actual.iterations, actual.faces_evaluated);
		EXPECT_LE(actual.linear_solves, actual.faces_evaluated);

		if (sample == 0) {
			EXPECT_FALSE(actual.warm_start_attempted);

		} else {
			EXPECT_TRUE(actual.warm_start_attempted);
			warm_hits += actual.warm_start_hit;
			warm_transitions += !actual.warm_start_hit;
		}

		problems.push_back(problem);
		first_results.push_back(actual);
		previous_command = actual.solution;
	}

	EXPECT_GT(warm_hits, 0);
	EXPECT_GT(warm_transitions, 0);

	Type4WeightedAllocator repeated_allocator;

	for (int sample = 0; sample < SampleCount; ++sample) {
		const auto repeated = repeated_allocator.solve(problems[sample]);
		const auto &first = first_results[sample];
		EXPECT_EQ(repeated.status, first.status);
		EXPECT_EQ(repeated.solution, first.solution);
		EXPECT_EQ(repeated.lower_active_mask, first.lower_active_mask);
		EXPECT_EQ(repeated.upper_active_mask, first.upper_active_mask);
		EXPECT_EQ(repeated.iterations, first.iterations);
		EXPECT_EQ(repeated.faces_evaluated, first.faces_evaluated);
		EXPECT_EQ(repeated.linear_solves, first.linear_solves);
		EXPECT_EQ(repeated.warm_start_attempted, first.warm_start_attempted);
		EXPECT_EQ(repeated.warm_start_hit, first.warm_start_hit);
	}
}
