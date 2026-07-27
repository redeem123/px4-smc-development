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

#include "Type4WeightedAllocator.hpp"

#include <math.h>

namespace type4_timing_bench
{

namespace
{

enum class FaceState : uint8_t {
	Free = 0,
	Lower,
	Upper,
};

bool finiteVector(const double vector[Type4WeightedAllocator::NumActuators])
{
	for (int i = 0; i < Type4WeightedAllocator::NumActuators; ++i) {
		if (!PX4_ISFINITE(vector[i])) {
			return false;
		}
	}

	return true;
}

void addCompensated(double value, double &sum, double &correction)
{
	const double updated_sum = sum + value;

	if (fabs(sum) >= fabs(value)) {
		correction += (sum - updated_sum) + value;

	} else {
		correction += (value - updated_sum) + sum;
	}

	sum = updated_sum;
}

double objectiveDifference(const Type4WeightedAllocator::Problem &problem,
			   const double candidate[Type4WeightedAllocator::NumActuators],
			   const double incumbent[Type4WeightedAllocator::NumActuators])
{
	double difference = 0.0;
	double difference_correction = 0.0;

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		double residual_delta = 0.0;
		double residual_delta_correction = 0.0;
		double incumbent_residual = -static_cast<double>(problem.target(axis));
		double incumbent_residual_correction = 0.0;

		for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
			const double effectiveness = static_cast<double>(problem.effectiveness(axis, actuator));
			addCompensated(effectiveness * (candidate[actuator] - incumbent[actuator]),
				       residual_delta, residual_delta_correction);
			addCompensated(effectiveness * incumbent[actuator],
				       incumbent_residual, incumbent_residual_correction);
		}

		const double delta = residual_delta + residual_delta_correction;
		const double residual = incumbent_residual + incumbent_residual_correction;
		const double weight = static_cast<double>(problem.axis_weight(axis));
		addCompensated(weight * weight * delta * (residual + 0.5 * delta),
			       difference, difference_correction);
	}

	const double regularization = static_cast<double>(problem.regularization);

	for (int actuator = 0; actuator < Type4WeightedAllocator::NumActuators; ++actuator) {
		const double reference = static_cast<double>(problem.reference(actuator));
		const double candidate_error = candidate[actuator] - reference;
		const double incumbent_error = incumbent[actuator] - reference;
		addCompensated(0.5 * regularization * (candidate_error - incumbent_error)
			       * (candidate_error + incumbent_error), difference, difference_correction);
	}

	return difference + difference_correction;
}

} // namespace

Type4WeightedAllocator::Result Type4WeightedAllocator::solve(const Problem &problem) const
{
	Result result{};

	if (!validateProblem(problem)) {
		return result;
	}

	double data_hessian[NumActuators][NumActuators] {};
	double data_linear_term[NumActuators] {};
	double data_scale = 0.0;

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		const double weight = static_cast<double>(problem.axis_weight(axis));
		const double weight_squared = weight * weight;

		for (int row = 0; row < NumActuators; ++row) {
			const double weighted_effectiveness = weight_squared * static_cast<double>(problem.effectiveness(axis, row));
			data_linear_term[row] += weighted_effectiveness * static_cast<double>(problem.target(axis));

			for (int column = 0; column < NumActuators; ++column) {
				data_hessian[row][column] += weighted_effectiveness
							     * static_cast<double>(problem.effectiveness(axis, column));
			}
		}
	}

	for (int row = 0; row < NumActuators; ++row) {
		data_scale = fmax(data_scale, fabs(data_linear_term[row]));

		for (int column = 0; column < NumActuators; ++column) {
			data_scale = fmax(data_scale, fabs(data_hessian[row][column]));
		}
	}

	if (!PX4_ISFINITE(data_scale)) {
		result.status = Status::NumericalFailure;
		return result;
	}

	const double objective_scale_double = fmax(1.0, data_scale);
	const double inverse_objective_scale = 1.0 / objective_scale_double;
	const double scaled_regularization = static_cast<double>(problem.regularization) * inverse_objective_scale;
	double hessian[NumActuators][NumActuators] {};
	double linear_term[NumActuators] {};

	if (!PX4_ISFINITE(scaled_regularization) || (scaled_regularization <= 0.0)) {
		result.status = Status::NumericalFailure;
		return result;
	}

	for (int row = 0; row < NumActuators; ++row) {
		linear_term[row] = data_linear_term[row] * inverse_objective_scale;

		for (int column = 0; column < NumActuators; ++column) {
			hessian[row][column] = data_hessian[row][column] * inverse_objective_scale;
		}

	}

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		hessian[actuator][actuator] += scaled_regularization;
		linear_term[actuator] += scaled_regularization * static_cast<double>(problem.reference(actuator));
	}

	bool resolved_curvature = true;
	int domain_indices[NumActuators] {};
	int domain_size = 0;

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		if (problem.lower_bound(actuator) < problem.upper_bound(actuator)) {
			domain_indices[domain_size++] = actuator;
		}
	}

	if (domain_size > 0) {
		double domain_hessian[NumActuators][NumActuators] {};
		double zero_rhs[NumActuators] {};
		double unused_solution[NumActuators] {};

		for (int row = 0; row < domain_size; ++row) {
			for (int column = 0; column < domain_size; ++column) {
				domain_hessian[row][column] = hessian[domain_indices[row]][domain_indices[column]];
			}
		}

		resolved_curvature = solveCholesky(domain_hessian, zero_rhs, unused_solution, domain_size);
	}

	bool solution_found = false;
	double best_solution[NumActuators] {};

	for (int face = 0; face < NumFaces; ++face) {
		++result.faces_evaluated;

		double candidate[NumActuators] {};
		FaceState face_state[NumActuators] {};
		int free_indices[NumActuators] {};
		int free_count = 0;
		int encoded_face = face;

		for (int actuator = 0; actuator < NumActuators; ++actuator) {
			face_state[actuator] = static_cast<FaceState>(encoded_face % 3);
			encoded_face /= 3;

			switch (face_state[actuator]) {
			case FaceState::Free:
				free_indices[free_count++] = actuator;
				break;

			case FaceState::Lower:
				candidate[actuator] = static_cast<double>(problem.lower_bound(actuator));
				break;

			case FaceState::Upper:
				candidate[actuator] = static_cast<double>(problem.upper_bound(actuator));
				break;
			}
		}

		if (free_count > 0) {
			double reduced_hessian[NumActuators][NumActuators] {};
			double reduced_rhs[NumActuators] {};
			double free_solution[NumActuators] {};

			for (int free_row = 0; free_row < free_count; ++free_row) {
				const int row = free_indices[free_row];
				reduced_rhs[free_row] = linear_term[row];

				for (int actuator = 0; actuator < NumActuators; ++actuator) {
					bool is_free = false;

					for (int free_column = 0; free_column < free_count; ++free_column) {
						if (free_indices[free_column] == actuator) {
							is_free = true;
							break;
						}
					}

					if (!is_free) {
						reduced_rhs[free_row] -= hessian[row][actuator] * static_cast<double>(candidate[actuator]);
					}
				}

				for (int free_column = 0; free_column < free_count; ++free_column) {
					reduced_hessian[free_row][free_column] = hessian[row][free_indices[free_column]];
				}
			}

			++result.linear_solves;

			if (!solveCholesky(reduced_hessian, reduced_rhs, free_solution, free_count)) {
				continue;
			}

			for (int free_index = 0; free_index < free_count; ++free_index) {
				candidate[free_indices[free_index]] = free_solution[free_index];
			}
		}

		bool candidate_feasible = finiteVector(candidate);

		for (int actuator = 0; candidate_feasible && actuator < NumActuators; ++actuator) {
			const double lower_bound = static_cast<double>(problem.lower_bound(actuator));
			const double upper_bound = static_cast<double>(problem.upper_bound(actuator));

			if ((candidate[actuator] < lower_bound) || (candidate[actuator] > upper_bound)) {
				candidate_feasible = false;
			}
		}

		if (!candidate_feasible) {
			continue;
		}

		bool candidate_better = !solution_found;

		if (solution_found) {
			const double objective_difference = objectiveDifference(problem, candidate, best_solution);
			candidate_better = PX4_ISFINITE(objective_difference) && (objective_difference < 0.0);
		}

		if (candidate_better) {
			for (int actuator = 0; actuator < NumActuators; ++actuator) {
				best_solution[actuator] = candidate[actuator];
			}

			solution_found = true;
		}
	}

	if (!resolved_curvature) {
		result.status = Status::NumericalFailure;
		return result;
	}

	if (!solution_found) {
		result.status = Status::NumericalFailure;
		return result;
	}

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		result.solution(actuator) = static_cast<float>(best_solution[actuator]);
	}

	double gradient[NumActuators] {};
	double gradient_roundoff_scale[NumActuators] {};

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		double modeled_wrench = 0.0;
		double modeled_wrench_scale = 0.0;

		for (int actuator = 0; actuator < NumActuators; ++actuator) {
			const double effectiveness = static_cast<double>(problem.effectiveness(axis, actuator));
			const double solution = static_cast<double>(result.solution(actuator));
			modeled_wrench += effectiveness * solution;
			modeled_wrench_scale += fabs(effectiveness * solution);
		}

		const double target = static_cast<double>(problem.target(axis));
		const double weight = static_cast<double>(problem.axis_weight(axis));
		const double weight_squared = weight * weight;
		const double weighted_residual = weight_squared * (modeled_wrench - target);
		const double weighted_residual_scale = weight_squared * (modeled_wrench_scale + fabs(target));

		for (int actuator = 0; actuator < NumActuators; ++actuator) {
			const double effectiveness = static_cast<double>(problem.effectiveness(axis, actuator));
			gradient[actuator] += effectiveness * weighted_residual;
			gradient_roundoff_scale[actuator] += fabs(effectiveness) * weighted_residual_scale;
		}
	}

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		const double regularization = static_cast<double>(problem.regularization);
		const double solution = static_cast<double>(result.solution(actuator));
		const double reference = static_cast<double>(problem.reference(actuator));
		gradient[actuator] += regularization * (solution - reference);
		gradient_roundoff_scale[actuator] += regularization * (fabs(solution) + fabs(reference));

		const bool equality_bound = problem.upper_bound(actuator) <= problem.lower_bound(actuator);
		const bool at_lower_bound = result.solution(actuator) <= problem.lower_bound(actuator);
		const bool at_upper_bound = result.solution(actuator) >= problem.upper_bound(actuator);
		double stationarity_violation = fabs(gradient[actuator]);

		if (equality_bound) {
			result.lower_active_mask |= 1u << actuator;
			result.upper_active_mask |= 1u << actuator;
			stationarity_violation = 0.0;

		} else if (at_lower_bound) {
			result.lower_active_mask |= 1u << actuator;
			stationarity_violation = fmax(-gradient[actuator], 0.0);

		} else if (at_upper_bound) {
			result.upper_active_mask |= 1u << actuator;
			stationarity_violation = fmax(gradient[actuator], 0.0);
		}

		result.kkt_residual_inf = fmax(result.kkt_residual_inf, stationarity_violation);

		if (gradient_roundoff_scale[actuator] > 0.0) {
			result.normalized_kkt_residual_inf = fmax(result.normalized_kkt_residual_inf,
					stationarity_violation / gradient_roundoff_scale[actuator]);

		} else if (stationarity_violation > 0.0) {
			result.normalized_kkt_residual_inf = HUGE_VAL;
		}

		const double lower_violation = fmax(static_cast<double>(problem.lower_bound(actuator)) - solution, 0.0);
		const double upper_violation = fmax(solution - static_cast<double>(problem.upper_bound(actuator)), 0.0);
		result.primal_residual_inf = fmax(result.primal_residual_inf, fmax(lower_violation, upper_violation));
	}

	result.normalized_kkt_residual_inf = fmax(result.normalized_kkt_residual_inf, result.primal_residual_inf);
	result.kkt_residual_inf = fmax(result.kkt_residual_inf, result.primal_residual_inf);
	result.objective = calculateObjective(problem, result.solution);
	result.objective_scale = objective_scale_double;

	if (!PX4_ISFINITE(result.objective) || !PX4_ISFINITE(result.kkt_residual_inf)
	    || !PX4_ISFINITE(result.normalized_kkt_residual_inf) || !PX4_ISFINITE(result.primal_residual_inf)) {
		result.status = Status::NumericalFailure;
		return result;
	}

	constexpr double ValidationKktTolerance = 1e-4;

	if ((result.primal_residual_inf > ValidationKktTolerance)
	    || (result.normalized_kkt_residual_inf > ValidationKktTolerance)) {
		const uint8_t faces_evaluated = result.faces_evaluated;
		const uint8_t linear_solves = result.linear_solves;
		result = Result{};
		result.faces_evaluated = faces_evaluated;
		result.linear_solves = linear_solves;
		result.status = Status::NumericalFailure;
		return result;
	}

	result.status = Status::Success;
	return result;
}

double Type4WeightedAllocator::calculateObjective(const Problem &problem, const ActuatorVector &solution)
{
	double objective = 0.0;

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		double modeled_wrench = 0.0;

		for (int actuator = 0; actuator < NumActuators; ++actuator) {
			modeled_wrench += static_cast<double>(problem.effectiveness(axis, actuator))
					   * static_cast<double>(solution(actuator));
		}

		const double weighted_residual = static_cast<double>(problem.axis_weight(axis))
						 * (modeled_wrench - static_cast<double>(problem.target(axis)));
		objective += 0.5 * weighted_residual * weighted_residual;
	}

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		const double reference_error = static_cast<double>(solution(actuator))
					       - static_cast<double>(problem.reference(actuator));
		objective += 0.5 * static_cast<double>(problem.regularization) * reference_error * reference_error;
	}

	return objective;
}

bool Type4WeightedAllocator::validateProblem(const Problem &problem)
{
	if ((problem.num_axes == 0) || (problem.num_axes > MaxWrenchAxes)
	    || !PX4_ISFINITE(problem.regularization) || (problem.regularization <= 0.f)) {
		return false;
	}

	for (int actuator = 0; actuator < NumActuators; ++actuator) {
		if (!PX4_ISFINITE(problem.lower_bound(actuator)) || !PX4_ISFINITE(problem.upper_bound(actuator))
		    || !PX4_ISFINITE(problem.reference(actuator))
		    || (problem.lower_bound(actuator) > problem.upper_bound(actuator))) {
			return false;
		}
	}

	for (int axis = 0; axis < problem.num_axes; ++axis) {
		if (!PX4_ISFINITE(problem.target(axis)) || !PX4_ISFINITE(problem.axis_weight(axis))
		    || (problem.axis_weight(axis) < 0.f)) {
			return false;
		}

		for (int actuator = 0; actuator < NumActuators; ++actuator) {
			if (!PX4_ISFINITE(problem.effectiveness(axis, actuator))) {
				return false;
			}
		}
	}

	return true;
}

bool Type4WeightedAllocator::solveCholesky(const double matrix[NumActuators][NumActuators],
		const double rhs[NumActuators], double solution[NumActuators], int size)
{
	double lower[NumActuators][NumActuators] {};
	double intermediate[NumActuators] {};

	for (int row = 0; row < size; ++row) {
		for (int column = 0; column <= row; ++column) {
			double value = matrix[row][column];

			for (int index = 0; index < column; ++index) {
				value -= lower[row][index] * lower[column][index];
			}

			if (row == column) {
				double row_scale = 0.0;

				for (int index = 0; index < size; ++index) {
					row_scale = fmax(row_scale, fabs(matrix[row][index]));
				}

				if (!PX4_ISFINITE(value) || (value <= 64.0 * __DBL_EPSILON__ * row_scale)) {
					return false;
				}

				lower[row][column] = sqrt(value);

			} else {
				lower[row][column] = value / lower[column][column];
			}
		}
	}

	for (int row = 0; row < size; ++row) {
		double value = rhs[row];

		for (int column = 0; column < row; ++column) {
			value -= lower[row][column] * intermediate[column];
		}

		intermediate[row] = value / lower[row][row];
	}

	for (int row = size - 1; row >= 0; --row) {
		double value = intermediate[row];

		for (int column = row + 1; column < size; ++column) {
			value -= lower[column][row] * solution[column];
		}

		solution[row] = value / lower[row][row];

		if (!PX4_ISFINITE(solution[row])) {
			return false;
		}
	}

	return true;
}

} // namespace type4_timing_bench
