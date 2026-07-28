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

#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include <cstdint>

namespace type4_timing_bench
{

class Type4WeightedAllocator
{
public:
	static constexpr int NumActuators = 4;
	static constexpr int MaxWrenchAxes = 6;
	static constexpr int NumFaces = 81;
	static constexpr int MaxIterations = NumFaces;

	using EffectivenessMatrix = matrix::Matrix<float, MaxWrenchAxes, NumActuators>;
	using WrenchVector = matrix::Vector<float, MaxWrenchAxes>;
	using ActuatorVector = matrix::Vector<float, NumActuators>;

	enum class Status : uint8_t {
		Success = 0,
		InvalidInput,
		NumericalFailure,
	};

	struct Problem {
		EffectivenessMatrix effectiveness{};
		WrenchVector target{};
		WrenchVector axis_weight{};
		ActuatorVector lower_bound{};
		ActuatorVector upper_bound{};
		ActuatorVector reference{};
		float regularization{0.f};
		uint8_t num_axes{0};
	};

	struct Result {
		ActuatorVector solution{};
		double objective{0.0};
		double objective_scale{1.0};
		double kkt_residual_inf{0.0};
		double normalized_kkt_residual_inf{0.0};
		double primal_residual_inf{0.0};
		uint8_t lower_active_mask{0};
		uint8_t upper_active_mask{0};
		uint8_t iterations{0};
		uint8_t faces_evaluated{0};
		uint8_t linear_solves{0};
		bool warm_start_attempted{false};
		bool warm_start_hit{false};
		Status status{Status::InvalidInput};
	};

	Result solve(const Problem &problem);
	void reset();

private:
	static double calculateObjective(const Problem &problem, const ActuatorVector &solution);
	static bool validateProblem(const Problem &problem);
	static bool solveCholesky(const double matrix[NumActuators][NumActuators], const double rhs[NumActuators],
				  double solution[NumActuators], int size);

	uint8_t _warm_face{0};
	bool _warm_state_valid{false};
};

} // namespace type4_timing_bench
