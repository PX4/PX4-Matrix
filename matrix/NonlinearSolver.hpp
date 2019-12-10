
// Nonlinear Solver
// Based on TinySolver from ceres-solver.org
// Original author: mierle@gmail.com (Keir Mierle)
// Ported by: julian@auterion.com (Julian Kent)
// Algorithm based off of:
//
// [1] K. Madsen, H. Nielsen, O. Tingleoff.
//     Methods for Non-linear Least Squares Problems.
//     http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf

#pragma once

#include <cassert>
#include <cmath>

#include "math.hpp"

namespace matrix {

// To use tiny solver, create a class or struct that allows computing the cost
// function (described below). This is similar to a ceres::CostFunction, but is
// different to enable statically allocating all memory for the solver
// (specifically, enum sizes). Key parts are the Scalar typedef, the enums to
// describe problem sizes (needed to remove all heap allocations), and the
// operator() overload to evaluate the cost and (optionally) jacobians.
//
//   struct NonlinearSolverCostFunctionTraits {
//     typedef double Scalar;
//     enum {
//       NUM_RESIDUALS = <int> OR Eigen::Dynamic,
//       NUM_PARAMETERS = <int> OR Eigen::Dynamic,
//     };
//     bool operator()(const double* parameters,
//                     double* residuals,
//                     double* jacobian) const;
//
//     int NumResiduals() const;  -- Needed if NUM_RESIDUALS == Eigen::Dynamic.
//     int NumParameters() const; -- Needed if NUM_PARAMETERS == Eigen::Dynamic.
//   };
//
// For operator(), the size of the objects is:
//
//   double* parameters -- NUM_PARAMETERS or NumParameters()
//   double* residuals  -- NUM_RESIDUALS or NumResiduals()
//   double* jacobian   -- NUM_RESIDUALS * NUM_PARAMETERS in column-major format
//                         (Eigen's default); or NULL if no jacobian requested.
//
// An example (fully statically sized):
//
//   struct MyCostFunctionExample {
//     typedef double Scalar;
//     enum {
//       NUM_RESIDUALS = 2,
//       NUM_PARAMETERS = 3,
//     };
//     bool operator()(const double* parameters,
//                     double* residuals,
//                     double* jacobian) const {
//       residuals[0] = x + 2*y + 4*z;
//       residuals[1] = y * z;
//       if (jacobian) {
//         jacobian[0 * 2 + 0] = 1;   // First column (x).
//         jacobian[0 * 2 + 1] = 0;
//
//         jacobian[1 * 2 + 0] = 2;   // Second column (y).
//         jacobian[1 * 2 + 1] = z;
//
//         jacobian[2 * 2 + 0] = 4;   // Third column (z).
//         jacobian[2 * 2 + 1] = y;
//       }
//       return true;
//     }
//   };
//
// The solver supports either statically or dynamically sized cost
// functions. If the number of residuals is dynamic then the Function
// must define:
//
//   int NumResiduals() const;
//
// If the number of parameters is dynamic then the Function must
// define:
//
//   int NumParameters() const;
//
template <typename Function>
class NonlinearSolver {
 public:
  // This class needs to have an Eigen aligned operator new as it contains
  // fixed-size Eigen types.

  enum {
    NUM_RESIDUALS = Function::NUM_RESIDUALS,
    NUM_PARAMETERS = Function::NUM_PARAMETERS
  };
  using Scalar = typename Function::Type;
  using Parameters = Matrix<Scalar, NUM_PARAMETERS, 1>;

  enum Status {
    GRADIENT_TOO_SMALL,            // eps > max(J'*f(x))
    RELATIVE_STEP_SIZE_TOO_SMALL,  // eps > ||dx|| / (||x|| + eps)
    COST_TOO_SMALL,                // eps > ||f(x)||^2 / 2
    HIT_MAX_ITERATIONS,
  };

  struct Options {
    Scalar gradient_tolerance = 1e-10;  // eps > max(J'*f(x))
    Scalar parameter_tolerance = 1e-8;  // eps > ||dx|| / ||x||
    Scalar cost_threshold = 1e-6;       // eps > ||f(x)||
    Scalar initial_trust_region_radius = 1e4;
    int max_num_iterations = 50;
  };

  struct Summary {
    Scalar initial_cost = -1;       // 1/2 ||f(x)||^2
    Scalar final_cost = -1;         // 1/2 ||f(x)||^2
    Scalar gradient_max_norm = -1;  // max(J'f(x))
    int iterations = -1;
    Status status = HIT_MAX_ITERATIONS;
  };

  bool Update(const Function& function, const Parameters& x) {
    if (!function(x.data(), error_.data(), jacobian_.data())) {
      return false;
    }

    error_ = -error_;

    // On the first iteration, compute a diagonal (Jacobi) scaling
    // matrix, which we store as a vector.
    if (summary.iterations == 0) {
      // jacobi_scaling = 1 / (1 + diagonal(J'J))
      //
      // 1 is added to the denominator to regularize small diagonal
      // entries.
      jacobi_scaling_ = 1.0 / (1.0 + jacobian_.colwise().norm().array());
    }

    // This explicitly computes the normal equations, which is numerically
    // unstable. Nevertheless, it is often good enough and is fast.
    //
    // TODO(sameeragarwal): Refactor this to allow for DenseQR
    // factorization.
    jacobian_ = jacobian_ * jacobi_scaling_.asDiagonal();
    jtj_ = jacobian_.transpose() * jacobian_;
    g_ = jacobian_.transpose() * error_;
    summary.gradient_max_norm = g_.array().abs().maxCoeff();
    cost_ = error_.squaredNorm() / 2;
    return true;
  }

  const Summary& Solve(const Function& function, Parameters* x_and_min) {
    Initialize<NUM_RESIDUALS, NUM_PARAMETERS>(function);
    assert(x_and_min);
    Parameters& x = *x_and_min;
    summary = Summary();
    summary.iterations = 0;

    // TODO(sameeragarwal): Deal with failure here.
    Update(function, x);
    summary.initial_cost = cost_;
    summary.final_cost = cost_;

    if (summary.gradient_max_norm < options.gradient_tolerance) {
      summary.status = GRADIENT_TOO_SMALL;
      return summary;
    }

    if (cost_ < options.cost_threshold) {
      summary.status = COST_TOO_SMALL;
      return summary;
    }

    Scalar u = 1.0 / options.initial_trust_region_radius;
    Scalar v = 2;

    for (summary.iterations = 1;
         summary.iterations < options.max_num_iterations;
         summary.iterations++) {
      jtj_regularized_ = jtj_;
      const Scalar min_diagonal = 1e-6;
      const Scalar max_diagonal = 1e32;
      for (int i = 0; i < lm_diagonal_.rows(); ++i) {
        lm_diagonal_[i] = sqrt(
            u * min(max(jtj_(i, i), min_diagonal), max_diagonal));
        jtj_regularized_(i, i) += lm_diagonal_[i] * lm_diagonal_[i];
      }

      // TODO(sameeragarwal): Check for failure and deal with it.
      auto linear_solver = LeastSquaresSolver<Scalar, NUM_PARAMETERS, NUM_PARAMETERS>(jtj_regularized_);
      lm_step_ = linear_solver.solve(g_);
      dx_ = jacobi_scaling_.asDiagonal() * lm_step_;

      // Adding parameter_tolerance to x.norm() ensures that this
      // works if x is near zero.
      const Scalar parameter_tolerance =
          options.parameter_tolerance *
          (x.norm() + options.parameter_tolerance);
      if (dx_.norm() < parameter_tolerance) {
        summary.status = RELATIVE_STEP_SIZE_TOO_SMALL;
        break;
      }
      x_new_ = x + dx_;

      // TODO(keir): Add proper handling of errors from user eval of cost
      // functions.
      function(&x_new_[0], &f_x_new_[0], NULL);

      const Scalar cost_change = (2 * cost_ - f_x_new_.squaredNorm());

      // TODO(sameeragarwal): Better more numerically stable evaluation.
      const Scalar model_cost_change = lm_step_.dot(2 * g_ - jtj_ * lm_step_);

      // rho is the ratio of the actual reduction in error to the reduction
      // in error that would be obtained if the problem was linear. See [1]
      // for details.
      Scalar rho(cost_change / model_cost_change);
      if (rho > 0) {
        // Accept the Levenberg-Marquardt step because the linear
        // model fits well.
        x = x_new_;

        // TODO(sameeragarwal): Deal with failure.
        Update(function, x);
        if (summary.gradient_max_norm < options.gradient_tolerance) {
          summary.status = GRADIENT_TOO_SMALL;
          break;
        }

        if (cost_ < options.cost_threshold) {
          summary.status = COST_TOO_SMALL;
          break;
        }

        Scalar tmp = Scalar(2 * rho - 1);
        u = u * max(1 / 3., 1 - tmp * tmp * tmp);
        v = 2;
        continue;
      }

      // Reject the update because either the normal equations failed to solve
      // or the local linear model was not good (rho < 0). Instead, increase u
      // to move closer to gradient descent.
      u *= v;
      v *= 2;
    }

    summary.final_cost = cost_;
    return summary;
  }

  Options options;
  Summary summary;

 private:
  // Preallocate everything, including temporary storage needed for solving the
  // linear system. This allows reusing the intermediate storage across solves.

  Scalar cost_;
  Parameters dx_, x_new_, g_, jacobi_scaling_, lm_diagonal_, lm_step_;
  Matrix<Scalar, NUM_RESIDUALS, 1> error_, f_x_new_;
  Matrix<Scalar, NUM_RESIDUALS, NUM_PARAMETERS> jacobian_;
  Matrix<Scalar, NUM_PARAMETERS, NUM_PARAMETERS> jtj_, jtj_regularized_;

  // The following definitions are needed for template metaprogramming.
  template <bool Condition, typename T>
  struct enable_if;

  template <typename T>
  struct enable_if<true, T> {
    typedef T type;
  };

  // The number of parameters and residuals are statically sized.
  template <int R, int P>
  void Initialize(const Function& /* function */) {}

  void Initialize(int num_residuals, int num_parameters) {
    dx_.resize(num_parameters);
    x_new_.resize(num_parameters);
    g_.resize(num_parameters);
    jacobi_scaling_.resize(num_parameters);
    lm_diagonal_.resize(num_parameters);
    lm_step_.resize(num_parameters);
    error_.resize(num_residuals);
    f_x_new_.resize(num_residuals);
    jacobian_.resize(num_residuals, num_parameters);
    jtj_.resize(num_parameters, num_parameters);
    jtj_regularized_.resize(num_parameters, num_parameters);
  }
};

}


