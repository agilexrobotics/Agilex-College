#pragma once

#ifdef WIN32
#define _USE_MATH_DEFINES
#endif
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Eigen>
#include <unsupported/Eigen/Polynomials>

namespace yaik_cpp {

// Indicate the value is invalid
constexpr double invalid_value = std::numeric_limits<double>::max();

// Safe operations
template <typename T>
T safe_sqrt(T a) {
  return a >= 0 ? std::sqrt(a) : 0.0;
}

template <typename T>
T safe_asin(T a) {
  if (a > 1)
    return 0.5 * M_PI;
  else if (a < -1)
    return -0.5 * M_PI;
  else
    return std::asin(a);
}

template <typename T>
T safe_acos(T a) {
  if (a > 1)
    return 0.0;
  else if (a < -1)
    return -M_PI;
  else
    return std::acos(a);
}

template <int N>
void removeDuplicate(std::vector<std::array<double, N>>& ik_output,
                      double tolerance) {
  // Do nothing if size less than 1
  if (ik_output.size() <= 1) return;

  // Functor to compare two vectors
  auto is_duplicate = [tolerance](const std::array<double, N>& a,
                                  const std::array<double, N>& b) -> bool {
    for (std::size_t j = 0; j < a.size(); j++) {
      if (std::abs(a[j] - b[j]) > tolerance) return false;
    }
    return true;
  };

  // There is no duplication in [0, no_duplicate_until]
  std::size_t no_duplicate_until = 0;
  std::size_t next_to_try = 1;

  // Processing loop
  while (next_to_try < ik_output.size()) {
    // Check duplication in [0, no_duplicate_until]
    bool contains_duplicate = false;
    assert(no_duplicate_until + 1 <= next_to_try);
    for (std::size_t i = 0; i <= no_duplicate_until; i++) {
      if (is_duplicate(ik_output[i], ik_output[next_to_try])) {
        contains_duplicate = true;
        break;
      }
    }

    // Update index
    if (contains_duplicate) {
      next_to_try += 1;
    } else {
      // Copy the result to no_duplicate_until + 1
      std::size_t copied_to = no_duplicate_until + 1;
      assert(copied_to < ik_output.size());
      if (copied_to != next_to_try) {
        ik_output[copied_to] = ik_output[next_to_try];
      }

      // Update the index
      no_duplicate_until = copied_to;
      next_to_try += 1;
    }
  }

  // [0, no_duplicate_until] is the range
  assert(no_duplicate_until < ik_output.size());
  ik_output.resize(no_duplicate_until + 1);
}

// Fixe-size array to hold ik solution
template <int N>
using IntermediateSolution = std::array<double, N>;

// Construct an ik solution with invalid valid
template <int N>
IntermediateSolution<N> make_intermediate_solution() {
  IntermediateSolution<N> solution;
  std::fill(solution.begin(), solution.end(), invalid_value);
  return solution;
}

// A queue with fixed capacity
template <typename T, int Capacity>
class FixedBufferQueue {
 private:
  std::array<T, Capacity> queue_element_;
  int top_index_;

 public:
  explicit FixedBufferQueue() : top_index_(0) {
    static_assert(Capacity >= 1, "Zero capacity not allowed");
  }
  void reset() { top_index_ = 0; }

  /// Push an element into the queue.
  /// If push succeed, then return the appended idx
  /// Else, return -1.
  int push(T element) {
    if (top_index_ >= queue_element_.size()) return -1;
    int appended_idx = top_index_;
    queue_element_[top_index_] = std::move(element);
    top_index_ += 1;
    return appended_idx;
  }

  /// Element access by index
  const T& operator[](int index) const { return queue_element_[index]; }
  T& operator[](int index) { return queue_element_[index]; }
  int size() const { return top_index_; }
};

// The queue to hold the solution
template <int Nq, int Capacity = 16>
struct SolutionQueue {
  FixedBufferQueue<IntermediateSolution<Nq>, Capacity> solutions;
  std::array<bool, Capacity> solutions_validity;

  explicit SolutionQueue() = default;
  void reset() {
    solutions.reset();
    std::fill(solutions_validity.begin(), solutions_validity.end(), false);
  }

  /// Push an solution into the queue
  int push_solution(IntermediateSolution<Nq> solution) {
    int appended_idx = solutions.push(std::move(solution));
    if (appended_idx >= 0) solutions_validity[appended_idx] = true;
    return appended_idx;
  }

  /// Interface for obtain solution and check its validity
  // clang-format off
  const IntermediateSolution<Nq>& get_solution(int solution_idx) const { return solutions[solution_idx]; }
  IntermediateSolution<Nq>& get_solution(int solution_idx) { return solutions[solution_idx]; }
  bool is_solution_valid(int solution_idx) const { return solutions_validity[solution_idx]; }
  int size() const { return solutions.size(); }
  // clang-format on
};

// The index workspace for different nodes
template <int max_n_solution = 32>
struct NodeIndexWorkspace {
  using IndexQueue = FixedBufferQueue<int, max_n_solution>;
  std::vector<IndexQueue> node_input_indices_vector;
  std::vector<bool> node_input_validity_vector;

  // Constructors and mutators
  NodeIndexWorkspace() = default;
  void reset(int n_node) {
    node_input_indices_vector.resize(n_node);
    node_input_validity_vector.resize(n_node);
    for (auto i = 0; i < n_node; i++) {
      node_input_indices_vector[i].reset();
      node_input_validity_vector[i] = false;
    }

    // Set the root node validity to true
    node_input_validity_vector[0] = true;
  }

  /// Add a index to a node
  bool append_index_to_node(int node_idx, int index_to_append) {
    auto appended_idx =
        node_input_indices_vector.at(node_idx).push(index_to_append);
    if (appended_idx >= 0) node_input_validity_vector[node_idx] = true;
    return appended_idx >= 0;
  }

  /// Access method
  // clang-format off
  const IndexQueue& get_input_indices_for_node(int node_idx) const { return node_input_indices_vector.at(node_idx); }
  bool is_input_indices_valid_for_node(int node_idx) const {return node_input_validity_vector.at(node_idx); }
  int num_nodes() const { return node_input_indices_vector.size(); }
  // clang-format on
};

/// A struct to hold all the buffers
template <int intermediate_solution_size, int max_n_solutions, int robot_nq>
struct RawIkWorkspaceTemplate {
  SolutionQueue<intermediate_solution_size, max_n_solutions> solution_queue;
  NodeIndexWorkspace<max_n_solutions> node_index_workspace;
  std::vector<std::array<double, robot_nq>> raw_ik_out;
};

/// The polynomial solver
struct PolynomialRealRoot {
  bool is_valid{false};
  double value{0.0};
};

template <int poly_order>
std::array<PolynomialRealRoot, poly_order> computePolynomialRealRoots(
    const std::array<double, poly_order + 1>& coefficient_high_to_low_order) {
  // Convert to eigen, note the reversed order
  Eigen::Matrix<double, poly_order + 1, 1> eigen_coefficient;
  for (auto i = 0; i < poly_order + 1; i++) {
    eigen_coefficient[i] = coefficient_high_to_low_order[poly_order - i];
  }

  // Call eigen solver
  Eigen::PolynomialSolver<double, poly_order> solver;
  solver.compute(eigen_coefficient);

  // The result
  using RootsType =
      typename Eigen::PolynomialSolver<double, poly_order>::RootsType;
  const RootsType& roots = solver.roots();
  std::array<PolynomialRealRoot, poly_order> poly_roots;
  for (auto i = 0; i < roots.size(); i++) {
    const std::complex<double>& root_i = roots[i];
    if (std::abs(root_i.imag()) < 1e-6) {
      const double real_root_i = root_i.real();
      PolynomialRealRoot root_i_return;
      root_i_return.is_valid = true;
      root_i_return.value = real_root_i;
      poly_roots[i] = root_i_return;
    } else {
      PolynomialRealRoot invalid_root;
      invalid_root.is_valid = false;
      poly_roots[i] = invalid_root;
    }
  }

  // Should be ok
  return poly_roots;
}

// For linear solver type2
namespace linear_solver {

template <typename T>
bool trySolveLinearType2(const Eigen::Ref<Eigen::Matrix<T, 3, 4>>& A,
                         T& solution_0, T& solution_1) {
  const T a = A(0, 0);
  const T b = A(0, 1);
  const T c = A(1, 0);
  const T d = A(1, 1);
  const T ad_minus_bc = a * d - b * c;
  constexpr T zero_tolerance = 1e-10;
  if (std::abs(ad_minus_bc) < zero_tolerance) return false;

  // Compute the inverse to top 2x2 matrix
  Eigen::Matrix<T, 2, 2> A_top2x2_inv;
  A_top2x2_inv(0, 0) = d / ad_minus_bc;
  A_top2x2_inv(0, 1) = -b / ad_minus_bc;
  A_top2x2_inv(1, 0) = -c / ad_minus_bc;
  A_top2x2_inv(1, 1) = a / ad_minus_bc;

  // [sin(x), cos(x)].T = B * [sin(y), cos(y)].T
  const Eigen::Matrix<T, 2, 2> B = -A_top2x2_inv * A.template block<2, 2>(0, 2);
  const T e = A(2, 2) + A(2, 0) * B(0, 0) + A(2, 1) * B(1, 0);
  const T f = A(2, 3) + A(2, 0) * B(0, 1) + A(2, 1) * B(1, 1);
  if (std::abs(e) < zero_tolerance && std::abs(f) < zero_tolerance)
    return false;

  // The solution should be OK
  constexpr T local_pi = 3.1415926;
  solution_0 = std::atan2(-f, e);
  solution_1 = solution_0 + local_pi;
  if (solution_1 > local_pi) solution_1 -= 2 * local_pi;
  return true;
}

template <int n_rows>
bool trySolveLinearType2SpecificRows(
    const Eigen::Ref<Eigen::Matrix<double, n_rows, 4>>& A, int row_0, int row_1,
    int row_2, double& solution_0, double& solution_1) {
  Eigen::Matrix<double, 3, 4> A_for_given_row;
  A_for_given_row.row(0) = A.row(row_0);
  A_for_given_row.row(1) = A.row(row_1);
  A_for_given_row.row(2) = A.row(row_2);
  return trySolveLinearType2<double>(A_for_given_row, solution_0, solution_1);
}

}  // namespace linear_solver

// For general-6dof solver
namespace general_6dof_internal {

template <int n_equations = 14, int n_lhs_unknowns = 9, int n_rhs_unknowns = 8>
bool numericalReduce(
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& A_sin,
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& A_cos,
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& C_const,
    const Eigen::Matrix<double, n_equations, n_rhs_unknowns>& rhs_matrix,
    const std::array<int, n_rhs_unknowns>& lines_to_reduce,
    const std::array<int, n_equations - n_rhs_unknowns>& remaining_lines,
    Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>*
        tau_sin,
    Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>*
        tau_cos,
    Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>*
        tau_const) {
  constexpr int remaining_rows = n_equations - n_rhs_unknowns;
  Eigen::Matrix<double, remaining_rows, n_rhs_unknowns> R_u;
  Eigen::Matrix<double, n_rhs_unknowns, n_rhs_unknowns> R_l;
  Eigen::Matrix<double, remaining_rows, n_lhs_unknowns> P_u_sin, P_u_cos,
      P_u_const;
  Eigen::Matrix<double, n_rhs_unknowns, n_lhs_unknowns> P_l_sin, P_l_cos,
      P_l_const;

  // Fill in the matrix for reduction
  int reduced_row_counter = 0;
  for (auto i = 0; i < lines_to_reduce.size(); i++) {
    auto row_idx = lines_to_reduce[i];
    R_l.row(reduced_row_counter) = rhs_matrix.row(row_idx);
    P_l_sin.row(reduced_row_counter) = A_sin.row(row_idx);
    P_l_cos.row(reduced_row_counter) = A_cos.row(row_idx);
    P_l_const.row(reduced_row_counter) = C_const.row(row_idx);
    reduced_row_counter += 1;
  }

  // Fill in the matrix for remaining rows
  int remaining_row_counter = 0;
  for (auto i = 0; i < remaining_lines.size(); i++) {
    auto row_idx = remaining_lines[i];
    R_u.row(remaining_row_counter) = rhs_matrix.row(row_idx);
    P_u_sin.row(remaining_row_counter) = A_sin.row(row_idx);
    P_u_cos.row(remaining_row_counter) = A_cos.row(row_idx);
    P_u_const.row(remaining_row_counter) = C_const.row(row_idx);
    remaining_row_counter += 1;
  }

  // Inverse and check nan
  const Eigen::Matrix<double, n_rhs_unknowns, n_rhs_unknowns> inv_R_l =
      R_l.inverse();
  for (auto r = 0; r < inv_R_l.rows(); r++) {
    for (auto c = 0; c < inv_R_l.cols(); c++) {
      if (std::isnan(inv_R_l(r, c)) || (!std::isfinite(inv_R_l(r, c)))) {
        return false;
      }
    }
  }

  // Compute the reduction
  const Eigen::Matrix<double, remaining_rows, n_rhs_unknowns> R_u_dot_R_l_inv =
      R_u * inv_R_l;
  *tau_sin = P_u_sin - R_u_dot_R_l_inv * P_l_sin;
  *tau_cos = P_u_cos - R_u_dot_R_l_inv * P_l_cos;
  *tau_const = P_u_const - R_u_dot_R_l_inv * P_l_const;
  return true;
}

template <int mat_rows = 14>
void sincosCoefficientToTanhalfCoefficient(
    const Eigen::Matrix<double, mat_rows, 9>& A_sincos,
    Eigen::Matrix<double, mat_rows, 9>* A_tanhalf) {
  A_tanhalf->col(0) =
      A_sincos.col(3) - A_sincos.col(5) - A_sincos.col(7) + A_sincos.col(8);
  A_tanhalf->col(1) = (-2.0) * (A_sincos.col(2) - A_sincos.col(6));
  A_tanhalf->col(2) = (-1.0 * A_sincos.col(3)) - A_sincos.col(5) +
                      A_sincos.col(7) + A_sincos.col(8);
  A_tanhalf->col(3) = (-2.0) * (A_sincos.col(1) - A_sincos.col(4));
  A_tanhalf->col(4) = 4 * A_sincos.col(0);
  A_tanhalf->col(5) = (2.0) * (A_sincos.col(1) + A_sincos.col(4));
  A_tanhalf->col(6) = (-1.0 * A_sincos.col(3)) + A_sincos.col(5) -
                      A_sincos.col(7) + A_sincos.col(8);
  A_tanhalf->col(7) = (2.0) * (A_sincos.col(2) + A_sincos.col(6));
  A_tanhalf->col(8) =
      A_sincos.col(3) + A_sincos.col(5) + A_sincos.col(7) + A_sincos.col(8);
}

template <typename T>
int computeSolutionFromTanhalfLME(const Eigen::Matrix<T, 6, 9>& A_x2,
                                  const Eigen::Matrix<T, 6, 9>& B_x,
                                  const Eigen::Matrix<T, 6, 9>& C,
                                  std::array<T, 16>* solution_buffer) {
  Eigen::Matrix<T, 12, 12> A_12;
  A_12.setZero();
  A_12.template block<6, 9>(0, 0) = A_x2;
  A_12.template block<6, 9>(6, 3) = A_x2;
  Eigen::Matrix<T, 12, 12> A_inv = A_12.inverse();
  for (auto r = 0; r < A_inv.rows(); r++) {
    for (auto c = 0; c < A_inv.cols(); c++) {
      if (std::isnan(A_inv(r, c)) || (!std::isfinite(A_inv(r, c)))) {
        return 0;
      }
    }
  }

  // Compute B and C
  Eigen::Matrix<T, 12, 12> B_12, C_12;
  B_12.setZero();
  C_12.setZero();
  B_12.template block<6, 9>(0, 0) = B_x;
  B_12.template block<6, 9>(6, 3) = B_x;
  C_12.template block<6, 9>(0, 0) = C;
  C_12.template block<6, 9>(6, 3) = C;

  Eigen::Matrix<T, 24, 24> M;
  M.setZero();
  M.template block<12, 12>(0, 12).setIdentity();
  M.template block<12, 12>(12, 0) = -(A_inv * C_12);
  M.template block<12, 12>(12, 12) = -(A_inv * B_12);

  // Compute the eigen-values of M
  Eigen::EigenSolver<Eigen::Matrix<T, 24, 24>> solver;
  solver.compute(M);
  const auto& eigenvalues = solver.eigenvalues();
  int solution_counter = 0;
  for (auto i = 0; i < eigenvalues.size(); i++) {
    const std::complex<T>& eigenvalue_i = eigenvalues[i];
    const double real_part = eigenvalue_i.real();
    const double imag_part = eigenvalue_i.imag();
    if (std::abs(imag_part) < 1e-6 &&
        solution_counter < solution_buffer->size()) {
      const double atan_value = std::atan(real_part);
      solution_buffer->at(solution_counter) = 2.0 * atan_value;
      solution_counter += 1;
    }
  }

  return solution_counter;
}

template <int mat_rows, int mat_cols>
void sincosLME2TanhalfLME(
    const Eigen::Matrix<double, mat_rows, mat_cols>& A_sin,
    const Eigen::Matrix<double, mat_rows, mat_cols>& A_cos,
    const Eigen::Matrix<double, mat_rows, mat_cols>& C_const,
    // Output
    Eigen::Matrix<double, mat_rows, mat_cols>* A,
    Eigen::Matrix<double, mat_rows, mat_cols>* B,
    Eigen::Matrix<double, mat_rows, mat_cols>* C) {
  *A = C_const - A_cos;
  *B = 2 * A_sin;
  *C = C_const + A_cos;
}

};  // namespace general_6dof_internal

template <int n_equations = 14, int n_lhs_unknowns = 9, int n_rhs_unknowns = 8>
int general6DofNumericalReduceSolve(
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& A_sin,
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& A_cos,
    const Eigen::Matrix<double, n_equations, n_lhs_unknowns>& C_const,
    const Eigen::Matrix<double, n_equations, n_rhs_unknowns>& rhs_matrix,
    const std::array<int, n_rhs_unknowns>& lines_to_reduce,
    const std::array<int, n_equations - n_rhs_unknowns>& remaining_lines,
    std::array<double, 16>* solution_buffer) {
  // First step, try reduce
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns> tau_sin;
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns> tau_cos;
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns> tau_const;
  bool reduced =
      general_6dof_internal::numericalReduce<n_equations, n_lhs_unknowns,
                                             n_rhs_unknowns>(
          A_sin, A_cos, C_const, rhs_matrix, lines_to_reduce, remaining_lines,
          &tau_sin, &tau_cos, &tau_const);

  // No solution if cannot reduce
  if (!reduced) return 0;

  // To tanhalf LME
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>
      tau_sin_tanhalf;
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>
      tau_cos_tanhalf;
  Eigen::Matrix<double, n_equations - n_rhs_unknowns, n_lhs_unknowns>
      tau_const_tanhalf;
  general_6dof_internal::sincosCoefficientToTanhalfCoefficient<n_equations -
                                                               n_rhs_unknowns>(
      tau_sin, &tau_sin_tanhalf);
  general_6dof_internal::sincosCoefficientToTanhalfCoefficient<n_equations -
                                                               n_rhs_unknowns>(
      tau_cos, &tau_cos_tanhalf);
  general_6dof_internal::sincosCoefficientToTanhalfCoefficient<n_equations -
                                                               n_rhs_unknowns>(
      tau_const, &tau_const_tanhalf);

  // Re-use the memory for A, B, C
  general_6dof_internal::sincosLME2TanhalfLME<n_equations - n_rhs_unknowns,
                                              n_lhs_unknowns>(
      tau_sin_tanhalf, tau_cos_tanhalf, tau_const_tanhalf, &tau_sin, &tau_cos,
      &tau_const);

  // Solve the equation
  int n_solution = general_6dof_internal::computeSolutionFromTanhalfLME(
      tau_sin, tau_cos, tau_const, solution_buffer);
  return n_solution;
}

template <typename T>
Eigen::Matrix<T, 4, 4> inverse_transform(
    const Eigen::Matrix<T, 4, 4>& transform) {
  Eigen::Matrix<T, 4, 4> inv;
  inv.setIdentity();
  auto rotation = transform.template block<3, 3>(0, 0).transpose();
  inv.template block<3, 3>(0, 0) = rotation;
  inv.template block<3, 1>(0, 3) =
      -rotation * transform.template block<3, 1>(0, 3);
  return inv;
}

template <typename T>
Eigen::Matrix<T, 4, 4> disturbTransform(const Eigen::Matrix<T, 4, 4>& transform,
                                        T disturb_angle = T(1e-2),
                                        T disturb_translation = T(1e-2)) {
  // Make angle axis
  Eigen::Matrix<T, 3, 1> axis;
  axis.setRandom();
  if (axis.squaredNorm() < 1e-4) {
    axis[0] += 0.1;
  }
  axis.normalize();
  Eigen::AngleAxis<T> angle_axis(disturb_angle, axis);

  // To matrix4 disturb
  Eigen::Matrix<T, 4, 4> disturb_matrix;
  disturb_matrix.setIdentity();
  disturb_matrix.template block<3, 3>(0, 0) = angle_axis.matrix();
  disturb_matrix(0, 3) -= disturb_translation;
  disturb_matrix(1, 3) -= disturb_translation;
  disturb_matrix(2, 3) -= disturb_translation;
  return disturb_matrix * transform;
}

template <int robot_nq = 6>
void numericalRefinement(
    const std::function<Eigen::Matrix4d(const std::array<double, robot_nq>&)>&
        fk_functor,
    const std::function<void(const std::array<double, robot_nq>&,
                             Eigen::Matrix<double, 6, robot_nq>&)>&
        twist_jacobian_functor,
    const Eigen::Matrix4d& ee_target, std::array<double, robot_nq>& q,
    double diagonal_damping = 1e-4) {
  Eigen::Matrix<double, 6, robot_nq> jacobian;
  Eigen::Matrix<double, robot_nq, robot_nq> jtj;
  Eigen::Matrix<double, robot_nq, 1> delta_q, jte;
  Eigen::Matrix<double, 6, 1> twist_target;

  constexpr int n_iterations = 10;
  for (auto i = 0; i < n_iterations; i++) {
    // Compute the twist target
    Eigen::Matrix4d ee_pose_i = fk_functor(q);
    Eigen::Matrix4d twist_target_4x4 = ee_target * inverse_transform(ee_pose_i);
    twist_target[0] = 0.5 * (twist_target_4x4(2, 1) - twist_target_4x4(1, 2));
    twist_target[1] = 0.5 * (twist_target_4x4(0, 2) - twist_target_4x4(2, 0));
    twist_target[2] = 0.5 * (twist_target_4x4(1, 0) - twist_target_4x4(0, 1));
    twist_target[3] = twist_target_4x4(0, 3);
    twist_target[4] = twist_target_4x4(1, 3);
    twist_target[5] = twist_target_4x4(2, 3);

    // Compute jacobian and solve it
    twist_jacobian_functor(q, jacobian);
    jtj = jacobian.transpose() * jacobian;
    jte = jacobian.transpose() * twist_target;
    for (auto j = 0; j < robot_nq; j++) {
      jtj(j, j) += diagonal_damping;
    }

    // Solve Ax = b and update
    delta_q = jtj.colPivHouseholderQr().solve(jte);
    for (auto j = 0; j < robot_nq; j++) {
      q[j] += delta_q[j];
    }
  }
}

};  // namespace yaik_cpp