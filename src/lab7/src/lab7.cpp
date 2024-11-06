#include "lab7/hand_eye_calib.hpp"

void findMeanErrorVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& mean_error_vec_out)
{
  if (error_vecs_in.empty())
    return;

  mean_error_vec_out = Eigen::Vector<double, 7>::Zero();

  /// TODO: Find the mean error vector by averaging the error vectors.
  double INV_SIZE = 1.0 / 7.0;

  for (const auto& vec : error_vecs_in)
    mean_error_vec_out += vec;

  mean_error_vec_out *= INV_SIZE;
}

void findCovarianceMatrix(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  const Eigen::Vector<double, 7>& mean_error_vec_in,
  Eigen::Matrix<double, 7, 7>& covariance_matrix_out)
{
  if (error_vecs_in.empty() || !mean_error_vec_in.allFinite())
    return;

  covariance_matrix_out = Eigen::Matrix<double, 7, 7>::Zero();

  /// TODO: Find the covariance matrix using the error vectors.
  double INV_POPULATION = 1.0 / 6.0;

  for (const auto& vec : error_vecs_in) {
    Eigen::Vector<double, 7> centered = vec - mean_error_vec_in;
    covariance_matrix_out += centered * centered.transpose();
  }

  covariance_matrix_out *= INV_POPULATION;
}

void findSumOfSquaredErrorsVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& least_squares_vec_out)
{
  if (error_vecs_in.empty())
    return;

  least_squares_vec_out = Eigen::Vector<double, 7>::Zero();

  /// TODO: Find the sum of the squared errors using the error vectors.

  for (const auto& vec : error_vecs_in)
    least_squares_vec_out += vec.cwiseProduct(vec);

}

void HandEyeCalibNode::verifyCalibration()
{
  /**
   * Measurement variables:
   *
   * - Reference positions of the end-effector are stored in `actual_eef_positions_`
   * - Reference orientations of the end-effector are stored in `actual_eef_orientations_`
   *
   * - Estimated positions of the end-effector std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};are stored in `estimated_eef_positions_`
   * - Estimated orientations of the end-effector are stored in `estimated_eef_orientations_`
   */

  std::vector<Eigen::Vector<double, 7>> error_vecs {};

  /**
   * Error vectors from the measurements consists of:
   *   - 3 components of position error vector
   *   - 4 components of orientation error quaternion
   *     (Check if the magnitude of this quaternion is close to 1)
   */
  for (unsigned int i {0}; i < estimated_eef_orientations_.size(); i++) {
    auto position_error {actual_eef_positions_.at(i) - estimated_eef_positions_.at(i)};

    auto orientation_error {
      actual_eef_orientations_.at(i).conjugate() * estimated_eef_orientations_.at(i)};

    Eigen::Vector<double, 7> error_vector;
    error_vector << position_error, orientation_error.coeffs();

    error_vecs.push_back(error_vector);
  }

  findMeanErrorVector(error_vecs, mean_error_vector_);
  findCovarianceMatrix(error_vecs, mean_error_vector_, covariance_matrix_);
  findSumOfSquaredErrorsVector(error_vecs, sum_of_squared_errors_vector_);
  root_sum_of_squared_errors_vector_ = sum_of_squared_errors_vector_.array().sqrt();

  RCLCPP_INFO(this->get_logger(), "Verification results can be saved now.");

  is_verification_complete_ = true;
}
