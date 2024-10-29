#include "lab7/hand_eye_calib.hpp"

void findMeanErrorVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& mean_error_vec_out)
{
  if (error_vecs_in.empty())
    return;

  mean_error_vec_out = Eigen::Vector<double, 7>::Zero();

  /// TODO: Find the mean error vector by averaging the error vectors.

}

void findCovarianceMatrix(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  const Eigen::Vector<double, 7>& mean_error_vec_in,
  Eigen::Matrix<double, 7, 7>& covariance_matrix_out)
{
  if (error_vecs_in.empty())
    return;

  covariance_matrix_out = Eigen::Matrix<double, 7, 7>::Zero();

  /// TODO: Find the covariance matrix using the error vectors.

}

void findLeastSquaresErrorVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& least_squares_vec_out)
{
  if (error_vecs_in.empty())
    return;

  least_squares_vec_out = Eigen::Vector<double, 7>::Zero();

  /**
   * TODO: Apply the least squares method to each component of the error vector separately to form
   *       the least squares error vector.
   */

}

void HandEyeCalibNode::verifyCalibration()
{
  /**
   * Measurement variables:
   *
   * - Reference positions of the end-effector are stored in `actual_eef_positions_`
   * - Reference orientations of the end-effector are stored in `actual_eef_orientations_`
   *
   * - Estimated positions of the end-effector are stored in `estimated_eef_positions_`
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
    Eigen::Vector<double, 7> estimated_pose_vec {};
    estimated_pose_vec << estimated_eef_positions_.at(i), estimated_eef_orientations_.at(i).coeffs();

    Eigen::Vector<double, 7> actual_pose_vec {};
    actual_pose_vec << actual_eef_positions_.at(i), actual_eef_orientations_.at(i).coeffs();

    error_vecs.push_back(estimated_pose_vec - actual_pose_vec);
  }

  findMeanErrorVector(error_vecs, mean_error_vector_);
  findCovarianceMatrix(error_vecs, mean_error_vector_, covariance_matrix_);
  findLeastSquaresErrorVector(error_vecs, least_squares_vector_);

  RCLCPP_INFO(this->get_logger(), "Verification results can be saved now.");

  is_verification_complete_ = true;
}
