#include "lab7/hand_eye_calib.hpp"

void findMeanErrorVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& mean_error_vec_out)
{
  if (error_vecs_in.empty())
    return;

  /// TODO: Find the mean error vector by averaging the error vectors.

  mean_error_vec_out = Eigen::Vector<double, 7>::Zero(); // Update this with calculated value
}

void findCovarianceMatrix(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Matrix<double, 7, 7>& covariance_matrix_out)
{
  if (error_vecs_in.empty())
    return;

  /// TODO: Find the covariance matrix using the error vectors.

  covariance_matrix_out = Eigen::Matrix<double, 7, 7>::Zero(); // Update this with calculated value
}

void findLeastSquaresError(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& least_squares_vec_out)
{
  if (error_vecs_in.empty())
    return;

  /**
   * TODO: Apply the least squares method to each component of the error vector separately, and
   *       then combine the results to form the least squares error vector.
   */

  least_squares_vec_out = Eigen::Vector<double, 7>::Zero(); // Update this with calculated value
}

void HandEyeCalibNode::verifyCalibration()
{
  /**
   * Measurement variables available for use:
   *
   * - Reference positions of the end-effector are stored in `actual_eef_positions_`
   * - Reference orientations of the end-effector are stored in `actual_eef_orientations_`
   *
   * - Estimated positions of the end-effector are stored in `estimated_eef_positions_`
   * - Estimated orientations of the end-effector are stored in `estimated_eef_orientations_`
   */

  std::vector<Eigen::Vector<double, 7>> error_vecs {};
  /**
   * TODO: Populate this variable with error vectors from the measurements which consists of:
   *       - 3 components of position error vector
   *       - 4 components of orientation error quaternion
   *           (Check if the magnitude of this quaternion is close to 1)
   */

  findMeanErrorVector(error_vecs, mean_error_vector_);
  findCovarianceMatrix(error_vecs, covariance_matrix_);
  findLeastSquaresError(error_vecs, least_squares_vector_);

  is_verification_complete_ = true;
}
