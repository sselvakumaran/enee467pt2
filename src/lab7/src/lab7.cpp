#include "lab7/hand_eye_calib.hpp"

void findMeanErrorVector(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& mean_error_vec_out)
{

}

void findCovarianceMatrix(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Matrix<double, 7, 7>& covariance_matrix_out)
{

}

void findLeastSquaresError(
  const std::vector<Eigen::Vector<double, 7>>& error_vecs_in,
  Eigen::Vector<double, 7>& least_squares_vec_out)
{

}

void HandEyeCalibNode::verifyCalibration()
{
/**
 * TODO: Implement this function as a part of this lab exercise. Feel free to define any additional
 *       functions within this file you may need.
 *
 * Measurement variables available for use:
 *
 * - Reference positions of the end-effector are stored in `actual_eef_positions_`
 * - Reference orientations of the end-effector are stored in `actual_eef_orientations_`
 *
 * - Estimated positions of the end-effector are stored in `estimated_eef_positions_`
 * - Estimated orientations of the end-effector are stored in `estimated_eef_orientations_`
 *
 *
 * TODO: Find the error vectors from the measurements which consists of:
 *        - 3 components of position error vector
 *        - 4 components of orientation error quaternion (Check if the magnitude of this quaternion is close to 1)
 *
 * TODO: Find the average error vector by averaging the error vectors.
 *
 * TODO: Find the covariance matrix using the error vectors.
 *
 * TODO: Apply the least squares method to each component of the error vector separately, and then
 *       combine the results to form the least squares error vector.
 */


}
