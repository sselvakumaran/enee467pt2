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
  is_verification_complete_ = true;
}
