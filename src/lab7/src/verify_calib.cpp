#include "lab7/hand_eye_calib.hpp"

void HandEyeCalibNode::verifyCalibration()
{
/**
 * TODO: Implement this function as a part of this lab exercise. Feel free to define any additional
 *       functions you may need.
 *
 * Variables available for use:
 *
 * - Reference positions of the end-effector are stored in `actual_eef_positions_`
 * - Reference orientations of the end-effector are stored in `actual_eef_orientations_`
 *
 * - Estimated positions of the end-effector are stored in `estimated_eef_positions_`
 * - Estimated orientations of the end-effector are stored in `estimated_eef_orientations_`
 *
 * Find the error vectors for positions and the error quaternions for orientations.
 *
 * Then find the average error for positions and orientations by finding the average of vectors
 * and quaternions. This is one of your results.
 *
 * Then find the covariance matrix using the calculated average errors for positions and orientations.
 * You need to convert error quaternions to rotation vectors for finding the mean.
 */
}
