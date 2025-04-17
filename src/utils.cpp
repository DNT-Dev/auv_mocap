#include "opencv2/calib3d.hpp"
#include "opencv2/core/matx.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

void quaternionFromRvecs(const cv::Vec3f &rvec, tf2::Quaternion &q) {

    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
    );

    tf_rotation.getRotation(q);
}