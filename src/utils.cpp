#include "opencv2/calib3d.hpp"
#include "opencv2/core/matx.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// In mocap/utils.hpp or directly in your .cpp file
void quaternionFromRvecs(const cv::Vec3d &rvec, tf2::Quaternion &q) { // Changed Vec3f to Vec3d
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix); // Now rvec is Vec3d, so rotation_matrix will be CV_64F by default

    // The .at<double> calls are now correct because rotation_matrix is CV_64F
    tf2::Matrix3x3 tf_rotation(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
    );

    tf_rotation.getRotation(q);
    q = q.normalized(); // Keep this line as a safeguard
}