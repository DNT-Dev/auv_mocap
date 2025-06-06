#include "opencv2/core/matx.hpp"
#include "tf2/LinearMath/Quaternion.h"

void quaternionFromRvecs(const cv::Vec3d &rvec, tf2::Quaternion &q);