#include "opencv2/aruco/dictionary.hpp"
#include <auv_mocap/ArucoPoseArray.h>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <map>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <tf/tf.h>

void setQuaternionFromRvec(cv::Vec<double, 3> &rvec,
                           geometry_msgs::Quaternion &q) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  tf::Matrix3x3 tfR(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
  tf::Quaternion tfQ;
  tfR.getRotation(tfQ);
  q.x = tfQ.x();
  q.y = tfQ.y();
  q.z = tfQ.z();
  q.w = tfQ.w();
}

class Camera {
public:
  Camera(ros::NodeHandle &nh, const std::string &topic,
         const std::string &calib_file,
         std::function<void(const sensor_msgs::ImageConstPtr &)> callback)
      : it_(nh) {
    sub_ = it_.subscribe(topic, 1, callback);
    loadCalibration(calib_file);
  }

  const cv::Mat &getCameraMatrix() const { return cameraMatrix_; }
  const cv::Mat &getDistCoeffs() const { return distCoeffs_; }

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_;
  cv::Mat cameraMatrix_, distCoeffs_;

  void loadCalibration(const std::string &calib_file) {
    std::ifstream file(calib_file);
    if (!file.is_open()) {
      ROS_ERROR("Failed to open calibration file: %s", calib_file.c_str());
      return;
    }

    std::string line;
    std::vector<double> cam_matrix_values, dist_coeffs_values;
    bool parsing_camera_matrix = false;
    bool parsing_distortion = false;

    while (std::getline(file, line)) {
      if (line.find("camera matrix") != std::string::npos) {
        parsing_camera_matrix = true;
        continue;
      }
      if (line.find("distortion") != std::string::npos) {
        parsing_camera_matrix = false;
        parsing_distortion = true;
        continue;
      }
      if (parsing_camera_matrix || parsing_distortion) {
        std::istringstream iss(line);
        double val;
        while (iss >> val) {
          if (parsing_camera_matrix) {
            cam_matrix_values.push_back(val);
          } else if (dist_coeffs_values.size() < 5) {
            dist_coeffs_values.push_back(val);
          }
        }
      }
    }

    if (cam_matrix_values.size() == 9 && dist_coeffs_values.size() == 5) {
      cameraMatrix_ = cv::Mat(3, 3, CV_64F, cam_matrix_values.data()).clone();
      distCoeffs_ = cv::Mat(1, dist_coeffs_values.size(), CV_64F,
                            dist_coeffs_values.data())
                        .clone();
      ROS_INFO_STREAM("Loaded Camera Matrix:\n"
                      << cameraMatrix_
                      << "Loaded "
                         "Distortion Coefficients:\n"
                      << distCoeffs_);
    } else {
      ROS_ERROR_STREAM("Invalid calibration file format. FAILED TRYING TO LOAD "
                       << calib_file);
    }
  }
};

class ArucoDetector {
public:
  ArucoDetector(ros::NodeHandle &nh, int num_cameras)
      : nh_(nh), it_(nh), objPoints(4, 1, CV_32FC3) {

    std::string package_path = ros::package::getPath("auv_mocap");

    for (int i = 0; i < num_cameras; ++i) {
      int camera_id = i + 1;
      cameras.push_back(Camera(nh_,
                               cv::format("/camera_%d/image_raw", camera_id),
                               cv::format("%s/calibration_files/camera_%d.txt",
                                          package_path.c_str(), camera_id),
                               std::bind(&ArucoDetector::imageCallback, this,
                                         std::placeholders::_1, camera_id)));
    }
    // cam2_ =
    //     std::make_unique<Camera>(nh_, "/four/image_raw", "./david.txt",
    //                              std::bind(&ArucoDetector::imageCallback,
    //                              this,
    //                                        std::placeholders::_1, "Camera
    //                                        2"));

    objPoints.ptr<cv::Vec3f>(0)[0] =
        cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] =
        cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] =
        cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] =
        cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
  }

  void spin() { ros::spin(); }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::map<std::string, ros::Publisher> camera_publishers_;
  cv::Mat objPoints;
  std::vector<Camera> cameras;
  // Length of the marker side in meters
  const float markerLength = 0.15f;
  void imageCallback(const sensor_msgs::ImageConstPtr &msg,
                     const int camera_id) {
    const std::string cameraName = cv::format("Camera %d", camera_id);
    cv::Mat imageCopy;
    try {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners;
      cv::Ptr<cv::aruco::Dictionary> dictionary =
          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
      image.copyTo(imageCopy);

      if (!markerIds.empty()) {
        const cv::Mat &cameraMatrix = cameras[camera_id - 1].getCameraMatrix();
        const cv::Mat &distCoeffs = cameras[camera_id - 1].getDistCoeffs();

        std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
        for (size_t i = 0; i < markerIds.size(); ++i) {
          cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs,
                       rvecs.at(i), tvecs.at(i));
        }

        // Create a single message per camera
        auv_mocap::ArucoPoseArray pose_array_msg;
        pose_array_msg.header.stamp = ros::Time::now();
        pose_array_msg.header.frame_id = cameraName;

        bool foundGlobalMarker = false;
        for (size_t i = 0; i < markerIds.size(); ++i) {
          
          if (markerIds[i] == 0) {
            foundGlobalMarker = true;
          }

          cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i],
                            tvecs[i], markerLength * 1.5f, 2);

          auv_mocap::ArucoPose pose_msg;
          pose_msg.header.stamp = ros::Time::now();
          pose_msg.header.frame_id = cameraName;
          pose_msg.header.seq = i;
          pose_msg.id = markerIds[i];
          pose_msg.pose.position.x = tvecs[i][0];
          pose_msg.pose.position.y = tvecs[i][1];
          pose_msg.pose.position.z = tvecs[i][2];

          geometry_msgs::Quaternion q;
          setQuaternionFromRvec(rvecs[i], q);
          pose_msg.pose.orientation = q;

          pose_array_msg.poses.push_back(pose_msg);
        }

        if (!foundGlobalMarker) {
          ROS_WARN("CAMERA %d: Global marker not detected", camera_id);
        }

        // Publish the ArucoPoseArray message
        if (camera_publishers_.find(cameraName) == camera_publishers_.end()) {
          camera_publishers_[cameraName] =
              nh_.advertise<auv_mocap::ArucoPoseArray>(
                  cv::format("/camera_%d/aruco_pose", camera_id), 10);
        }
        camera_publishers_[cameraName].publish(pose_array_msg);
      }

      if (!markerIds.empty()) {
        cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
      }

      cv::imshow(cameraName, imageCopy);
      cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;

  int num_cameras;
  nh.param("num_cameras", num_cameras, 1);

  ArucoDetector detector(nh, num_cameras);
  detector.spin();
  return 0;
}
