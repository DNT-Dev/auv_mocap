#include "geometry_msgs/TransformStamped.h"
#include "mocap/utils.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "ros/publisher.h"
#include "ros/spinner.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <string>
#include <unordered_map>

class Camera {
public:
  Camera(ros::NodeHandle &nh, const std::string &topic,
         const std::string &calib_file,
         std::function<void(const sensor_msgs::ImageConstPtr &)> callback)
      : it_(nh) {
    sub_ = it_.subscribe(topic, 0, callback);
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
  int _num_markers;
  std::vector<ros::Publisher> _marker_publishers;

public:
  ArucoDetector(ros::NodeHandle &nh, int num_cameras, int num_markers)
      : nh_(nh), it_(nh), objPoints(4, 1, CV_32FC3), _num_markers(num_markers) {

    std::string package_path = ros::package::getPath("auv_mocap");

    for (int i = 0; i < num_cameras; ++i) {
      int camera_id = i + 1;
      cameras.push_back(Camera(nh_,
                               cv::format("/camera_%d/image_raw", camera_id),
                               cv::format("%s/calibration_files/camera_%d.txt",
                                          package_path.c_str(), camera_id),
                               std::bind(&ArucoDetector::imageCallback, this,
                                         std::placeholders::_1, camera_id)));
      _marker_publishers.push_back(nh_.advertise<sensor_msgs::Image>(
          cv::format("/camera_%d/detected_markers", camera_id), 10));
    }

    objPoints.ptr<cv::Vec3f>(0)[0] =
        cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] =
        cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] =
        cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] =
        cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
  }

  void spin() {
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  cv::Mat objPoints;
  std::vector<Camera> cameras;
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  std::vector<cv::Vec3d> rvecs = std::vector<cv::Vec3d>(10, cv::Vec3d(0, 0, 0));
  std::vector<cv::Vec3d> tvecs = std::vector<cv::Vec3d>(10, cv::Vec3d(0, 0, 0));
  geometry_msgs::TransformStamped transform;

  // Length of the marker side in meters
  const float markerLength = 0.15f;
  void imageCallback(const sensor_msgs::ImageConstPtr &msg,
                     const int camera_id) {
    static tf2_ros::TransformBroadcaster tf_br;
    const std::string cameraName = cv::format("camera_%d", camera_id);
    cv::Mat imageCopy;
    try {
      // detect aruco markers
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
      image.copyTo(imageCopy);

      if (!markerIds.empty()) {
        const cv::Mat &cameraMatrix = cameras[camera_id - 1].getCameraMatrix();
        const cv::Mat &distCoeffs = cameras[camera_id - 1].getDistCoeffs();

        // convert maker points (2d) to 3d
        for (size_t i = 0; i < markerIds.size(); ++i) {
          cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs,
                       rvecs.at(i), tvecs.at(i));
        }

        bool foundGlobalMarker = false;
        for (size_t i = 0; i < markerIds.size(); ++i) {

          if (markerIds[i] > 1) {
            continue;
          }

          if (markerIds[i] == 0) {
            foundGlobalMarker = true;
          }

          transform.header.stamp = ros::Time::now();
          // Swap frame_id and child_frame_id for inverted transform
          transform.header.frame_id =
              cv::format("aruco_marker_%d", markerIds[i]);
          transform.child_frame_id = cameraName;

          // Original transform: camera -> marker
          tf2::Vector3 t_orig(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
          tf2::Quaternion q_orig;
          quaternionFromRvecs(rvecs[i], q_orig);
          q_orig.normalize();

          tf2::Transform tf_orig(q_orig, t_orig);

          // Invert the transform: marker -> camera
          tf2::Transform tf_inv = tf_orig.inverse();

          tf2::Vector3 t_inv = tf_inv.getOrigin();
          tf2::Quaternion q_inv = tf_inv.getRotation();

          transform.transform.translation.x = t_inv.x();
          transform.transform.translation.y = t_inv.y();
          transform.transform.translation.z = t_inv.z();

          ROS_INFO_STREAM("CAMERA " << camera_id << ": Detected marker "
                                    << markerIds[i] << " with translation: "
                                    << transform.transform.translation.x << ", "
                                    << transform.transform.translation.y << ", "
                                    << transform.transform.translation.z
                                    << " and rotation: " << q_inv.x() << ", "
                                    << q_inv.y() << ", " << q_inv.z() << ", "
                                    << q_inv.w());

          transform.transform.rotation.x = q_inv.x();
          transform.transform.rotation.y = q_inv.y();
          transform.transform.rotation.z = q_inv.z();
          transform.transform.rotation.w = q_inv.w();

          tf_br.sendTransform(transform);
          // ROS_INFO("CAMERA %d: Sent transform to marker %d", camera_id,
          //  markerIds[i]);

          cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i],
                            tvecs[i], markerLength * 1.5f, 2);
        }

        if (!foundGlobalMarker) {
          ROS_WARN("CAMERA %d: Global marker not detected", camera_id);
        }
      }

      if (!markerIds.empty()) {
        try {
          cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
        } catch (const std::exception &e) {
          ROS_ERROR("Failed to draw detected markers: %s", e.what());
        }
      }

      _marker_publishers[camera_id - 1].publish(
          cv_bridge::CvImage(msg->header, "bgr8", imageCopy).toImageMsg());

      // cv::imshow(cameraName, imageCopy);
      // cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh("~");

  int num_cameras;
  nh.param("num_cameras", num_cameras, 1);

  int num_markers;
  nh.param("num_markers", num_markers, 1);

  ROS_INFO("Initializing %d cameras", num_cameras);

  ArucoDetector detector(nh, num_cameras, num_markers);
  detector.spin();
  return 0;
}
