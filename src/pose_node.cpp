#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf2_ros/transform_listener.h"
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Transform.h>

void waitTillReady(tf2_ros::Buffer &tf_buffer, int num_cameras) {
  while (ros::ok()) {
    int num_ready = 0;
    for (int i = 0; i < num_cameras; i++) {
      if (tf_buffer.canTransform("camera_" + std::to_string(i), "aruco_maker_0",
                                 ros::Time(0))) {
        num_ready++;
      } else {
        ROS_WARN("Waiting for camera %d to be ready...", i);
      }
    }

    if (num_ready == num_cameras)
      break;
    else {
      ROS_WARN("Waiting for all cameras to be ready...");
      ros::Duration(1.0).sleep();
    }
  }
}

class StateEstimator {
  tf2::Transform m0_to_m1;
  ros::Publisher m0_to_m1_pub;

public:
  StateEstimator(ros::NodeHandle &nh) {
    m0_to_m1.setIdentity();
    m0_to_m1_pub =
        nh.advertise<geometry_msgs::TransformStamped>("m0_to_m1", 10);
  }
  void addTransformToState(std::vector<tf2::Transform> &transforms) {
    // THIS PART IS SUPER SKETCHY???
    if (transforms.empty()) {
      throw std::runtime_error("Cannot average empty transform list");
    }

    // Average translation
    tf2::Vector3 avg_translation(0, 0, 0);
    for (const auto &tf : transforms) {
      avg_translation += tf.getOrigin();
    }
    avg_translation /= transforms.size();

    // Average rotation
    // We'll average the quaternion components and then normalize
    double avg_x = 0, avg_y = 0, avg_z = 0, avg_w = 0;
    for (const auto &tf : transforms) {
      tf2::Quaternion q = tf.getRotation();
      // Ensure consistent quaternion orientation
      if (q.getW() < 0) {
        q = tf2::Quaternion(-q.getX(), -q.getY(), -q.getZ(), -q.getW());
      }
      avg_x += q.getX();
      avg_y += q.getY();
      avg_z += q.getZ();
      avg_w += q.getW();
    }
    avg_x /= transforms.size();
    avg_y /= transforms.size();
    avg_z /= transforms.size();
    avg_w /= transforms.size();

    tf2::Quaternion avg_quat(avg_x, avg_y, avg_z, avg_w);
    avg_quat.normalize();

    m0_to_m1 = tf2::Transform(avg_quat, avg_translation);
    auto transform_msg =
        tf2::toMsg<typeof(m0_to_m1), geometry_msgs::TransformStamped>(m0_to_m1);
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "aruco_maker_0";
    transform_msg.child_frame_id = "aruco_marker_1";
    m0_to_m1_pub.publish(transform_msg);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;

  StateEstimator state_estimator(nh);

  int num_cameras;
  nh.param("num_cameras", num_cameras, 1);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  waitTillReady(tf_buffer, num_cameras);
  ROS_INFO("All cameras are ready!");

  // now all cameras can see global frame
  while (ros::ok()) {
    std::vector<tf2::Transform> transforms;
    for (int i = 0; i < num_cameras; i++) {
      auto camera_id = "camera_" + std::to_string(i);
      tf2::Transform m0_to_ci, ci_to_m1;
      tf2::fromMsg(
          tf_buffer.lookupTransform("aruco_maker_0", camera_id, ros::Time(0)),
          m0_to_ci);
      tf2::fromMsg(
          tf_buffer.lookupTransform(camera_id, "aruco_marker_1", ros::Time(0)),
          ci_to_m1);
      tf2::Transform m0_to_m1 = m0_to_ci * ci_to_m1;
      transforms.push_back(m0_to_m1);
    }
    state_estimator.addTransformToState(transforms);
  }

  return 0;
}