#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <boost/range/distance.hpp>
#include <numeric>
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
protected:
  tf2::Transform m0_to_m1;
  ros::Publisher m0_to_m1_pub;
  std::vector<double> t_x;
  std::vector<double> t_y;
  std::vector<double> t_z;
  std::vector<double> r_r;
  std::vector<double> r_p;
  std::vector<double> r_y;
  std::vector<double> t_dist;

public:
  StateEstimator(ros::NodeHandle &nh) {
    m0_to_m1.setIdentity();
    m0_to_m1_pub =
        nh.advertise<geometry_msgs::TransformStamped>("m0_to_m1", 10);
  }

  virtual void Computation() {}

  public:
  void Process(std::vector<tf2::Transform> &transforms) {
    // Clear old lists for new batch
    t_x.clear();
    t_y.clear();
    t_z.clear();
    r_r.clear();
    r_p.clear();
    r_y.clear();

    for (const auto &tf : transforms) {
      double r, p, y;
      auto origin = tf.getOrigin();
      auto rotation = tf2::Matrix3x3(tf.getRotation());

      rotation.getRPY(r, p, y);

      t_x.push_back(origin.x());
      t_y.push_back(origin.y());
      t_z.push_back(origin.z());
      t_dist.push_back(std::sqrt(origin.x() * origin.x() +
                                 origin.y() * origin.y() +
                                 origin.z() * origin.z()));
      r_r.push_back(r);
      r_p.push_back(p);
      r_y.push_back(y);
    }

    Computation();
  }
};

class AverageEstimator : public StateEstimator {
public:
  AverageEstimator(ros::NodeHandle &nh) : StateEstimator(nh) {}
  void Computation() override {
    double avg_x = std::accumulate(t_x.begin(), t_x.end(), 0.0) / t_x.size();
    double avg_y = std::accumulate(t_y.begin(), t_y.end(), 0.0) / t_y.size();
    double avg_z = std::accumulate(t_z.begin(), t_z.end(), 0.0) / t_z.size();
    double avg_r = std::accumulate(r_r.begin(), r_r.end(), 0.0) / r_r.size();
    double avg_p = std::accumulate(r_p.begin(), r_p.end(), 0.0) / r_p.size();
    double avg_yaw = std::accumulate(r_y.begin(), r_y.end(), 0.0) / r_y.size();

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "aruco_marker_0";
    msg.child_frame_id = "aruco_marker_1";
    msg.transform.translation.x = avg_x;
    msg.transform.translation.y = avg_y;
    msg.transform.translation.z = avg_z;

    tf2::Quaternion q;
    q.setRPY(avg_r, avg_p, avg_yaw);
    msg.transform.rotation =
        tf2::toMsg<typeof q, typeof msg.transform.rotation>(q);

    m0_to_m1_pub.publish(msg);
  }
};

class GaussianEstimator : public StateEstimator {

  std::pair<double, double> GaussianMul(std::vector<double> &mu_list,
                                        std::vector<double> &std_list,
                                        double std_mul = 1.0) {
    std::vector<double> precisions(std_list.size());
    std::transform(std_list.begin(), std_list.end(), precisions.begin(),
                   [=](double std) {
                     std *= std_mul;
                     return 1.0 / (std * std);
                   });
    double precision_sum =
        std::accumulate(precisions.begin(), precisions.end(), 0.0);

    double mu_w_sum = std::inner_product(mu_list.begin(), mu_list.end(),
                                         precisions.begin(), 0.0);
    double mu_w = mu_w_sum / precision_sum;
    double std_w = std::sqrt(1.0 / precision_sum);

    return {mu_w, std_w};
  }

public:
  GaussianEstimator(ros::NodeHandle &nh) : StateEstimator(nh) {}
  void Computation() override {
    auto [avg_x, std_x] = GaussianMul(t_x, t_dist, 0.1228);
    auto [avg_y, std_y] = GaussianMul(t_y, t_dist, 0.1204);
    auto [avg_z, std_z] = GaussianMul(t_z, t_dist, 0.1091);
    auto [avg_r, std_r] = GaussianMul(r_r, t_dist, 1.1584);
    auto [avg_p, std_p] = GaussianMul(r_p, t_dist, 1.1584);
    auto [avg_yaw, std_yaw] = GaussianMul(r_y, t_dist, 1.1584);

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "aruco_marker_0";
    msg.child_frame_id = "aruco_marker_1";
    msg.transform.translation.x = avg_x;
    msg.transform.translation.y = avg_y;
    msg.transform.translation.z = avg_z;

    tf2::Quaternion q;
    q.setRPY(avg_r, avg_p, avg_yaw);
    msg.transform.rotation =
        tf2::toMsg<typeof q, typeof msg.transform.rotation>(q);

    m0_to_m1_pub.publish(msg);
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "aruco_detector");
  ros::NodeHandle nh;

  AverageEstimator state_estimator(nh);

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
    state_estimator.Process(transforms);
  }

  return 0;
}
