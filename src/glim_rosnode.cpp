#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Two executor threads: the sensor callbacks and the frame-handoff timer stay in the node's default
  // (mutually exclusive) callback group, and the save_map service runs in its own group (see GlimROS::snapshot).
  // With the single-threaded executor a live map snapshot (tens of seconds) stalled every IMU / points
  // callback; when they resumed the queued IMU and points were ~1 s apart and the odometry aborted with
  // "IndexedSlidingWindow: index out of range" (aspa3 2026-10-23 05:21 and 11:58).
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  exec.add_node(glim);
  exec.spin();
  rclcpp::shutdown();

  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  glim->wait();
  glim->save(dump_path);

  return 0;
}