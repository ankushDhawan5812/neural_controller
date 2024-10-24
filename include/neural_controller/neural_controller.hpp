#pragma once

#include <RTNeural/RTNeural.h>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

// auto-generated by generate_parameter_library
#include "neural_controller_parameters.hpp"

namespace neural_controller {

template <typename T>
bool contains_nan(const T &container) {
  return std::any_of(container.begin(), container.end(),
                     [](double val) { return std::isnan(val); });
}

using CmdType = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;

class NeuralController : public controller_interface::ControllerInterface {
 public:
  NeuralController();

  ~NeuralController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

 protected:
  bool check_param_vector_size();

  /* ----------------- Layer sizes ----------------- */
  // TODO: Could make observation a struct with named fields
  static constexpr int kActionSize = 12;
  int kJointPositionIdx = 12;
  int kLastActionIdx = kJointPositionIdx + kActionSize;
  int kSingleObservationSize = 3              /* base link angular velocity */
                               + 3            /* projected gravity vector */
                               + 3            /* x, y, yaw velocity commands
                                               */
                               + 3            /* desired world z in body frame
                                               */
                               + kActionSize  /* joint positions */
                               + kActionSize; /* previous action */
  static constexpr int kGravityZIndx = 5;     // Index of gravity z component in the observation
  /* ----------------------------------------------- */

  std::shared_ptr<RTNeural::Model<float>> model_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  // Observation vector. Size is determined at runtime by the observation history parameter
  std::vector<float> observation_ = {};

  // Action vector
  std::array<float, kActionSize> action_ = {};

  // Initial joint positions
  std::array<double, kActionSize> init_joint_pos_ = {};

  // Command velocities
  float cmd_x_vel_ = 0;
  float cmd_y_vel_ = 0;
  float cmd_yaw_vel_ = 0;

  // Command body orientation
  tf2::Vector3 desired_world_z_in_body_frame_ = tf2::Vector3(0, 0, 1);

  // Map from joint names to command types to command interfaces
  std::map<
      std::string,
      std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>>>
      command_interfaces_map_;

  // Map from joint/sensor names to state types to state interfaces
  std::map<std::string,
           std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>>
      state_interfaces_map_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr cmd_subscriber_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<Joy>> rt_joy_command_ptr_;
  rclcpp::Subscription<Joy>::SharedPtr joy_subscriber_;

  rclcpp::Time init_time_;

  int repeat_action_counter_;

  bool estop_active_ = false;
};

}  // namespace neural_controller
