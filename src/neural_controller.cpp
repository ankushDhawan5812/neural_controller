#include "neural_controller/neural_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace neural_controller
{
  NeuralController::NeuralController()
      : controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr),
        cmd_subscriber_(nullptr)
  {
  }

  controller_interface::CallbackReturn NeuralController::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();

      std::ifstream json_stream(params_.model_path, std::ifstream::binary);
      nlohmann::json model_json;
      json_stream >> model_json;
      RTNeural::torch_helpers::loadLSTM<float>(model_json, "memory.", model_.get<0>());
      RTNeural::torch_helpers::loadDense<float>(model_json, "actor.0.", model_.get<1>());
      RTNeural::torch_helpers::loadDense<float>(model_json, "actor.2.", model_.get<3>());
      RTNeural::torch_helpers::loadDense<float>(model_json, "actor.4.", model_.get<5>());
      RTNeural::torch_helpers::loadDense<float>(model_json, "actor.6.", model_.get<7>());
      model_.reset();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn NeuralController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Initialize the command subscriber
    cmd_subscriber_ = get_node()->create_subscription<CmdType>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  NeuralController::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::ALL};
  }

  controller_interface::InterfaceConfiguration NeuralController::state_interface_configuration()
      const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::ALL};
  }

  controller_interface::CallbackReturn NeuralController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

    // Populate the command interfaces map
    for (auto &command_interface : command_interfaces_)
    {
      command_interfaces_map_[command_interface.get_prefix_name()].emplace(
          command_interface.get_interface_name(),
          std::ref(command_interface));
    }

    // Populate the state interfaces map
    for (auto &state_interface : state_interfaces_)
    {
      state_interfaces_map_[state_interface.get_prefix_name()].emplace(
          state_interface.get_interface_name(),
          std::ref(state_interface));
    }

    // Set the kp and kd gains for each joint
    for (int i = 0; i < ACTION_SIZE; i++)
    {
      command_interfaces_map_.at(params_.joint_names[i]).at("kp").get().set_value(params_.kps[i]);
      command_interfaces_map_.at(params_.joint_names[i]).at("kd").get().set_value(params_.kds[i]);
    }

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn NeuralController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    for (auto &command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type NeuralController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // Get the latest commanded velocities
    auto command = rt_command_ptr_.readFromRT();
    if (command && command->get())
    {
      cmd_x_vel_ = command->get()->linear.x;
      cmd_y_vel_ = command->get()->linear.y;
      cmd_yaw_vel_ = command->get()->angular.z;
    }

    // Get the latest observation
    double pitch_vel, roll_vel, yaw_vel, orientation_w, orientation_x, orientation_y, orientation_z;
    try
    {
      // read IMU states from hardware interface
      pitch_vel = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.y").get().get_value();
      roll_vel = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.x").get().get_value();
      yaw_vel = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.z").get().get_value();
      orientation_w = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.w").get().get_value();
      orientation_x = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.x").get().get_value();
      orientation_y = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.y").get().get_value();
      orientation_z = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.z").get().get_value();

      // Calculate the projected gravity vector
      tf2::Quaternion q(
        orientation_x,
        orientation_y,
        orientation_z,
        orientation_w);
      tf2::Matrix3x3 m(q);
      tf2::Vector3 world_gravity_vector(0, 0, -1);
      tf2::Vector3 projected_gravity_vector = m.inverse() * world_gravity_vector;

      // Fill the observation vector
      // Linear velocity (zeroed out, this isn't observed)
      observation_[0] = 0.0;
      observation_[1] = 0.0;
      observation_[2] = 0.0;
      // Angular velocity
      observation_[3] = (float) pitch_vel * params_.ang_vel_scale;
      observation_[4] = (float) roll_vel * params_.ang_vel_scale;
      observation_[5] = (float) yaw_vel * params_.ang_vel_scale;
      // Projected gravity vector
      observation_[6] = (float) projected_gravity_vector[0];
      observation_[7] = (float) projected_gravity_vector[1];
      observation_[8] = (float) projected_gravity_vector[2];
      // Commands
      observation_[9] = (float) cmd_x_vel_ * params_.lin_vel_scale;
      observation_[10] = (float) cmd_y_vel_ * params_.lin_vel_scale;
      observation_[11] = (float) cmd_yaw_vel_ * params_.ang_vel_scale;
      // Joint positions
      for (int i = 0; i < ACTION_SIZE; i++)
      {
        observation_[12 + i] = (float) wrap_angle((state_interfaces_map_.at(params_.joint_names[i]).at("position").get().get_value() - params_.default_joint_pos[i]) * params_.joint_pos_scale, -2 * M_PI, 2 * M_PI);
      }
      // Joint velocities
      for (int i = 0; i < ACTION_SIZE; i++)
      {
        observation_[12 + ACTION_SIZE + i] = (float) state_interfaces_map_.at(params_.joint_names[i]).at("velocity").get().get_value() * params_.joint_vel_scale;
      }
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_INFO(get_node()->get_logger(), "failed to read joint states from hardware interface");
      return controller_interface::return_type::OK;
    }

    // Perform policy inference
    model_.forward(observation_);

    // Process the actions
    const float* policy_output = model_.getOutputs();
    for (int i = 0; i < ACTION_SIZE; i++)
    {
      // Copy policy_output to the observation vector
      observation_[12 + ACTION_SIZE * 2 + i] = policy_output[i];
      // Scale and de-normalize to get the action vector
      if (params_.action_type == "position")
      {
        action_[i] = policy_output[i] * params_.action_scale + params_.default_joint_pos[i];
      }
      else {
        action_[i] = policy_output[i] * params_.action_scale;
      }
      // Send the action to the hardware interface
      command_interfaces_map_.at(params_.joint_names[i]).at(params_.action_type).get().set_value((double) action_[i]);
    }

    return controller_interface::return_type::OK;
  }

  float NeuralController::wrap_angle(float angle, float angle_min, float angle_max)
  {
      /// Wraps an angle to the range [angle_min, angle_max] ///
      float span = angle_max - angle_min;
      return angle - span * floor((angle - angle_min) / span);
  }

} // namespace neural_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    neural_controller::NeuralController,
    controller_interface::ControllerInterface)