#include <chrono>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#define RACING_PARAM "mission.racing"
#define APPROACH_AWARENESS_PARAM "mission.approach_awareness"
#define DYNAMIC_SPEED_PARAM "mission.dynamic_speed"

class MissionControllerNode : public rclcpp::Node
{
public:
  MissionControllerNode() : Node("mission_controller")
  {
    m_toggleRacingPublisher =
      this->create_publisher<std_msgs::msg::Bool>("/trajectory_planner/control_planning", 10);

    m_is_racing = this->declare_parameter(RACING_PARAM, false);

    m_param_subscriber = this->add_on_set_parameters_callback(
      std::bind(&MissionControllerNode::parameterChangeCallback, this, std::placeholders::_1));

    // Republish the state so a planner that starts later, or a startup value, still takes effect.
    m_publishTimer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      m_toggleRacingPublisher->publish(std_msgs::msg::Bool().set__data(m_is_racing));
    });

    RCLCPP_INFO(
      this->get_logger(),
      "Mission Controller initialized. Waiting for parameter changes via rqt...");
  }

private:
  bool m_is_racing = false;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_subscriber;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_toggleRacingPublisher;
  rclcpp::TimerBase::SharedPtr m_publishTimer;

  rcl_interfaces::msg::SetParametersResult parameterChangeCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    RCLCPP_INFO(
      this->get_logger(), "Received parameter change. Setting %s to value %s",
      parameters[0].get_name().c_str(), parameters[0].value_to_string().c_str());

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";

    for (const auto & param : parameters) {
      if (
        param.get_name() == RACING_PARAM &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        m_is_racing = param.as_bool();
        m_toggleRacingPublisher->publish(std_msgs::msg::Bool().set__data(m_is_racing));
      } else {
        result.successful = false;
        result.reason = "Invalid parameter or type";
        RCLCPP_WARN(
          this->get_logger(), "mission_controller: Could not set parameter %s",
          param.get_name().c_str());
        break;
      }
    }
    return result;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MissionControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Mission Controller Node failed: " << e.what() << '\n';
  } catch (...) {
    std::cerr << "Mission Controller Node failed: Unknown error occurred" << '\n';
  }

  rclcpp::shutdown();
  return 0;
}
