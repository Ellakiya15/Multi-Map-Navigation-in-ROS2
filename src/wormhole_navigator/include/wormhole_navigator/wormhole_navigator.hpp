#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <sqlite3.h>

#include <string>
#include <vector>

#include "wormhole_interfaces/action/switch_map.hpp"

class WormholeNavigator : public rclcpp::Node
{
public:
  using SwitchMap = wormhole_interfaces::action::SwitchMap;
  using GoalHandleSwitchMap = rclcpp_action::ServerGoalHandle<SwitchMap>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  explicit WormholeNavigator();

private:
  struct Wormhole
  {
    std::string from_map;
    std::string to_map;
    double exit_x, exit_y, exit_yaw;
    double entry_x, entry_y, entry_yaw;
  };

  // --- Action server callback ---
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SwitchMap::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSwitchMap> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleSwitchMap> goal_handle);

  void execute(const std::shared_ptr<GoalHandleSwitchMap> goal_handle);

  // --- Helpers ---
  void loadWormholes();
  bool loadMap(const std::string & map_file);
  void sendNavigationGoal(double x, double y, double yaw);

  // --- ROS interfaces ---
  rclcpp_action::Server<SwitchMap>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;

  // --- DB ---
  sqlite3 * db_;
  std::vector<Wormhole> wormholes_;
};