#include "wormhole_navigator/wormhole_navigator.hpp"
#include <sqlite3.h>
#include <thread>

WormholeNavigator::WormholeNavigator()
: Node("wormhole_navigator"), db_(nullptr)
{
  // Open database
  if (sqlite3_open("wormholes.db", &db_) != SQLITE_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to open wormholes.db");
  } else {
    loadWormholes();
  }

  // Action server
  action_server_ = rclcpp_action::create_server<SwitchMap>(
    this,
    "switch_map",
    std::bind(&WormholeNavigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&WormholeNavigator::handle_cancel, this, std::placeholders::_1),
    std::bind(&WormholeNavigator::handle_accepted, this, std::placeholders::_1));

  // Nav2 action client
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Initialpose publisher
  initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10);
}

void WormholeNavigator::loadWormholes()
{
  const char * sql = "SELECT * FROM wormholes;";
  sqlite3_stmt * stmt;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    RCLCPP_ERROR(get_logger(), "Failed to read wormholes table");
    return;
  }

  while (sqlite3_step(stmt) == SQLITE_ROW) {
    Wormhole wh;
    wh.from_map = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    wh.to_map = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
    wh.exit_x = sqlite3_column_double(stmt, 3);
    wh.exit_y = sqlite3_column_double(stmt, 4);
    wh.exit_yaw = sqlite3_column_double(stmt, 5);
    wh.entry_x = sqlite3_column_double(stmt, 6);
    wh.entry_y = sqlite3_column_double(stmt, 7);
    wh.entry_yaw = sqlite3_column_double(stmt, 8);
    wormholes_.push_back(wh);
  }

  sqlite3_finalize(stmt);
  RCLCPP_INFO(get_logger(), "Loaded %zu wormholes", wormholes_.size());
}

rclcpp_action::GoalResponse WormholeNavigator::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const SwitchMap::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received request to switch from %s -> %s",
              goal->from_map.c_str(), goal->to_map.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WormholeNavigator::handle_cancel(
  const std::shared_ptr<GoalHandleSwitchMap>)
{
  RCLCPP_INFO(get_logger(), "Cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WormholeNavigator::handle_accepted(
  const std::shared_ptr<GoalHandleSwitchMap> goal_handle)
{
  std::thread{std::bind(&WormholeNavigator::execute, this, goal_handle)}.detach();
}

void WormholeNavigator::execute(
  const std::shared_ptr<GoalHandleSwitchMap> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<SwitchMap::Result>();

  // Find wormhole
  auto it = std::find_if(wormholes_.begin(), wormholes_.end(),
    [&](const Wormhole & w) {
      return w.from_map == goal->from_map && w.to_map == goal->to_map;
    });

  if (it == wormholes_.end()) {
    result->success = false;
    result->message = "No matching wormhole found";
    goal_handle->abort(result);
    return;
  }

  // Simulate switching map
  RCLCPP_INFO(get_logger(), "Switching map to %s...", it->to_map.c_str());
  loadMap(it->to_map);

  // Send initial pose
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.pose.pose.position.x = it->entry_x;
  init_pose.pose.pose.position.y = it->entry_y;
  init_pose.pose.pose.orientation.z = sin(it->entry_yaw / 2.0);
  init_pose.pose.pose.orientation.w = cos(it->entry_yaw / 2.0);
  initialpose_pub_->publish(init_pose);

  // Send navigation goal
  sendNavigationGoal(it->entry_x, it->entry_y, it->entry_yaw);

  result->success = true;
  result->message = "Switched and navigated";
  goal_handle->succeed(result);
}

bool WormholeNavigator::loadMap(const std::string & map_file)
{
  auto client = this->create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
  auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  req->map_url = map_file;

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Map service not available");
    return false;
  }

  auto future = client->async_send_request(req,
    [this](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
        if (future.get()->result) {
            RCLCPP_INFO(this->get_logger(), "Map loaded successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map");
        }
    });
  return true;
}

void WormholeNavigator::sendNavigationGoal(double x, double y, double yaw)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
    return;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation.z = sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = cos(yaw / 2.0);

  nav_client_->async_send_goal(goal_msg);
}

// --- main ---
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WormholeNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}