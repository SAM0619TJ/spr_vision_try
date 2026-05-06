#ifndef AUTO_AIM__PLANNER_HERO_HPP
#define AUTO_AIM__PLANNER_HERO_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <string>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/target.hpp"

#include "tinympc/tiny_api.hpp"

namespace auto_aim {

class Planner_Hero {
public:
  Eigen::Vector4d debug_xyza;
  Planner_Hero(const std::string &config_path);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver *yaw_solver_;
  TinySolver *pitch_solver_;

  void setup_yaw_solver(const std::string &config_path);
  void setup_pitch_solver(const std::string &config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target &target, double bullet_speed);
  Trajectory get_trajectory(Target &target, double yaw0, double bullet_speed);
};

} // namespace auto_aim

#endif // AUTO_AIM__PLANNER_HERO_HPP
