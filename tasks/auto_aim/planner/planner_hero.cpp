#include "planner_hero.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/trajectory_hero.hpp"
#include "tools/yaml.hpp"

namespace auto_aim {
Planner_Hero::Planner_Hero(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Plan Planner_Hero::plan(Target target, double bullet_speed) {
  if (bullet_speed < 10.0) {
    bullet_speed = 11.0;
  } else if (bullet_speed > 12.0) {
    bullet_speed = 11.0;
  }

  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto &xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }

  auto bullet_traj = tools::Trajectory_Hero(bullet_speed, min_dist, xyz.z());
  double fly_time;
  if (!bullet_traj.unsolvable) {
    fly_time = bullet_traj.fly_time;
  } else {
    auto bullet_traj_no_drag =
        tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj_no_drag.unsolvable)
      return {false};
    fly_time = bullet_traj_no_drag.fly_time;
  }
  target.predict(fly_time);

  double yaw0;
  auto_aim::Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception &e) {
    tools::logger()->warn("Unsolvable hero target {:.2f}", bullet_speed);
    return {false};
  }

  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_);

  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
      std::hypot(traj(0, HALF_HORIZON + shoot_offset_) -
                     yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
                 traj(2, HALF_HORIZON + shoot_offset_) -
                     pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) <
      fire_thresh_;
  return plan;
}

Plan Planner_Hero::plan(std::optional<Target> target, double bullet_speed) {
  if (!target.has_value())
    return {false};

  double delay_time = std::abs(target->ekf_x()[7]) > decision_speed_
                          ? high_speed_delay_time_
                          : low_speed_delay_time_;

  auto future = std::chrono::steady_clock::now() +
                std::chrono::microseconds(int(delay_time * 1e6));
  target->predict(future);

  return plan(*target, bullet_speed);
}

void Planner_Hero::setup_yaw_solver(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto max_yaw_acc = tools::read<double>(yaml, "max_yaw_acc");
  auto Q_yaw = tools::read<std::vector<double>>(yaml, "Q_yaw");
  auto R_yaw = tools::read<std::vector<double>>(yaml, "R_yaw");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
  Eigen::Matrix<double, 1, 1> R(R_yaw.data());
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
             HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min =
      Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max =
      Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner_Hero::setup_pitch_solver(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto max_pitch_acc = tools::read<double>(yaml, "max_pitch_acc");
  auto Q_pitch = tools::read<std::vector<double>>(yaml, "Q_pitch");
  auto R_pitch = tools::read<std::vector<double>>(yaml, "R_pitch");

  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
  Eigen::Matrix<double, 1, 1> R(R_pitch.data());
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
             HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min =
      Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max =
      Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner_Hero::aim(const Target &target,
                                              double bullet_speed) {
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto &xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory_Hero(bullet_speed, min_dist, xyz.z());
  double pitch_solved;
  if (!bullet_traj.unsolvable) {
    pitch_solved = bullet_traj.pitch;
  } else {
    // Trajectory_Hero 失败时，用无阻力解析解兜底，避免 MPC 轨迹采样整段失败
    auto bullet_traj_no_drag =
        tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj_no_drag.unsolvable)
      throw std::runtime_error("Unsolvable hero bullet trajectory!");
    pitch_solved = bullet_traj_no_drag.pitch;
  }

  return {tools::limit_rad(azim + yaw_offset_),
          -pitch_solved - pitch_offset_};
}

auto_aim::Trajectory Planner_Hero::get_trajectory(Target &target, double yaw0,
                                                  double bullet_speed) {
  auto_aim::Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel =
        tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1),
        pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

} // namespace auto_aim