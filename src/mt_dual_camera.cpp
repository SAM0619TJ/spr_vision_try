#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aux_solver.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  // 主/辅相机配置路径（与主配置同目录）
  const std::string config_dir =
    config_path.substr(0, config_path.find_last_of("/\\") + 1);
  const std::string main_cam_config = config_dir + "dual_camera_main.yaml";
  const std::string aux_cam_config = config_dir + "dual_camera_aux.yaml";

  io::Gimbal gimbal(config_path);
  io::Camera camera_main(main_cam_config);
  io::Camera camera_aux(aux_cam_config);

  auto_aim::multithread::MultiThreadDetector detector_main(config_path, true);
  auto_aim::multithread::MultiThreadDetector detector_aux(config_path, false);

  auto_aim::Solver solver(config_path);
  auto_aim::AuxSolver aux_solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;

  // 辅助相机最新检测结果
  std::mutex aux_mtx;
  std::optional<std::pair<std::list<auto_aim::Armor>, std::chrono::steady_clock::time_point>>
    latest_aux_result;

  // 主相机采集线程
  auto main_cam_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    while (!exiter.exit() && !quit) {
      if (gimbal.mode() == io::GimbalMode::AUTO_AIM) {
        camera_main.read(img, t);
        detector_main.push(img, t);
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  });

  // 辅助相机采集线程
  auto aux_cam_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    while (!exiter.exit() && !quit) {
      if (gimbal.mode() == io::GimbalMode::AUTO_AIM) {
        camera_aux.read(img, t);
        detector_aux.push(img, t);
      } else {
        std::this_thread::sleep_for(10ms);
      }
    }
  });

  // 辅助相机检测消费线程（取最新结果，try_pop 便于退出）
  auto aux_pop_thread = std::thread([&]() {
    while (!exiter.exit() && !quit) {
      if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
        std::this_thread::sleep_for(50ms);
        continue;
      }
      if (auto result = detector_aux.try_pop()) {
        auto [armors, t] = std::move(*result);
        std::lock_guard<std::mutex> lock(aux_mtx);
        latest_aux_result = {std::move(armors), t};
      } else {
        std::this_thread::sleep_for(5ms);
      }
    }
  });

  // 统一命令发送线程：主有目标用 Planner，主无且辅有时仅发 yaw
  auto command_thread = std::thread([&]() {
    while (!quit) {
      if (gimbal.mode() == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();
        auto gs = gimbal.state();

        if (target.has_value()) {
          auto plan = planner.plan(target, gs.bullet_speed);
          gimbal.send(
            plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
            plan.pitch_acc);
        } else {
          std::list<auto_aim::Armor> armors_aux_copy;
          {
            std::lock_guard<std::mutex> lock(aux_mtx);
            if (latest_aux_result.has_value() && !latest_aux_result->first.empty()) {
              armors_aux_copy = latest_aux_result->first;
            }
          }
          if (!armors_aux_copy.empty()) {
            auto best = armors_aux_copy.front();
            aux_solver.solve(best);
            float aux_yaw =
              static_cast<float>(tools::limit_rad(auto_aim::AuxSolver::yaw_from_armor(best)));
            gimbal.send(true, false, aux_yaw, 0, 0, gs.pitch, 0, 0);
          } else {
            gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
          }
        }
        std::this_thread::sleep_for(10ms);
      } else {
        gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
        std::this_thread::sleep_for(200ms);
      }
    }
  });

  io::GimbalMode last_mode = io::GimbalMode::IDLE;

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }

    if (mode != io::GimbalMode::AUTO_AIM) {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    // 主循环：从主相机获取检测结果，更新 target_queue
    auto [img, armors_main, t_main] = detector_main.debug_pop();
    auto q = gimbal.q(t_main);
    solver.set_R_gimbal2world(q);

    auto targets = tracker.track(armors_main, t_main);

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);
  }

  quit = true;

  if (main_cam_thread.joinable()) main_cam_thread.join();
  if (aux_cam_thread.joinable()) aux_cam_thread.join();
  if (aux_pop_thread.joinable()) aux_pop_thread.join();
  if (command_thread.joinable()) command_thread.join();

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
