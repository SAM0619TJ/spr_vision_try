/**
 * mt_dual_camera 双相机 + 模拟云台 测试
 * 使用真实主/辅相机（io::Camera），云台不接串口，用内存中的模拟状态与 q(t)，send 仅打印。
 * 流程与 mt_dual_camera 一致，便于有相机无串口时联调。
 */

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
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

static const char * mode_str(io::GimbalMode mode)
{
  switch (mode) {
    case io::GimbalMode::IDLE:
      return "IDLE";
    case io::GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case io::GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case io::GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "?";
  }
}

/** 模拟云台：不打开串口，mode/state/q 固定，send 仅打印 */
struct MockGimbal
{
  io::GimbalMode mode_ = io::GimbalMode::AUTO_AIM;
  io::GimbalState state_{};
  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity();

  io::GimbalMode mode() const { return mode_; }
  io::GimbalState state() const { return state_; }
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point) const { return q_; }

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc)
  {
    tools::logger()->info(
      "[MockGimbal] send control={} fire={} yaw={:.4f} yaw_vel={:.4f} yaw_acc={:.4f} pitch={:.4f} "
      "pitch_vel={:.4f} pitch_acc={:.4f}",
      control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
  }
};

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  const std::string config_dir =
    config_path.substr(0, config_path.find_last_of("/\\") + 1);
  const std::string main_cam_config = config_dir + "dual_camera_main.yaml";
  const std::string aux_cam_config = config_dir + "dual_camera_aux.yaml";

  MockGimbal gimbal;
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

  std::mutex aux_mtx;
  std::optional<std::pair<std::list<auto_aim::Armor>, std::chrono::steady_clock::time_point>>
    latest_aux_result;
  cv::Mat latest_aux_img;

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

  auto aux_pop_thread = std::thread([&]() {
    while (!exiter.exit() && !quit) {
      if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
        std::this_thread::sleep_for(50ms);
        continue;
      }
      auto [img_aux, armors, t] = detector_aux.debug_pop();
      std::lock_guard<std::mutex> lock(aux_mtx);
      latest_aux_result = {std::move(armors), t};
      latest_aux_img = img_aux.clone();
    }
  });

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

  tools::logger()->info(
    "mt_dual_camera_mock_gimbal_test: 双相机 + 模拟云台，主/辅画面见 main / aux 窗口，按 q 或 Ctrl+C 退出");

  while (!exiter.exit()) {
    auto mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", mode_str(mode));
      last_mode = mode;
    }

    if (mode != io::GimbalMode::AUTO_AIM) {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    auto [img, armors_main, t_main] = detector_main.debug_pop();
    auto q = gimbal.q(t_main);
    solver.set_R_gimbal2world(q);

    auto targets = tracker.track(armors_main, t_main);

    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});
    cv::Mat display;
    cv::resize(img, display, {}, 0.5, 0.5);
    cv::imshow("main", display);
    {
      std::lock_guard<std::mutex> lock(aux_mtx);
      cv::Mat display_aux;
      if (!latest_aux_img.empty()) {
        cv::resize(latest_aux_img, display_aux, {}, 0.5, 0.5);
      } else {
        display_aux = cv::Mat(360, 640, CV_8UC3, cv::Scalar(40, 40, 40));
        cv::putText(
          display_aux, "Aux (MindVision) not ready", {100, 160}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
          {200, 200, 200}, 2);
        cv::putText(
          display_aux, "err:-18: device in use? Unplug or close other app.", {40, 200},
          cv::FONT_HERSHEY_SIMPLEX, 0.5, {150, 150, 150}, 1);
      }
      cv::imshow("aux", display_aux);
    }
    if (cv::waitKey(1) == 'q') break;

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
  tools::logger()->info("mt_dual_camera_mock_gimbal_test 退出");

  return 0;
}
