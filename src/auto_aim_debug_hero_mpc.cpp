#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "debug/web_debugger.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner_hero.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

namespace {

// Check if an environment variable is set and not empty
bool has_env(const char *name) {
  const char *value = std::getenv(name);
  return value != nullptr && value[0] != '\0';
}

bool has_display_server() {
  return has_env("DISPLAY") || has_env("WAYLAND_DISPLAY");
}

bool is_ssh_session() {
  return has_env("SSH_CONNECTION") || has_env("SSH_CLIENT") ||
         has_env("SSH_TTY");
}

bool yaml_bool_or(const YAML::Node &node, bool default_value) {
  return node ? node.as<bool>() : default_value;
}

int to_debug_color(auto_aim::Color color) {
  if (color == auto_aim::Color::blue)
    return 0;
  if (color == auto_aim::Color::red)
    return 1;
  if (color == auto_aim::Color::purple)
    return 3;
  return 2;
}

int to_debug_number(auto_aim::ArmorName name) {
  switch (name) {
  case auto_aim::ArmorName::sentry:
    return 0;
  case auto_aim::ArmorName::one:
    return 1;
  case auto_aim::ArmorName::two:
    return 2;
  case auto_aim::ArmorName::three:
    return 3;
  case auto_aim::ArmorName::four:
    return 4;
  case auto_aim::ArmorName::five:
    return 5;
  case auto_aim::ArmorName::outpost:
    return 6;
  case auto_aim::ArmorName::base:
    return 7;
  default:
    return 8;
  }
}

std::vector<debug::DetectionData>
to_debug_detections(const std::list<auto_aim::Armor> &armors) {
  std::vector<debug::DetectionData> detections;
  detections.reserve(armors.size());

  for (const auto &armor : armors) {
    debug::DetectionData detection;
    detection.pts = armor.points;
    detection.color = to_debug_color(armor.color);
    detection.number = to_debug_number(armor.name);
    detection.conf = static_cast<float>(armor.confidence);
    detections.push_back(detection);
  }

  return detections;
}

} // namespace

const std::string keys =
    "{help h usage ? |                     | 输出命令行参数说明}"
    "{@config-path   | configs/hero.yaml   | 位置参数，yaml配置文件路径 }";

int main(int argc, char *argv[]) {
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto config = YAML::LoadFile(config_path);
  auto plotter_host = config["plotter"] && config["plotter"]["host"]
                          ? config["plotter"]["host"].as<std::string>()
                          : "127.0.0.1";
  auto plotter_port = config["plotter"] && config["plotter"]["port"]
                          ? config["plotter"]["port"].as<uint16_t>()
                          : 9870;
  tools::Plotter plotter(plotter_host, plotter_port);

  auto debug_display_config = config["debug_display"];
  auto window_config =
      debug_display_config ? debug_display_config["window"] : YAML::Node();
  auto web_config =
      debug_display_config ? debug_display_config["web"] : YAML::Node();

  auto window_enabled = yaml_bool_or(window_config["enabled"], true);
  auto window_auto_detect = yaml_bool_or(window_config["auto_detect"], true);
  if (window_enabled && window_auto_detect && !has_display_server()) {
    tools::logger()->warn(
        "{} display server found, local debug window disabled. Use the web "
        "debugger instead.",
        is_ssh_session() ? "SSH session without" : "No");
    window_enabled = false;
  }
  if (window_enabled) {
    cv::namedWindow("reprojection", cv::WINDOW_NORMAL);
  }

  auto web_debug_enabled = yaml_bool_or(web_config["enabled"], true);
  auto web_debug_port =
      web_config && web_config["port"]
          ? web_config["port"].as<int>()
          : config["web_debugger"] && config["web_debugger"]["port"]
                ? config["web_debugger"]["port"].as<int>()
                : 8080;
  std::unique_ptr<debug::WebDebugger> web_debugger;
  if (web_debug_enabled) {
    web_debugger = std::make_unique<debug::WebDebugger>(web_debug_port);
    web_debugger->start();
    tools::logger()->info("Web debugger listening on http://0.0.0.0:{}",
                          web_debug_port);
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, window_enabled);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner_Hero planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc,
                  plan.pitch, plan.pitch_vel, plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["bullet_speed"] = gs.bullet_speed;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];
        data["target_vz"] = target->ekf_x()[5];
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    auto frame_start = std::chrono::steady_clock::now();
    camera.read(img, t);
    auto web_frame = img.clone();
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto detections = to_debug_detections(armors);
    std::vector<debug::ReprojectionData> reprojections;

    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);

    if (!targets.empty()) {
      auto target = targets.front();

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d &xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(
            xyza.head(3), xyza[3], target.armor_type, target.name);
        reprojections.push_back({image_points});
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points = solver.reproject_armor(
          aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      reprojections.push_back({image_points});
      tools::draw_points(img, image_points, {0, 0, 255});
    }

    auto latency_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - frame_start)
                          .count();
    if (web_debugger) {
      web_debugger->push(web_frame, detections, reprojections, latency_ms);
    }

    if (window_enabled) {
      cv::Mat display_img;
      cv::resize(img, display_img, {}, 0.5, 0.5);
      cv::imshow("reprojection", display_img);
      auto key = cv::waitKey(1);
      if (key == 'q')
        break;
    }
  }

  quit = true;
  if (plan_thread.joinable())
    plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}