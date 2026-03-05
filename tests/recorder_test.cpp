#include "tools/recorder.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"

const std::string keys =
    "{help h usage ? |                     | 输出命令行参数说明}"
    "{fps f          | 30                 | 录制帧率          }"
    "{duration d     | 5                  | 录制时长（秒）    }"
    "{camera c       | 0                  | 摄像头索引（0,1,2...）}"
    "{video v        |                    | 从视频文件读取录制}"
    "{generate g     |                    | 生成测试图像录制  }"
    "{fallback       |                    | 摄像头打开失败时自动使用生成模式}";

// 生成测试图像（彩色渐变）
cv::Mat generateTestImage(int width, int height, int frame_count) {
  cv::Mat img(height, width, CV_8UC3);

  // 创建彩色渐变效果
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int r = (x * 255 / width + frame_count * 2) % 256;
      int g = (y * 255 / height + frame_count * 3) % 256;
      int b = ((x + y) * 255 / (width + height) + frame_count * 5) % 256;
      img.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r); // BGR格式
    }
  }

  // 添加帧号文本
  cv::putText(img, "Frame: " + std::to_string(frame_count), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

  return img;
}

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  double fps = cli.get<double>("fps");
  double duration = cli.get<double>("duration");
  bool use_camera = cli.has("camera");
  bool use_video = cli.has("video");
  bool generate = cli.has("generate");
  bool fallback = cli.has("fallback");

  // 获取摄像头索引（如果使用摄像头）
  int camera_index = 0;
  if (use_camera) {
    camera_index = cli.get<int>("camera");
  }

  // 如果没有指定任何输入源，默认使用生成测试图像
  if (!use_camera && !use_video) {
    generate = true;
  }

  tools::Recorder recorder(fps);
  tools::logger()->info("开始录制，帧率: {} fps, 时长: {} 秒", fps, duration);

  cv::VideoCapture cap;
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto start_time = std::chrono::steady_clock::now();
  int frame_count = 0;

  // 初始化输入源
  if (use_camera) {
    tools::logger()->info("尝试打开摄像头索引: {}", camera_index);
    cap.open(camera_index);

    // 如果指定索引失败，尝试其他索引
    if (!cap.isOpened() && camera_index == 0) {
      tools::logger()->warn("摄像头索引 {} 打开失败，尝试其他索引...",
                            camera_index);
      for (int i = 1; i <= 3; i++) {
        cap.open(i);
        if (cap.isOpened()) {
          camera_index = i;
          tools::logger()->info("成功打开摄像头索引: {}", i);
          break;
        }
      }
    }

    if (!cap.isOpened()) {
      tools::logger()->error("无法打开摄像头（已尝试索引 0-3）");
      if (fallback) {
        tools::logger()->warn("自动切换到生成测试图像模式");
        generate = true;
        use_camera = false;
      } else {
        tools::logger()->error(
            "提示：可以使用 --fallback 参数在摄像头失败时自动使用生成模式");
        tools::logger()->error("或者使用 --generate 直接使用生成测试图像模式");
        return -1;
      }
    } else {
      // 设置摄像头参数
      cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
      cap.set(cv::CAP_PROP_FPS, fps);

      // 读取实际设置的参数
      double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
      double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
      double actual_fps = cap.get(cv::CAP_PROP_FPS);

      tools::logger()->info("摄像头打开成功！索引: {}, 分辨率: {}x{}, 帧率: {}",
                            camera_index, actual_width, actual_height,
                            actual_fps);

      // 测试读取一帧以验证摄像头是否正常工作
      cv::Mat test_frame;
      if (cap.read(test_frame)) {
        tools::logger()->info("摄像头测试帧读取成功，尺寸: {}x{}",
                              test_frame.cols, test_frame.rows);
      } else {
        tools::logger()->warn("摄像头打开但无法读取帧，可能存在问题");
        if (fallback) {
          tools::logger()->warn("自动切换到生成测试图像模式");
          generate = true;
          use_camera = false;
          cap.release();
        } else {
          tools::logger()->error("摄像头无法读取帧，请检查摄像头连接");
          return -1;
        }
      }
    }
  } else if (use_video) {
    std::string video_path = cli.get<std::string>("video");
    cap.open(video_path);
    if (!cap.isOpened()) {
      tools::logger()->error("无法打开视频文件: {}", video_path);
      return -1;
    }
    tools::logger()->info("从视频文件录制: {}", video_path);
  }

  // 模拟四元数（单位四元数，表示无旋转）
  Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);

  while (!exiter.exit()) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration<double>(current_time - start_time).count();

    if (elapsed >= duration) {
      tools::logger()->info("录制时长达到 {} 秒，停止录制", duration);
      break;
    }

    timestamp = std::chrono::steady_clock::now();

    // 获取图像
    if (generate) {
      img = generateTestImage(640, 480, frame_count);
    } else {
      if (!cap.read(img)) {
        tools::logger()->warn("无法读取图像帧");
        break;
      }
    }

    if (img.empty()) {
      tools::logger()->warn("图像为空，跳过此帧");
      continue;
    }

    // 模拟四元数变化（简单的旋转）
    double angle = frame_count * 0.01;
    q = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

    // 录制帧
    recorder.record(img, q, timestamp);

    frame_count++;

    // 显示进度
    if (frame_count % static_cast<int>(fps) == 0) {
      tools::logger()->info("已录制 {} 秒，共 {} 帧", static_cast<int>(elapsed),
                            frame_count);
    }

    // 显示图像（可选）
    cv::imshow("Recording", img);
    if (cv::waitKey(1) == 'q') {
      tools::logger()->info("用户中断录制");
      break;
    }

    // 控制帧率（如果生成测试图像）
    if (generate) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(1000.0 / fps)));
    }
  }

  if (cap.isOpened()) {
    cap.release();
  }
  cv::destroyAllWindows();

  tools::logger()->info("录制完成！共录制 {} 帧", frame_count);
  tools::logger()->info("视频和姿态数据已保存到 records/ 目录");

  return 0;
}
