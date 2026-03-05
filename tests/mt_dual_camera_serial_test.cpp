/**
 * mt_dual_camera 串口协议模拟测试
 * 不打开真实串口、不实例化 Gimbal/Camera，仅构造并打印模拟的收发协议包。
 */

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"

#include <cstdint>

static void fill_and_set_tail(io::GimbalToVision & rx)
{
  rx.tail = 0xef;  // 帧尾校验（原为 CRC16）
}

static void fill_and_set_tail(io::VisionToGimbal & tx)
{
  tx.tail = 0xef;  // 帧尾校验（原为 CRC16）
}

static const char * mode_str(uint8_t mode)
{
  switch (mode) {
    case 0: return "IDLE";
    case 1: return "AUTO_AIM";
    case 2: return "SMALL_BUFF";
    case 3: return "BIG_BUFF";
    default: return "?";
  }
}

static void print_rx(const io::GimbalToVision & rx, int index)
{
  tools::logger()->info("--- 模拟接收 [云台->视觉] #{} ---", index);
  tools::logger()->info("  head: 0x{:02X} 0x{:02X}", rx.head[0], rx.head[1]);
  tools::logger()->info("  mode: {} ({})", static_cast<int>(rx.mode), mode_str(rx.mode));
  tools::logger()->info(
    "  q: w={:.4f} x={:.4f} y={:.4f} z={:.4f}", rx.q[0], rx.q[1], rx.q[2], rx.q[3]);
  tools::logger()->info("  yaw={:.4f} yaw_vel={:.4f} pitch={:.4f} pitch_vel={:.4f}",
                        rx.yaw, rx.yaw_vel, rx.pitch, rx.pitch_vel);
  tools::logger()->info("  bullet_speed={:.4f} bullet_count={}", rx.bullet_speed, rx.bullet_count);
  tools::logger()->info("  tail=0x{:02X}", rx.tail);
  tools::logger()->info("  frame_tail_check: {}", rx.tail == 0xef ? "OK" : "FAIL");
}

static void print_tx(const io::VisionToGimbal & tx, int index)
{
  tools::logger()->info("--- 模拟发送 [视觉->云台] #{} ---", index);
  tools::logger()->info("  head: 0x{:02X} 0x{:02X}", tx.head[0], tx.head[1]);
  tools::logger()->info("  mode: {} (0=不控制 1=控制 2=控制+开火)", static_cast<int>(tx.mode));
  tools::logger()->info(
    "  yaw={:.4f} yaw_vel={:.4f} yaw_acc={:.4f}", tx.yaw, tx.yaw_vel, tx.yaw_acc);
  tools::logger()->info(
    "  pitch={:.4f} pitch_vel={:.4f} pitch_acc={:.4f}", tx.pitch, tx.pitch_vel, tx.pitch_acc);
  tools::logger()->info("  tail=0x{:02X}", tx.tail);
  tools::logger()->info("  frame_tail_check: {}", tx.tail == 0xef ? "OK" : "FAIL");
}

int main(int, char **)
{
  tools::logger()->info("========== mt_dual_camera 串口协议模拟测试 ==========");
  tools::logger()->info("（无真实串口、无 Gimbal/Camera 实例）");

  // ----- 模拟接收：云台 -> 视觉 (GimbalToVision) -----
  io::GimbalToVision rx1{};
  rx1.head[0] = 'S';
  rx1.head[1] = 'P';
  rx1.mode = 0;  // IDLE
  rx1.q[0] = 1.f;
  rx1.q[1] = 0.f;
  rx1.q[2] = 0.f;
  rx1.q[3] = 0.f;
  rx1.yaw = 0.f;
  rx1.yaw_vel = 0.f;
  rx1.pitch = 0.f;
  rx1.pitch_vel = 0.f;
  rx1.bullet_speed = 0.f;
  rx1.bullet_count = 0;
  fill_and_set_tail(rx1);
  print_rx(rx1, 1);

  io::GimbalToVision rx2{};
  rx2.head[0] = 'S';
  rx2.head[1] = 'P';
  rx2.mode = 1;  // AUTO_AIM
  rx2.q[0] = 0.9239f;
  rx2.q[1] = 0.f;
  rx2.q[2] = 0.3827f;
  rx2.q[3] = 0.f;
  rx2.yaw = 0.1f;
  rx2.yaw_vel = 0.02f;
  rx2.pitch = -0.05f;
  rx2.pitch_vel = 0.01f;
  rx2.bullet_speed = 22.f;
  rx2.bullet_count = 100;
  fill_and_set_tail(rx2);
  print_rx(rx2, 2);

  io::GimbalToVision rx3{};
  rx3.head[0] = 'S';
  rx3.head[1] = 'P';
  rx3.mode = 2;  // SMALL_BUFF
  rx3.q[0] = 1.f;
  rx3.q[1] = 0.f;
  rx3.q[2] = 0.f;
  rx3.q[3] = 0.f;
  rx3.yaw = 0.5f;
  rx3.yaw_vel = 0.f;
  rx3.pitch = -0.1f;
  rx3.pitch_vel = 0.f;
  rx3.bullet_speed = 18.f;
  rx3.bullet_count = 50;
  fill_and_set_tail(rx3);
  print_rx(rx3, 3);

  // ----- 模拟发送：视觉 -> 云台 (VisionToGimbal) -----
  io::VisionToGimbal tx1{};
  tx1.head[0] = 'S';
  tx1.head[1] = 'P';
  tx1.mode = 0;  // 不控制
  tx1.yaw = 0.f;
  tx1.yaw_vel = 0.f;
  tx1.yaw_acc = 0.f;
  tx1.pitch = 0.f;
  tx1.pitch_vel = 0.f;
  tx1.pitch_acc = 0.f;
  fill_and_set_tail(tx1);
  print_tx(tx1, 1);

  io::VisionToGimbal tx2{};
  tx2.head[0] = 'S';
  tx2.head[1] = 'P';
  tx2.mode = 1;  // 控制但不开火
  tx2.yaw = 0.2f;
  tx2.yaw_vel = 0.1f;
  tx2.yaw_acc = 0.f;
  tx2.pitch = -0.05f;
  tx2.pitch_vel = 0.f;
  tx2.pitch_acc = 0.f;
  fill_and_set_tail(tx2);
  print_tx(tx2, 2);

  io::VisionToGimbal tx3{};
  tx3.head[0] = 'S';
  tx3.head[1] = 'P';
  tx3.mode = 2;  // 控制且开火
  tx3.yaw = 0.15f;
  tx3.yaw_vel = 0.05f;
  tx3.yaw_acc = 0.f;
  tx3.pitch = -0.08f;
  tx3.pitch_vel = 0.02f;
  tx3.pitch_acc = 0.f;
  fill_and_set_tail(tx3);
  print_tx(tx3, 3);

  tools::logger()->info("========== 测试结束 ==========");
  return 0;
}
