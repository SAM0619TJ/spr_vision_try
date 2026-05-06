#include "trajectory_hero.hpp"

#include <cmath>

namespace tools {

// 弹道模型说明
// ----------------------------------------------------------------------------
// 阻力模型：F_drag = 0.5 * rho * Cd * A * v^2，沿速度反方向。
// 简化假设：水平方向独立处理，认为水平速度 vx 满足
//     dvx/dt = -k1 * vx^2,   k1 = 0.5 * rho * Cd * A / m
// 解析解：
//     vx(t) = v0x / (1 + k1 * v0x * t)
//     x(t)  = ln(1 + k1 * v0x * t) / k1
// 由 x(T) = d 反解得：
//     T(theta) = (exp(k1 * d) - 1) / (k1 * v0 * cos(theta))
//
// 垂直方向：忽略垂直阻力（量级较小且与水平耦合复杂），仍按自由落体处理：
//     z(T) = v0 * sin(theta) * T - 0.5 * g * T^2
// 用 x 参数化（t = x / vx_avg 对纯重力情形等价）的等价形式：
//     z(d) = d * tan(theta) - 0.5 * g * T^2 / cos^2(theta) ?  ——不等价！
// 严格地，把 T(theta) 直接代入 z(T) 即可，无需二次变量替换。
// 这里采用最直接的形式：
//     f(theta) = h - v0 * sin(theta) * T(theta) + 0.5 * g * T(theta)^2
// 其中 T(theta) 已包含水平阻力效应。
//
// 解析导数（设 C = (exp(k1*d) - 1)/(k1*v0)，则 T = C/cos(theta)）：
//     v0 * sin(theta) * T = v0 * C * tan(theta)
//     0.5 * g * T^2       = 0.5 * g * C^2 / cos^2(theta)
// =>  f(theta) = h - v0*C*tan(theta) + 0.5*g*C^2/cos^2(theta)
//     f'(theta) = -v0*C/cos^2(theta) + g*C^2*sin(theta)/cos^3(theta)

constexpr double g = 9.7833;
constexpr double k1 =
    0.5 * 1.225 * 0.47 * (3.1415926535 * 0.021 * 0.021) / 0.042;

constexpr int    kMaxIter   = 30;
constexpr double kTolMeters = 1e-5;
constexpr double kMaxStep   = 0.1;   // 单步仰角变化上限 (rad)
constexpr double kPitchLim  = 1.4;   // 仰角合理范围 (rad)，约 80°
constexpr double kCosMin    = 1e-3;  // 防止 cos(theta) 过小

inline double clamp_d(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

Trajectory_Hero::Trajectory_Hero(const double v0, const double d,
                                 const double h) {
  unsolvable = false;
  fly_time   = 0.0;
  pitch      = 0.0;

  if (v0 <= 0.0 || d <= 0.0) {
    unsolvable = true;
    return;
  }

  const double C = (std::exp(k1 * d) - 1.0) / (k1 * v0);

  double theta = std::atan2(h, d);
  bool   converged = false;

  for (int i = 0; i < kMaxIter; ++i) {
    double cos_t = std::cos(theta);
    if (std::abs(cos_t) < kCosMin) {
      unsolvable = true;
      return;
    }
    double sin_t  = std::sin(theta);
    double tan_t  = sin_t / cos_t;
    double sec2_t = 1.0 / (cos_t * cos_t);

    double T = C / cos_t;

    double f  = h - v0 * C * tan_t + 0.5 * g * C * C * sec2_t;
    double df = -v0 * C * sec2_t + g * C * C * sin_t / (cos_t * cos_t * cos_t);

    if (std::abs(f) < kTolMeters) {
      fly_time  = T;
      pitch     = theta;
      converged = true;
      break;
    }

    if (std::abs(df) < 1e-12) {
      unsolvable = true;
      return;
    }

    double step = f / df;
    step = clamp_d(step, -kMaxStep, kMaxStep);
    theta -= step;

    theta = clamp_d(theta, -kPitchLim, kPitchLim);
  }

  if (!converged) {
    unsolvable = true;
    return;
  }
}

}  // namespace tools
