#ifndef TOOLS__TRAJECTORY_HERO_HPP
#define TOOLS__TRAJECTORY_HERO_HPP

namespace tools
{
struct Trajectory_Hero
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正
  // 考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory_Hero(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HERO_HPP