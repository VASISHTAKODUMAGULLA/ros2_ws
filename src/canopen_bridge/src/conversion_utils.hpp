#ifndef CANOPEN_BRIDGE_CONVERSION_UTILS_HPP
#define CANOPEN_BRIDGE_CONVERSION_UTILS_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace canopen_bridge
{

/// Convert a ROS velocity (m/s) into a scaled CANopen 16-bit integer.
///
/// The result is rounded (not truncated) and clamped to the int16_t range so
/// that an out-of-range command can never trigger undefined behaviour from a
/// narrowing cast — it saturates instead.
///
/// Example: to_canopen_velocity(1.5) -> 1500
inline int16_t to_canopen_velocity(double ros_vel, double scale = 1000.0)
{
  const double scaled = std::round(ros_vel * scale);
  const double clamped = std::clamp(
    scaled,
    static_cast<double>(std::numeric_limits<int16_t>::min()),
    static_cast<double>(std::numeric_limits<int16_t>::max()));
  return static_cast<int16_t>(clamped);
}

/// Convert a scaled CANopen 16-bit integer back into a ROS velocity (m/s).
///
/// Example: from_canopen_velocity(1450) -> 1.45
inline double from_canopen_velocity(int16_t can_vel, double scale = 1000.0)
{
  return static_cast<double>(can_vel) / scale;
}

}  // namespace canopen_bridge

#endif  // CANOPEN_BRIDGE_CONVERSION_UTILS_HPP
