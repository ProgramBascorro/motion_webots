#ifndef OP3_DEMO_KICK_ALIGNMENT_POLICY_H_
#define OP3_DEMO_KICK_ALIGNMENT_POLICY_H_

#include <cmath>
#include <limits>

namespace robotis_op
{

enum class KickPositionDecision
{
  Wait,
  Approach,
  Shift,
  Ready
};

inline double kickGroundDistance(double camera_height, double ball_radius,
                                 double depression_rad, double min_depression_rad)
{
  const double max_depression_rad = 89.0 * std::acos(-1.0) / 180.0;
  if (!std::isfinite(camera_height) || !std::isfinite(ball_radius) ||
      !std::isfinite(depression_rad) || !std::isfinite(min_depression_rad) ||
      ball_radius <= 0.0 || camera_height <= ball_radius ||
      min_depression_rad <= 0.0 || depression_rad < min_depression_rad ||
      depression_rad >= max_depression_rad)
    return std::numeric_limits<double>::quiet_NaN();
  return (camera_height - ball_radius) / std::tan(depression_rad);
}

inline KickPositionDecision evaluateKickPosition(bool observation_valid, double distance,
                                                 double kick_distance, bool foot_in_window)
{
  if (!observation_valid || !std::isfinite(distance) || distance <= 0.0 ||
      !std::isfinite(kick_distance) || kick_distance <= 0.0)
    return KickPositionDecision::Wait;
  if (!foot_in_window)
    return KickPositionDecision::Shift;
  if (distance > kick_distance)
    return KickPositionDecision::Approach;
  return KickPositionDecision::Ready;
}

inline bool kickHeadingReady(bool alignment_enabled, bool yaw_valid,
                             double yaw_error_deg, double tolerance_deg)
{
  return !alignment_enabled ||
      (yaw_valid && std::isfinite(yaw_error_deg) && std::isfinite(tolerance_deg) &&
       tolerance_deg >= 0.0 && std::abs(yaw_error_deg) <= tolerance_deg);
}

}

#endif
