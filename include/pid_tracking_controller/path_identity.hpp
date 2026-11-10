#ifndef PID_TRACKING_CONTROLLER__PATH_IDENTITY_HPP_
#define PID_TRACKING_CONTROLLER__PATH_IDENTITY_HPP_
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include "nav_msgs/msg/path.hpp"

// Shared v1 wire identity. Stamps are matched separately; q and -q are equivalent.
namespace pid_tracking_controller::path_identity
{
namespace detail
{

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

inline void hashByte(std::uint64_t & hash, std::uint8_t value)
{
  hash ^= value;
  hash *= kFnvPrime;
}

inline void hashUint64(std::uint64_t & hash, std::uint64_t value)
{
  for (unsigned int shift = 0; shift < 64; shift += 8) {
    hashByte(hash, static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

inline void hashString(std::uint64_t & hash, const std::string & value)
{
  hashUint64(hash, static_cast<std::uint64_t>(value.size()));
  for (const unsigned char character : value) {
    hashByte(hash, character);
  }
}

inline bool hashDouble(std::uint64_t & hash, double value)
{
  if (!std::isfinite(value)) {
    return false;
  }
  if (value == 0.0) {
    value = 0.0;
  }
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  hashUint64(hash, bits);
  return true;
}

inline bool hashPose(
  std::uint64_t & hash, const geometry_msgs::msg::PoseStamped & pose,
  const std::string & fallback_frame)
{
  const std::string & frame = pose.header.frame_id.empty() ? fallback_frame : pose.header.frame_id;
  if (frame.empty()) {
    return false;
  }
  hashString(hash, frame);
  if (!hashDouble(hash, pose.pose.position.x) ||
    !hashDouble(hash, pose.pose.position.y) ||
    !hashDouble(hash, pose.pose.position.z))
  {
    return false;
  }

  double qx = pose.pose.orientation.x;
  double qy = pose.pose.orientation.y;
  double qz = pose.pose.orientation.z;
  double qw = pose.pose.orientation.w;
  const double squared_norm = qx * qx + qy * qy + qz * qz + qw * qw;
  if (!std::isfinite(squared_norm) || squared_norm <= 1e-12) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  qx *= inverse_norm;
  qy *= inverse_norm;
  qz *= inverse_norm;
  qw *= inverse_norm;
  const bool negate = qw < 0.0 ||
    (qw == 0.0 && (qz < 0.0 ||
    (qz == 0.0 && (qy < 0.0 || (qy == 0.0 && qx < 0.0)))));
  if (negate) {
    qx = -qx;
    qy = -qy;
    qz = -qz;
    qw = -qw;
  }
  return hashDouble(hash, qx) && hashDouble(hash, qy) &&
         hashDouble(hash, qz) && hashDouble(hash, qw);
}

inline std::uint64_t finishHash(std::uint64_t hash)
{
  return hash == 0U ? 1U : hash;
}

}  // namespace detail

inline std::uint64_t fingerprintPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & fallback_frame = {})
{
  std::uint64_t hash = detail::kFnvOffset;
  detail::hashString(hash, "rbs_handoff_goal_v1");
  if (!detail::hashPose(hash, pose, fallback_frame)) {
    return 0U;
  }
  return detail::finishHash(hash);
}

inline std::uint64_t fingerprintPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty() || path.header.frame_id.empty()) {
    return 0U;
  }
  std::uint64_t hash = detail::kFnvOffset;
  detail::hashString(hash, "rbs_handoff_path_v1");
  detail::hashString(hash, path.header.frame_id);
  detail::hashUint64(hash, static_cast<std::uint64_t>(path.poses.size()));
  for (const auto & pose : path.poses) {
    if (!detail::hashPose(hash, pose, path.header.frame_id)) {
      return 0U;
    }
  }
  return detail::finishHash(hash);
}

inline std::uint64_t fingerprintPathGoal(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return 0U;
  }
  return fingerprintPose(path.poses.back(), path.header.frame_id);
}

}  // namespace pid_tracking_controller::path_identity
#endif
