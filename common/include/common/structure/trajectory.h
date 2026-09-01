/**
 * *********************************************************
 *
 * @file: trajectory.h
 * @brief: Trajectory with time, position, velocity and acceletation
 * @author: Yang Haodong
 * @date: 2024-10-03
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_STRUCTURE_TRAJECTORY_H_
#define RMP_COMMON_STRUCTURE_TRAJECTORY_H_

#include "common/geometry/point.h"

namespace rmp
{
namespace common
{
namespace structure
{
template <typename Point>
class Trajectory
{
public:
  Trajectory(int size = 0) : size_(size)
  {
    time.reserve(size);
    position.reserve(size);
    velocity.reserve(size);
    acceletation.reserve(size);
  }

  ~Trajectory()
  {
    reset();
  }

  void clear()
  {
    time.clear();
    position.clear();
    velocity.clear();
    acceletation.clear();
  }
  void reset(int size = 0)
  {
    size_ = size;

    if (!time.empty())
    {
      std::vector<double>().swap(time);
    }

    if (!position.empty())
    {
      std::vector<Point>().swap(position);
    }

    if (!velocity.empty())
    {
      std::vector<Point>().swap(velocity);
    }

    if (!acceletation.empty())
    {
      std::vector<Point>().swap(acceletation);
    }

    if (size > 0)
    {
      time.reserve(size);
      position.reserve(size);
      velocity.reserve(size);
      acceletation.reserve(size);
    }
  }

public:
  std::vector<double> time;
  std::vector<Point> position;
  std::vector<Point> velocity;
  std::vector<Point> acceletation;

private:
  int size_;
};

using Trajectory2d = Trajectory<rmp::common::geometry::Point2d>;
using Trajectory3d = Trajectory<rmp::common::geometry::Point3d>;

}  // namespace structure
}  // namespace common
}  // namespace rmp

#endif