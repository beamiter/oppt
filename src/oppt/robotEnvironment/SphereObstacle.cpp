/**
 * Copyright 2017
 *
 * This file is part of On-line POMDP Planning Toolkit (OPPT).
 * OPPT is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License published by the Free Software Foundation,
 * either version 2 of the License, or (at your option) any later version.
 *
 * OPPT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with OPPT.
 * If not, see http://www.gnu.org/licenses/.
 */
#include "include/SphereObstacle.hpp"
#include "oppt/opptCore/geometric/Sphere.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using namespace fcl;

namespace oppt
{

SphereObstacle::SphereObstacle(const std::string &name,
                               const geometric::Pose& worldPose,
                               const FloatType &radius):
  ObstacleImpl(name, worldPose),
  radius_(radius)
{
  collisionGeometry_ = std::make_shared<geometric::Sphere>(name, radius, worldPose);
  createCollisionObject();
}

ObstacleSharedPtr SphereObstacle::clone() const
{
  VectorFloat color = getColor();
  ObstacleSharedPtr clonedObstacle = std::make_shared<SphereObstacle>(name_,
                                     worldPose_,
                                     radius_);
  clonedObstacle->setStatic(static_);
  clonedObstacle->setEnabled(enabled_);
  clonedObstacle->setColor(color);
  auto clonedCollisionGeometry = clonedObstacle->getCollisionGeometry();
  clonedCollisionGeometry->setWorldPose(getCollisionGeometry()->getWorldPose());
  if (visualGeometry_) {
    auto clonedVisualGeometry = visualGeometry_->copy();
    clonedObstacle->initVisualGeometry(clonedVisualGeometry);
  }
  return clonedObstacle;
}
}