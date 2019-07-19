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
#include "include/MeshObstacle.hpp"
#include "oppt/opptCore/geometric/Mesh.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{
MeshObstacle::MeshObstacle(const std::string &name,
                           const VectorString& meshFiles,
                           const geometric::Pose& worldPose,
                           const VectorFloat& scale,
                           const bool &loadFromFile):
  ObstacleImpl(name, worldPose),
  meshFiles_(meshFiles),
  identity_transform_(),
  scale_( {1.0, 1.0, 1.0})
{
  collisionGeometry_ = std::make_shared<geometric::Mesh>(name, meshFiles[0], worldPose, scale);
  createCollisionObject();
}

ObstacleSharedPtr MeshObstacle::clone() const
{
  VectorString clonedMeshFiles = meshFiles_;
  VectorFloat clonedScale = scale_;  
  VectorFloat color = getColor();
  ObstacleSharedPtr clonedObstacle = std::make_shared<MeshObstacle>(name_,
                                     clonedMeshFiles,
                                     worldPose_,
                                     clonedScale,
                                     true);
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

void MeshObstacle::removeVertices(std::vector<VectorFloat>& vertices)
{
  createCollisionObject();
}

}
