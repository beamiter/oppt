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
#include "include/BoxObstacle.hpp"
#include "oppt/opptCore/geometric/Box.hpp"
#include "oppt/opptCore/CollisionObject.hpp"
#include <iostream>

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;

namespace oppt
{

BoxObstacle::BoxObstacle(const std::string &name,
                         const geometric::Pose& worldPose,
                         const VectorFloat& dimensions):
    ObstacleImpl(name, worldPose)
{
    collisionGeometry_ = std::make_shared<geometric::Box>(name, dimensions, worldPose);
    createCollisionObject();
}

ObstacleSharedPtr BoxObstacle::clone() const
{
    VectorFloat dimensions = static_cast<geometric::Box*>(collisionGeometry_.get())->getDimensions();
    VectorFloat color = getColor();
    ObstacleSharedPtr clonedObstacle = std::make_shared<BoxObstacle>(name_,
                                       worldPose_,
                                       dimensions);
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
