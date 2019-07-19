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
#ifndef _SDF_ENVIRONMENT_PARSER_HPP_
#define _SDF_ENVIRONMENT_PARSER_HPP_
#include "oppt/opptCore/core.hpp"
#include <sdf/parser.hh>

namespace oppt
{
class SDFEnvironmentParser
{
public:
  SDFEnvironmentParser();

  VectorObstaclePtr parseObstaclesFromSDFString(const std::string& sdfString) const;

  VectorObstaclePtr parseObstaclesFromFile(std::string& filename, const std::string& robotName) const;

private:
  VectorFloat toDoubleVec(const VectorString& stringVec) const;

  geometric::Pose processPoseElement(const sdf::ElementPtr& poseElement) const;

  std::pair<GeometrySharedPtr, VectorFloat> processVisualElement(const sdf::ElementPtr& visualElement, const geometric::Pose &parentLinkPose) const;

  ObstacleSharedPtr processLinkElement(const std::string& modelName,
                                       const sdf::ElementPtr& poseElement,
                                       const sdf::ElementPtr& linkElement) const;

  bool processBoxElement(const std::string& name,
                         const sdf::ElementPtr& boxElement,
                         const geometric::Pose& parentLinkPose,
                         const sdf::ElementPtr& collisionElement,
                         const sdf::ElementPtr& visualElement,
                         bool& enabled,
                         VectorObstaclePtr& obstacles) const;

  bool processMeshElement(const std::string& name,
                          const sdf::ElementPtr& meshElement,
                          const geometric::Pose& parentLinkPose,
                          const sdf::ElementPtr& collisionElement,
                          const sdf::ElementPtr& visualElement,
                          bool& enabled,
                          VectorObstaclePtr& obstacles) const;

  bool processSphereElement(const std::string& name,
                            const sdf::ElementPtr& sphereElement,
                            const geometric::Pose& parentLinkPose,
                            const sdf::ElementPtr& collisionElement,
                            const sdf::ElementPtr& visualElement,
                            bool& enabled,
                            VectorObstaclePtr& obstacles) const;

  bool processCylinderElement(const std::string& name,
                              const sdf::ElementPtr& cylinderElement,
                              const geometric::Pose& parentLinkPose,
                              const sdf::ElementPtr& collisionElement,
                              const sdf::ElementPtr& visualElement,
                              bool& enabled,
                              VectorObstaclePtr& obstacles) const;

  bool processStateElement(const sdf::ElementPtr &stateElement, const std::string& robotName, VectorObstaclePtr &obstacles) const;

  bool processObstacleState(const std::string &modelName, const geometric::Pose &parentModelPose, const sdf::ElementPtr &linkElement, VectorObstaclePtr &obstacles) const;

  GeometrySharedPtr makeBoxGeometry(const sdf::ElementPtr &geometryElement, const geometric::Pose &visualPose) const;

  GeometrySharedPtr makeCylinderGeometry(const sdf::ElementPtr &geometryElement, const geometric::Pose &visualPose) const;

  GeometrySharedPtr makeSphereGeometry(const sdf::ElementPtr &geometryElement, const geometric::Pose &visualPose) const;

  GeometrySharedPtr makeMeshGeometry(const sdf::ElementPtr &geometryElement, const geometric::Pose &visualPose) const;

  geometric::Pose getCollisionPose(const sdf::ElementPtr &collisionElement, const geometric::Pose &parentLinkPose) const;
};
}

#endif
