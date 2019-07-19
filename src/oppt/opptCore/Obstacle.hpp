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
#ifndef __OPPT_OBSTACLE_HPP__
#define __OPPT_OBSTACLE_HPP__
#include "includes.hpp"
#include "typedefs.hpp"
#include "CollisionReport.hpp"
#include "logging.hpp"
#include "MathUtils.hpp"
#include "geometric/Geometry.hpp"

using std::cout;
using std::endl;

namespace oppt
{

/**
 * Virtual base class for an obstacle
 */
class Obstacle
{
public:
    Obstacle(const std::string& name, const geometric::Pose &worldPose):
        name_(name),
        worldPose_(worldPose),
        opptCollisionObject_(nullptr),
        enabled_(true),
        static_(true),
        collisionGeometry_(nullptr),
        visualGeometry_(nullptr)
    {

    }

    virtual ObstacleSharedPtr clone() const = 0;

    virtual void updateCollisionObject() = 0;

    virtual void initVisualGeometry(const GeometrySharedPtr &visualGeometry) = 0;
    
    /**
     * Check if this obstacle collides with the obstacles provided in otherObstacles
     */
    virtual bool collides(const std::vector<oppt::ObstacleSharedPtr>& otherObstacles) const = 0;

    /**
     * Check if this obstacle collides with the collision objects on collisionObjects
     */
    virtual bool collides(const std::vector<CollisionObjectSharedPtr>& collisionObjects,
                          unsigned int& collidingBodyIndex) const = 0;

    /**
     * Check if the 3D-point collides with this obstacle
     * @param point A 3D-point
     */
    virtual bool collides(const VectorFloat& point) const = 0;

    /**
     * Checks if the obstacle collides with another moving collision object.
     * The motion of of the other collision object is determined by a start
     * and goal transformation
     */
    virtual bool collidesContinuous(const CollisionObjectSharedPtr& collisionObjectStart,
                                    const CollisionObjectSharedPtr& collisionObjectGoal) const = 0;

    /**
    * Computes the smallest distance between the obstacle and the given collision objects
    */
    virtual FloatType distance(const std::vector<oppt::CollisionObjectSharedPtr>& collisionObjects) const = 0;

    void setEnabled(const bool& enabled)
    {
        enabled_ = enabled;
    }

    bool isEnabled() const
    {
        return enabled_;
    }

    void setStatic(const bool& isStatic)
    {
        static_ = isStatic;
    }

    bool isStatic() const
    {
        return static_;
    }

    void serializeToSDF(std::ostream& os) const
    {
        std::string sdfString = toSDFString();
        os << sdfString;
    }

    virtual std::string toSDFString() const = 0;

    OpptCollisionObjectSharedPtr getOpptCollisionObject() const
    {
        return opptCollisionObject_;
    }

    std::string getName()
    {
        return name_;
    }

    GeometrySharedPtr getCollisionGeometry() const
    {
        return collisionGeometry_;
    }

    GeometrySharedPtr getVisualGeometry() const {
        return visualGeometry_;
    }

    void setColor(const VectorFloat& color)
    {
        if (visualGeometry_)
            visualGeometry_->setColor(color);
    }

    VectorFloat getColor() const
    {
        VectorFloat color({0.0, 0.0, 0.0, 1.0});
        if (visualGeometry_)
            color = visualGeometry_->getColor();
        return color;
    }

    /**
     * Set the world pose of this obstacle
     */
    void setWorldPose(const geometric::Pose &pose) {        
        geometric::Pose relativeCollisionPose = collisionGeometry_->getWorldPose() - worldPose_;
        collisionGeometry_->setWorldPose(relativeCollisionPose + pose);
        //geometric::Pose collisionWorldPose(collisionGeometry_->getRelativePose().toGZPose() + pose.toGZPose());        
        //collisionGeometry_->setWorldPose(collisionWorldPose);
        if (visualGeometry_) {
            geometric::Pose relativeVisualPose = visualGeometry_->getWorldPose() - worldPose_;
            visualGeometry_->setWorldPose(relativeVisualPose + pose);
            //geometric::Pose visualWorldPose(visualGeometry_->getRelativePose().toGZPose() + pose.toGZPose());
            //visualGeometry_->setWorldPose(visualWorldPose);
        }      

        worldPose_ = pose;
    }

    /**
     * @brief Get the world pose of this oppt::Obstacle
     *
     * @return The world pose of this obstacle
     */
    geometric::Pose getWorldPose() const {
        return worldPose_;
    }

protected:
    std::string name_;

    bool enabled_;

    bool static_;

    geometric::Pose worldPose_;

    GeometrySharedPtr collisionGeometry_;

    GeometrySharedPtr visualGeometry_;

    oppt::OpptCollisionObjectSharedPtr opptCollisionObject_;

protected:
    virtual void createCollisionObject() = 0;

};

/// @brief Collision data stores the collision request and the result given by collision algorithm.
struct CollisionData {
    CollisionData()
    {
        done = false;
    }

    /// @brief Collision request
    fcl::CollisionRequest request;

    /// @brief Collision result
    fcl::CollisionResult result;

    /// @brief Whether the collision iteration can stop
    bool done;

    std::vector<fcl::CollisionObject*> o1;

    std::vector<fcl::CollisionObject*> o2;
};

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata);

}

#endif
