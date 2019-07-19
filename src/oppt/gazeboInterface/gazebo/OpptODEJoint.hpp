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
#ifndef _OPPT_ODE_JOINT_HPP_
#define _OPPT_ODE_JOINT_HPP_
#include "OpptJoint.hpp"
#include "opende/src/joints/hinge.h"
#include "opende/src/joints/ball.h"
#include "opende/src/joints/slider.h"
#include "opende/src/joints/screw.h"
#include "opende/src/joints/gearbox.h"
#include "opende/src/joints/universal.h"
#include "opende/src/joints/fixed.h"

namespace gazebo
{
namespace physics
{

template<class JointType, class ODEJointType>
class OpptODEJoint: public JointType, public OpptJoint
{
public:
    OpptODEJoint(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }

    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }

    virtual std::vector<double> getCumulativeAngles() const override {
        return std::vector<double>();
    }

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override {

    }

    virtual std::string getName() const override {
        return this->GetName();
    }
};

template<class JointType, class ODEJointType>
class OpptODEJointExtended: public JointType, public OpptJoint
{
public:
    OpptODEJointExtended(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }

    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }

    virtual std::vector<double> getCumulativeAngles() const override {
        return std::vector<double>({static_cast<ODEJointType* const>(this->jointId)->cumulative_angle});
    }

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override {
        static_cast<ODEJointType*>(this->jointId)->cumulative_angle = cumulativeAngles[0];
    }

    virtual std::string getName() const override {
        return this->GetName();
    }

    virtual JointWrench GetForceTorque(unsigned int _index) override {
        // Note that:
        // f2, t2 are the force torque measured on parent body's cg
        // f1, t1 are the force torque measured on child body's cg        
        dJointFeedback *fb = this->GetFeedback();
        if (fb)
        {
            // kind of backwards here, body1 (parent) corresponds go f2, t2
            // and body2 (child) corresponds go f1, t1
            this->wrench.body2Force.Set(fb->f1[0], fb->f1[1], fb->f1[2]);
            this->wrench.body2Torque.Set(fb->t1[0], fb->t1[1], fb->t1[2]);
            this->wrench.body1Force.Set(fb->f2[0], fb->f2[1], fb->f2[2]);
            this->wrench.body1Torque.Set(fb->t2[0], fb->t2[1], fb->t2[2]);

            // get force applied through SetForce
            physics::JointWrench wrenchAppliedWorld;
            if (this->HasType(physics::Base::HINGE_JOINT))
            {
                // rotate force into child link frame
                // LocalAxis is the axis specified in parent link frame!!!
                wrenchAppliedWorld.body2Torque =
                    this->GetForce(0u) * this->GlobalAxis(0u);

                // gzerr << "body2Torque [" << wrenchAppliedWorld.body2Torque
                //       << "] axis [" << this->LocalAxis(0u)
                //       << "]\n";

                wrenchAppliedWorld.body1Torque = -wrenchAppliedWorld.body2Torque;
            }
            else if (this->HasType(physics::Base::SLIDER_JOINT))
            {
                // rotate force into child link frame
                wrenchAppliedWorld.body2Force =
                    this->GetForce(0u) * this->GlobalAxis(0u);
                wrenchAppliedWorld.body1Force = -wrenchAppliedWorld.body2Force;
            }
            else if (this->HasType(physics::Base::FIXED_JOINT))
            {
                // no correction are necessary for fixed joint
            }
            else
            {
                /// \TODO: fix for multi-axis joints
                gzerr << "force torque for joint type [" << this->GetType()
                      << "] not implemented, returns false results!!\n";
            }

            // add wrench in world frame before transform into child and parent frame
            this->wrench = this->wrench + wrenchAppliedWorld;

            // convert wrench from child cg location to child link frame
            if (this->childLink)
            {
                ignition::math::Pose3d childPose = this->childLink->WorldPose();

                // convert torque from about child CG to joint anchor location
                // cg position specified in child link frame
                ignition::math::Pose3d cgPose;
                auto inertial = this->childLink->GetInertial();
                if (inertial)
                    cgPose = inertial->Pose();

                // anchorPose location of joint in child frame
                // childMomentArm: from child CG to joint location in child link frame
                // moment arm rotated into world frame (given feedback is in world frame)
                ignition::math::Vector3d childMomentArm = childPose.Rot().RotateVector(
                            (this->anchorPose -
                             ignition::math::Pose3d(cgPose.Pos(),
                                                    ignition::math::Quaterniond::Identity)).Pos());

                // gzerr << "anchor [" << anchorPose
                //       << "] iarm[" << this->childLink->GetInertial()->GetPose().Pos()
                //       << "] childMomentArm[" << childMomentArm
                //       << "] f1[" << this->wrench.body2Force
                //       << "] t1[" << this->wrench.body2Torque
                //       << "] fxp[" << this->wrench.body2Force.Cross(childMomentArm)
                //       << "]\n";

                this->wrench.body2Torque += this->wrench.body2Force.Cross(childMomentArm);

                // rotate resulting body2Force in world frame into link frame
                this->wrench.body2Force = childPose.Rot().RotateVectorReverse(
                                              -this->wrench.body2Force);

                // rotate resulting body2Torque in world frame into link frame
                this->wrench.body2Torque = childPose.Rot().RotateVectorReverse(
                                               -this->wrench.body2Torque);
            }

            // convert torque from about parent CG to joint anchor location
            if (this->parentLink)
            {
                // get child pose, or it's the inertial world if childLink is nullptr
                ignition::math::Pose3d childPose;
                if (this->childLink)
                    childPose = this->childLink->WorldPose();
                else
                    gzerr << "missing child link, double check model.";

                ignition::math::Pose3d parentPose = this->parentLink->WorldPose();

                // if parent link exists, convert torque from about parent
                // CG to joint anchor location

                // parent cg specified in parent link frame
                ignition::math::Pose3d cgPose;
                auto inertial = this->parentLink->GetInertial();
                if (inertial)
                    cgPose = inertial->Pose();

                // get parent CG pose in child link frame
                ignition::math::Pose3d parentCGInChildLink =
                    ignition::math::Pose3d(cgPose.Pos(),
                                           ignition::math::Quaterniond::Identity) - (childPose - parentPose);

                // paretnCGFrame in world frame
                ignition::math::Pose3d parentCGInWorld = cgPose + parentPose;

                // rotate momeent arms into world frame
                ignition::math::Vector3d parentMomentArm =
                    parentCGInWorld.Rot().RotateVector(
                        (this->anchorPose - parentCGInChildLink).Pos());

                // gzerr << "anchor [" << this->anchorPose
                //       << "] pcginc[" << parentCGInChildLink
                //       << "] iarm[" << cgPose
                //       << "] anc2pcg[" << this->anchorPose - parentCGInChildLink
                //       << "] parentMomentArm[" << parentMomentArm
                //       << "] f1[" << this->wrench.body1Force
                //       << "] t1[" << this->wrench.body1Torque
                //       << "] fxp[" << this->wrench.body1Force.Cross(parentMomentArm)
                //       << "]\n";

                this->wrench.body1Torque +=
                    this->wrench.body1Force.Cross(parentMomentArm);

                // rotate resulting body1Force in world frame into link frame
                this->wrench.body1Force = parentPose.Rot().RotateVectorReverse(
                                              -this->wrench.body1Force);

                // rotate resulting body1Torque in world frame into link frame
                this->wrench.body1Torque = parentPose.Rot().RotateVectorReverse(
                                               -this->wrench.body1Torque);

                if (!this->childLink)
                {
                    gzlog << "Joint [" << this->GetScopedName()
                          << "] with parent Link [" << this->parentLink->GetScopedName()
                          << "] but no child Link.  Child Link must be world.\n";
                    // if child link does not exist, use equal and opposite
                    this->wrench.body2Force = -this->wrench.body1Force;
                    this->wrench.body2Torque = -this->wrench.body1Torque;

                    // force/torque are in parent link frame, transform them into
                    // child link(world) frame.
                    auto parentToWorldTransform = this->parentLink->WorldPose();
                    this->wrench.body1Force =
                        parentToWorldTransform.Rot().RotateVector(
                            this->wrench.body1Force);
                    this->wrench.body1Torque =
                        parentToWorldTransform.Rot().RotateVector(
                            this->wrench.body1Torque);
                }
            }
            else
            {
                if (!this->childLink)
                {
                    gzerr << "Both parent and child links are invalid, abort.\n";
                    return JointWrench();
                }
                else
                {
                    gzlog << "Joint [" << this->GetScopedName()
                          << "] with child Link [" << this->childLink->GetScopedName()
                          << "] but no parent Link.  Parent Link must be world.\n";
                    // if parentLink does not exist, use equal opposite body1 wrench
                    this->wrench.body1Force = -this->wrench.body2Force;
                    this->wrench.body1Torque = -this->wrench.body2Torque;

                    // force/torque are in child link frame, transform them into
                    // parent link frame.  Here, parent link is world, so zero transform.
                    auto childToWorldTransform = this->childLink->WorldPose();
                    this->wrench.body1Force =
                        childToWorldTransform.Rot().RotateVector(
                            this->wrench.body1Force);
                    this->wrench.body1Torque =
                        childToWorldTransform.Rot().RotateVector(
                            this->wrench.body1Torque);
                }
            }
            //this->wrench = this->wrench - wrenchAppliedWorld;
        }
        else
        {
            // forgot to set provide_feedback?
            gzwarn << "GetForceTorque: forgot to set <provide_feedback>?\n";
        }

        return this->wrench;
    }

};

template<class JointType, class ODEJointType>
class OpptODEJointExtended2: public JointType, public OpptJoint
{
public:
    OpptODEJointExtended2(dWorldID _worldId, BasePtr _parent):
        JointType(_worldId, _parent),
        OpptJoint() {

    }

    virtual void SetForceImpl(unsigned int _index, double _effort) override {
        if (!blockSetForce_)
            JointType::SetForceImpl(_index, _effort);
    }

    virtual std::vector<double> getCumulativeAngles() const override {
        return std::vector<double>({static_cast<ODEJointType* const>(this->jointId)->cumulative_angle1,
                                    static_cast<ODEJointType* const>(this->jointId)->cumulative_angle2
                                   });
    }

    virtual void setCumulativeAngles(const std::vector<double> &cumulativeAngles) const override {
        static_cast<ODEJointType*>(this->jointId)->cumulative_angle1 = cumulativeAngles[0];
        static_cast<ODEJointType*>(this->jointId)->cumulative_angle2 = cumulativeAngles[1];
    }

    virtual std::string getName() const override {
        return this->GetName();
    }
};

}
}

#endif
