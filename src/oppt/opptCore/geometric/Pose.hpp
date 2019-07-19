#ifndef _OPPT_POSE_HPP_
#define _OPPT_POSE_HPP_
#include "oppt/opptCore/typedefs.hpp"
#include "oppt/opptCore/MathUtils.hpp"

namespace oppt {
namespace geometric {
struct Pose {
	Pose():
		position(0.0, 0.0, 0.0),
		orientation(1.0, 0.0, 0.0, 0.0) {
	}

	Pose(const Vector3f &position_, const Quaternionf &orientation_):
		position(position_),
		orientation(orientation_) {

	}

	Pose (const FloatType &x, const FloatType &y, const FloatType &z, const FloatType &oX, const FloatType &oY, const FloatType &oZ):
		position(x, y, z),
		orientation(oppt::math::eulerAnglesToQuaternion(VectorFloat( {oX, oY, oZ}))) {

	}

	Pose(const Pose &other):
		position(other.position),
		orientation(other.orientation) {
	}

	Pose (const GZPose &gzPose):
		position(gzPose.Pos().X(), gzPose.Pos().Y(), gzPose.Pos().Z()),
		orientation(gzPose.Rot().W(), gzPose.Rot().X(), gzPose.Rot().Y(), gzPose.Rot().Z()) {
	}

	Pose (const Vector3f &position_, const Vector3f &orientation_):
		position(position_),
		orientation(oppt::math::eulerAnglesToQuaternion(orientation_.x(),
		            orientation_.y(),
		            orientation_.z())) {
	}

	Pose &operator=(const Pose &other) {
		if (&other == this) {
			return *this;
		}

		position = other.position;
		orientation = other.orientation;

		return *this;
	}

	Pose operator+(const Pose &other) {
		return Pose(toGZPose() + other.toGZPose());
	}

	Pose operator-(const Pose &other) {
		return Pose(toGZPose() - other.toGZPose());
	}

	GZPose toGZPose() const {
		return GZPose(position.x(), position.y(), position.z(), orientation.w(), orientation.x(), orientation.y(), orientation.z());
	}

	VectorFloat toStdVecEuler() const {
		Vector3f euler = oppt::math::quaternionToEulerAngles(orientation);
		VectorFloat vec({position.x(), position.y(), position.z(), euler.x(), euler.y(), euler.z()});
		return vec;
	}

	friend std::ostream& operator<< (std::ostream& out, const Pose& pose) {
		out << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ", ";
		out << pose.orientation.w() << ", " << pose.orientation.x() << ", " << pose.orientation.y() << ", " << pose.orientation.z();
		return out;
	}

	Vector3f position;
	Quaternionf orientation;
};

/** @brief A std::shared_ptr to oppt::geometric::Pose*/
typedef std::shared_ptr<geometric::Pose> PoseSharedPtr;
/** @brief A std::unique_ptr to oppt::geometric::Pose*/
typedef std::unique_ptr<geometric::Pose> PoseUniquePtr;
}
}
#endif