/*
	MIT License

	Copyright (c) 2024 Ssebunya Umar

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/


//@ssebunya_umar - X(twitter)

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include"collision/collider.h"
#include"../math/transform.h"

namespace mech {

	class PhysicsData;

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct RigidBodySettings {

		Vec3 gravity = Vec3(decimal(0.0), decimal(-9.8), decimal(0.0));
		decimal linearDamping = decimal(0.5);
		decimal angularDamping = decimal(0.5);
		decimal sleepEpsilon = decimal(0.00001);
		decimal maxMotion = sleepEpsilon * decimal(10.0);
		decimal leastMotion = sleepEpsilon * decimal(1.2);
	};

	RigidBodySettings* getRigidBodySettings();

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct RigidBody {

		/*
			------rigid body flags---------
			body can go to sleep            - 0b00000001
			body is active                  - 0b00000010
		*/

		Transform3D transform;
		Transform3D prevTransform;
		Mat3x3 invInertiaTensor;
		Vec3 linearVelocity;
		Vec3 angularVelocity;
		Vec3 forceAccumulated;
		Vec3 torqueAccumulated;
		Vec3 deltaPosition;
		Vec3 deltaOrientaion;
		decimal motion = decimal(0.0);
		decimal invMass = decimal(0.0);
		uint32 colliderID = -1;
		byte flags = 0b00000011;

		RigidBody();
		~RigidBody() {}

		void update(PhysicsData* physicsData, const decimal& deltaTime);
		void subStep(PhysicsData* physicsData, const decimal& t);
		void addForce(const Vec3& force);
		void addForceAtPoint(const Vec3& force, const Vec3& point);
		void updatePositionAndOrientaion(const Vec3& deltaPos, const Vec3& deltaOrient);
		void updateLinearAndAngularVelocity(const Vec3& deltaLinVel, const Vec3& deltaAngVel);
		void activate();
		void deactivate();
		void setMotionToMax();
		void clearForces();

		bool isActive() { return this->flags & 0b00000010; }
		bool canSleep() { return this->flags & 0b00000001; }
	
		Mat4x4 getTransformMatrix() const { return this->transform.toMatrix(); }
		Vec3 getDisplacement() { return this->transform.position - this->prevTransform.position; }

		void setTransform(const Transform3D& transform) { this->transform = transform; }
		void setMass(const decimal& mass) { this->invMass = decimal(1.0) / mass; }
		void setTensor(const Mat3x3& tensor) { this->invInertiaTensor = getInverse(tensor); }
	};
}

#endif
