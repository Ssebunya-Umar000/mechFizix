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

//https://t.me/mechFizix - Telegram (if you have a question, remark or complaint)
//@ssebunya_umar - X(twitter)

#include"rigidBody.h"

#include"physicsData.h"

namespace mech {

	//////////////////////////////////////////////////////////////////////////////////////////////////
	void transformConvexHull(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t) { physicsData->convexHullColliders[colliderIndex].transform(t); }
	void transformSphere(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t) { physicsData->sphereColliders[colliderIndex].transform(t); }
	void transformCapsule(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t) { physicsData->capsuleColliders[colliderIndex].transform(t); }

	void (*transformFunctionPointers[3]) (PhysicsData*, const uint32&, const Transform3D&) = { transformConvexHull, transformSphere, transformCapsule };
	//////////////////////////////////////////////////////////////////////////////////////////////////

	RigidBodyConfigurations rbConfigurations;
	
	RigidBodyConfigurations* getRigidBodyConfigurations()
	{
		return &rbConfigurations;
	}

	RigidBody::RigidBody()
	{
		this->motion = rbConfigurations.maxMotion;
	}

	void RigidBody::update(PhysicsData* physicsData, const decimal& deltaTime)
	{
		this->deltaPosition += this->linearVelocity * deltaTime;
		this->deltaOrientaion += this->angularVelocity * deltaTime;

		if (this->canSleep()) {

			decimal bias = mathPOW(decimal(0.5), deltaTime);
			this->motion = bias * this->motion + (decimal(1.0) - bias) * (magnitudeSq(this->deltaPosition) + magnitudeSq(this->deltaOrientaion));

			if (this->motion < rbConfigurations.sleepEpsilon) {
				this->deactivate();
				return;
			}
			else if (this->motion > rbConfigurations.maxMotion) {
				this->motion = rbConfigurations.maxMotion;
			}
		}

		this->prevTransform = this->transform;
		this->transform.position += this->deltaPosition;
		this->transform.orientation = normalise(rotationQuaternion(this->deltaOrientaion) * this->transform.orientation);
		transformFunctionPointers[(uint32)(this->colliderType)](physicsData, this->colliderIndex, this->transform * getInverse(this->prevTransform));

		this->linearVelocity += (rbConfigurations.gravity + this->forceAccumulated * this->invMass) * deltaTime;
		this->angularVelocity += (this->invInertiaTensor * this->torqueAccumulated) * deltaTime;
		this->linearVelocity *= mathPOW(rbConfigurations.linearDamping, deltaTime);
		this->angularVelocity *= mathPOW(rbConfigurations.angularDamping, deltaTime);

		this->clearForces();
	}

	void RigidBody::subStep(PhysicsData* physicsData, const decimal& t)
	{
		Transform3D trans = Transform3DRange(this->prevTransform, this->transform).interpolate(t);
		this->transform = trans * this->transform;
		transformFunctionPointers[(uint32)(this->colliderType)](physicsData, this->colliderIndex, trans);
	}

	void RigidBody::addForce(const Vec3& force)
	{
		this->forceAccumulated += force;
		this->activate();
	}

	void RigidBody::addForceAtPoint(const Vec3& force, const Vec3& point)
	{
		this->forceAccumulated += force;
		this->torqueAccumulated += crossProduct(point - this->transform.position, force);
		this->activate();
	}

	void RigidBody::updatePositionAndOrientaion(const Vec3& deltaPos, const Vec3& deltaOrient)
	{
		this->deltaPosition += deltaPos;
		this->deltaOrientaion += deltaOrient;
	}

	void RigidBody::updateLinearAndAngularVelocity(const Vec3& deltaLinVel, const Vec3& deltaAngVel)
	{
		this->linearVelocity += deltaLinVel;
		this->angularVelocity += deltaAngVel;
	}

	void RigidBody::activate()
	{
		if (this->isActive()) return;
		this->motion = rbConfigurations.leastMotion;
		this->flags |= 0b00000010;
	}

	void RigidBody::deactivate()
	{
		this->linearVelocity = Vec3();
		this->angularVelocity = Vec3();
		this->clearForces();
		this->flags &= 0b11111101;
	}

	void RigidBody::setMotionToMax()
	{
		this->motion = rbConfigurations.maxMotion;
	}

	void RigidBody::clearForces()
	{
		this->forceAccumulated = Vec3();
		this->torqueAccumulated = Vec3();
		this->deltaPosition = Vec3();
		this->deltaOrientaion = Vec3();
	}
}
