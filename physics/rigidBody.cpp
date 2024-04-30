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

#include"rigidBody.h"

#include"physicsData.h"

namespace mech {

	struct ColliderTransformer {
		
		void transformConvexHull(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t)
		{ 
			physicsData->convexHullColliders[colliderIndex].transform(t);
		}

		void transformSphere(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t)
		{ 
			physicsData->sphereColliders[colliderIndex].transform(t);
		}

		void transformCapsule(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t)
		{ 
			physicsData->capsuleColliders[colliderIndex].transform(t);
		}

		void transformCompoundCollider(PhysicsData* physicsData, const uint32& colliderIndex, const Transform3D& t)
		{
			for (byte x = 0, len = physicsData->compoundColliders[colliderIndex].components.size(); x < len; ++x) {
				(this->*transformPtrs[(uint32)(physicsData->colliderIdentifiers[physicsData->compoundColliders[colliderIndex].components[x]].type)])(physicsData, physicsData->colliderIdentifiers[physicsData->compoundColliders[colliderIndex].components[x]].colliderIndex, t);
			}

			physicsData->compoundColliders[colliderIndex].bound = physicsData->getColliderAABB(physicsData->compoundColliders[colliderIndex].components[0]);
			for (byte x = 1, len = physicsData->compoundColliders[colliderIndex].components.size(); x < len; ++x) {
			
				const AABB& aabb = physicsData->getColliderAABB(physicsData->compoundColliders[colliderIndex].components[x]);
				physicsData->compoundColliders[colliderIndex].bound.max = maxVec(physicsData->compoundColliders[colliderIndex].bound.max, aabb.max);
				physicsData->compoundColliders[colliderIndex].bound.min = minVec(physicsData->compoundColliders[colliderIndex].bound.min, aabb.min);
			}
		}

		void (ColliderTransformer::*transformPtrs[4]) (PhysicsData*, const uint32&, const Transform3D&);

		ColliderTransformer()
		{
			transformPtrs[0] = &ColliderTransformer::transformConvexHull;
			transformPtrs[1] = &ColliderTransformer::transformSphere;
			transformPtrs[2] = &ColliderTransformer::transformCapsule;
			transformPtrs[3] = &ColliderTransformer::transformCompoundCollider;
		}

		void transform(PhysicsData* physicsData, const uint32& colliderID, const Transform3D& t)
		{
			(this->*transformPtrs[(uint32)(physicsData->colliderIdentifiers[colliderID].type)])(physicsData, physicsData->colliderIdentifiers[colliderID].colliderIndex, t);
		}
	};
	ColliderTransformer colliderTransformer;


	RigidBodySettings rbSettings;
	
	RigidBodySettings* getRigidBodySettings()
	{
		return &rbSettings;
	}

	RigidBody::RigidBody()
	{
		this->motion = rbSettings.maxMotion;
	}

	void RigidBody::update(PhysicsData* physicsData, const decimal& deltaTime)
	{
		BEGIN_PROFILE("RigidBody::update");

		this->deltaPosition += this->linearVelocity * deltaTime;
		this->deltaOrientaion += this->angularVelocity * deltaTime;

		if (this->canSleep()) {

			decimal bias = mathPOW(decimal(0.5), deltaTime);
			this->motion = bias * this->motion + (decimal(1.0) - bias) * (magnitudeSq(this->deltaPosition) + magnitudeSq(this->deltaOrientaion));

			if (this->motion < rbSettings.sleepEpsilon) {
				this->deactivate();
				
				END_PROFILE;
				return;
			}
			else if (this->motion > rbSettings.maxMotion) {
				this->motion = rbSettings.maxMotion;
			}
		}

		this->prevTransform = this->transform;

		this->transform.position += this->deltaPosition;
		this->transform.orientation = normalise(rotationQuaternion(this->deltaOrientaion) * this->transform.orientation);
		
		colliderTransformer.transform(physicsData, this->colliderID, this->transform * getInverse(this->prevTransform));

		this->linearVelocity += (rbSettings.gravity + this->forceAccumulated * this->invMass) * deltaTime;
		this->angularVelocity += (this->invInertiaTensor * this->torqueAccumulated) * deltaTime;
		this->linearVelocity *= mathPOW(rbSettings.linearDamping, deltaTime);
		this->angularVelocity *= mathPOW(rbSettings.angularDamping, deltaTime);

		this->clearForces();

		END_PROFILE;
	}

	void RigidBody::subStep(PhysicsData* physicsData, const decimal& t)
	{
		Transform3D trans = Transform3DRange(this->prevTransform, this->transform).interpolate(t);
		this->transform = trans * this->transform;
		colliderTransformer.transform(physicsData, this->colliderID, trans);
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
		this->motion = rbSettings.leastMotion;
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
		this->motion = rbSettings.maxMotion;
	}

	void RigidBody::clearForces()
	{
		this->forceAccumulated = Vec3();
		this->torqueAccumulated = Vec3();
		this->deltaPosition = Vec3();
		this->deltaOrientaion = Vec3();
	}
}
