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
	
	void RigidBody::update(PhysicsData* physicsData, const decimal& deltaTime)
	{
		this->deltaPosition += this->linearVelocity * deltaTime;
		this->deltaOrientaion += this->angularVelocity * deltaTime;

		this->prevTransform3D = this->transform;
		
		this->transform.position += this->deltaPosition;
		this->transform.orientation = normalise(rotationQuaternion(this->deltaOrientaion) * this->transform.orientation);
		this->transformCollider(physicsData, this->transform.toMatrix() * getInverse(this->prevTransform3D.toMatrix()));

		if (this->flags & 0b00000001) {

			decimal bias = mathPOW(decimal(0.5), deltaTime);
			this->motion = bias * this->motion + (decimal(1.0) - bias) * (magnitudeSq(this->deltaPosition) + magnitudeSq(this->deltaOrientaion));

			if (this->motion < physicsData->sleepEpsilon) {

				this->linearVelocity = Vec3();
				this->angularVelocity = Vec3();
				this->flags &= 0b11111101;

				return;
			}
			else if (this->motion > physicsData->maxMotion) {
				this->motion = physicsData->maxMotion;
			}
		}

		this->deltaPosition = Vec3();
		this->deltaOrientaion = Vec3();

		this->linearVelocity += (physicsData->gravity + this->forceAccumulated * this->invMass) * deltaTime;
		this->angularVelocity += (this->invInertiaTensor * this->torqueAccumulated) * deltaTime;

		this->linearVelocity *= mathPOW(physicsData->linearDamping, deltaTime);
		this->angularVelocity *= mathPOW(physicsData->angularDamping, deltaTime);

		this->forceAccumulated = Vec3();
		this->torqueAccumulated = Vec3();
	}

	void RigidBody::subStep(PhysicsData* physicsData, const decimal& t)
	{
		Transform3D prev = this->transform;
		this->transform = Transform3DPair(this->prevTransform3D, this->transform).interpolate(t);
		this->transformCollider(physicsData, getInverse(prev.toMatrix()) * this->transform.toMatrix());
	}

	void RigidBody::activate(PhysicsData* physicsData)
	{
		struct ActivateInternal {
			static void fcn(RigidBody& rigidBody, const decimal& baseMotion)
			{
				rigidBody.motion = baseMotion;
				rigidBody.flags |= 0b00000010;
			}
		};

		if (isAValidIndex(this->islandIndex)) {
			for (auto it = physicsData->islands[this->islandIndex].begin(), end = physicsData->islands[this->islandIndex].end(); it != end; ++it) {
				ActivateInternal::fcn(physicsData->physicsObjects[it.data().second.objectIndex].rigidBody, physicsData->baseMotion);
			}
		}
		else {
			ActivateInternal::fcn(*this, physicsData->baseMotion);
		}
	}

	void RigidBody::addForce(PhysicsData* physicsData, const Vec3& force)
	{
		this->forceAccumulated += force;
		this->activate(physicsData);
	}

	void RigidBody::addForceAtPoint(PhysicsData* physicsData, const Vec3& force, const Vec3& point)
	{
		this->forceAccumulated += force;
		this->torqueAccumulated += crossProduct(point - this->transform.position, force);
		this->activate(physicsData);
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

	void RigidBody::transformCollider(PhysicsData* physicsData, const Mat4x4& mat)
	{
		if (this->colliderType == ColliderType::convexHull) {
			physicsData->convexHullColliders[this->colliderIndex].transform(mat);
		}
		else if (this->colliderType == ColliderType::sphere) {
			physicsData->sphereColliders[this->colliderIndex].transform(mat);
		}
		else if (this->colliderType == ColliderType::capsule) {
			physicsData->capsuleColliders[this->colliderIndex].transform(mat);
		}
	}
}
