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

#include"physicsWorld.h"

namespace mech {

#define LARGE_VELOCITY_SQ decimal(4000.0)

	PhysicsWorld::PhysicsWorld()
	{
		this->mConstraintSolver.physicsData = &this->mPhysicsData;
		this->mBroadPhase.narrowPhase.physicsData = &this->mPhysicsData;
		this->mCacheManager.physicsData = &this->mPhysicsData;
	}
	
	void PhysicsWorld::update(const decimal& deltaTime)
	{
		for (auto it = this->mPhysicsData.physicsObjects.begin(), end = this->mPhysicsData.physicsObjects.end(); it != end;) {

			auto temp = it;
			++it;

			if (temp.data().isTerminated()) {
				this->mPhysicsData.physicsObjects.eraseDataAtIndex(temp.index());
			}
			else {
				if (temp.data().rigidBody.isActive()) {
				
					temp.data().rigidBody.update(&this->mPhysicsData, deltaTime);

					if (magnitudeSq(temp.data().rigidBody.linearVelocity + temp.data().rigidBody.angularVelocity) >= LARGE_VELOCITY_SQ) {
						this->mBroadPhase.continousCollisionDetection(temp.data());
					}
					else {
						this->mPhysicsData.octree.repositionCollider(&this->mPhysicsData, temp.data().getIdentifier());
					}
				}
			}
		}

		this->mBroadPhase.discreteCollisionDetection();
		this->mConstraintSolver.solve(deltaTime);
		this->mCacheManager.update();
	}

	void PhysicsWorld::initialiseOctree(const AABB& bounds, const byte& depth)
	{
		this->mPhysicsData.octree.initialise(bounds, depth);
	}

	void PhysicsWorld::initialiseHeightField(const HeightField::Parameters& parameters, const PhysicsMaterial& material)
	{
		this->mPhysicsData.heightFieldCollider.collider.initialise(parameters);
		this->mPhysicsData.heightFieldCollider.properties.restitution = material.restitution;
		this->mPhysicsData.heightFieldCollider.properties.frictionSqrt = material.frictionSqrt;
	}

	uint32 PhysicsWorld::add(const Sphere& sphere, const PhysicsMaterial& material, const Vec3& pos)
	{
		uint32 colliderIndex = this->mPhysicsData.sphereColliders.insert(SphereCollider(sphere));
		uint32 objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
		
		this->mPhysicsData.sphereColliders[colliderIndex].properties.restitution = material.restitution;
		this->mPhysicsData.sphereColliders[colliderIndex].properties.frictionSqrt = material.frictionSqrt;

		this->mPhysicsData.sphereColliders[colliderIndex].identifier.colliderIndex = colliderIndex;
		this->mPhysicsData.sphereColliders[colliderIndex].identifier.objectIndex = objectIndex;

		this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, this->mPhysicsData.sphereColliders[colliderIndex].identifier, material.density, pos);

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.sphereColliders[colliderIndex].identifier);

		return objectIndex;
	}

	uint32 PhysicsWorld::add(const Capsule& capsule, const PhysicsMaterial& material, const Vec3& pos)
	{
		uint32 colliderIndex = this->mPhysicsData.capsuleColliders.insert(CapsuleCollider(capsule));
		uint32 objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());

		this->mPhysicsData.capsuleColliders[colliderIndex].properties.restitution = material.restitution;
		this->mPhysicsData.capsuleColliders[colliderIndex].properties.frictionSqrt = material.frictionSqrt;

		this->mPhysicsData.capsuleColliders[colliderIndex].identifier.colliderIndex = colliderIndex;
		this->mPhysicsData.capsuleColliders[colliderIndex].identifier.objectIndex = objectIndex;

		this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, this->mPhysicsData.capsuleColliders[colliderIndex].identifier, material.density, pos);

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.capsuleColliders[colliderIndex].identifier);

		return objectIndex;
	}

	uint32 PhysicsWorld::add(const ConvexHull& convexHull, const PhysicsMaterial& material, const Vec3& pos)
	{
		uint32 colliderIndex = this->mPhysicsData.convexHullColliders.insert(ConvexHullCollider(convexHull));
		uint32 objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());

		this->mPhysicsData.convexHullColliders[colliderIndex].properties.restitution = material.restitution;
		this->mPhysicsData.convexHullColliders[colliderIndex].properties.frictionSqrt = material.frictionSqrt;

		this->mPhysicsData.convexHullColliders[colliderIndex].identifier.colliderIndex = colliderIndex;
		this->mPhysicsData.convexHullColliders[colliderIndex].identifier.objectIndex = objectIndex;

		this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[colliderIndex].identifier, material.density, pos);

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[colliderIndex].identifier);

		return objectIndex;
	}

	void PhysicsWorld::add(const Sphere& sphere, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.sphereColliders.insert(SphereCollider(sphere));
		this->mPhysicsData.sphereColliders[index].properties.restitution = material.restitution;
		this->mPhysicsData.sphereColliders[index].properties.frictionSqrt = material.frictionSqrt;
		this->mPhysicsData.sphereColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.sphereColliders[index].identifier);
	}

	void PhysicsWorld::add(const Capsule& capsule, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.capsuleColliders.insert(CapsuleCollider(capsule));
		this->mPhysicsData.capsuleColliders[index].properties.restitution = material.restitution;
		this->mPhysicsData.capsuleColliders[index].properties.frictionSqrt = material.frictionSqrt;
		this->mPhysicsData.capsuleColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.capsuleColliders[index].identifier);
	}

	void PhysicsWorld::add(const ConvexHull& convexHull, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.convexHullColliders.insert(ConvexHullCollider(convexHull));
		this->mPhysicsData.convexHullColliders[index].properties.restitution = material.restitution;
		this->mPhysicsData.convexHullColliders[index].properties.frictionSqrt = material.frictionSqrt;
		this->mPhysicsData.convexHullColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[index].identifier);
	}

	void PhysicsWorld::add(const TriangleMesh& mesh, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.triangleMeshColliders.insert(TriangleMeshCollider(mesh));
		this->mPhysicsData.triangleMeshColliders[index].properties.restitution = material.restitution;
		this->mPhysicsData.triangleMeshColliders[index].properties.frictionSqrt = material.frictionSqrt;
		this->mPhysicsData.triangleMeshColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.triangleMeshColliders[index].identifier);
	}

	void PhysicsWorld::add(const HingeConstraint::Parameters& parameters)
	{
		if (parameters.disableCollisions == true) {
			this->mPhysicsData.physicsObjects[parameters.object1].diableCollision(&this->mPhysicsData, parameters.object2);
		}
		this->mPhysicsData.hingeConstraints.insert(HingeConstraint(&this->mPhysicsData, parameters));
	}

	void PhysicsWorld::add(const ConeConstraint::Parameters& parameters)
	{
		if (parameters.disableCollisions == true) {
			this->mPhysicsData.physicsObjects[parameters.object1].diableCollision(&this->mPhysicsData, parameters.object2);
		}
		this->mPhysicsData.coneConstraints.insert(ConeConstraint(&this->mPhysicsData, parameters));
	}

	void PhysicsWorld::add(const MotorConstraint::Parameters& parameters)
	{
		if (parameters.disableCollisions == true) {
			this->mPhysicsData.physicsObjects[parameters.object1].diableCollision(&this->mPhysicsData, parameters.object2);
		}
		this->mPhysicsData.motorConstraints.insert(MotorConstraint(&this->mPhysicsData, parameters));
	}

	bool PhysicsWorld::isObjectIntheWorld(const uint32& objectIndex)
	{
		return this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex);
	}

	void PhysicsWorld::removePhysicsObject(const uint32& objectIndex)
	{
		assert(this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex));
		this->mPhysicsData.physicsObjects.eraseDataAtIndex(objectIndex);
	}

	Mat4x4 PhysicsWorld::getObjectTransformationMatrix(const uint32& objectIndex)
	{
		assert(this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex));
		return this->mPhysicsData.physicsObjects[objectIndex].rigidBody.getTransformMatrix();
	}
	
	void PhysicsWorld::addForceToRigidBody(const uint32& objectIndex, const Vec3& force)
	{
		assert(this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex));
		this->mPhysicsData.physicsObjects[objectIndex].rigidBody.addForce(&this->mPhysicsData, force);
	}

	void PhysicsWorld::addForceToRigidBodyAtaPoint(const uint32& objectIndex, const Vec3& force, const Vec3& point)
	{
		assert(this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex));
		this->mPhysicsData.physicsObjects[objectIndex].rigidBody.addForceAtPoint(&this->mPhysicsData, force, point);
	}
}
