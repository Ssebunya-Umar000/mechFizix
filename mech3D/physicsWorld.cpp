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
#include"../core/utilities.h"

namespace mech {

#define LARGE_VELOCITY_SQ decimal(3600.0)

	PhysicsWorld::PhysicsWorld()
	{
		this->mConstraintSolver.physicsData = &this->mPhysicsData;
	
		this->mNarrowPhase.physicsData = &this->mPhysicsData;
		
		this->mBroadPhase.physicsData = &this->mPhysicsData;
		this->mBroadPhase.narrowPhase = &this->mNarrowPhase;
		this->mBroadPhase.constraintSolver = &this->mConstraintSolver;
		
		this->mCacheManager.physicsData = &this->mPhysicsData;

		this->mPhysicsData.configurations.rigidBodyConfigs = getRigidBodyConfigurations();
		this->mPhysicsData.configurations.constraintConfigs = &this->mConstraintSolver.configurations;
	}
	
	void PhysicsWorld::update(const decimal& deltaTime)
	{
		PROFILE("PhysicsWorld::update");

		for (auto it = this->mPhysicsData.physicsObjects.begin(), end = this->mPhysicsData.physicsObjects.end(); it != end;) {

			auto temp = it;
			++it;

			PhysicsObject& object = temp.data();

			if (object.rigidBody.isActive()) {

				object.rigidBody.update(&this->mPhysicsData, deltaTime);

				if (object.rigidBody.isActive()) {
					
					if (magnitudeSq(object.rigidBody.linearVelocity) >= LARGE_VELOCITY_SQ) {
						this->mBroadPhase.continousCollisionDetection(object);
					}
					else {
						this->mBroadPhase.discreteCollisionDetection(object);
					}
				}
			}
		}

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
		this->mPhysicsData.heightFieldCollider.material = material;
	}

	uint32 PhysicsWorld::add(const Sphere& sphere, const PhysicsMaterial& material, const Vec3& pos)
	{
		uint32 colliderIndex = this->mPhysicsData.sphereColliders.insert(SphereCollider(sphere));
		uint32 objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
		
		this->mPhysicsData.sphereColliders[colliderIndex].material = material;
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

		this->mPhysicsData.capsuleColliders[colliderIndex].material = material;
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

		this->mPhysicsData.convexHullColliders[colliderIndex].material = material;
		this->mPhysicsData.convexHullColliders[colliderIndex].identifier.colliderIndex = colliderIndex;
		this->mPhysicsData.convexHullColliders[colliderIndex].identifier.objectIndex = objectIndex;

		this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[colliderIndex].identifier, material.density, pos);

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[colliderIndex].identifier);

		return objectIndex;
	}

	void PhysicsWorld::add(const Sphere& sphere, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.sphereColliders.insert(SphereCollider(sphere));
		this->mPhysicsData.sphereColliders[index].material = material;
		this->mPhysicsData.sphereColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.sphereColliders[index].identifier);
	}

	void PhysicsWorld::add(const Capsule& capsule, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.capsuleColliders.insert(CapsuleCollider(capsule));
		this->mPhysicsData.capsuleColliders[index].material = material;
		this->mPhysicsData.capsuleColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.capsuleColliders[index].identifier);
	}

	void PhysicsWorld::add(const ConvexHull& convexHull, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.convexHullColliders.insert(ConvexHullCollider(convexHull));
		this->mPhysicsData.convexHullColliders[index].material = material;
		this->mPhysicsData.convexHullColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.convexHullColliders[index].identifier);
	}

	void PhysicsWorld::add(const TriangleMesh& mesh, const PhysicsMaterial& material)
	{
		uint32 index = this->mPhysicsData.triangleMeshColliders.insert(TriangleMeshCollider(mesh));
		this->mPhysicsData.triangleMeshColliders[index].material = material;
		this->mPhysicsData.triangleMeshColliders[index].identifier.colliderIndex = index;

		this->mPhysicsData.octree.add(&this->mPhysicsData, this->mPhysicsData.triangleMeshColliders[index].identifier);
	}
}
