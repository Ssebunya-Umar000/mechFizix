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

#include"physicsWorld.h"
#include"tensorCalculation.h"

namespace mech {

	PhysicsWorld::PhysicsWorld()
	{
		this->mConstraintSolver.physicsData = &this->mPhysicsData;
	
		this->mNarrowPhase.physicsData = &this->mPhysicsData;

		this->mTimeOfImpact.physicsData = &this->mPhysicsData;
		
		this->mBroadPhase.physicsData = &this->mPhysicsData;
		this->mBroadPhase.narrowPhase = &this->mNarrowPhase;
		this->mBroadPhase.timeOfImpact = &this->mTimeOfImpact;
		this->mBroadPhase.constraintSolver = &this->mConstraintSolver;
		
		this->mCacheManager.physicsData = &this->mPhysicsData;

		this->mPhysicsData.settings.rigidBodySettings = getRigidBodySettings();

		this->mHeightFieldTest.heightField = &this->mPhysicsData.heightFieldCollider.collider;
	}
	
	void PhysicsWorld::update(const decimal& deltaTime)
	{
		BEGIN_PROFILE("PhysicsWorld::update");

		for (auto it = this->mPhysicsData.physicsObjects.begin(), end = this->mPhysicsData.physicsObjects.end(); it != end;) {

			BEGIN_PROFILE("PhysicsObjectUpdateLoop");

			auto temp = it;
			++it;

			PhysicsObject& object = temp.data();

			if (object.rigidBody.isActive()) {

				object.rigidBody.update(&this->mPhysicsData, deltaTime);

				if (object.rigidBody.isActive()) {
					this->mBroadPhase.handle(object, deltaTime);
				}
			}

			END_PROFILE;
		}

		this->mConstraintSolver.solve(deltaTime);
		this->mCacheManager.update();

#if mech_ENABLE_DEBUG_RENDERER
	
		for (auto it = this->mPhysicsData.octree.nodes.begin(), end = this->mPhysicsData.octree.nodes.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLACK);
		}

		for (auto it = this->mPhysicsData.convexHullColliders.begin(), end = this->mPhysicsData.convexHullColliders.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLUE);
		}
		for (auto it = this->mPhysicsData.sphereColliders.begin(), end = this->mPhysicsData.sphereColliders.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLUE);
		}
		for (auto it = this->mPhysicsData.capsuleColliders.begin(), end = this->mPhysicsData.capsuleColliders.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLUE);
		}
		for (auto it = this->mPhysicsData.compoundColliders.begin(), end = this->mPhysicsData.compoundColliders.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLUE);
		}
		for (auto it = this->mPhysicsData.triangleMeshColliders.begin(), end = this->mPhysicsData.triangleMeshColliders.end(); it != end; ++it) {
			DEBUG_RENDERER_ADD(it.data().bound, BLUE);
		}
#endif

		END_PROFILE;
	}

	void PhysicsWorld::initialiseOctree(const AABB& bounds, const byte& depth)
	{
		this->mPhysicsData.octree.initialise(bounds, &this->mHeightFieldTest, depth);
	}

	void PhysicsWorld::initialiseHeightField(BumpyTerrainParameters* parameters, const PhysicsMaterial& material)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::heightField));
		this->mPhysicsData.heightFieldCollider.collider.initialise(parameters);

		this->mPhysicsData.colliderIdentifiers[colliderID].colliderID = colliderID;
		this->mPhysicsData.colliderIdentifiers[colliderID].material = material;

		this->mHeightFieldTest.heightFieldID = colliderID;
	}

	void PhysicsWorld::initialiseHeightField(FlatTerrainParameters* parameters, const PhysicsMaterial& material)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::heightField));
		this->mPhysicsData.heightFieldCollider.collider.initialise(parameters);

		this->mPhysicsData.colliderIdentifiers[colliderID].colliderID = colliderID;
		this->mPhysicsData.colliderIdentifiers[colliderID].material = material;

		this->mHeightFieldTest.heightFieldID = colliderID;
	}

	void setUp(PhysicsData* physicsData, const uint32& colliderID, const uint32& colliderIndex, const uint32& objectIndex, const PhysicsMaterial& material, const ColliderMotionState& state)
	{
		physicsData->colliderIdentifiers[colliderID].colliderID = colliderID;
		physicsData->colliderIdentifiers[colliderID].colliderIndex = colliderIndex;
		physicsData->colliderIdentifiers[colliderID].objectIndex = objectIndex;
		physicsData->colliderIdentifiers[colliderID].material = material;
		physicsData->colliderIdentifiers[colliderID].state = state;
	}

	uint32 PhysicsWorld::addSphere(const Sphere& sphere, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::sphere));
		uint32 colliderIndex = this->mPhysicsData.sphereColliders.insert(SphereCollider(sphere));

		this->mPhysicsData.sphereColliders[colliderIndex].transform(offset);

		uint32 objectIndex = -1;
		if (state == ColliderMotionState::dynamic) {
			objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
			decimal mass = material.density * this->mPhysicsData.sphereColliders[colliderIndex].getVolume();
			this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, colliderID, mass, calculateTensor(mass, this->mPhysicsData.sphereColliders[colliderIndex].collider), offset);
		}

		setUp(&this->mPhysicsData, colliderID, colliderIndex, objectIndex, material, state);

		StackArray<uint16, 8> nodes = this->mPhysicsData.octree.addEntity(colliderID, this->mPhysicsData.getColliderAABB(colliderID));
		if (state == ColliderMotionState::dynamic) {
			this->mPhysicsData.physicsObjects[objectIndex].nodesIntersected = nodes;
		}

		return colliderID;
	}

	uint32 PhysicsWorld::addCapsule(const Capsule& capsule, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::capsule));
		uint32 colliderIndex = this->mPhysicsData.capsuleColliders.insert(CapsuleCollider(capsule));

		this->mPhysicsData.capsuleColliders[colliderIndex].transform(offset);

		uint32 objectIndex = -1;
		if (state == ColliderMotionState::dynamic) {
			objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
			this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, colliderID, material.density * this->mPhysicsData.capsuleColliders[colliderIndex].getVolume(), calculateTensor(material.density, this->mPhysicsData.capsuleColliders[colliderIndex].collider), offset);
		}

		setUp(&this->mPhysicsData, colliderID, colliderIndex, objectIndex, material, state);

		StackArray<uint16, 8> nodes = this->mPhysicsData.octree.addEntity(colliderID, this->mPhysicsData.getColliderAABB(colliderID));
		if (state == ColliderMotionState::dynamic) {
			this->mPhysicsData.physicsObjects[objectIndex].nodesIntersected = nodes;
		}

		return colliderID;
	}

	uint32 PhysicsWorld::addConvexHull(const ConvexHull& convexHull, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::convexHull));
		uint32 colliderIndex = this->mPhysicsData.convexHullColliders.insert(ConvexHullCollider(convexHull));

		this->mPhysicsData.convexHullColliders[colliderIndex].transform(offset);

		uint32 objectIndex = -1;
		if (state == ColliderMotionState::dynamic) {
			objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
			decimal mass = material.density * this->mPhysicsData.convexHullColliders[colliderIndex].getVolume();
			this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, colliderID, mass, calculateTensor(mass, this->mPhysicsData.convexHullColliders[colliderIndex].collider.vertices.toDynamicArray()), offset);
		}

		setUp(&this->mPhysicsData, colliderID, colliderIndex, objectIndex, material, state);

		StackArray<uint16, 8> nodes = this->mPhysicsData.octree.addEntity(colliderID, this->mPhysicsData.getColliderAABB(colliderID));
		if (state == ColliderMotionState::dynamic) {
			this->mPhysicsData.physicsObjects[objectIndex].nodesIntersected = nodes;
		}

		return colliderID;
	}

	uint32 PhysicsWorld::addTriangleMesh(const TriangleMesh& mesh, const ColliderMotionState& state, const PhysicsMaterial& material)
	{
		ASSERT(state == ColliderMotionState::motionless, "dynamic triangle meshes are not supported!!");

		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::triangleMesh));
		uint32 colliderIndex = this->mPhysicsData.triangleMeshColliders.insert(TriangleMeshCollider(mesh));

		setUp(&this->mPhysicsData, colliderID, colliderIndex, -1, material, state);

		StackArray<uint16, 8> nodes = this->mPhysicsData.octree.addEntity(colliderID, this->mPhysicsData.getColliderAABB(colliderID));
		
		return colliderID;
	}

	uint32 PhysicsWorld::addCompoundCollider(const DynamicArray<Pair<ConvexHull, Transform3D>, uint32> convexHulls, const DynamicArray<Pair<Sphere, Transform3D>, uint32> spheres, const DynamicArray<Pair<Capsule, Transform3D>, uint32> capsules, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset)
	{
		uint32 colliderID = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::compound));
		uint32 colliderIndex = this->mPhysicsData.compoundColliders.insert(CompoundCollider(colliderID));

		/////////////////////////////////////////////////////////////
		decimal mass = decimal(0.0);
		Mat3x3 tensor(decimal(0.0));
		/////////////////////////////////////////////////////////////

		uint32 objectIndex = -1;
		if (state == ColliderMotionState::dynamic) {
			objectIndex = this->mPhysicsData.physicsObjects.insert(PhysicsObject());
			this->mPhysicsData.colliderIdentifiers[colliderID].objectIndex = objectIndex;
		}

		this->mPhysicsData.compoundColliders[colliderIndex].bound = AABB(offset.position, offset.position);
		for (uint32 x = 0, len = convexHulls.size(); x < len; ++x) {

			uint32 id = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::convexHull));
			uint32 c = this->mPhysicsData.convexHullColliders.insert(ConvexHullCollider(convexHulls[x].first));
			this->mPhysicsData.convexHullColliders[c].transform(offset * convexHulls[x].second);

			setUp(&this->mPhysicsData, id, c, objectIndex, material, state);

			this->mPhysicsData.compoundColliders[colliderIndex].bound.max = maxVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.max, this->mPhysicsData.convexHullColliders[c].bound.max);
			this->mPhysicsData.compoundColliders[colliderIndex].bound.min = minVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.min, this->mPhysicsData.convexHullColliders[c].bound.min);

			decimal m = material.density * this->mPhysicsData.convexHullColliders[c].getVolume();
			mass += m;
			tensor += calculateTensor(m, this->mPhysicsData.convexHullColliders[c].collider.vertices.toDynamicArray());

			this->mPhysicsData.compoundColliders[colliderIndex].components.pushBack(id);
		}

		for (uint32 x = 0, len = spheres.size(); x < len; ++x) {

			uint32 id = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::sphere));
			uint32 c = this->mPhysicsData.sphereColliders.insert(SphereCollider(spheres[x].first));
			this->mPhysicsData.sphereColliders[c].transform(offset * spheres[x].second);

			setUp(&this->mPhysicsData, id, c, objectIndex, material, state);

			this->mPhysicsData.compoundColliders[colliderIndex].bound.max = maxVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.max, this->mPhysicsData.sphereColliders[c].bound.max);
			this->mPhysicsData.compoundColliders[colliderIndex].bound.min = minVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.min, this->mPhysicsData.sphereColliders[c].bound.min);

			decimal m = material.density * this->mPhysicsData.sphereColliders[c].getVolume();
			mass += m;
			tensor += calculateTensor(m, this->mPhysicsData.sphereColliders[c].collider);

			this->mPhysicsData.compoundColliders[colliderIndex].components.pushBack(id);
		}

		for (uint32 x = 0, len = capsules.size(); x < len; ++x) {

			uint32 id = this->mPhysicsData.colliderIdentifiers.insert(ColliderIdentifier(ColliderType::capsule));
			uint32 c = this->mPhysicsData.capsuleColliders.insert(CapsuleCollider(capsules[x].first));
			this->mPhysicsData.capsuleColliders[c].transform(offset * capsules[x].second);

			setUp(&this->mPhysicsData, id, c, objectIndex, material, state);

			this->mPhysicsData.compoundColliders[colliderIndex].bound.max = maxVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.max, this->mPhysicsData.capsuleColliders[c].bound.max);
			this->mPhysicsData.compoundColliders[colliderIndex].bound.min = minVec(this->mPhysicsData.compoundColliders[colliderIndex].bound.min, this->mPhysicsData.capsuleColliders[c].bound.min);

			mass += material.density * this->mPhysicsData.capsuleColliders[c].getVolume();
			tensor += calculateTensor(material.density, this->mPhysicsData.capsuleColliders[c].collider);

			this->mPhysicsData.compoundColliders[colliderIndex].components.pushBack(id);
		}

		this->mPhysicsData.compoundColliders[colliderIndex].convexRadius = this->mPhysicsData.compoundColliders[colliderIndex].bound.getRadius();

		if (state == ColliderMotionState::dynamic) {
			for (auto it = this->mPhysicsData.compoundColliders[colliderIndex].components.begin(), end = this->mPhysicsData.compoundColliders[colliderIndex].components.end(); it != end; ++it) {
				this->mPhysicsData.physicsObjects[objectIndex].disabledCollisions.pushBack(it.data());
			}

			this->mPhysicsData.physicsObjects[objectIndex].initialise(&this->mPhysicsData, colliderID, mass, tensor, offset);
		}

		setUp(&this->mPhysicsData, colliderID, colliderIndex, objectIndex, material, state);

		StackArray<uint16, 8> nodes = this->mPhysicsData.octree.addEntity(colliderID, this->mPhysicsData.getColliderAABB(colliderID));
		if (state == ColliderMotionState::dynamic) {
			this->mPhysicsData.physicsObjects[objectIndex].nodesIntersected = nodes;
		}

		return colliderID;
	}
}
