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

#ifndef PHYSICSDATA_H
#define PHYSICSDATA_H

#include"octree.h"
#include"physicsObject.h"
#include"collision/collider.h"
#include"constraints/constraints.h"
#include"../containers/AVLTree.h"

namespace mech {

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct PhysicsSettings {
		RigidBodySettings* rigidBodySettings = nullptr;
		decimal baumgarteFactor = decimal(0.3);
		decimal linearSlop = decimal(0.01);
		decimal minVelocityForRestitution = decimal(1.5);
		decimal minimalDispacement = decimal(0.015);
		byte velocityIterations = 5;
		byte positionIterations = 3;
		byte framesToRetainCache = 10;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class PhysicsData {

	private:

		const AABB& (PhysicsData::* mAABBPtrs[5]) (const uint32&) = {};
		void (PhysicsData::* mErasePtrs[5]) (const uint32&) = {};

		const AABB& convexHullAABB(const uint32& colliderIndex) { return this->convexHullColliders[colliderIndex].bound; }
		const AABB& sphereAABB(const uint32& colliderIndex) { return this->sphereColliders[colliderIndex].bound; }
		const AABB& capsuleAABB(const uint32& colliderIndex) { return this->capsuleColliders[colliderIndex].bound; }
		const AABB& compoundAABB(const uint32& colliderIndex) { return this->compoundColliders[colliderIndex].bound; }
		const AABB& triangleMeshAABB(const uint32& colliderIndex) { return this->triangleMeshColliders[colliderIndex].bound; }

		void eraseConvexHull(const uint32& colliderIndex) { this->convexHullColliders.eraseDataAtIndex(colliderIndex); }
		void eraseSphere(const uint32& colliderIndex) { this->sphereColliders.eraseDataAtIndex(colliderIndex); }
		void eraseCapsule(const uint32& colliderIndex) { this->capsuleColliders.eraseDataAtIndex(colliderIndex); }
		void eraseCompoundCollider(const uint32& colliderIndex) { this->compoundColliders.eraseDataAtIndex(colliderIndex); }
		void eraseTriangleMesh(const uint32& colliderIndex) { this->triangleMeshColliders.eraseDataAtIndex(colliderIndex); }

	public:

		PhysicsData()
		{
			this->mAABBPtrs[0] = &PhysicsData::convexHullAABB;
			this->mAABBPtrs[1] = &PhysicsData::sphereAABB;
			this->mAABBPtrs[2] = &PhysicsData::capsuleAABB;
			this->mAABBPtrs[3] = &PhysicsData::compoundAABB;
			this->mAABBPtrs[4] = &PhysicsData::triangleMeshAABB;

			this->mErasePtrs[0] = &PhysicsData::eraseConvexHull;
			this->mErasePtrs[1] = &PhysicsData::eraseSphere;
			this->mErasePtrs[2] = &PhysicsData::eraseCapsule;
			this->mErasePtrs[3] = &PhysicsData::eraseCompoundCollider;
			this->mErasePtrs[4] = &PhysicsData::eraseTriangleMesh;
		}

		PhysicsData(const PhysicsData&) = delete;
		PhysicsData& operator=(const PhysicsData&) = delete;

		const AABB& getColliderAABB(const uint32& id)
		{
			return (this->*mAABBPtrs[(uint32)(this->colliderIdentifiers[id].type)])(this->colliderIdentifiers[id].colliderIndex);
		}

		void erase(const uint32& id)
		{ 
			if (this->colliderIdentifiers[id].state == ColliderMotionState::dynamic) {
				this->physicsObjects.eraseDataAtIndex(this->colliderIdentifiers[id].objectIndex);
			}
			(this->*mErasePtrs[(uint32)(this->colliderIdentifiers[id].type)])(this->colliderIdentifiers[id].colliderIndex);
			this->colliderIdentifiers.eraseDataAtIndex(id);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		Octree octree;

		RigidArray<ColliderIdentifier, uint32> colliderIdentifiers;

		RigidArray<PhysicsObject, uint32> physicsObjects;
		
		RigidArray<AVLTree<uint32, uint32>, uint16> islands; //RigidArray<AVLTree<colliderID, ............

		HeightFieldCollider heightFieldCollider;
		RigidArray<ConvexHullCollider, uint32> convexHullColliders;
		RigidArray<SphereCollider, uint32> sphereColliders;
		RigidArray<CapsuleCollider, uint32> capsuleColliders;
		RigidArray<CompoundCollider, uint32> compoundColliders;
		RigidArray<TriangleMeshCollider, uint32> triangleMeshColliders;

		DynamicArray<ContactConstraint, uint32> contactConstraints;
		HashTable<Pair<uint32, ContactConstraint::ImpulseCache>, uint32> contactImpulseCache; //HashTable<Pair<manifoldID, ContactConstraint::ImpulseCache>............
		HashTable<Pair<uint32, HullVsHullContactCache>, uint32> hullVsHullContactCache; //HashTable<Pair<manifoldID, HullVsHullContactCache>............
		HashTable<Pair<uint32, CollisionFlag>, uint32> finishedCollisions; //HashTable<Pair<manifoldID, CollisionFlag>............

		RigidArray<HingeConstraint, uint16> hingeConstraints;
		RigidArray<ConeConstraint, uint16> coneConstraints;
		RigidArray<MotorConstraint, uint16> motorConstraints;

		PhysicsSettings settings;
	};
}

#endif