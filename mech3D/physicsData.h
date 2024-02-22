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

#ifndef PHYSICSDATA_H
#define PHYSICSDATA_H

#include"octree.h"
#include"physicsObject.h"
#include"collision/collider.h"
#include"constraints/constraints.h"
#include"../containers/AVLTree.h"

namespace mech {

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct PhysicsConfigurations {
		RigidBodyConfigurations* rigidBodyConfigs = nullptr;
		ConstraintConfigurations* constraintConfigs = nullptr;
		decimal minimalDispacement = decimal(0.025);
		decimal timeOfImpactBias = decimal(0.01);
		byte framesToRetainCache = 10;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class PhysicsData {

	public:

		Octree octree;

		RigidArray<PhysicsObject, uint32> physicsObjects;
		
		RigidArray<AVLTree<Pair<uint32, ColliderIdentifier>, uint32>, uint16> islands; //RigidArray<AVLTree<Pair<colliderID, ColliderIdentifier>............

		HeightFieldCollider heightFieldCollider;
		RigidArray<ConvexHullCollider, uint32> convexHullColliders;
		RigidArray<SphereCollider, uint32> sphereColliders;
		RigidArray<CapsuleCollider, uint32> capsuleColliders;
		RigidArray<TriangleMeshCollider, uint32> triangleMeshColliders;

		DynamicArray<ContactConstraint, uint32> contactConstraints;
		HashTable<Pair<uint32, ContactConstraint::ImpulseCache>, uint32> contactImpulseCache; //HashTable<Pair<manifoldID, ContactConstraint::ImpulseCache>............
		HashTable<Pair<uint32, HullVsHullContactCache>, uint32> hullVsHullContactCache; //HashTable<Pair<manifoldID, HullVsHullContactCache>............
		HashTable<Pair<uint32, CollisionFlag>, uint32> finishedCollisions; //HashTable<Pair<manifoldID, CollisionFlag>............

		RigidArray<HingeConstraint, uint16> hingeConstraints;
		RigidArray<ConeConstraint, uint16> coneConstraints;
		RigidArray<MotorConstraint, uint16> motorConstraints;

		PhysicsConfigurations configurations;

		PhysicsData()
		{
			this->mAABBFunctionPointers[0] = &PhysicsData::convexHullAABB;
			this->mAABBFunctionPointers[1] = &PhysicsData::sphereAABB;
			this->mAABBFunctionPointers[2] = &PhysicsData::capsuleAABB;
			this->mAABBFunctionPointers[3] = &PhysicsData::triangleMeshAABB;
		}
		
		PhysicsData(const PhysicsData&) = delete;
		PhysicsData& operator=(const PhysicsData&) = delete;

		const AABB& getColliderAABB(const ColliderIdentifier& colliderID) { return (this->*mAABBFunctionPointers[(uint32)(colliderID.type)])(colliderID.colliderIndex); }

	private:

		const AABB& (PhysicsData::* mAABBFunctionPointers[4]) (const uint32&) = {};

		const AABB& convexHullAABB(const uint32& colliderIndex) { return this->convexHullColliders[colliderIndex].bound; }
		const AABB& sphereAABB(const uint32& colliderIndex) { return this->sphereColliders[colliderIndex].bound; }
		const AABB& capsuleAABB(const uint32& colliderIndex) { return this->capsuleColliders[colliderIndex].bound; }
		const AABB& triangleMeshAABB(const uint32& colliderIndex) { return this->triangleMeshColliders[colliderIndex].bound; }
	};
}

#endif