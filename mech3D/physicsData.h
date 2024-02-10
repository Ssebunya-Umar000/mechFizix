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

	using Island = AVLTree<Pair<uint32, ColliderIdentifier>, uint32>; //AVLTree<Pair<collider id, identifier>, ...

	struct PhysicsData {

		PhysicsData() {}
		PhysicsData(const PhysicsData&) = delete;
		PhysicsData& operator=(const PhysicsData&) = delete;

		Octree octree;
		HashTable<uint16, uint16> activeOctreeNodes;

		RigidArray<PhysicsObject, uint32> physicsObjects;
		RigidArray<Island, uint16> islands;

		HeightFieldCollider heightFieldCollider;
		RigidArray<ConvexHullCollider, uint32> convexHullColliders;
		RigidArray<SphereCollider, uint32> sphereColliders;
		RigidArray<CapsuleCollider, uint32> capsuleColliders;
		RigidArray<TriangleMeshCollider, uint32> triangleMeshColliders;

		DynamicArray<ContactConstraint, uint32> contactConstraints;
		HashTable<Pair<uint32, ContactConstraint::ImpulseCache>, uint32> contactImpulseCache;
		HashTable<Pair<uint32, HullVsHullContactCache>, uint32> hullVsHullContactCache;
		HashTable<uint32, uint32> manifoldIDs;

		RigidArray<HingeConstraint, uint16> hingeConstraints;
		RigidArray<ConeConstraint, uint16> coneConstraints;
		RigidArray<MotorConstraint, uint16> motorConstraints;

		const byte framesToRetainCache = 10;

		//rigid body motion settings
		const Vec3 gravity = Vec3(decimal(0.0), decimal(-9.8), decimal(0.0));
		const decimal linearDamping = decimal(0.5);
		const decimal angularDamping = decimal(0.5);
		const decimal sleepEpsilon = decimal(1e-4);
		const decimal maxMotion = sleepEpsilon * decimal(10.0);
		const decimal baseMotion = sleepEpsilon * decimal(1.015);

		//constraint solver settings
		const decimal baumgarteFactor = decimal(0.3);
		const byte velocityIterations = 5;
		const byte positionIterations = 3;

		//contact constraint settings
		const decimal linearSlop = decimal(0.01);
		const decimal minVelocityForRestitution = decimal(1.5);

		//contact generator settings
		const decimal minimalDispacement = decimal(0.025);
	};
}

#endif