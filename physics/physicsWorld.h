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

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include"constraintSolver.h"
#include"broadPhase.h"
#include"cacheManager.h"

namespace mech {

	struct HeightFieldTest : public OctreeToHeightFieldLink {
		HeightField* heightField = nullptr;

		bool intersects(const AABB& aabb) override
		{
			return this->heightField->intersects(aabb);
		}
	};

	class PhysicsWorld
	{
	private:

		PhysicsData mPhysicsData;
		BroadPhase mBroadPhase;
		TimeOfImpact mTimeOfImpact;
		NarrowPhase mNarrowPhase;
		ConstraintSolver mConstraintSolver;
		CacheManager mCacheManager;
		HeightFieldTest mHeightFieldTest;

	public:
		PhysicsWorld();
		PhysicsWorld(const PhysicsWorld&) = delete;
		PhysicsWorld& operator=(const PhysicsWorld&) = delete;

		void update(const decimal& deltaTime);

		void initialiseOctree(const AABB& bounds, const byte& depth);
		void initialiseHeightField(const HeightField::Parameters& parameters, const PhysicsMaterial& material);

		//colliders
		uint32 addSphere(const Sphere& sphere, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset = Transform3D()); //returns id of the collider
		uint32 addCapsule(const Capsule& capsule, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset = Transform3D()); //returns id of the collider
		uint32 addConvexHull(const ConvexHull& convexHull, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset = Transform3D()); //returns id of the collider
		uint32 addTriangleMesh(const TriangleMesh& mesh, const ColliderMotionState& state, const PhysicsMaterial& material); //returns id of the collider
		uint32 addCompoundCollider(const DynamicArray<Pair<ConvexHull, Transform3D>, uint32> convexHulls, const DynamicArray<Pair<Sphere, Transform3D>, uint32> spheres, const DynamicArray<Pair<Capsule, Transform3D>, uint32> capsules, const ColliderMotionState& state, const PhysicsMaterial& material, const Transform3D& offset = Transform3D()); //returns id of the collider

		//constraints
		void addHingeConstraint(const HingeConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }
		void addConeConstraint(const ConeConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }
		void addMotorConstraint(const MotorConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }

		bool isObjectIntheWorld(const uint32& id) { return this->mPhysicsData.colliderIdentifiers.isIndexOccupied(id); }
		void erase(const uint32& id) { this->mPhysicsData.erase(id); }

		RigidBody* getRigidBody(const uint32& id) { return &this->mPhysicsData.physicsObjects[this->mPhysicsData.colliderIdentifiers[id].objectIndex].rigidBody; }
		const ColliderIdentifier* getColliderIdentifier(const uint32& id) { return &this->mPhysicsData.colliderIdentifiers[id]; }
		PhysicsSettings* getPhysicsSettings() { return &this->mPhysicsData.settings; }
	};
}

#endif

