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

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include"constraintSolver.h"
#include"broadPhase.h"
#include"cacheManager.h"

namespace mech {

	class PhysicsWorld
	{
	private:

		PhysicsData mPhysicsData;
		BroadPhase mBroadPhase;
		NarrowPhase mNarrowPhase;
		ConstraintSolver mConstraintSolver;
		CacheManager mCacheManager;

	public:
		PhysicsWorld();
		PhysicsWorld(const PhysicsWorld&) = delete;
		PhysicsWorld& operator=(const PhysicsWorld&) = delete;

		void update(const decimal& deltaTime);

		void initialiseOctree(const AABB& bounds, const byte& depth);
		void initialiseHeightField(const HeightField::Parameters& parameters, const PhysicsMaterial& material);

		//dynamic colliders
		uint32 add(const Sphere& sphere, const PhysicsMaterial& material, const Vec3& pos); //returns index of physics object
		uint32 add(const Capsule& capsule, const PhysicsMaterial& material, const Vec3& pos); //returns index of physics object
		uint32 add(const ConvexHull& convexHull, const PhysicsMaterial& material, const Vec3& pos); //returns index of physics object
		
		//static colliders
		void add(const Sphere& sphere, const PhysicsMaterial& material);
		void add(const Capsule& capsule, const PhysicsMaterial& material);
		void add(const ConvexHull& convexHull, const PhysicsMaterial& material);
		void add(const TriangleMesh& mesh, const PhysicsMaterial& material);

		//constraints
		void add(const HingeConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }
		void add(const ConeConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }
		void add(const MotorConstraint::Parameters& parameters) { this->mConstraintSolver.add(parameters); }

		bool isObjectIntheWorld(const uint32& objectIndex) { return this->mPhysicsData.physicsObjects.isIndexOccupied(objectIndex); }
		void removePhysicsObject(const uint32& objectIndex) { this->mPhysicsData.physicsObjects.eraseDataAtIndex(objectIndex); }

		RigidBody* getRigidBody(const uint32& objectIndex) { return &this->mPhysicsData.physicsObjects[objectIndex].rigidBody; }
		PhysicsConfigurations* getPhysicsConfigurations() { return &this->mPhysicsData.configurations; }
	};
}

#endif

