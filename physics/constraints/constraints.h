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

#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include"helpers/anchorPointConstraint.h"
#include"helpers/angularRotationConstraint.h"
#include"helpers/hingeAxisConstraint.h"
#include"helpers/axisConstraint.h"
#include"../collision/contact.h"
#include"../../containers/pair.h"

namespace mech {

#define MAXIMUM_CONTACT_POINTS 4

	class PhysicsData;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class ConstraintType : byte { contact = 1 << 0, cone = 1 << 1, hinge = 1 << 2, motor = 1 << 3 };

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ContactConstraint {

		struct ImpulseCache {

			struct Impulse {
				Vec3 frictionImpulse1;
				Vec3 frictionImpulse2;
				decimal antiPenetrationImpulse = decimal(0.0);
			};

			StackArray<Pair<uint32, Impulse>, MAXIMUM_CONTACT_POINTS> impulses = StackArray<Pair<uint32, Impulse>, MAXIMUM_CONTACT_POINTS>(-1);
			byte retention = 0;
		};

		struct ContactData {
			AxisConstraint penetrationConstraint;
			AxisConstraint frictionConstraint1;
			AxisConstraint frictionConstraint2;
			Vec3 normal;
			Vec3 tangent1;
			Vec3 tangent2;
			decimal penetration = decimal(0.0);
			uint32 ID = -1;
		};

		ContactData contactData[MAXIMUM_CONTACT_POINTS] = {};
		decimal frictionCoefficient = decimal(0.0);
		StackArray<uint32, 2> objectIndex = StackArray<uint32, 2>(-1);
		uint32 impulseCacheID = -1;
		byte contactPointCount = 0;

		ContactConstraint() {}
		ContactConstraint(PhysicsData* physicsData, const ContactManifold& manifold, const uint32& objectIndex1, const uint32& objectIndex2);

		void warmStart(PhysicsData* physicsData);
		void solve(PhysicsData* physicsData, const decimal& baumgarteFactor, const decimal& linearSlop, const bool& solvePosition, const bool& lastIteration);
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ConeConstraint {

		//parameters should be in world space
		struct Parameters {
			Vec3 twistAxis1 = nanVEC3;
			Vec3 twistAxis2 = nanVEC3;
			Vec3 anchorPoint = nanVEC3;
			decimal halfConeAngle = decimalNAN;
			uint32 colliderID1 = -1;
			uint32 colliderID2 = -1;
			bool disableCollisions = true;
		};

		AnchorPointConstraint pointConstraint;
		AngularRotationConstraint angleConstraint;
		StackArray<Vec3, 2> localSpacePoint;
		StackArray<Vec3, 2> localSpaceTwistAxis;
		Vec3 worldSpaceRotationAxis;
		decimal cosHalfConeAngle = decimal(0.0);
		decimal cosTheta = decimal(0.0);
		StackArray<uint32, 2> objectIndex = StackArray<uint32, 2>(-1);
		bool isActive = true;

		ConeConstraint() {}
		ConeConstraint(PhysicsData* physicsData, const Parameters& parameters);

		bool isValid(PhysicsData* physicsData);
		void warmStart(PhysicsData* physicsData);
		void solve(PhysicsData* physicsData, const decimal& baumgarteFactor, const bool& solvePosition);
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HingeConstraint {

		//parameters should be in world space
		struct Parameters {
			Vec3 hingeAxis1 = nanVEC3;
			Vec3 hingeAxis2 = nanVEC3;
			Vec3 normalAxis1 = nanVEC3;
			Vec3 normalAxis2 = nanVEC3;
			Vec3 anchorPoint = nanVEC3;
			decimal limitsMin = decimalNAN;
			decimal limitsMax = decimalNAN;
			uint32 colliderID1 = -1;
			uint32 colliderID2 = -1;
			bool disableCollisions = true;
		};

		AnchorPointConstraint pointConstraint;
		AngularRotationConstraint angleConstraint;
		HingeAxisConstraint singleAxisRotationConstraint;
		Quaternion invInitialOrientation;
		StackArray<Vec3, 2> localSpacePoint;
		StackArray<Vec3, 2> localSpaceHingeAxis;
		Vec3 a1;
		decimal theta = decimal(0.0);
		decimal limitsMin = decimal(0.0);
		decimal limitsMax = decimal(0.0);
		StackArray<uint32, 2> objectIndex = StackArray<uint32, 2>(-1);
		bool isActive = true;

		HingeConstraint() {}
		HingeConstraint(PhysicsData* physicsData, const Parameters& parameters);

		bool isValid(PhysicsData* physicsData);
		void warmStart(PhysicsData* physicsData);
		void solve(PhysicsData* physicsData, const decimal& baumgarteFactor, const bool& solvePosition);
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct MotorConstraint {

		//parameters should be in world space
		struct Parameters {
			Vec3 hingeAxis1 = nanVEC3;
			Vec3 hingeAxis2 = nanVEC3;
			Vec3 normalAxis1 = nanVEC3;
			Vec3 normalAxis2 = nanVEC3;
			Vec3 anchorPoint = nanVEC3;
			decimal targetAngularVelocity = decimalNAN;
			decimal minTorque = decimalNAN;
			decimal maxTorque = decimalNAN;
			uint32 colliderID1 = -1;
			uint32 colliderID2 = -1;
			bool disableCollisions = true;
		};

		AnchorPointConstraint pointConstraint;
		AngularRotationConstraint angleConstraint;
		HingeAxisConstraint singleAxisRotationConstraint;
		Quaternion invInitialOrientation;
		StackArray<Vec3, 2> localSpacePoint;
		StackArray<Vec3, 2> localSpaceHingeAxis;
		Vec3 a1;
		decimal targetAngularVelocity = decimal(0.0);
		decimal minTorque = decimal(0.0);
		decimal maxTorque = decimal(0.0);
		StackArray<uint32, 2> objectIndex = StackArray<uint32, 2>(-1);
		bool isActive = true;

		MotorConstraint() {}
		MotorConstraint(PhysicsData* physicsData, const Parameters& parameters);

		bool isValid(PhysicsData* physicsData);
		void warmStart(PhysicsData* physicsData);
		void solve(PhysicsData* physicsData, const decimal& deltaTime, const decimal& baumgarteFactor, const bool& solvePosition);
	};
}

#endif