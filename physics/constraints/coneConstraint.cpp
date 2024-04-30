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

#include"constraints.h"

#include"../physicsData.h"

namespace mech {

	ConeConstraint::ConeConstraint(PhysicsData* physicsData, const Parameters& parameters)
	{
		this->objectIndex[0] = physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex;
		this->objectIndex[1] = physicsData->colliderIdentifiers[parameters.colliderID2].objectIndex;

		this->cosHalfConeAngle = mathCOS(parameters.halfConeAngle);

		this->worldSpaceRotationAxis = getPerpendicularVectorNormalised(parameters.twistAxis1);

		StackArray<RigidBody*, 2> bodies;
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
		}

		Mat4x4 invT1 = getInverse(bodies[0]->getTransformMatrix());
		this->localSpacePoint[0] = invT1 * parameters.anchorPoint;
		this->localSpaceTwistAxis[0] = invT1.toMat3x3() * parameters.twistAxis1;

		if (bodies[1]) {
			Mat4x4 invT2 = getInverse(bodies[1]->getTransformMatrix());
			this->localSpacePoint[1] = invT2 * parameters.anchorPoint;
			this->localSpaceTwistAxis[1] = invT2.toMat3x3() * parameters.twistAxis1;
		}
		else {
			this->localSpacePoint[1] = parameters.anchorPoint;
			this->localSpaceTwistAxis[1] = this->localSpaceTwistAxis[0];
		}
	}

	bool ConeConstraint::isValid(PhysicsData* physicsData)
	{
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			if (physicsData->physicsObjects.isIndexOccupied(this->objectIndex[x]) == false) {
				return false;
			}
		}

		return true;
	}

	void ConeConstraint::warmStart(PhysicsData* physicsData)
	{
		StackArray<RigidBody*, 2> bodies;
		bool active[2] = { false, false };
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
			active[x] = bodies[x]->isActive();
		}

		this->isActive = active[0] || (bodies[1] ? active[1] : false);

		if (this->isActive) {

			StackArray<Quaternion, 2> orient(IDENTITY_QUATERNION);
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				orient[x] = bodies[x]->transform.orientation;
				if (active[x] == false) bodies[x]->activate();
			}

			this->pointConstraint.initialise(bodies, this->localSpacePoint, orient);
			this->pointConstraint.warmStart(bodies);

			Vec3 twist[2] = { orient[0] * this->localSpaceTwistAxis[0], orient[1] * this->localSpaceTwistAxis[1] };
			this->cosTheta = dotProduct(twist[0], twist[1]);
			if (this->cosTheta < this->cosHalfConeAngle) {

				Vec3 rotationAxis = crossProduct(twist[1], twist[0]);
				decimal mag = magnitude(rotationAxis);

				if (mag > decimal(0.0)) {
					this->worldSpaceRotationAxis = rotationAxis / mag;
				}

				this->angleConstraint.initialise(bodies, this->worldSpaceRotationAxis, decimal(0.0));
				this->angleConstraint.warmStart(bodies);
			}
			else {
				this->angleConstraint.deactivate();
			}
		}
	}

	void ConeConstraint::solve(PhysicsData* physicsData, const decimal& baumgarteFactor, const bool& solvePosition)
	{
		if (this->isActive) {

			StackArray<RigidBody*, 2> bodies;
			for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
				bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
			}

			this->pointConstraint.solveVelocity(bodies);
			if (solvePosition) {
				this->pointConstraint.solvePosition(bodies, baumgarteFactor);
			}

			if (this->angleConstraint.isActive()) {
				this->angleConstraint.solveVelocity(bodies, this->worldSpaceRotationAxis, decimal(0.0), decimalMAX);
				if (solvePosition) {
					this->angleConstraint.solvePosition(bodies, this->cosTheta - this->cosHalfConeAngle, baumgarteFactor);
				}
			}
		}
	}
}