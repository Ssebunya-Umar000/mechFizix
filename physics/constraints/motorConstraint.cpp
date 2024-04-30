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

	MotorConstraint::MotorConstraint(PhysicsData* physicsData, const Parameters& parameters)
	{
		this->objectIndex[0] = physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex;
		this->objectIndex[1] = physicsData->colliderIdentifiers[parameters.colliderID2].objectIndex;

		this->targetAngularVelocity = parameters.targetAngularVelocity;
		this->minTorque = parameters.minTorque;
		this->maxTorque = parameters.maxTorque;

		StackArray<RigidBody*, 2> bodies;
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
		}

		Mat4x4 invT1 = getInverse(bodies[0]->getTransformMatrix());
		this->localSpacePoint[0] = invT1 * parameters.anchorPoint;

		Mat3x3 r1 = invT1.toMat3x3();
		this->localSpaceHingeAxis[0] = normalise(r1 * parameters.hingeAxis1);

		if (bodies[1]) {
			Mat4x4 invT2 = getInverse(bodies[1]->getTransformMatrix());
			this->localSpacePoint[1] = invT2 * parameters.anchorPoint;

			Mat3x3 r2 = invT2.toMat3x3();
			this->localSpaceHingeAxis[1] = normalise(r2 * parameters.hingeAxis2);
		}
		else {
			this->localSpacePoint[1] = parameters.anchorPoint;
			this->localSpaceHingeAxis[1] = this->localSpaceHingeAxis[0];
		}

		if (parameters.normalAxis1 == parameters.normalAxis2 && parameters.hingeAxis1 == parameters.hingeAxis2) {
			this->invInitialOrientation = IDENTITY_QUATERNION;
		}
		else {
			Mat3x3 constraint1(parameters.normalAxis1, crossProduct(parameters.hingeAxis1, parameters.normalAxis1), parameters.hingeAxis1);
			Mat3x3 constraint2(parameters.normalAxis2, crossProduct(parameters.hingeAxis2, parameters.normalAxis2), parameters.hingeAxis2);
			this->invInitialOrientation = normalise(quaternionFromMatrix(constraint1) * getConjugate(quaternionFromMatrix(constraint2)));
		}

		this->invInitialOrientation = normalise((bodies[1] ? getConjugate(bodies[1]->transform.orientation) : IDENTITY_QUATERNION) * this->invInitialOrientation * bodies[0]->transform.orientation);
	}

	bool MotorConstraint::isValid(PhysicsData* physicsData)
	{
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			if (physicsData->physicsObjects.isIndexOccupied(this->objectIndex[x]) == false) {
				return false;
			}
		}

		return true;
	}

	void MotorConstraint::warmStart(PhysicsData* physicsData)
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

			this->singleAxisRotationConstraint.initialise(bodies, orient[0] * this->localSpaceHingeAxis[0], orient[1] * this->localSpaceHingeAxis[1]);
			this->singleAxisRotationConstraint.warmStart(bodies);

			this->a1 = orient[0] * this->localSpaceHingeAxis[0];
			this->angleConstraint.initialise(bodies, this->a1, -this->targetAngularVelocity);
			this->angleConstraint.warmStart(bodies);
		}
	}

	void MotorConstraint::solve(PhysicsData* physicsData, const decimal & deltaTime, const decimal & baumgarteFactor, const bool& solvePosition)
	{
		StackArray<RigidBody*, 2> bodies;
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
		}

		this->angleConstraint.solveVelocity(bodies, this->a1, this->minTorque * deltaTime, this->maxTorque * deltaTime);

		this->pointConstraint.solveVelocity(bodies);
		if (solvePosition) {
			this->pointConstraint.solvePosition(bodies, baumgarteFactor);
		}

		this->singleAxisRotationConstraint.solveVelocity(bodies);
		if (solvePosition) {
			this->singleAxisRotationConstraint.solvePosition(bodies, baumgarteFactor);
		}
	}
}