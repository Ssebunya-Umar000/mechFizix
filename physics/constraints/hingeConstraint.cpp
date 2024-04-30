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

	HingeConstraint::HingeConstraint(PhysicsData* physicsData, const Parameters& parameters)
	{
		this->objectIndex[0] = physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex;
		this->objectIndex[1] = physicsData->colliderIdentifiers[parameters.colliderID2].objectIndex;

		this->limitsMax = parameters.limitsMax >= decimal(0.0) && parameters.limitsMax <= mathPI ? parameters.limitsMax : decimal(0.0);
		this->limitsMin = parameters.limitsMin <= decimal(0.0) && parameters.limitsMin >= -mathPI ? parameters.limitsMin : decimal(0.0);

		StackArray<RigidBody*, 2> bodies;
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
		}

		Mat4x4 invT1 = getInverse(bodies[0]->getTransformMatrix());
		this->localSpacePoint[0] = invT1 * parameters.anchorPoint;
		this->localSpaceHingeAxis[0] = normalise(invT1.toMat3x3() * parameters.hingeAxis1);

		if (bodies[1]) {
			Mat4x4 invT2 = getInverse(bodies[1]->getTransformMatrix());
			this->localSpacePoint[1] = invT2 * parameters.anchorPoint;
			this->localSpaceHingeAxis[1] = normalise(invT2.toMat3x3() * parameters.hingeAxis2);
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

	bool HingeConstraint::isValid(PhysicsData* physicsData)
	{
		for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
			if (physicsData->physicsObjects.isIndexOccupied(this->objectIndex[x]) == false) {
				return false;
			}
		}

		return true;
	}

	void HingeConstraint::warmStart(PhysicsData* physicsData)
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
			this->theta = getRotationAngle((bodies[1] ? bodies[1]->transform.orientation : IDENTITY_QUATERNION) * this->invInitialOrientation * getConjugate(orient[0]), this->a1);

			if (this->limitsMax != this->limitsMin && (this->theta <= this->limitsMin || this->theta >= this->limitsMax)) {
				this->angleConstraint.initialise(bodies, this->a1, decimal(0.0));
				this->angleConstraint.warmStart(bodies);
			}
			else {
				this->angleConstraint.deactivate();
			}
		}
	}

	void HingeConstraint::solve(PhysicsData* physicsData, const decimal & baumgarteFactor, const bool& solvePosition)
	{
		struct TaskExecutor {
			decimal centerAngleAroundZero(const decimal& angleInRadians)
			{
				decimal angle = angleInRadians;
				if (angle < -mathPI) {
					do {
						angle += math2PI;
					} while (angle < -mathPI);
				}
				else if (angle > mathPI) {
					do {
						angle -= math2PI;
					} while (angle > mathPI);
				}

				return angle;
			}
		};

		if (this->isActive) {

			StackArray<RigidBody*, 2> bodies;
			for (byte x = 0; x < 2; ++x) if (isAValidIndex(this->objectIndex[x])) {
				bodies[x] = &physicsData->physicsObjects[this->objectIndex[x]].rigidBody;
			}

			this->pointConstraint.solveVelocity(bodies);
			if (solvePosition) {
				this->pointConstraint.solvePosition(bodies, baumgarteFactor);
			}

			this->singleAxisRotationConstraint.solveVelocity(bodies);
			if (solvePosition) {
				this->singleAxisRotationConstraint.solvePosition(bodies, baumgarteFactor);
			}

			if (this->angleConstraint.isActive()) {

				TaskExecutor ex;
				decimal distMin = ex.centerAngleAroundZero(this->theta - this->limitsMin);
				decimal distMax = ex.centerAngleAroundZero(this->theta - this->limitsMax);

				decimal smallest = mathABS(distMin) < mathABS(distMax) ? distMin : distMax;

				decimal min = smallest < decimal(0.0) ? decimal(0.0) : -decimalMAX;
				decimal max = smallest < decimal(0.0) ? decimalMAX : decimal(0.0);

				this->angleConstraint.solveVelocity(bodies, this->a1, min, max);
				if (solvePosition) {
					this->angleConstraint.solvePosition(bodies, smallest, baumgarteFactor);
				}
			}
		}
	}
}