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

#ifndef ANGULAROTATIONCONSTRAINT_H
#define ANGULAROTATIONCONSTRAINT_H

#include"../../rigidBody.h"
#include"../../../containers/stackArray.h"

namespace mech {

	struct AngularRotationConstraint {

		StackArray<Vec3, 2> invIxAxis;
		decimal invEffectiveMass = decimal(0.0);
		decimal totalLambda = decimal(0.0);
		decimal bias = decimal(0.0);

		void initialise(StackArray<RigidBody*, 2>& bodies, const Vec3& axis, const decimal& inBias)
		{
			for (uint32 x = 0; x < 2; ++x) if(bodies[x]) {
				Mat3x3 t = bodies[x]->getTransformMatrix().toMat3x3();
				this->invIxAxis[x] = (t * bodies[x]->invInertiaTensor * getTranspose(t)) * axis;
			}

			this->invEffectiveMass = decimal(1.0) / dotProduct(axis, this->invIxAxis[0] + this->invIxAxis[1]);
			this->bias = inBias;
		}

		void warmStart(StackArray<RigidBody*, 2>& bodies)
		{
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(Vec3(), this->invIxAxis[x] * this->totalLambda * sign);
			}
		}

		void solveVelocity(StackArray<RigidBody*, 2>& bodies, const Vec3& axis, const decimal& minLambda, const decimal& maxLambda)
		{
			Vec3 angularVel[2];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				angularVel[x] = bodies[x]->angularVelocity;
			}

			decimal lambda = this->invEffectiveMass * dotProduct(axis, angularVel[0] - angularVel[1]) - this->bias;

			decimal temp = this->totalLambda;
			this->totalLambda = clamp(this->totalLambda + lambda, minLambda, maxLambda);
			lambda = this->totalLambda - temp;

			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(Vec3(), this->invIxAxis[x] * lambda * sign);
			}
		}

		void solvePosition(StackArray<RigidBody*, 2>& bodies, const decimal& C, const decimal& BaumgarteFactor)
		{
			decimal lambda = -this->invEffectiveMass * BaumgarteFactor * C;
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updatePositionAndOrientaion(Vec3(), this->invIxAxis[x] * lambda * sign);
			}
		}

		void deactivate()
		{
			this->invEffectiveMass = decimal(0.0);
			this->totalLambda = decimal(0.0);
		}

		bool isActive()
		{
			return this->invEffectiveMass != decimal(0.0);
		}
	};
}

#endif

