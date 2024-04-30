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

#ifndef AXISCONSTRAINT_H
#define AXISCONSTRAINT_H

#include"../../rigidBody.h"
#include"../../../containers/stackArray.h"

namespace mech {

	struct AxisConstraint {

		StackArray<Vec3, 2> rCrossA;
		StackArray<Vec3, 2> ixrCrossA;
		decimal invEffectiveMass = decimal(0.0);
		decimal totalLambda = decimal(0.0);
		decimal bias = decimal(0.0);

		void initialise(const Vec3& axis, StackArray<RigidBody*, 2>& bodies, const StackArray<Vec3, 2>& r, const StackArray<Mat3x3, 2>& invI, const StackArray<decimal, 2>& invM, const decimal& inBias)
		{
			decimal effectiveMass = decimal(0.0);
			for (uint32 x = 0; x < 2; ++x) if(bodies[x]) {
				this->rCrossA[x] = crossProduct(r[x], axis);
				this->ixrCrossA[x] = invI[x] * this->rCrossA[x];
				effectiveMass += invM[x] + dotProduct(this->ixrCrossA[x], this->rCrossA[x]);
			}

			this->invEffectiveMass = decimal(1.0) / effectiveMass;
			this->bias = inBias;
		}

		void warmStart(const Vec3& axis, StackArray<Vec3, 2>& deltaLinVel, StackArray<Vec3, 2>& deltaAngVel, const StackArray<decimal, 2>& invM)
		{
			Vec3 linearImpulse = axis * this->totalLambda;

			deltaLinVel[0] -= linearImpulse * invM[0];
			deltaAngVel[0] -= this->ixrCrossA[0] * this->totalLambda;

			deltaLinVel[1] += linearImpulse * invM[1];
			deltaAngVel[1] += this->ixrCrossA[1] * this->totalLambda;
		}

		void solveVelocity(const Vec3& axis, StackArray<RigidBody*, 2>& bodies, const StackArray<decimal, 2>& invM, const decimal& minLambda, const decimal& maxLambda)
		{
			Vec3 linearVel[2];
			Vec3 angularVel[2];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				linearVel[x] = bodies[x]->linearVelocity;
				angularVel[x] = bodies[x]->angularVelocity;
			}

			decimal jv = dotProduct(axis, linearVel[1] - linearVel[0]) + dotProduct(this->rCrossA[1], angularVel[1]) - dotProduct(this->rCrossA[0], angularVel[0]);

			decimal lambda = -(jv - this->bias) * this->invEffectiveMass;

			decimal temp = this->totalLambda;
			this->totalLambda = clamp(this->totalLambda + lambda, minLambda, maxLambda);
			lambda = this->totalLambda - temp;

			Vec3 linearImpulse = axis * lambda;
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(linearImpulse * invM[x] * sign, this->ixrCrossA[x] * lambda * sign);
			}
		}

		void solvePosition(const Vec3& axis, StackArray<RigidBody*, 2>& bodies, const StackArray<decimal, 2>& invM, const decimal& BaumgarteFactor, decimal& C)
		{
			decimal lambda = -this->invEffectiveMass * BaumgarteFactor * C;

			Vec3 linearImpulse = axis * lambda;
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updatePositionAndOrientaion(linearImpulse * invM[x] * sign, this->ixrCrossA[x] * lambda * sign);
			}

			C += lambda;
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

