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

#ifndef ANCHORPOINTCONSTRAINT_H
#define ANCHORPOINTCONSTRAINT_H

#include"../../rigidBody.h"
#include"../../../containers/stackArray.h"

namespace mech {

	struct AnchorPointConstraint {

		StackArray<Mat3x3, 2> invIxR;
		Mat3x3 invEffectiveMass;
		StackArray<Vec3, 2> r;
		Vec3 totalLambda;

		void initialise(StackArray<RigidBody*, 2>& bodies, const StackArray<Vec3, 2>& inR, const StackArray<Quaternion, 2>& orient)
		{
			Mat3x3 effectiveMass;
			decimal invMassSum = decimal(0.0);
			for (uint32 x = 0; x < 2; ++x) {

				this->r[x] = orient[x] * -inR[x];
				if (bodies[x]) {

					Mat3x3 mat = matrixFromQuarternion(orient[x]);
					Mat3x3 invI = mat * bodies[x]->invInertiaTensor * getTranspose(mat);
					Mat3x3 rx = getSkewSymmetricMatrix(this->r[x]);
					this->invIxR[x] = invI * rx;
					invMassSum += bodies[x]->invMass;
					effectiveMass += rx * invI * getTranspose(rx);
				}
			}

			effectiveMass += scaleMatrix(Vec3(invMassSum)).toMat3x3();
			this->invEffectiveMass = getInverse(effectiveMass);
		}

		void warmStart(StackArray<RigidBody*, 2>& bodies)
		{
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(this->totalLambda * bodies[x]->invMass * sign, this->invIxR[x] * this->totalLambda * sign);
			}
		}

		void solveVelocity(StackArray<RigidBody*, 2>& bodies)
		{
			Vec3 linearVel[2];
			Vec3 angularVel[2];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				linearVel[x] = bodies[x]->linearVelocity;
				angularVel[x] = bodies[x]->angularVelocity;
			}

			Vec3 lambda = this->invEffectiveMass * (linearVel[0] - crossProduct(this->r[0], angularVel[0]) - linearVel[1] + crossProduct(this->r[1], angularVel[1]));

			this->totalLambda += lambda;

			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(lambda * bodies[x]->invMass * sign, this->invIxR[x] * lambda * sign);
			}
		}

		void solvePosition(StackArray<RigidBody*, 2>& bodies, const decimal& BaumgarteFactor)
		{
			Vec3 position[2];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				position[x] = bodies[x]->transform.position;
			}

			Vec3 separation = (position[1] - position[0]) - (this->r[1] - this->r[0]);
			if (magnitudeSq(separation) != decimal(0.0)) {

				Vec3 lambda = this->invEffectiveMass * separation * -BaumgarteFactor;

				for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
					decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
					bodies[x]->updatePositionAndOrientaion(lambda * bodies[x]->invMass * sign, this->invIxR[x] * lambda * sign);
				}
			}
		}
	};
}

#endif

