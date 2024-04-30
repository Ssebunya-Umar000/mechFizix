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

#ifndef HINGEAXISCONSTRAINT_H
#define HINGEAXISCONSTRAINT_H

#include"../../rigidBody.h"
#include"../../../containers/stackArray.h"

namespace mech {

	struct HingeAxisConstraint {

		Mat2x2 invEffectiveMass;
		Vec3 a1;
		Vec3 b2;
		Vec3 c2;
		Vec3 b2CrossA1;
		Vec3 c2CrossA1;
		Vec2 totalLambda;

		void initialise(StackArray<RigidBody*, 2>& bodies, const Vec3& hingeAxis1, const Vec3& hingeAxis2)
		{
			this->a1 = hingeAxis1;
			Vec3 a2 = hingeAxis2;
			
			decimal dot = dotProduct(this->a1, a2);
			if (dot <= 1e-3) {

				Vec3 perp = a2 - (this->a1 * dot);
				if (magnitudeSq(perp) < 1e-6) {
					perp = getPerpendicularVectorNormalised(this->a1);
				}

				a2 = normalise(perp * decimal(0.99) + this->a1 * decimal(0.01));
			}

			this->b2 = getPerpendicularVectorNormalised(a2);
			this->c2 = crossProduct(a2, this->b2);

			this->b2CrossA1 = crossProduct(this->b2, this->a1);
			this->c2CrossA1 = crossProduct(this->c2, this->a1);

			Mat3x3 sumInvInertia = bodies[0]->invInertiaTensor + (bodies[1] ?  bodies[1]->invInertiaTensor : Mat3x3());
			Vec3 v1 = sumInvInertia * this->b2CrossA1;
			Vec3 v2 = sumInvInertia * this->c2CrossA1;

			Mat2x2 effectiveMass;
			effectiveMass.setRow(0, Vec2(dotProduct(this->b2CrossA1, v1), dotProduct(this->b2CrossA1, v2)));
			effectiveMass.setRow(1, Vec2(dotProduct(this->c2CrossA1, v1), dotProduct(this->c2CrossA1, v2)));

			this->invEffectiveMass = getInverse(effectiveMass);
		}

		void warmStart(StackArray<RigidBody*, 2>& bodies)
		{
			Vec3 impulse = this->b2CrossA1 * this->totalLambda[0] + this->c2CrossA1 * this->totalLambda[1];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(Vec3(), bodies[x]->invInertiaTensor * impulse * sign);
			}
		}

		void solveVelocity(StackArray<RigidBody*, 2>& bodies)
		{
			Vec3 angularVel[2];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				angularVel[x] = bodies[x]->angularVelocity;
			}

			Vec3 deltaAngularVel = angularVel[0]- angularVel[1];
			Vec2 jv = Vec2(dotProduct(this->b2CrossA1, deltaAngularVel), dotProduct(this->c2CrossA1, deltaAngularVel));
			Vec2 lambda = this->invEffectiveMass * jv;

			this->totalLambda += lambda;

			Vec3 impulse = this->b2CrossA1 * lambda[0] + this->c2CrossA1 * lambda[1];
			for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
				decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
				bodies[x]->updateLinearAndAngularVelocity(Vec3(), bodies[x]->invInertiaTensor * impulse * sign);
			}
		}

		void solvePosition(StackArray<RigidBody*, 2>& bodies, const decimal& BaumgarteFactor)
		{
			Vec2 C = Vec2(dotProduct(this->a1, this->b2), dotProduct(this->a1, this->c2));
			if (magnitudeSq(C) != decimal(0.0)) {

				Vec2 lambda = -(this->invEffectiveMass * C) * BaumgarteFactor;

				Vec3 impulse = this->b2CrossA1 * lambda[0] + this->c2CrossA1 * lambda[1];
				for (uint32 x = 0; x < 2; ++x) if (bodies[x]) {
					decimal sign = x == 0 ? decimal(-1.0) : decimal(1.0);
					bodies[x]->updatePositionAndOrientaion(Vec3(), bodies[x]->invInertiaTensor * impulse * sign);
				}
			}
		}
	};
}

#endif

