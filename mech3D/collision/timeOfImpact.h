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

#ifndef TIMEOTIMPACT_H
#define TIMEOFIMPACT_H

#include"../../math/transform.h"
#include"../geometry/algorithms/GJK.h"

namespace mech {

#define MAXIMUM_NUM_POINTS 10
#define MAXIMUM_TOI_ITERATIONS 50
#define MAXIMUM_ROOT_ITERATIONS 50

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class TOIState : byte { overlaping = 1 << 0, touching = 1 << 1, separated = 1 << 2 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TOIResult {
		decimal t = decimalNAN;
		TOIState state = TOIState::separated;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	template<class T1, class T2>
	TOIResult timeOfImpact(const T1& convexObject1, const T2& convexObject2, const Transform3DPair& transformA, const Transform3DPair& transformB, const decimal& tMax = decimal(1.0), const decimal& tolerance = decimal(0.25))
	{

		struct SeperationFunction {

			Vec3 axis;
			Vec3 supportA = nanVEC3;
			Vec3 supportB = nanVEC3;
			const Transform3DPair& transformA;
			const Transform3DPair& transformB;
			const T1& convexObjectA;
			const T2& convexObjectB;

			SeperationFunction(const decimal& t, const T1& object1, const T2& object2, const Transform3DPair& tA, const Transform3DPair& tB, const GJKResult& gjk) : convexObjectA(object1), convexObjectB(object2), transformA(tA), transformB(tB)
			{
				this->axis = normalise(gjk.closest2 - gjk.closest1);
			}

			decimal calculateSeparation(const decimal & t)
			{
				Transform3D tA = transformA.interpolate(t);
				Transform3D tB = transformB.interpolate(t);

				this->supportA = this->convexObjectA.getSupportPoint(tA.toMatrix().toMat3x3() * this->axis);
				this->supportB = this->convexObjectB.getSupportPoint(tB.toMatrix().toMat3x3() * -this->axis);

				return dotProduct(tB.toMatrix() * this->supportB - tA.toMatrix() * this->supportA, this->axis);
			}

			decimal reCalculateSeparation(const decimal & t)
			{
				Transform3D tA = transformA.interpolate(t);
				Transform3D tB = transformB.interpolate(t);

				return dotProduct(tB.toMatrix() * this->supportB - tA.toMatrix() * this->supportA, this->axis);
			}
		};

		TOIResult result;
		result.t = tMax;

		decimal toleranceSq = square(tolerance);
		decimal t1 = decimal(0.0);

		byte toiIterations = 0;
		while (toiIterations < MAXIMUM_TOI_ITERATIONS) {

			GJKResult gjk = GJKAlgorithm(convexObject1.transformed(transformA.interpolate(t1).toMatrix()), convexObject2.transformed(transformB.interpolate(t1).toMatrix()), false);

			if (gjk.overlap == true) {
				result.state = TOIState::overlaping;
				result.t = t1;
				return result;
			}
			
			if (magnitudeSq(gjk.closest2 - gjk.closest1) < toleranceSq) {
				result.state = TOIState::touching;
				result.t = t1;
				return result;
			}

			SeperationFunction fcn(t1, convexObject1, convexObject2, transformA, transformB, gjk);

			decimal t2 = tMax;
			byte deepestPointIterations = 0;
			while (deepestPointIterations < MAXIMUM_NUM_POINTS) {

				decimal s2 = fcn.calculateSeparation(t2);

				if (s2 > tolerance) {
					result.state = TOIState::separated;
					result.t = t2;
					return result;
				}

				if (s2 > -tolerance) {
					t1 = t2;
					break;
				}

				decimal s1 = fcn.reCalculateSeparation(t1);

				if (s1 <= tolerance) {
					result.state = TOIState::touching;
					result.t = t1;
					return result;
				}

				byte rootIterations = 0;
				decimal rootT1 = t1;
				decimal rootT2 = t2;
				while (rootIterations < MAXIMUM_ROOT_ITERATIONS) {

					decimal t;
					if (rootIterations & 1) {
						t = rootT1 + (-s1) * (rootT2 - rootT1) / (s2 - s1);
					}
					else {
						t = decimal(0.5) * (rootT1 + rootT2);
					}

					decimal s = fcn.reCalculateSeparation(t);

					if (mathABS(s) < tolerance) {
						t2 = t;
						break;
					}

					if (s > decimal(0.0)) {
						rootT1 = t;
						s1 = s;
					}
					else {
						rootT2 = t;
						s2 = s;
					}

					++rootIterations;
				}

				assert(rootIterations < MAXIMUM_ROOT_ITERATIONS);
				++deepestPointIterations;
			}

			++toiIterations;
		}

		assert(toiIterations < MAXIMUM_TOI_ITERATIONS);

		return TOIResult();
	}
}

#endif


