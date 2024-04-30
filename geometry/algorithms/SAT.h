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

#ifndef SAT_H
#define SAT_H

#include"../../math/vec.h"

namespace mech {

	template<typename T1, typename T2>
	static bool SATOverlap(const T1& convexShapeA, const T2& convexShapeB, const Vec3& axis)
	{
		if (almostEqual(magnitudeSq(axis), decimal(0.0))) return true;

		Vec3 supportA1, supportA2;
		convexShapeA.getSupportPoints(axis, supportA1, supportA2);
		decimal min1 = dotProduct(axis, supportA1);
		decimal max1 = dotProduct(axis, supportA2);

		Vec3 supportB1, supportB2;
		convexShapeB.getSupportPoints(axis, supportB1, supportB2);
		decimal min2 = dotProduct(axis, supportB1);
		decimal max2 = dotProduct(axis, supportB2);

		return (min2 <= max1) && (min1 <= max2);
	}
}

#endif

