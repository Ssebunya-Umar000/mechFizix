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

#ifndef SAT_H
#define SAT_H

#include"../../../math/vec.h"

namespace mech {

	template<typename T1, typename T2>
	bool SATOverlap(const T1& convexObject1, const T2& convexObject2, const Vec3& axis)
	{
		if (almostEqual(magnitudeSq(axis), decimal(0.0))) return true;

		Vec3 s1, s2;
		convexObject1.getSupportPoints(axis, s1, s2);
		decimal min1 = dotProduct(axis, s1);
		decimal max1 = dotProduct(axis, s2);

		Vec3 s3, s4;
		convexObject2.getSupportPoints(axis, s3, s4);
		decimal min2 = dotProduct(axis, s3);
		decimal max2 = dotProduct(axis, s4);

		return (min2 <= max1) && (min1 <= max2);
	}
}

#endif

