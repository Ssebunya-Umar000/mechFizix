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

#ifndef CPTOLS_H
#define CPTOLS_H

#include"../lineSegment.h"

namespace mech {

	template<typename T>
	static Vec3 closestPointToLineSegment(const T& convexShape, const LineSegment& lineSegment, const decimal& tolerance = decimal(0.01))
	{
		BEGIN_PROFILE("closestPointToLineSegment");

		Vec3 closestA = convexShape.closestPoint(lineSegment.pointA);
		Vec3 closestB = convexShape.closestPoint(lineSegment.pointB);

		Vec3 pointA = lineSegment.closestPoint(closestA);
		Vec3 pointB = lineSegment.closestPoint(closestB);

		decimal magA = magnitudeSq(closestA - pointA);
		decimal magB = magnitudeSq(closestB - pointB);

		Vec3 closest = nanVEC3;

		if (magnitudeSq(crossProduct(pointA - closestA, pointB - closestB)) == decimal(0.0) && dotProduct(pointA - closestA, pointB - closestB) > decimal(0.0)) {

			if (magA == magB) {
				closest = (closestA + closestB) / decimal(2.0);
			}
			else {
				closest = magnitudeSq(closestA - lineSegment.pointA) <= magnitudeSq(closestB - lineSegment.pointB) ? closestA : closestB;
			}
		}
		else {

			closest = magA < magB ? closestA : closestB;

			while (magnitudeSq(pointA - pointB) > tolerance) {

				Vec3 p = (pointA + pointB) / decimal(2.0);
				closest = convexShape.closestPoint(p);

				decimal m = magnitudeSq(closest - p);

				if (magA > magB && magA > m) {
					magA = m;
					pointA = p;
				}
				else {
					magB = m;
					pointB = p;
				}
			}
		}

		END_PROFILE;
		return closest;
	}
}

#endif

