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

#ifndef CAPSULE_H
#define CAPSULE_H

#include"lineSegment.h"

namespace mech {

	struct Capsule {
		decimal radius = decimal(0.0);
		union {
			LineSegment capsuleLine;
			struct {
				Vec3 pointA;
				Vec3 pointB;
			};
		};

		Capsule() {}
		explicit Capsule(decimal r, const Vec3& a, const Vec3& b) : radius(r), pointA(a), pointB(b) {}
		~Capsule() {};

		AABB toAABB();

		Vec3 getSupportPoint(const Vec3& direction) const;
		void getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const;

		Vec3 closestPoint(const Vec3& point) const;

		bool contains(const Vec3& point) const;
		bool contains(const ConvexHull& convexHull) const;
		bool contains(const AABB& aabb) const;
		bool contains(const OBB& obb) const;
		bool contains(const Capsule& capsule) const;
		bool contains(const Sphere& sphere) const;
		bool contains(const Polygon& polygon) const;
		bool contains(const Triangle& triangle) const;
		bool contains(const LineSegment& lineSegment) const;

		bool intersects(const ConvexHull& convexHull) const;
		bool intersects(const AABB& aabb) const;
		bool intersects(const OBB& obb) const;
		bool intersects(const Capsule& capsule) const ;
		bool intersects(const Sphere& sphere) const;
		bool intersects(const Plane& plane) const;
		bool intersects(const Polygon& polygon) const;
		bool intersects(const Triangle& triangle) const;
		bool intersects(const Line& line) const;
		bool intersects(const Ray& ray) const;
		bool intersects(const LineSegment& lineSegment) const;

		void transform(const Transform3D& t);
		Capsule transformed(const Transform3D& t) const;

		String toString() const { return "Capsule(radius:" + toString2(this->radius) + " pointA:" + this->pointA.toString() + " pointB:" + this->pointB.toString() + ")"; }
	};
}

#endif
