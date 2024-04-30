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

#ifndef PLANE_H
#define PLANE_H

#include"../math/vec.h"

namespace mech {

	struct ConvexHull;
	struct AABB;
	struct OBB;
	struct Capsule;
	struct Sphere;
	struct Polygon;
	struct Triangle;
	struct Line;
	struct Ray;
	struct LineSegment;

	struct Plane {
		Vec3 normal;
		decimal distance = decimal(0.0);

		Plane() {}
		explicit Plane(const Vec3& normalIn, decimal distanceIn) :normal(normalIn), distance(distanceIn) { normal = normalise(normal); }
		explicit Plane(const Vec3& p1, const Vec3& p2, const Vec3& p3) { this->normal = normalise(crossProduct(p2 - p1, p3 - p1)); this->distance = dotProduct(this->normal, p1); }
		~Plane() {};

		Vec3 getSupportPoint(const Vec3& direction) const;
		void getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const;

		decimal getDistanceFromPlane(const Vec3& point) const;

		Vec3 closestPoint(const Vec3& point) const;
		Vec3 closestPoint(const LineSegment& lineSegment) const;

		bool contains(const Vec3& point) const;
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

		Vec3 clip(const LineSegment& lineSegment) const;

		String toString() const { return "Plane(normal:" + this->normal.toString() + " distance:" + toString2(this->distance) + ")"; }
	};
}

#endif