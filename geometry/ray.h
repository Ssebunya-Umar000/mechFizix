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

#ifndef RAY_H
#define RAY_H

#include"../math/vec.h"

namespace mech {

	struct ConvexHull;
	struct AABB;
	struct OBB;
	struct Capsule;
	struct Sphere;
	struct Plane;
	struct Polygon;
	struct Triangle;
	struct Line;
	struct LineSegment;

	struct Transform3D;

	struct Ray {
		Vec3 origin;
		Vec3 direction;

		Ray() {}
		explicit Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
		~Ray() {};

		Vec3 getSupportPoint(const Vec3& dir) const;
		void getSupportPoints(const Vec3& dir, Vec3& min, Vec3& max) const;

		Vec3 closestPoint(const Vec3& point) const;
		Vec3 closestPoint(const LineSegment& lineSegment) const;

		decimal rayCastTime(const ConvexHull& convexHull) const;
		decimal rayCastTime(const AABB& aabb) const;
		decimal rayCastTime(const OBB& obb) const;
		decimal rayCastTime(const Capsule& capsule) const;
		decimal rayCastTime(const Sphere& sphere) const;
		decimal rayCastTime(const Plane& plane) const;
		decimal rayCastTime(const Polygon& polygon) const;
		decimal rayCastTime(const Triangle& triangle) const;
		decimal rayCastTime(const Line& line) const;
		decimal rayCastTime(const Ray& ray) const;

		Vec3 rayCastPoint(const ConvexHull& convexHull) const;
		Vec3 rayCastPoint(const AABB& aabb) const;
		Vec3 rayCastPoint(const OBB& obb) const;
		Vec3 rayCastPoint(const Capsule& capsule) const;
		Vec3 rayCastPoint(const Sphere& sphere) const;
		Vec3 rayCastPoint(const Plane& plane) const;
		Vec3 rayCastPoint(const Polygon& polygon) const;
		Vec3 rayCastPoint(const Triangle& triangle) const;
		Vec3 rayCastPoint(const Line& line) const;
		Vec3 rayCastPoint(const Ray& ray) const;

		bool contains(const Vec3& point);
		bool contains(const LineSegment& lineSegment);

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
		Ray transformed(const Transform3D& t) const;

		String toString() const { return "Ray(origin:" + this->origin.toString() + " direction:" + this->direction.toString() + ")"; }
	};
}

#endif

