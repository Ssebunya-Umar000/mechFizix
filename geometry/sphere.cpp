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

#include"sphere.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"../math/transform.h"

namespace mech {

	AABB Sphere::toAABB()
	{
		return AABB(this->center - Vec3(this->radius), this->center + Vec3(this->radius));
	}

	Vec3 Sphere::getSupportPoint(const Vec3& direction) const
	{
		decimal len = magnitude(direction);
		if (len <= decimal(0.0)) return nanVEC3;
		return this->center + direction * this->radius / len;
	}

	void Sphere::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		decimal len = magnitude(direction);
		if (len <= decimal(0.0)) return;
		max = this->center + direction * this->radius / len;
		min = this->center - direction * this->radius / len;
	}

	Vec3 Sphere::closestPoint(const Vec3& point) const
	{
		return this->contains(point) ? point : this->center + (normalise(point - this->center) * this->radius);
	}

	bool Sphere::contains(const Vec3& point) const
	{
		return magnitudeSq(this->center - point) <= (square(this->radius));
	}

	bool Sphere::contains(const ConvexHull& convexHull) const
	{
		for (uint32 i = 0; i < convexHull.vertices.size(); ++i)
			if (!this->contains(convexHull.vertices[i])) return false;

		return true;
	}

	bool Sphere::contains(const AABB& aabb) const
	{
		StackArray<Vec3, 8> v = aabb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool Sphere::contains(const OBB& obb) const
	{
		StackArray<Vec3, 8> v = obb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool Sphere::contains(const Capsule& capsule) const
	{
		if (this->contains(capsule.capsuleLine) == false) return false;

		return magnitudeSq(capsule.pointA - this->closestPoint(capsule.pointA)) < square(capsule.radius) &&
			magnitudeSq(capsule.pointB - this->closestPoint(capsule.pointB)) < square(capsule.radius);
	}

	bool Sphere::contains(const Sphere& sphere) const
	{
		if (this->contains(sphere.center) == false) return false;
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) < square(this->radius);
	}

	bool Sphere::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i)
			if (!this->contains(polygon.vertices[i])) return false;

		return true;
	}

	bool Sphere::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool Sphere::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool Sphere::intersects(const ConvexHull& convexHull) const
	{
		return magnitudeSq(this->center - convexHull.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const AABB& aabb) const
	{
		return magnitudeSq(this->center - aabb.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const OBB& obb) const
	{
		return magnitudeSq(this->center - obb.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Capsule& capsule) const
	{
		return magnitudeSq(this->center - capsule.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(this->center - sphere.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Plane& plane) const
	{
		return magnitudeSq(this->center - plane.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Polygon& polygon) const
	{
		return magnitudeSq(this->center - polygon.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Triangle& triangle) const
	{
		return magnitudeSq(this->center - triangle.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Line& line) const
	{
		return magnitudeSq(this->center - line.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const Ray& ray) const
	{
		return magnitudeSq(this->center - ray.closestPoint(this->center)) <= square(this->radius);
	}

	bool Sphere::intersects(const LineSegment& lineSegment) const
	{
		return magnitudeSq(this->center - lineSegment.closestPoint(this->center)) <= square(this->radius);
	}

	void Sphere::transform(const Transform3D& t)
	{
		this->center = t * this->center;
	}

	Sphere Sphere::transformed(const Transform3D& t) const
	{
		Sphere s = *this;
		s.transform(t);
		return s;
	}
}