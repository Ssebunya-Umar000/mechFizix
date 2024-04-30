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

#include"capsule.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"sphere.h"
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
	
	AABB Capsule::toAABB()
	{
		Vec3 d = Vec3(this->radius);
		return AABB(minVec(this->pointA, this->pointB) - d, maxVec(this->pointA, this->pointB) + d);
	}

	Vec3 Capsule::getSupportPoint(const Vec3& direction) const
	{
		decimal len = magnitude(direction);
		if (len == decimal(0.0)) return nanVEC3;
		return (dotProduct(direction, this->pointB - this->pointA) >= decimal(0.0) ? this->pointB : this->pointA) + direction * this->radius / len;
	}

	void Capsule::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		decimal len = magnitude(direction);
		if (len == decimal(0.0)) return;
		min = (dotProduct(direction, this->pointB - this->pointA) >= decimal(0.0) ? this->pointB : this->pointA) - direction * this->radius / len;
		max = (dotProduct(direction, this->pointB - this->pointA) >= decimal(0.0) ? this->pointB : this->pointA) + direction * this->radius / len;
	}

	Vec3 Capsule::closestPoint(const Vec3& point) const
	{
		Vec3 closestToLine = this->capsuleLine.closestPoint(point);
		Vec3 d = point - closestToLine;
		return magnitudeSq(d) > square(this->radius) ? closestToLine + (normalise(d) * this->radius) : point;
	}

	bool Capsule::contains(const Vec3& point) const
	{
		return magnitudeSq(point - this->capsuleLine.closestPoint(point)) < square(this->radius);
	}

	bool Capsule::contains(const ConvexHull& convexHull) const
	{
		for (uint32 i = 0; i < convexHull.vertices.size(); ++i) {
			if (this->contains(convexHull.vertices[i]) == false) return false;
		}

		return true;
	}

	bool Capsule::contains(const AABB& aabb) const
	{
		StackArray<Vec3, 8> v = aabb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool Capsule::contains(const OBB& obb) const
	{
		StackArray<Vec3, 8> v = obb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool Capsule::contains(const Capsule& capsule) const
	{
		if (this->contains(capsule.capsuleLine) == false) return false;

		return magnitudeSq(capsule.pointA - this->closestPoint(capsule.pointA)) < square(capsule.radius) && 
			   magnitudeSq(capsule.pointB - this->closestPoint(capsule.pointB)) < square(capsule.radius);
	}

	bool Capsule::contains(const Sphere& sphere) const
	{
		if (this->contains(sphere.center) == false) return false;
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) < square(sphere.radius);
	}

	bool Capsule::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i) {
			if (this->contains(polygon.vertices[i]) == false) return false;
		}

		return true;
	}

	bool Capsule::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool Capsule::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool Capsule::intersects(const ConvexHull& convexHull) const
	{
		Vec3 closest = convexHull.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const AABB& aabb) const
	{
		Vec3 closest = aabb.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const OBB& obb) const
	{
		Vec3 closest = obb.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->capsuleLine.closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool Capsule::intersects(const Plane& plane) const
	{
		Vec3 closest = plane.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Polygon& polygon) const
	{
		Vec3 closest = polygon.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Triangle& triangle) const
	{
		Vec3 closest = triangle.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Line& line) const
	{
		Vec3 closest = line.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const Ray& ray) const
	{
		Vec3 closest = ray.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	bool Capsule::intersects(const LineSegment& lineSegment) const
	{
		Vec3 closest = lineSegment.closestPoint(this->capsuleLine);
		return magnitudeSq(closest - this->capsuleLine.closestPoint(closest)) <= square(this->radius);
	}

	void Capsule::transform(const Transform3D& t)
	{
		this->capsuleLine.transform(t);
	}

	Capsule Capsule::transformed(const Transform3D& t) const
	{
		Capsule c = *this;
		c.transform(t);
		return c;
	}
}