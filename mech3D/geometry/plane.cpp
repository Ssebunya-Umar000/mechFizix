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

#include"plane.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"

namespace mech {
	
	Vec3 Plane::getSupportPoint(const Vec3& direction) const
	{
		Vec3 point = direction * decimal(1e30);
		return point + this->normal * (dotProduct(this->normal, point) - this->distance);
	}

	void Plane::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		Vec3 point = direction * decimal(1e30);
		min = point - this->normal * (dotProduct(this->normal, point) - this->distance);
		max = point + this->normal * (dotProduct(this->normal, point) - this->distance);
	}

	decimal Plane::getDistanceFromPlane(const Vec3& point) const
	{
		return dotProduct(point, this->normal) - this->distance;
	}

	Vec3 Plane::closestPoint(const Vec3& point) const
	{
		return point - this->normal * (dotProduct(this->normal, point) - this->distance);
	}

	Vec3 Plane::closestPoint(const LineSegment& lineSegment) const
	{
		if (this->intersects(lineSegment)) {
			return lineSegment.toRay().rayCastPoint(*this);
		}

		Vec3 p1 = this->closestPoint(lineSegment.start);
		Vec3 p2 = this->closestPoint(lineSegment.end);
		return magnitudeSq(lineSegment.start - p1) < magnitudeSq(lineSegment.end - p2) ? p1 : p2;
	}

	bool Plane::contains(const Vec3& point) const
	{
		return almostEqual(dotProduct(point, this->normal), this->distance);
	}

	bool Plane::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i)
			if (!this->contains(polygon.vertices[i])) return false;

		return true;
	}

	bool Plane::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool Plane::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.start) && contains(lineSegment.end);
	}

	bool Plane::intersects(const ConvexHull& convexHull) const
	{
		return SATOverlap(*this, convexHull, this->normal);
	}

	bool Plane::intersects(const AABB& aabb) const
	{
		return SATOverlap(*this, aabb, this->normal);
	}

	bool Plane::intersects(const OBB& obb) const
	{
		return SATOverlap(*this, obb, this->normal);
	}

	bool Plane::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= capsule.radius * capsule.radius;
	}

	bool Plane::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= sphere.radius * sphere.radius;
	}

	bool Plane::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, this->normal);
	}

	bool Plane::intersects(const Polygon& polygon) const
	{
		return SATOverlap(*this, polygon, this->normal);
	}

	bool Plane::intersects(const Triangle& triangle) const
	{
		return SATOverlap(*this, triangle, this->normal);
	}

	bool Plane::intersects(const Line& line) const
	{
		return SATOverlap(*this, line, this->normal);
	}

	bool Plane::intersects(const Ray& ray) const
	{
		return SATOverlap(*this, ray, this->normal);
	}

	bool Plane::intersects(const LineSegment& lineSegment) const
	{
		Vec3 dir = lineSegment.getDirection();
		if (dotProduct(this->normal, dir) == decimal(0.0)) return false;
		decimal t = (this->distance - dotProduct(this->normal, lineSegment.start)) / dotProduct(this->normal, dir);
		return t >= decimal(0.0) && t <= decimal(1.0);
	}

	Vec3 Plane::clip(const LineSegment& lineSegment) const
	{
		Vec3 dir = lineSegment.getDirection();

		decimal normalDotdir = dotProduct(this->normal, dir);

		if (almostEqual(normalDotdir, decimal(0.0))) return nanVEC3;

		decimal t = (this->distance - dotProduct(this->normal, lineSegment.start)) / normalDotdir;

		if (t >= decimal(0.0) && t <= decimal(1.0)) return lineSegment.start + dir * t;

		return nanVEC3;
	}
}