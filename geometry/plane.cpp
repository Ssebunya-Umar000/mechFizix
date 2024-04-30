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
#include"algorithms/CPToLS.h"

namespace mech {
	
	Vec3 Plane::getSupportPoint(const Vec3& direction) const
	{
		Vec3 point = direction * decimal(1e30);
		return point + this->normal * (dotProduct(this->normal, point) - this->distance); //wrong!!
	}

	void Plane::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		Vec3 point = direction * decimal(1e30);
		min = max = point + this->normal * (dotProduct(this->normal, point) - this->distance); //wrong!!
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
		return closestPointToLineSegment(*this, lineSegment);
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
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool Plane::intersects(const ConvexHull& convexHull) const
	{
		return SATOverlap(*this, convexHull, this->normal); //wrong!!
	}

	bool Plane::intersects(const AABB& aabb) const
	{
		return SATOverlap(*this, aabb, this->normal); //wrong!!
	}

	bool Plane::intersects(const OBB& obb) const
	{
		return SATOverlap(*this, obb, this->normal); //wrong!!
	}

	bool Plane::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool Plane::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool Plane::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, this->normal); //wrong!!
	}

	bool Plane::intersects(const Polygon& polygon) const
	{
		return SATOverlap(*this, polygon, this->normal); //wrong!!
	}

	bool Plane::intersects(const Triangle& triangle) const
	{
		return SATOverlap(*this, triangle, this->normal); //wrong!!
	}

	bool Plane::intersects(const Line& line) const
	{
		return SATOverlap(*this, line, this->normal); //wrong!!
	}

	bool Plane::intersects(const Ray& ray) const
	{
		return SATOverlap(*this, ray, this->normal); //wrong!!
	}

	bool Plane::intersects(const LineSegment& lineSegment) const
	{
		return this->getDistanceFromPlane(lineSegment.pointA) <= decimal(0.0) != this->getDistanceFromPlane(lineSegment.pointB) <= decimal(0.0);
	}

	Vec3 Plane::clip(const LineSegment& lineSegment) const
	{
		if (this->intersects(lineSegment)) {
			return lineSegment.toRay().rayCastPoint(*this);
		}

		return nanVEC3;
	}
}