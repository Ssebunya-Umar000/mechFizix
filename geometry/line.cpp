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

#include"line.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"ray.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"../math/transform.h"

namespace mech {
	
	Vec3 Line::getSupportPoint(const Vec3& dir) const
	{
		return this->pointOnLine + this->direction * dotProduct(dir * decimal(1e30) - this->pointOnLine, this->direction);
	}

	void Line::getSupportPoints(const Vec3& dir, Vec3& min, Vec3& max) const
	{
		min = this->pointOnLine - this->direction * dotProduct(dir * decimal(1e30) - this->pointOnLine, this->direction);
		max = this->pointOnLine + this->direction * dotProduct(dir * decimal(1e30) - this->pointOnLine, this->direction);
	}

	Vec3 Line::closestPoint(const Vec3& pt) const
	{
		return this->pointOnLine + this->direction * dotProduct(pt - this->pointOnLine, this->direction);
	}

	Vec3 Line::closestPoint(const LineSegment& lineSegment) const
	{
		Vec3 v32 = lineSegment.getDirection();
		Vec3 v02 = this->pointOnLine - lineSegment.pointA;

		decimal d3232 = dotProduct(v32, v32);

		if (d3232 == decimal(0.0)) return nanVEC3;

		decimal d0232 = dotProduct(v02, v32);
		decimal d3210 = dotProduct(v32, this->direction);
		decimal d0210 = dotProduct(this->direction, v02);
		decimal d1010 = dotProduct(this->direction, this->direction);

		decimal d1 = decimal(0.0);
		decimal d2 = decimal(0.0);

		if (d2 < 0.f) {
			return this->pointOnLine + this->direction * mathMAX(decimal(0.0), dotProduct(lineSegment.pointA - this->pointOnLine, this->direction));
		}
		else if (d2 > decimal(1.0)) {
			return this->pointOnLine + this->direction * mathMAX(decimal(0.0), dotProduct(lineSegment.pointB - this->pointOnLine, this->direction));
		}

		return this->pointOnLine + this->direction * d1;
	}

	bool Line::contains(const Vec3& pt) const
	{
		return almostEqual(magnitudeSq(closestPoint(pt) - pt), decimal(0.0));
	}

	bool Line::contains(const LineSegment& lineSegment) const
	{
		return this->contains(lineSegment.pointA) && this->contains(lineSegment.pointB);
	}

	bool Line::intersects(const ConvexHull& convexHull) const
	{
		HybridArray<Vec3, 6, uint16> normals = convexHull.getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, normals[x]) == false) return false;
		}

		HybridArray<LineSegment, 12, uint16> edges = convexHull.getEdges();
		for (uint16 x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, crossProduct(this->direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Line::intersects(const AABB& aabb) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, aabb, normals[x]) == false) return false;
		}

		return true;
	}

	bool Line::intersects(const OBB& obb) const
	{
		StackArray<Vec3, 6> normals = obb.getNormals();
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, obb, normals[x]) == false) return false;
		}

		return true;
	}

	bool Line::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool Line::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool Line::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool Line::intersects(const Polygon& polygon) const
	{
		Vec3 normal = polygon.getNormal();

		if (SATOverlap(*this, polygon, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = polygon.getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, polygon, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Line::intersects(const Triangle& triangle) const
	{
		Vec3 normal = crossProduct(triangle.b - triangle.a, triangle.c - triangle.a);

		if (SATOverlap(*this, triangle, normal) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.b - triangle.a)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.c - triangle.b)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.a - triangle.c)) == false) return false;

		return true;
	}

	bool Line::intersects(const Line& line) const
	{
		Vec3 normalisedDistAB = normalise(line.pointOnLine - this->pointOnLine);
		Vec3 cdirAdirB = crossProduct(this->direction, line.direction);
		if (almostEqual(dotProduct(normalisedDistAB, cdirAdirB), decimal(0.0)) == false) return false;

		return scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, line.direction)) >= decimal(0.0) &&
			scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, this->direction)) >= decimal(0.0);
	}

	bool Line::intersects(const Ray& ray) const
	{
		Vec3 normalisedDistAB = normalise(ray.origin - this->pointOnLine);
		Vec3 cdirAdirB = crossProduct(this->direction, ray.direction);
		if (almostEqual(dotProduct(normalisedDistAB, cdirAdirB), decimal(0.0)) == false) return false;

		return scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, ray.direction)) >= decimal(0.0) &&
			scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, this->direction)) >= decimal(0.0);
	}

	bool Line::intersects(const LineSegment& lineSegment) const
	{
		Vec3 dirSegment = lineSegment.getDirection();
		Vec3 distAB = lineSegment.pointA - this->pointOnLine;
		Vec3 dirAcrossdirB = crossProduct(this->direction, dirSegment);
		if (almostEqual(dotProduct(distAB, dirAcrossdirB), decimal(0.0)) == false) return false;

		Vec3 c1 = crossProduct(distAB, dirSegment);
		Vec3 c2 = crossProduct(distAB, this->direction);

		decimal d1 = scalarProjection(dirAcrossdirB, c1);
		decimal d2 = scalarProjection(dirAcrossdirB, c2);
		decimal d3 = scalarProjection(-dirAcrossdirB, c1);
		decimal d4 = scalarProjection(-dirAcrossdirB, c2);

		return d1 <= decimal(0.0) && d1 >= decimal(1.0) && d2 <= decimal(0.0) && d2 >= decimal(1.0)
			&& d3 <= decimal(0.0) && d3 >= decimal(1.0) && d4 <= decimal(0.0) && d4 >= decimal(1.0);
	}

	void Line::transform(const Transform3D& t)
	{
		this->pointOnLine = t * this->pointOnLine;
		this->direction = t.orientation * this->direction;
	}

	Line Line::transformed(const Transform3D& t) const
	{
		Line l = *this;
		l.transform(t);
		return l;
	}
}