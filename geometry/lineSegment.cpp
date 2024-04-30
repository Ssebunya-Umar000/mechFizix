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

#include"lineSegment.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"../math/transform.h"

namespace mech { 

	Ray LineSegment::toRay() const
	{ 
		return Ray(this->pointA, getDirection());
	}

	Vec3 LineSegment::getSupportPoint(const Vec3& direction) const
	{
		return dotProduct(direction, this->pointB - this->pointA) >= decimal(0.0) ? this->pointB : this->pointA;
	}

	void LineSegment::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		min = dotProduct(direction, this->pointB - this->pointA) <= decimal(0.0) ? this->pointB : this->pointA;
		max = dotProduct(direction, this->pointB - this->pointA) >= decimal(0.0) ? this->pointB : this->pointA;
	}

	Vec3 LineSegment::closestPoint(const Vec3& point) const
	{
		Vec3 lineDir = this->getDirection();
		return this->pointA + (lineDir * clamp(scalarProjection(lineDir, point - this->pointA), decimal(0.0), decimal(1.0)));
	}

	Vec3 LineSegment::closestPoint(const LineSegment& lineSegment) const
	{
		Vec3 d1 = this->getDirection();
		Vec3 d2 = lineSegment.getDirection();

		decimal d2d2 = magnitudeSq(d2);
		decimal d1d2 = dotProduct(d1, d2);

		decimal denominator = dotProduct(d1, d1) * d2d2 - square(d1d2);

		if (denominator == decimal(0.0)) return this->pointA;

		Vec3 dist = this->pointA - lineSegment.pointA;
		decimal t = clamp((d1d2 * dotProduct(d2, dist) - dotProduct(d1, dist) * d2d2) / denominator, decimal(0.0), decimal(1.0));
		return this->pointA + d1 * t;
	}

	bool LineSegment::contains(const Vec3& point) const
	{
		return magnitudeSq(this->closestPoint(point) - point) < mathEPSILON;
	}

	bool LineSegment::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool LineSegment::intersects(const ConvexHull& convexHull) const
	{
		HybridArray<Vec3, 6, uint16> normals = convexHull.getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, normals[x]) == false) return false;
		}

		Vec3 dir = this->getDirection();
		HybridArray<LineSegment, 12, uint16> edges = convexHull.getEdges();
		for (uint16 x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, crossProduct(dir, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool LineSegment::intersects(const AABB& aabb) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, aabb, normals[x]) == false) return false;
		}

		return true;
	}

	bool LineSegment::intersects(const OBB& obb) const
	{
		StackArray<Vec3, 6> normals = obb.getNormals();
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, obb, normals[x]) == false) return false;
		}

		return true;
	}

	bool LineSegment::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool LineSegment::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool LineSegment::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool LineSegment::intersects(const Polygon& polygon) const
	{
		Vec3 normal = polygon.getNormal();

		if (SATOverlap(*this, polygon, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = polygon.getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, polygon, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool LineSegment::intersects(const Triangle& triangle) const
	{
		Vec3 normal = crossProduct(triangle.b - triangle.a, triangle.c - triangle.a);

		if (SATOverlap(*this, triangle, normal) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.b - triangle.a)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.c - triangle.b)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.a - triangle.c)) == false) return false;

		return true;
	}

	bool LineSegment::intersects(const Line& line) const
	{
		Vec3 dirSegment = this->getDirection();
		Vec3 distAB = this->pointA - line.pointOnLine;
		Vec3 cdirAdirB = crossProduct(line.direction, dirSegment);
		if (almostEqual(dotProduct(distAB, cdirAdirB), decimal(0.0)) == false) return false;

		decimal d1 = scalarProjection(cdirAdirB, crossProduct(distAB, dirSegment));
		decimal d2 = scalarProjection(cdirAdirB, crossProduct(distAB, line.direction));

		return d1 <= decimal(0.0) && d2 <= decimal(0.0);
	}

	bool LineSegment::intersects(const Ray& ray) const
	{
		Vec3 dirSegment = this->getDirection();
		Vec3 distAB = this->pointA - ray.origin;
		Vec3 cdirAdirB = crossProduct(ray.direction, dirSegment);
		if (almostEqual(dotProduct(distAB, cdirAdirB), decimal(0.0)) == false) return false;

		decimal d1 = scalarProjection(cdirAdirB, crossProduct(distAB, dirSegment));
		decimal d2 = scalarProjection(cdirAdirB, crossProduct(distAB, ray.direction));

		return d1 <= decimal(0.0) && d1 >= decimal(1.0) && d2 <= decimal(0.0) && d2 >= decimal(1.0);
	}

	bool LineSegment::intersects(const LineSegment& lineSegment) const
	{
		Vec3 dirA = this->getDirection();
		Vec3 dirB = lineSegment.getDirection();
		Vec3 ss = lineSegment.pointA - this->pointA;

		Vec3 dirAcrossdirB = crossProduct(dirA, dirB);
		if (dotProduct(ss, dirAcrossdirB) != decimal(0.0)) return false;

		decimal d1 = scalarProjection(dirAcrossdirB, crossProduct(ss, dirB));
		decimal d2 = scalarProjection(dirAcrossdirB, crossProduct(ss, dirA));

		return d1 <= decimal(0.0) && d1 >= decimal(1.0) && d2 <= decimal(0.0) && d2 >= decimal(1.0);
	}

	void LineSegment::transform(const Transform3D& t)
	{
		this->pointA = t * this->pointA;
		this->pointB = t * this->pointB;
	}

	LineSegment LineSegment::transformed(const Transform3D& t) const
	{
		LineSegment l = *this;
		l.transform(t);
		return l;
	}
}