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

#include"obb.h"

#include"convexHull.h"
#include"aabb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"algorithms/CPToLS.h"
#include"../math/transform.h"

namespace mech {

	ConvexHull OBB::toConvexHull() const
	{
		return ConvexHull(HybridArray<Polygon, 6, uint16>(this->getFacePolygons()));
	}

	AABB OBB::toAABB() const
	{
		Vec3 h = absVec(this->orientation.getColumn(0) * this->halfExtents.x) + absVec(this->orientation.getColumn(1) * this->halfExtents.y) + absVec(this->orientation.getColumn(2) * this->halfExtents.z);

		return AABB(this->center - h, this->center + h);
	}

	Vec3 OBB::getSupportPoint(const Vec3& direction) const
	{
		Vec3 pt = this->center;
		Vec3 x = this->orientation.getColumn(0);
		Vec3 y = this->orientation.getColumn(1);
		Vec3 z = this->orientation.getColumn(2);
		pt += x * (dotProduct(direction, x) >= decimal(0.0) ? this->halfExtents.x : -this->halfExtents.x);
		pt += y * (dotProduct(direction, y) >= decimal(0.0) ? this->halfExtents.y : -this->halfExtents.y);
		pt += z * (dotProduct(direction, z) >= decimal(0.0) ? this->halfExtents.z : -this->halfExtents.z);
		return pt;
	}

	void OBB::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		min = this->center;
		max = this->center;
		Vec3 x = this->orientation.getColumn(0);
		Vec3 y = this->orientation.getColumn(1);
		Vec3 z = this->orientation.getColumn(2);
		
		min -= x * (dotProduct(direction, x) >= decimal(0.0) ? this->halfExtents.x : -this->halfExtents.x);
		min -= y * (dotProduct(direction, y) >= decimal(0.0) ? this->halfExtents.y : -this->halfExtents.y);
		min -= z * (dotProduct(direction, z) >= decimal(0.0) ? this->halfExtents.z : -this->halfExtents.z);

		max += x * (dotProduct(direction, x) >= decimal(0.0) ? this->halfExtents.x : -this->halfExtents.x);
		max += y * (dotProduct(direction, y) >= decimal(0.0) ? this->halfExtents.y : -this->halfExtents.y);
		max += z * (dotProduct(direction, z) >= decimal(0.0) ? this->halfExtents.z : -this->halfExtents.z);
	}

	StackArray<Vec3, 8> OBB::getVertices() const
	{
		Vec3 hx = this->orientation.getColumn(0) * this->halfExtents.x;
		Vec3 hy = this->orientation.getColumn(1) * this->halfExtents.y;
		Vec3 hz = this->orientation.getColumn(2) * this->halfExtents.z;

		StackArray<Vec3, 8> vertices;

		vertices[0] = this->center - hx - hy - hz;
		vertices[1] = this->center + hx - hy - hz;
		vertices[2] = this->center - hx + hy - hz;
		vertices[3] = this->center + hx + hy - hz;
		vertices[4] = this->center - hx - hy + hz;
		vertices[5] = this->center + hx - hy + hz;
		vertices[6] = this->center - hx + hy + hz;
		vertices[7] = this->center + hx + hy + hz;

		return vertices;
	}

	StackArray<LineSegment, 12> OBB::getEdges() const
	{
		StackArray<Vec3, 8> vertices = this->getVertices();
		StackArray<LineSegment, 12> edges;

		edges[0] = LineSegment(vertices[0], vertices[1]);
		edges[1] = LineSegment(vertices[0], vertices[2]);
		edges[2] = LineSegment(vertices[0], vertices[4]);
		edges[3] = LineSegment(vertices[1], vertices[3]);
		edges[4] = LineSegment(vertices[1], vertices[5]);
		edges[5] = LineSegment(vertices[2], vertices[3]);
		edges[6] = LineSegment(vertices[2], vertices[6]);
		edges[7] = LineSegment(vertices[3], vertices[7]);
		edges[8] = LineSegment(vertices[4], vertices[6]);
		edges[9] = LineSegment(vertices[4], vertices[5]);
		edges[10] = LineSegment(vertices[5], vertices[7]);
		edges[11] = LineSegment(vertices[6], vertices[7]);

		return edges;
	}

	StackArray<Vec3, 6> OBB::getNormals() const
	{
		StackArray<Vec3, 6> normals;

		normals[0] = this->orientation.getColumn(0);
		normals[1] = this->orientation.getColumn(1);
		normals[2] = this->orientation.getColumn(2);
		normals[3] = -normals[0];
		normals[4] = -normals[1];
		normals[5] = -normals[2];

		return normals;
	}

	StackArray<Polygon, 6> OBB::getFacePolygons() const
	{
		StackArray<Vec3, 8> vertices = this->getVertices();

		Vec3 a1[4] = { vertices[4],vertices[0],vertices[1],vertices[5] };
		Polygon p1 = Polygon(DynamicArray<Vec3, byte>(a1, 4));

		Vec3 a2[4] = { vertices[7],vertices[3],vertices[2],vertices[6] };
		Polygon p2 = Polygon(DynamicArray<Vec3, byte>(a2, 4));

		Vec3 a3[4] = { vertices[1],vertices[3],vertices[7],vertices[5] };
		Polygon p3 = Polygon(DynamicArray<Vec3, byte>(a3, 4));

		Vec3 a4[4] = { vertices[4],vertices[6],vertices[2],vertices[0] };
		Polygon p4 = Polygon(DynamicArray<Vec3, byte>(a4, 4));

		Vec3 a5[4] = { vertices[4],vertices[5],vertices[7],vertices[6] };
		Polygon p5 = Polygon(DynamicArray<Vec3, byte>(a5, 4));

		Vec3 a6[4] = { vertices[0],vertices[2],vertices[3],vertices[1] };
		Polygon p6 = Polygon(DynamicArray<Vec3, byte>(a6, 4));

		Polygon a7[6] = { p1, p2, p3, p4, p5, p6 };
		return StackArray<Polygon, 6>(a7, 6);
	}

	Vec3 OBB::closestPoint(const Vec3& point) const
	{
		Vec3 d = point - this->center;
		Vec3 closest = this->center;
		for (byte i = 0; i < 3; ++i) {
			closest += this->orientation.getColumn(i) * clamp(dotProduct(d, this->orientation.getColumn(i)), -this->halfExtents[i], this->halfExtents[i]);
		}

		return closest;
	}

	Vec3 OBB::closestPoint(const LineSegment& lineSegment) const
	{
		return closestPointToLineSegment(*this, lineSegment);
	}

	bool OBB::contains(const Vec3& point) const
	{
		Vec3 pt = point - this->center;
		return  mathABS(dotProduct(pt, this->orientation.getColumn(0))) < this->halfExtents.x &&
			mathABS(dotProduct(pt, this->orientation.getColumn(1))) < this->halfExtents.y &&
			mathABS(dotProduct(pt, this->orientation.getColumn(2))) < this->halfExtents.z;
	}

	bool OBB::contains(const ConvexHull& convexHull) const
	{
		for (uint32 i = 0; i < convexHull.vertices.size(); ++i) {
			if (this->contains(convexHull.vertices[i]) == false) return false;
		}

		return true;
	}

	bool OBB::contains(const AABB& aabb) const
	{
		StackArray<Vec3, 8> v = aabb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool OBB::contains(const OBB& obb) const
	{
		StackArray<Vec3, 8> v = obb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool OBB::contains(const Capsule& capsule) const
	{
		if (this->contains(capsule.capsuleLine) == false) return false;

		return magnitudeSq(capsule.pointA - this->closestPoint(capsule.pointA)) < square(capsule.radius) &&
			magnitudeSq(capsule.pointB - this->closestPoint(capsule.pointB)) < square(capsule.radius);
	}

	bool OBB::contains(const Sphere& sphere) const
	{
		if (this->contains(sphere.center) == false) return false;

		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) < square(sphere.radius);
	}

	bool OBB::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i) {
			if (this->contains(polygon.vertices[i]) == false) return false;
		}

		return true;
	}

	bool OBB::contains(const Triangle& triangle) const
	{
		return this->contains(triangle.a) && this->contains(triangle.b) && this->contains(triangle.c);
	}

	bool OBB::contains(const LineSegment& lineSegment) const
	{
		return this->contains(lineSegment.pointA) && this->contains(lineSegment.pointB);
	}

	bool OBB::intersects(const ConvexHull& convexHull) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, convexHull, this->orientation.getColumn(x)) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const AABB& aabb) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, aabb, this->orientation.getColumn(x)) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const OBB& obb) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, obb, this->orientation.getColumn(x)) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool OBB::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool OBB::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool OBB::intersects(const Polygon& polygon) const
	{
		if (SATOverlap(*this, polygon, crossProduct(polygon.vertices[1] - polygon.vertices[0], polygon.vertices.back() - polygon.vertices[0])) == false) return false;

		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, polygon, this->orientation.getColumn(x)) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const Triangle& triangle) const
	{
		if (SATOverlap(*this, triangle, crossProduct(triangle.b - triangle.a, triangle.c - triangle.a)) == false) return false;

		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, triangle, this->orientation.getColumn(x)) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const Line& line) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, line, this->orientation.getColumn(x)) == false) return false;
		}

		StackArray<LineSegment, 12> edges = this->getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, line, crossProduct(line.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const Ray& ray) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, ray, this->orientation.getColumn(x)) == false) return false;
		}

		StackArray<LineSegment, 12> edges = this->getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, ray, crossProduct(ray.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool OBB::intersects(const LineSegment& lineSegment) const
	{
		return lineSegment.contains(this->closestPoint(lineSegment));
	}

	void OBB::transform(const Transform3D& t)
	{
		this->center = t * this->center;
		this->orientation.setColumn(0, normalise(t.orientation * this->orientation.getColumn(0)));
		this->orientation.setColumn(1, normalise(t.orientation * this->orientation.getColumn(1)));
		this->orientation.setColumn(2, normalise(t.orientation * this->orientation.getColumn(2)));
	}

	OBB OBB::transformed(const Transform3D& t) const
	{
		OBB o = *this;
		o.transform(t);
		return o;
	}
}