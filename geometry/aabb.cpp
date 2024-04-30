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

#include"aabb.h"

#include"convexHull.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"
#include"point.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"algorithms/CPToLS.h"
#include"../math/transform.h"

namespace mech {

	AABB::AABB(const Vec3& minIn, const Vec3& maxIn)
	{
		Vec3 c = (minIn + maxIn) / decimal(2.0);
		Vec3 h = absVec(minIn - maxIn) / decimal(2.0);

		this->min = c - h;
		this->max = c + h;
	}

	ConvexHull AABB::toConvexHull() const
	{
		return ConvexHull(HybridArray<Polygon, 6, uint16>(this->getFacePolygons()));
	}

	OBB AABB::toOBB() const
	{
		OBB obb;

		obb.center = this->getCenter();
		obb.halfExtents = this->getDimensions() / decimal(2.0);

		obb.orientation.setColumn(0, XAXIS);
		obb.orientation.setColumn(1, YAXIS);
		obb.orientation.setColumn(2, ZAXIS);

		return obb;
	}

	Vec3 AABB::getSupportPoint(const Vec3& direction) const
	{
		return Vec3((direction.x >= decimal(0.0) ? this->max.x : this->min.x), (direction.y >= decimal(0.0) ? this->max.y : this->min.y), (direction.z >= decimal(0.0) ? this->max.z : this->min.z));
	}

	void AABB::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		min = Vec3((direction.x <= decimal(0.0) ? this->max.x : this->min.x), (direction.y <= decimal(0.0) ? this->max.y : this->min.y), (direction.z <= decimal(0.0) ? this->max.z : this->min.z));
		max = Vec3((direction.x >= decimal(0.0) ? this->max.x : this->min.x), (direction.y >= decimal(0.0) ? this->max.y : this->min.y), (direction.z >= decimal(0.0) ? this->max.z : this->min.z));
	}

	StackArray<Vec3, 8> AABB::getVertices() const
	{
		StackArray<Vec3, 8> vertices;

		Vec3 xWidth = Vec3(this->max.x - this->min.x, decimal(0.0), decimal(0.0));
		Vec3 yWidth = Vec3(decimal(0.0), this->max.y - this->min.y, decimal(0.0));
		Vec3 zWidth = Vec3(decimal(0.0), decimal(0.0), this->max.z - this->min.z);

		vertices[0] = this->min;
		vertices[1] = this->min + xWidth;

		vertices[2] = this->min + yWidth;
		vertices[3] = this->max - zWidth;

		vertices[4] = this->min + zWidth;
		vertices[5] = this->max - yWidth;

		vertices[6] = this->max - xWidth;
		vertices[7] = this->max;

		return vertices;
	}

	StackArray<LineSegment, 12> AABB::getEdges() const
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

	StackArray<Vec3, 6> AABB::getNormals() const
	{
		StackArray<Vec3, 6> normals;

		normals[0] = XAXIS;
		normals[1] = YAXIS;
		normals[2] = ZAXIS;
		normals[3] = -XAXIS;
		normals[4] = -YAXIS;
		normals[5] = -ZAXIS;

		return normals;
	}

	StackArray<Polygon, 6> AABB::getFacePolygons() const
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

	Vec3 AABB::closestPoint(const Vec3& point) const
	{
		Vec3 closest = point;
		closest.x = closest.x < this->min.x ? this->min.x : closest.x;
		closest.y = closest.y < this->min.y ? this->min.y : closest.y;
		closest.z = closest.z < this->min.z ? this->min.z : closest.z;

		closest.x = closest.x > this->max.x ? this->max.x : closest.x;
		closest.y = closest.y > this->max.y ? this->max.y : closest.y;
		closest.z = closest.z > this->max.z ? this->max.z : closest.z;

		return closest;
	}

	Vec3 AABB::closestPoint(const LineSegment& lineSegment) const
	{
		return closestPointToLineSegment(*this, lineSegment);
	}

	bool AABB::contains(const Vec3& point) const
	{
		return point.x > this->min.x && point.x < this->max.x &&
			point.y > this->min.y && point.y < this->max.y && 
			point.z > this->min.z && point.z < this->max.z;
	}

	bool AABB::contains(const ConvexHull& convexHull) const
	{
		for (uint32 i = 0; i < convexHull.vertices.size(); ++i) {
			if (this->contains(convexHull.vertices[i]) == false) return false;
		}

		return true;
	}

	bool AABB::contains(const AABB& aabb) const
	{
		return contains(aabb.min) && contains(aabb.max);
	}

	bool AABB::contains(const OBB& obb) const
	{
		StackArray<Vec3, 8> v = obb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (contains(v[i]) == false) return false;
		}

		return true;
	}

	bool AABB::contains(const Capsule& capsule) const
	{
		if (this->contains(capsule.capsuleLine) == false) return false;

		return magnitudeSq(capsule.pointA - this->closestPoint(capsule.pointA)) < square(capsule.radius) &&
			magnitudeSq(capsule.pointB - this->closestPoint(capsule.pointB)) < square(capsule.radius);
	}

	bool AABB::contains(const Sphere& sphere) const
	{
		if (contains(sphere.center) == false) return false;

		return (mathABS(this->max.x - sphere.center.x) <= sphere.radius || mathABS(sphere.center.x - this->min.x) <= sphere.radius ||
			mathABS(this->max.y - sphere.center.y) <= sphere.radius || mathABS(sphere.center.y - this->min.y) <= sphere.radius ||
			mathABS(this->max.z - sphere.center.z) <= sphere.radius || mathABS(sphere.center.z - this->min.z) <= sphere.radius);
	}

	bool AABB::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i) {
			if (this->contains(polygon.vertices[i]) == false) return false;
		}

		return true;
	}

	bool AABB::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool AABB::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool AABB::intersects(const ConvexHull& convexHull) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, convexHull, normals[x]) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const AABB& aabb) const
	{
		return (this->min.x <= aabb.max.x && this->max.x >= aabb.min.x) &&
			(this->min.y <= aabb.max.y && this->max.y >= aabb.min.y) &&
			(this->min.z <= aabb.max.z && this->max.z >= aabb.min.z);
	}

	bool AABB::intersects(const OBB& obb) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, obb, normals[x]) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool AABB::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool AABB::intersects(const Plane& plane) const
	{
		StackArray<Vec3, 8> vertices = this->getVertices();

		bool b[2] = { false, false };
		for (byte x = 0; x < 8; ++x) {
			
			decimal d = plane.getDistanceFromPlane(vertices[x]);
			if (d == decimal(0.0)) {
				return true;
			}
			else if(d < decimal(0.0)) {
				b[0] = true;
			}
			else {
				b[1] = true;
			}

			if (b[0] == true && b[1] == true) return true;
		}

		return false;
	}

	bool AABB::intersects(const Polygon& polygon) const
	{
		if (SATOverlap(*this, polygon, polygon.getNormal()) == false) return false;

		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, polygon, normals[x]) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const Triangle& triangle) const
	{
		//return GJKOverlap(*this, triangle); //inaccurate

		if (SATOverlap(*this, triangle, crossProduct(triangle.b - triangle.a, triangle.c - triangle.a)) == false) return false;

		Vec3 normals[3] = {XAXIS, YAXIS, ZAXIS};
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, triangle, normals[x]) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const Line& line) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, line, normals[x]) == false) return false;
		}

		StackArray<LineSegment, 12> edges = this->getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, line, crossProduct(line.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const Ray& ray) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, ray, normals[x]) == false) return false;
		}

		StackArray<LineSegment, 12> edges = this->getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, ray, crossProduct(ray.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool AABB::intersects(const LineSegment& lineSegment) const
	{
		return lineSegment.contains(this->closestPoint(lineSegment));
	}

	void AABB::transform(const Transform3D& t)
	{
		this->min = t * this->min;
		this->max = t * this->max;
	}

	AABB AABB::transformed(const Transform3D& t) const
	{
		return AABB(t * this->min, t * this->max);
	}

	AABB AABB::partitionTo8(const byte& partition)
	{
		Vec3 center = this->getCenter();
		if (partition == 0) {
			return AABB(this->min, center);
		}
		else if (partition == 1) {
			return AABB(Vec3(center.x, this->min.y, this->min.z), Vec3(this->max.x, center.y, center.z));
		}
		else if (partition == 2) {
			return AABB(Vec3(this->min.x, center.y, this->min.z), Vec3(center.x, this->max.y, center.z));
		}
		else if (partition == 3) {
			return AABB(Vec3(center.x, center.y, this->min.z), Vec3(this->max.x, this->max.y, center.z));
		}
		else if (partition == 4) {
			return AABB(Vec3(this->min.x, this->min.y, center.z), Vec3(center.x, center.y, this->max.z));
		}
		else if (partition == 5) {
			return AABB(Vec3(center.x, this->min.y, center.z), Vec3(this->max.x, center.y, this->max.z));
		}
		else if (partition == 6) {
			return AABB(Vec3(this->min.x, center.y, center.z), Vec3(center.x, this->max.y, this->max.z));
		}
		else if (partition == 7) {
			return AABB(center, this->max);
		}

		return AABB();
	}
}