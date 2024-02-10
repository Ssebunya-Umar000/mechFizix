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

#include"polygon.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"triangle.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"
#include"point.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"

namespace mech {

	Plane Polygon::toPlane() const
	{
		Vec3 normal = this->getNormal();
		return Plane(normal, dotProduct(normal, vertices[0]));
	}

	decimal Polygon::calculateArea() const
	{
		Vec3 area;
		int len = this->vertices.size();
		int x = len - 1;
		for (byte y = 0; y < len; ++y) {
			area += crossProduct(this->vertices[x], this->vertices[y]);
			x = y;
		}
		return decimal(0.5) * mathABS(dotProduct(this->getNormal(), area));
	}

	HybridArray<Triangle, 2, byte> Polygon::triangulate() const
	{
		HybridArray<Triangle, 2, byte> triangles;

		if (this->vertices.size() < 3) return triangles;

		for (uint32 x = 0, len = this->vertices.size(); x < len; x += 2) {

			triangles.pushBack(Triangle());
			Triangle& t = triangles.back();

			if (len % 2 == 0) {
				if (x == len - 2) {
					t.a = vertices[x];
					t.b = vertices[x + 1];
					t.c = vertices[0];
					break;
				}
			}
			else {
				if (x == len - 1) {
					t.a = vertices[x - 1];
					t.b = vertices[x];
					t.c = vertices[0];
					break;
				}
			}

			t.a = vertices[0];
			t.b = vertices[x + 1];
			t.c = vertices[x + 2];
		}

		return triangles;
	}

	Vec3 Polygon::getSupportPoint(const Vec3& direction) const
	{
		Vec3 mostExtreme;
		decimal mostExtremeDist = -decimalMAX;
		for (uint32 x = 0, len = this->vertices.size(); x < len; ++x) {

			decimal d = dotProduct(direction, this->vertices[x]);
			if (d > mostExtremeDist) {
				mostExtremeDist = d;
				mostExtreme = this->vertices[x];
			}
		}
		return mostExtreme;
	}

	void Polygon::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		decimal d1 = -decimalMAX;
		decimal d2 = decimalMAX;
		for (uint32 x = 0, len = this->vertices.size(); x < len; ++x) {

			decimal d = dotProduct(direction, this->vertices[x]);
			if (d > d1) {
				d1 = d;
				max = this->vertices[x];
			}
			if (d < d2) {
				d2 = d;
				min = this->vertices[x];
			}
		}
	}

	Vec3 Polygon::getNormal() const
	{
		assert(vertices.size() >= 3);
		return normalise(crossProduct(this->vertices[1] - this->vertices[0], this->vertices[2] - this->vertices[1]));
	}

	Vec3 Polygon::getCentroid() const
	{
		Vec3 centroid;
		uint32 len = this->vertices.size();
		for (uint32 x = 0; x < len; ++x) {
			centroid += this->vertices[x];
		}

		return centroid / len;
	}

	HybridArray<LineSegment, 4, byte> Polygon::getEdges() const
	{
		HybridArray<LineSegment, 4, byte> edges;

		for (byte x = 0, len = this->vertices.size(); x < len; ++x) {
			if (x == vertices.size() - 1) edges.pushBack(LineSegment(vertices[x], vertices[0]));
			if (x < vertices.size() - 1) edges.pushBack(LineSegment(vertices[x], vertices[x + 1]));
		}

		return edges;
	}

	HybridArray<Vec3, 4, byte> Polygon::getVertices() const
	{
		HybridArray<Vec3, 4, byte> vertices;

		HybridArray<Triangle, 2, byte> triangles = this->triangulate();
		for (uint32 x = 0, len = triangles.size(); x < len; ++x) {
			vertices.pushBack(triangles[x].a);
			vertices.pushBack(triangles[x].b);
			vertices.pushBack(triangles[x].c);
		}

		return vertices;
	}

	Vec3 Polygon::intersectionPoint(const LineSegment& lineSegment) const
	{
		Ray ray = lineSegment.toRay();

		decimal t = ray.rayCastTime(this->toPlane());

		if (t < decimal(0.0)) return nanVEC3;

		return ray.origin + ray.direction * t;
	}

	Vec3 Polygon::closestPoint(const Vec3& point) const
	{
		return GJKAlgorithm(*this, Point(point), false).closest1;
	}

	Vec3 Polygon::closestPoint(const LineSegment& lineSegment) const
	{
		return GJKAlgorithm(*this, lineSegment, false).closest1;
	}

	bool Polygon::contains(const Vec3& point) const
	{
		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		decimal total = 0;
		for (uint32 x = 0, len = edges.size(); x < len; ++x) {
			decimal angle = mathABS(angleBetween(edges[x].start - point, edges[x].end - point));
			total += almostEqual(angle, mathPI) ? decimal(0.0) : angle;
		}

		return almostEqual(total, decimal(2.0) * mathPI);
	}

	bool Polygon::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool Polygon::contains(const Polygon& polygon) const
	{
		for (uint32 i = 0; i < polygon.vertices.size(); ++i) {
			if (this->contains(polygon.vertices[i]) == false) return false;
		}

		return true;
	}

	bool Polygon::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.start) && contains(lineSegment.end);
	}

	bool Polygon::intersects(const ConvexHull& convexHull) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, convexHull, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const AABB& aabb) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, aabb, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, aabb, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const OBB& obb) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, obb, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, obb, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= capsule.radius * capsule.radius;
	}

	bool Polygon::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= sphere.radius * sphere.radius;
	}

	bool Polygon::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool Polygon::intersects(const Polygon& polygon) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, polygon, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, polygon, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Triangle& triangle) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, triangle, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, triangle, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Line& line) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, line, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, line, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Ray& ray) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, ray, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, ray, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const LineSegment& lineSegment) const
	{
		Vec3 normal = crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0]);

		if (SATOverlap(*this, lineSegment, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, lineSegment, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	void Polygon::transform(const Mat4x4& mat)
	{
		for (uint32 i = 0, len = this->vertices.size(); i < len; ++i) {
			this->vertices[i] = mat * this->vertices[i];
		}
	}

	Polygon Polygon::transformed(const Mat4x4& mat) const
	{
		Polygon p = *this;
		p.transform(mat);
		return p;
	}

	Vec3 Polygon::clip(const LineSegment& lineSegment) const
	{
		Ray ray = lineSegment.toRay();

		decimal t = ray.rayCastTime(this->toPlane());

		if (t < decimal(0.0)) return nanVEC3;

		Vec3 pt = ray.origin + ray.direction * t;
		if (this->contains(pt) && t * t <= magnitudeSq(lineSegment.getDirection())) return pt;

		return nanVEC3;
	}
}
