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
#include"algorithms/CPToLS.h"
#include"../math/transform.h"

namespace mech {

	Plane Polygon::toPlane() const
	{
		Vec3 normal = this->getNormal();
		return Plane(normal, dotProduct(normal, vertices[0]));
	}

	decimal Polygon::calculateArea() const
	{
		return mathABS(dotProduct(this->getNormal(), crossProduct(this->vertices[1] - this->vertices[0], this->vertices.back() - this->vertices[0])));
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
		uint16 index  = -1;
		decimal dist = -decimalMAX;
		for (uint32 x = 0, len = this->vertices.size(); x < len; ++x) {

			decimal d = dotProduct(direction, this->vertices[x]);
			if (d > dist) {
				dist = d;
				index = x;
			}
		}
		return this->vertices[index];
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
		ASSERT(vertices.size() >= 3, "polygon should have atleast 3 vertices");
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

	Vec3 Polygon::closestPoint(const Vec3& point) const
	{
		return GJKDistance(*this, Point(point)).closest1;

		/*HybridArray<Triangle, 2, byte> triangles = this->triangulate();
		Vec3 closest = nanVEC3;
		decimal least = decimalMAX;
		for (uint16 x = 0, len = triangles.size(); x < len; ++x) {
			Vec3 c = triangles[x].closestPoint(point);
			decimal d = magnitudeSq(point - c);
			if (d < least) {
				least = d;
				closest = c;
			}
		}

		return closest;*/
	}

	Vec3 Polygon::closestPoint(const LineSegment& lineSegment) const
	{
		return closestPointToLineSegment(*this, lineSegment);
	}

	bool Polygon::contains(const Vec3& point) const
	{
		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		decimal total = 0;
		for (uint32 x = 0, len = edges.size(); x < len; ++x) {
			decimal angle = mathABS(angleBetween(edges[x].pointA - point, edges[x].pointB - point));
			total += almostEqual(angle, mathPI) ? decimal(0.0) : angle;
		}

		return almostEqual(total, math2PI);
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
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool Polygon::intersects(const ConvexHull& convexHull) const
	{
		if (SATOverlap(*this, convexHull, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;

		HybridArray<Vec3, 6, uint16> normals = convexHull.getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, normals[x]) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const AABB& aabb) const
	{
		if (SATOverlap(*this, aabb, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;

		StackArray<Vec3, 6> normals = aabb.getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, aabb, normals[x]) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const OBB& obb) const
	{
		if (SATOverlap(*this, obb, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;

		StackArray<Vec3, 6> normals = obb.getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, obb, normals[x]) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool Polygon::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool Polygon::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool Polygon::intersects(const Polygon& polygon) const
	{
		if (SATOverlap(*this, polygon, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;
		if (SATOverlap(*this, polygon, crossProduct(polygon.vertices[0] - polygon.vertices[1], polygon.vertices.back() - polygon.vertices[0])) == false) return false;

		return true;
	}

	bool Polygon::intersects(const Triangle& triangle) const
	{
		if (SATOverlap(*this, triangle, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(triangle.b - triangle.a, triangle.c - triangle.a)) == false) return false;

		return true;
	}

	bool Polygon::intersects(const Line& line) const
	{
		if (SATOverlap(*this, line, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, line, crossProduct(line.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const Ray& ray) const
	{
		if (SATOverlap(*this, ray, crossProduct(this->vertices[0] - this->vertices[1], this->vertices.back() - this->vertices[0])) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = this->getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, ray, crossProduct(ray.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Polygon::intersects(const LineSegment& lineSegment) const
	{
		return lineSegment.contains(this->closestPoint(lineSegment));
	}

	void Polygon::transform(const Transform3D& t)
	{
		for (uint32 i = 0, len = this->vertices.size(); i < len; ++i) {
			this->vertices[i] = t * this->vertices[i];
		}
	}

	Polygon Polygon::transformed(const Transform3D& t) const
	{
		Polygon p = *this;
		p.transform(t);
		return p;
	}

	Vec3 Polygon::clip(const LineSegment& lineSegment) const
	{
		Vec3 p = this->closestPoint(lineSegment);
		if (lineSegment.contains(p)) return p;
		return nanVEC3;
	}
}
