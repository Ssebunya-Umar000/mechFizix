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

#include"triangle.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"line.h"
#include"ray.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"../../math/transform.h"

namespace mech {

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	Vec3 FatTriangle::getSupportPoint(const Vec3& direction) const
	{
		byte index = -1;
		decimal maxDot = -decimalMAX;
		for (byte x = 0; x < 6; ++x) {
			decimal d = dotProduct(direction, this->vertices[x]);
			if (d > maxDot) {
				maxDot = d;
				index = x;
			}
		}
		return this->vertices[index];
	}

	void FatTriangle::transform(const Transform3D& t)
	{
		for (byte x = 0; x < 6; ++x) {
			this->vertices[x] = t * this->vertices[x];
		}
	}

	FatTriangle FatTriangle::transformed(const Transform3D& t) const
	{
		FatTriangle ft = *this;
		ft.transform(t);
		return ft;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////
	Plane Triangle::toPlane() const
	{
		Vec3 normal = this->getNormal();
		return Plane(normal, dotProduct(normal, vertices[0]));
	}

	Polygon Triangle::toPolygon() const
	{
		return Polygon(DynamicArray<Vec3, byte>(this->vertices, 3));
	}

	FatTriangle Triangle::toFatTriangle() const
	{
		FatTriangle ft;

		ft.vertices[0] = this->vertices[0];
		ft.vertices[1] = this->vertices[1];
		ft.vertices[2] = this->vertices[2];

		Vec3 thickness = -this->getNormal() * decimal(0.5);
		for (byte x = 0; x < 3; ++x) {
			ft.vertices[x + 3] = this->vertices[x] + thickness;
		}

		return ft;
	}

	decimal Triangle::calculateArea() const
	{
		return decimal(0.5) * magnitude(crossProduct(this->b - this->a, this->c - this->a));
	}

	Vec3 Triangle::getSupportPoint(const Vec3& direction) const
	{
		Vec3 support = nanVEC3;
		decimal maxDotProduct = -decimalMAX;
		for (byte i = 0; i < 3; i++) {

			decimal dot = dotProduct(direction, this->vertices[i]);
			if (dot > maxDotProduct) {
				support = this->vertices[i];
				maxDotProduct = dot;
			}
		}

		return support;
	}

	void Triangle::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		decimal d1 = -decimalMAX;
		decimal d2 = decimalMAX;
		for (byte i = 0; i < 3; i++) {

			decimal d = dotProduct(direction, this->vertices[i]);
			if (d > d1) {
				max = this->vertices[i];
				d1 = d;
			}
			if (d < d2) {
				min = this->vertices[i];
				d2 = d;
			}
		}
	}

	Vec3 Triangle::getBarryCentricCordinates(const Vec3& point) const
	{
		Vec3 v0 = this->b - this->a;
		Vec3 v1 = this->c - this->a;
		Vec3 v2 = point - this->a;

		decimal d00 = dotProduct(v0, v0);
		decimal d01 = dotProduct(v0, v1);
		decimal d11 = dotProduct(v1, v1);
		decimal d20 = dotProduct(v2, v0);
		decimal d21 = dotProduct(v2, v1);

		Vec3 bCoords;
		decimal denom = d00 * d11 - d01 * d01;
		bCoords[1] = (d11 * d20 - d01 * d21) / denom;
		bCoords[2] = (d00 * d21 - d01 * d20) / denom;
		bCoords[0] = decimal(1.0) - bCoords[1] - bCoords[2];

		return bCoords;
	}

	StackArray<LineSegment, 3> Triangle::getEdges() const
	{
		LineSegment edges[3] = { LineSegment(this->a, this->b), LineSegment(this->b, this->c), LineSegment(this->c, this->a) };
		return StackArray<LineSegment, 3>(edges);
	}

	Vec3 Triangle::closestPoint(const Vec3& point) const
	{
		Vec3 ab = this->b - this->a;
		Vec3 ac = this->c - this->a;
		Vec3 ap = point - this->a;
		decimal dot1 = dotProduct(ab, ap);
		decimal dot2 = dotProduct(ac, ap);		
		if (dot1 <= decimal(0.0) && dot2 <= decimal(0.0)) return this->a;

		Vec3 bp = point - this->b;
		decimal dot3 = dotProduct(ab, bp);
		decimal dot4 = dotProduct(ac, bp);
		if (dot3 >= decimal(0.0) && dot4 <= dot3) return this->b;

		decimal vc = dot1 * dot4 - dot3 * dot2;
		if (vc <= decimal(0.0) && dot1 >= decimal(0.0) && dot3 <= decimal(0.0)) return this->a + ab * (dot1 / (dot1 - dot3));

		Vec3 cp = point - this->c;
		decimal dot5 = dotProduct(ab, cp);
		decimal dot6 = dotProduct(ac, cp);
		if (dot6 >= decimal(0.0) && dot5 <= dot6) return this->c;

		decimal vb = dot5 * dot2 - dot1 * dot6;
		if (vb <= decimal(0.0) && dot2 >= decimal(0.0) && dot6 <= decimal(0.0)) return this->a + ac * (dot2 / (dot2 - dot6));

		decimal va = dot3 * dot6 - dot5 * dot4;
		if (va <= decimal(0.0) && dot4 - dot3 >= decimal(0.0) && dot5 - dot6 >= decimal(0.0)) return this->b + (this->c - this->b) * ((dot4 - dot3) / (dot4 - dot3 + dot5 - dot6));

		decimal d = decimal(1.0) / (va + vb + vc);
		return this->a + ab * vb * d + ac * vc * d;
	}

	Vec3 Triangle::closestPoint(const LineSegment& lineSegment) const
	{
		//return GJKAlgorithm(*this, lineSegment, false).closest1;

		if (this->intersects(lineSegment)) {
			return lineSegment.toRay().rayCastPoint(this->toPlane());
		}

		Vec3 p1 = this->closestPoint(lineSegment.start);
		Vec3 p2 = this->closestPoint(lineSegment.end);
		return magnitudeSq(lineSegment.start - p1) < magnitudeSq(lineSegment.end - p2) ? p1 : p2;
	}

	bool Triangle::contains(const Vec3& point) const
	{
		StackArray<LineSegment, 3> edges = this->getEdges();
		decimal total = 0;
		for (byte x = 0; x < 3; ++x) {
			decimal angle = mathABS(angleBetween(edges[x].start - point, edges[x].end - point));
			total += almostEqual(angle, mathPI) ? decimal(0.0) : angle;
		}

		return almostEqual(total, decimal(2.0) * mathPI);
	}

	bool Triangle::contains(const Triangle& triangle) const
	{
		return contains(triangle.a) && contains(triangle.b) && contains(triangle.c);
	}

	bool Triangle::contains(const LineSegment& lineSegment) const
	{
		return contains(lineSegment.start) && contains(lineSegment.end);
	}

	bool Triangle::intersects(const ConvexHull& convexHull) const
	{
		Vec3 normal = crossProduct(this->b - this->a, this->c - this->a);

		if (SATOverlap(*this, convexHull, normal) == false) return false;
		if (SATOverlap(*this, convexHull, crossProduct(normal, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, convexHull, crossProduct(normal, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, convexHull, crossProduct(normal, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const AABB& aabb) const
	{
		Vec3 normal = crossProduct(this->b - this->a, this->c - this->a);

		if (SATOverlap(*this, aabb, normal) == false) return false;
		if (SATOverlap(*this, aabb, crossProduct(normal, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, aabb, crossProduct(normal, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, aabb, crossProduct(normal, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const OBB& obb) const
	{
		Vec3 normal = crossProduct(this->b - this->a, this->c - this->a);

		if (SATOverlap(*this, obb, normal) == false) return false;
		if (SATOverlap(*this, obb, crossProduct(normal, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, obb, crossProduct(normal, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, obb, crossProduct(normal, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= capsule.radius * capsule.radius;
	}

	bool Triangle::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= sphere.radius * sphere.radius;
	}

	bool Triangle::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool Triangle::intersects(const Polygon& polygon) const
	{
		Vec3 normal = crossProduct(this->b - this->a, this->c - this->a);

		if (SATOverlap(*this, polygon, normal) == false) return false;
		if (SATOverlap(*this, polygon, crossProduct(normal, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, polygon, crossProduct(normal, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, polygon, crossProduct(normal, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const Triangle& triangle) const
	{
		Vec3 normal = crossProduct(this->b - this->a, this->c - this->a);

		if (SATOverlap(*this, triangle, normal) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const Line& line) const
	{
		if (SATOverlap(*this, line, crossProduct(this->b - this->a, this->c - this->a)) == false) return false;
		if (SATOverlap(*this, line, crossProduct(line.direction, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, line, crossProduct(line.direction, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, line, crossProduct(line.direction, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const Ray& ray) const
	{
		if (SATOverlap(*this, ray, crossProduct(this->b - this->a, this->c - this->a)) == false) return false;
		if (SATOverlap(*this, ray, crossProduct(ray.direction, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, ray, crossProduct(ray.direction, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, ray, crossProduct(ray.direction, this->a - this->c)) == false) return false;

		return true;
	}

	bool Triangle::intersects(const LineSegment& lineSegment) const
	{
		Vec3 dir = lineSegment.getDirection();
		if (SATOverlap(*this, lineSegment, crossProduct(this->b - this->a, this->c - this->a)) == false) return false;
		if (SATOverlap(*this, lineSegment, crossProduct(dir, this->b - this->a)) == false) return false;
		if (SATOverlap(*this, lineSegment, crossProduct(dir, this->c - this->b)) == false) return false;
		if (SATOverlap(*this, lineSegment, crossProduct(dir, this->a - this->c)) == false) return false;

		return true;
	}

	void Triangle::transform(const Transform3D& t)
	{
		this->a = t * this->a;
		this->b = t * this->b;
		this->c = t * this->c;
	}

	Triangle Triangle::transformed(const Transform3D& t) const
	{
		Triangle triangle = *this;
		triangle.transform(t);
		return triangle;
	}

	Vec3 Triangle::clip(const LineSegment& lineSegment) const
	{
		Ray ray = lineSegment.toRay();

		decimal t = ray.rayCastTime(this->toPlane());

		if (t < decimal(0.0)) return nanVEC3;

		Vec3 pt = ray.origin + ray.direction * t;
		if (this->contains(pt) && t * t <= magnitudeSq(lineSegment.getDirection())) return pt;

		return nanVEC3;
	}
}