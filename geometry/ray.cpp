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

#include"ray.h"

#include"convexHull.h"
#include"aabb.h"
#include"obb.h"
#include"capsule.h"
#include"sphere.h"
#include"plane.h"
#include"polygon.h"
#include"triangle.h"
#include"line.h"
#include"lineSegment.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"../math/transform.h"

namespace mech {
	
	Vec3 Ray::getSupportPoint(const Vec3& dir) const
	{
		return this->origin + this->direction * mathMAX(dotProduct(dir * decimal(1e30) - this->origin, this->direction), decimal(0.0));
	}

	void Ray::getSupportPoints(const Vec3& dir, Vec3& min, Vec3& max) const
	{
		min = this->origin - this->direction * mathMAX(dotProduct(dir * decimal(1e30) - this->origin, this->direction), decimal(0.0));
		max = this->origin + this->direction * mathMAX(dotProduct(dir * decimal(1e30) - this->origin, this->direction), decimal(0.0));
	}

	Vec3 Ray::closestPoint(const Vec3& point) const
	{
		return this->origin + this->direction * mathMAX(dotProduct(point - this->origin, this->direction), decimal(0.0));
	}

	Vec3 Ray::closestPoint(const LineSegment& lineSegment) const
	{
		Vec3 v32 = lineSegment.getDirection();
		Vec3 v02 = this->origin - lineSegment.pointA;
		
		decimal d3232 = dotProduct(v32, v32);
		
		if (d3232 == decimal(0.0)) return nanVEC3;

		decimal d0232 = dotProduct(v02, v32);
		decimal d3210 = dotProduct(v32, this->direction);
		decimal d0210 = dotProduct(this->direction, v02);
		decimal d1010 = dotProduct(this->direction, this->direction);

		decimal d1 = decimal(0.0);
		decimal d2 = decimal(0.0);

		decimal denom = d1010 * d3232 - d3210 * d3210;
		if (denom != decimal(0.0)) {
			d1 = (d0232 * d3210 - d0210 * d3232) / denom;
		}
		else {
			d1 = decimal(0.0);
			d2 = (d0232 + d1 * d3210) / d3232;
		}

		if (d1 < decimal(0.0)) {
			d1 = decimal(0.0);
			if (d2 >= decimal(0.0) && d2 <= decimal(1.0)) {
				return this->origin;
			}

			Vec3 p;
			if (d2 < decimal(0.0)) {
				p = lineSegment.pointA;
			}
			else {
				p = lineSegment.pointB;
			}

			Vec3 closest1 = this->origin + this->direction * mathMAX(decimal(0.0), dotProduct(p - this->origin, this->direction));
			Vec3 closest2 = lineSegment.pointA + v32 * mathMAX(decimal(0.0), dotProduct(this->origin - lineSegment.pointA, v32));

			if(magnitudeSq(closest1 - p) <= magnitudeSq(closest2 - this->origin)) {
				return closest1;
			}
			else {
				return this->origin;
			}
		}
		else if (d2 < decimal(0.0)) {
			return this->origin + this->direction * mathMAX(decimal(0.0), dotProduct(lineSegment.pointA - this->origin, this->direction));
		}
		else if (d2 > decimal(1.0)) {
			return this->origin + this->direction * mathMAX(decimal(0.0), dotProduct(lineSegment.pointB - this->origin, this->direction));
		}

		return this->origin + this->direction * d1;
	}

	decimal Ray::rayCastTime(const ConvexHull& convexHull) const
	{
		decimal tmin = decimalMAX;

		for (uint32 x = 0, len = convexHull.halfEdgeMesh.faces.size(); x < len; ++x) {
			decimal t = rayCastTime(convexHull.getFacePolygon(x));
			if (t != decimal(-1.0) && t < tmin) tmin = t;
		}

		return tmin == decimalMAX ? decimal(-1.0) : tmin;
	}

	decimal Ray::rayCastTime(const AABB& aabb) const
	{
		Vec3 minT = (aabb.min - this->origin) / this->direction;
		Vec3 maxT = (aabb.max - this->origin) / this->direction;

		decimal tmin = mathMAX(mathMAX(mathMIN(minT.x, maxT.x), mathMIN(minT.y, maxT.y)), mathMIN(minT.z, maxT.z));
		decimal tmax = mathMIN(mathMIN(mathMAX(minT.x, maxT.x), mathMAX(minT.y, maxT.y)), mathMAX(minT.z, maxT.z));

		if (tmin > tmax || tmax < decimal(0.0)) return decimal(-1.0);
		
		if (tmin < decimal(0.0)) return tmax;

		return tmin;
	}

	decimal Ray::rayCastTime(const OBB& obb) const
	{
		Vec3 dist = obb.center - this->origin;
		Vec3 projectionOfDistOntoAxes = Vec3(dotProduct(obb.orientation.getColumn(0), dist), dotProduct(obb.orientation.getColumn(1), dist), dotProduct(obb.orientation.getColumn(2), dist));
		Vec3 angleBtnRayDirAndAxes = Vec3(dotProduct(obb.orientation.getColumn(0), this->direction), dotProduct(obb.orientation.getColumn(1), this->direction), dotProduct(obb.orientation.getColumn(2), this->direction));

		StackArray<decimal, 6> t(decimal(0.0));

		for (byte i = 0; i < 3; ++i) {
			if (almostEqual(angleBtnRayDirAndAxes[i], decimal(0.0))) {
				if (-projectionOfDistOntoAxes[i] - obb.halfExtents[i] > decimal(0.0) || -projectionOfDistOntoAxes[i] + obb.halfExtents[i] < decimal(0.0)) 
					return decimal(-1.0);

				angleBtnRayDirAndAxes[i] = mathEPSILON;
			}

			t[i * 2 + 0] = (projectionOfDistOntoAxes[i] + obb.halfExtents[i]) / angleBtnRayDirAndAxes[i];
			t[i * 2 + 1] = (projectionOfDistOntoAxes[i] - obb.halfExtents[i]) / angleBtnRayDirAndAxes[i];
		}

		decimal tmin = mathMAX(mathMAX(mathMIN(t[0], t[1]), mathMIN(t[2], t[3])), mathMIN(t[4], t[5]));
		decimal tmax = mathMIN(mathMIN(mathMAX(t[0], t[1]), mathMAX(t[2], t[3])), mathMAX(t[4], t[5]));

		if (tmin > tmax || tmax < decimal(0.0)) return decimal(-1.0);
		if (tmin < decimal(0.0)) return tmax;

		return tmin;
	}

	decimal Ray::rayCastTime(const Capsule& capsule) const
	{
		Vec3 ab = capsule.capsuleLine.getDirection();
		Vec3 ao = this->origin - capsule.pointA;

		decimal p1 = scalarProjection(ab, this->direction);
		decimal p2 = scalarProjection(ab, ao);

		Vec3 rayDirRejection = this->direction - (ab * p1);
		Vec3 aoRejection = ao - (ab * p2);

		//ray equation plugged into cylinder equation
		decimal a = magnitudeSq(rayDirRejection);
		decimal b = decimal(2.0) * dotProduct(rayDirRejection, aoRejection);
		decimal c = magnitudeSq(aoRejection) - (square(capsule.radius));

		if (a == decimal(0.0)) {
			decimal ta = this->rayCastTime(Sphere(capsule.pointA, capsule.radius));
			decimal tb = this->rayCastTime(Sphere(capsule.pointB, capsule.radius));
			
			if (ta < decimal(0.0) && tb < decimal(0.0)) return decimal(-1.0);

			return ta < tb ? ta : tb;
		}

		decimal discriminant = b * b - decimal(4.0) * a * c;
		if (discriminant < decimal(0.0)) return decimal(-1.0);

		decimal sqD = mathSQRT(discriminant);
		decimal tmin = (-b - sqD) / (decimal(2.0) * a);
		decimal tmax = (-b + sqD) / (decimal(2.0) * a);

		if (tmin > tmax) {
			decimal temp = tmin;
			tmin = tmax;
			tmax = temp;
		}

		decimal t1 = tmin * p1 + p2;
		if (t1 < decimal(0.0)) return this->rayCastTime(Sphere(capsule.pointA, capsule.radius));
		else if (t1 > decimal(1.0)) return this->rayCastTime(Sphere(capsule.pointB, capsule.radius));

		return tmin;
	}

	decimal Ray::rayCastTime(const Sphere& sphere) const
	{
		Vec3 rayToSphere = sphere.center - this->origin;
		decimal distBtnSphereAndRaySq = magnitudeSq(rayToSphere);

		decimal radiusSq = square(sphere.radius);

		decimal projOfDistOntoRayDir = dotProduct(rayToSphere, this->direction);
		decimal projOfDistOntoRayDirSq = projOfDistOntoRayDir * projOfDistOntoRayDir;
		decimal rejOfDistOntoRayDirSq = distBtnSphereAndRaySq - projOfDistOntoRayDirSq;
		decimal fromRadiusToSurface = mathSQRT(radiusSq - rejOfDistOntoRayDirSq);

		if (radiusSq - (distBtnSphereAndRaySq - projOfDistOntoRayDirSq) < decimal(0.0)) return decimal(-1.0);

		else if (distBtnSphereAndRaySq < radiusSq) return projOfDistOntoRayDir + fromRadiusToSurface;

		return projOfDistOntoRayDir - fromRadiusToSurface;
	}

	decimal Ray::rayCastTime(const Plane& plane) const
	{
		decimal nd = dotProduct(this->direction, plane.normal);
		if (nd == decimal(0.0)) return decimal(-1.0);

		decimal t = (plane.distance - dotProduct(this->origin, plane.normal)) / nd;

		return t >= decimal(0.0) ? t : decimal(-1.0);
	}

	decimal Ray::rayCastTime(const Polygon& polygon) const
	{
		decimal t = rayCastTime(polygon.toPlane());

		if (t < decimal(0.0)) return decimal(-1.0);

		if (polygon.contains(this->origin + this->direction * t)) return t;

		return decimal(-1.0);
	}

	decimal Ray::rayCastTime(const Triangle& triangle) const
	{
		Vec3 vE1 = triangle.b - triangle.a;
		Vec3 vE2 = triangle.c - triangle.a;

		Vec3 vP = crossProduct(this->direction, vE2);

		decimal det = dotProduct(vE1, vP);
		if (mathABS(det) <= mathEPSILON) return decimal(-1.0);
		decimal recipDet = decimal(1.0) / det;

		Vec3 vT = this->origin - triangle.a;

		decimal u = dotProduct(vT, vP) * recipDet;
		if (u < -mathEPSILON || u > (decimal(1.0) + mathEPSILON)) return decimal(-1.0);

		Vec3 vQ = crossProduct(vT, vE1);

		decimal v = dotProduct(this->direction, vQ) * recipDet;
		if (v < -mathEPSILON || u + v >(decimal(1.0) + mathEPSILON)) return decimal(-1.0);

		return dotProduct(vE2, vQ) * recipDet;
	}

	decimal Ray::rayCastTime(const Line& line) const
	{
		decimal t1 = Ray(line.pointOnLine, line.direction).rayCastTime(*this);
		decimal t2 = Ray(line.pointOnLine, -line.direction).rayCastTime(*this);

		decimal t = decimal(-1.0);
		if (t1 >= decimal(0.0)) t = t1;
		if (t2 >= decimal(0.0)) t = t2;

		return t;
	}

	decimal Ray::rayCastTime(const Ray& ray) const
	{
		Vec3 normalisedDistAB = normalise(ray.origin - this->origin);
		Vec3 cdirAdirB = crossProduct(this->direction, ray.direction);
		if (almostEqual(dotProduct(normalisedDistAB, cdirAdirB), decimal(0.0)) == false) return decimal(-1.0);

		decimal t = scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, this->direction));

		return t >= decimal(0.0) ? t : decimal(-1.0);
	}

	Vec3 Ray::rayCastPoint(const ConvexHull& convexHull) const
	{
		decimal t = this->rayCastTime(convexHull);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const AABB& aabb) const
	{
		decimal t = this->rayCastTime(aabb);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const OBB& obb) const
	{
		decimal t = this->rayCastTime(obb);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Capsule& capsule) const
	{
		decimal t = this->rayCastTime(capsule);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Sphere& sphere) const
	{
		decimal t = this->rayCastTime(sphere);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Plane& plane) const
	{
		decimal t = this->rayCastTime(plane);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Polygon& polygon) const
	{
		decimal t = this->rayCastTime(polygon);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Triangle& triangle) const
	{
		decimal t = this->rayCastTime(triangle);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Line& line) const
	{
		decimal t = this->rayCastTime(line);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	Vec3 Ray::rayCastPoint(const Ray& ray) const
	{
		decimal t = this->rayCastTime(ray);
		if (t < decimal(0.0)) return nanVEC3;

		return this->origin + this->direction * t;
	}

	bool Ray::contains(const Vec3& point)
	{
		if (point == this->origin) return true;

		return almostEqual(dotProduct(normalise(point - this->origin), this->direction), decimal(1.0));
	}

	bool Ray::contains(const LineSegment& lineSegment)
	{
		return contains(lineSegment.pointA) && contains(lineSegment.pointB);
	}

	bool Ray::intersects(const ConvexHull& convexHull) const
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

	bool Ray::intersects(const AABB& aabb) const
	{
		Vec3 normals[3] = { XAXIS, YAXIS, ZAXIS };
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, aabb, normals[x]) == false) return false;
		}

		StackArray<LineSegment, 12> edges = aabb.getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, aabb, crossProduct(this->direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Ray::intersects(const OBB& obb) const
	{
		for (byte x = 0; x < 3; ++x) {
			if (SATOverlap(*this, obb, obb.orientation.getColumn(x)) == false) return false;
		}

		StackArray<LineSegment, 12> edges = obb.getEdges();
		for (uint16 x = 0; x < 12; ++x) {
			if (SATOverlap(*this, obb, crossProduct(this->direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Ray::intersects(const Capsule& capsule) const
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool Ray::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool Ray::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool Ray::intersects(const Polygon& polygon) const
	{
		Vec3 normal = polygon.getNormal();

		if (SATOverlap(*this, polygon, normal) == false) return false;

		HybridArray<LineSegment, 4, byte> edges = polygon.getEdges();
		for (byte x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, polygon, crossProduct(normal, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool Ray::intersects(const Triangle& triangle) const
	{
		Vec3 normal = crossProduct(triangle.b - triangle.a, triangle.c - triangle.a);

		if (SATOverlap(*this, triangle, normal) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.b - triangle.a)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.c - triangle.b)) == false) return false;
		if (SATOverlap(*this, triangle, crossProduct(normal, triangle.a - triangle.c)) == false) return false;

		return true;
	}

	bool Ray::intersects(const Line& line) const
	{
		Vec3 normalisedDistAB = normalise(line.pointOnLine - this->origin);
		Vec3 cdirAdirB = crossProduct(this->direction, line.direction);
		if (almostEqual(dotProduct(normalisedDistAB, cdirAdirB), decimal(0.0)) == false) return false;

		return scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, line.direction)) >= decimal(0.0) &&
			scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, this->direction)) >= decimal(0.0);
	}

	bool Ray::intersects(const Ray& ray) const
	{
		Vec3 normalisedDistAB = normalise(ray.origin - this->origin);
		Vec3 cdirAdirB = crossProduct(this->direction, ray.direction);
		if (almostEqual(dotProduct(normalisedDistAB, cdirAdirB), decimal(0.0)) == false) return false;

		return scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, ray.direction)) >= decimal(0.0) &&
			scalarProjection(cdirAdirB, crossProduct(normalisedDistAB, this->direction)) >= decimal(0.0);
	}

	bool Ray::intersects(const LineSegment& lineSegment) const
	{
		Vec3 dirSegment = lineSegment.getDirection();
		Vec3 distAB = lineSegment.pointA - this->origin;
		Vec3 cdirAdirB = crossProduct(this->direction, dirSegment);
		if (almostEqual(dotProduct(distAB, cdirAdirB), decimal(0.0)) == false) return false;

		decimal d1 = scalarProjection(cdirAdirB, crossProduct(distAB, dirSegment));
		decimal d2 = scalarProjection(cdirAdirB, crossProduct(distAB, this->direction));

		return d1 <= decimal(0.0) && d1 >= decimal(1.0) && d2 <= decimal(0.0) && d2 >= decimal(1.0);
	}

	void Ray::transform(const Transform3D& t)
	{
		this->origin = t * this->origin;
		this->direction = t.orientation * this->direction;
	}

	Ray Ray::transformed(const Transform3D& t) const
	{
		Ray r = *this;
		r.transform(t);
		return r;
	}
}
