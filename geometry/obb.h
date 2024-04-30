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

#ifndef OBB_H
#define OBB_H

#include"../math/matrix.h"
#include"../containers/stackArray.h"

namespace mech {

	struct ConvexHull;
	struct AABB;
	struct Capsule;
	struct Sphere;
	struct Plane;
	struct Polygon;
	struct Triangle;
	struct Line;
	struct Ray;
	struct LineSegment;

	struct Transform3D;

	//oriented bounding box
	struct OBB {
		Vec3 center;
		Vec3 halfExtents;
		Mat3x3 orientation = Mat3x3(decimal(1.0));

		OBB() {}
		explicit OBB(const Vec3& c, const Vec3& h) : center(c), halfExtents(h) {}
		~OBB() {};

		ConvexHull toConvexHull() const;
		AABB toAABB() const;

		Vec3 getSupportPoint(const Vec3& direction) const;
		void getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const;

		StackArray<Vec3, 8> getVertices() const;
		StackArray<LineSegment, 12> getEdges() const;
		StackArray<Vec3, 6> getNormals() const;
		StackArray<Polygon, 6> getFacePolygons() const;

		Vec3 closestPoint(const Vec3& point) const;
		Vec3 closestPoint(const LineSegment& lineSegment) const;

		bool contains(const Vec3& point) const;
		bool contains(const ConvexHull& convexHull) const;
		bool contains(const AABB& aabb) const;
		bool contains(const OBB& obb) const;
		bool contains(const Capsule& capsule) const;
		bool contains(const Sphere& sphere) const;
		bool contains(const Polygon& polygon) const;
		bool contains(const Triangle& triangle) const;
		bool contains(const LineSegment& lineSegment) const;

		bool intersects(const ConvexHull& convexHull) const;
		bool intersects(const AABB& aabb) const;
		bool intersects(const OBB& obb) const;
		bool intersects(const Capsule& capsule) const ;
		bool intersects(const Sphere& sphere) const;
		bool intersects(const Plane& plane) const;
		bool intersects(const Polygon& polygon) const;
		bool intersects(const Triangle& triangle) const;
		bool intersects(const Line& line) const;
		bool intersects(const Ray& ray) const;
		bool intersects(const LineSegment& lineSegment) const;

		void transform(const Transform3D& t);
		OBB transformed(const Transform3D& t) const;

		String toString() const { return "OBB(center:" + this->center.toString() + " halfExtents:" + this->halfExtents.toString() + " orientation:" + this->orientation.toString() + ")"; }
	};
}

#endif