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

#ifndef POLYHEDRON_H
#define POLYHEDRON_H

#include"../math/vec.h"
#include"../containers/hybridArray.h"

namespace mech {

	struct AABB;
	struct OBB;
	struct Capsule;
	struct Sphere;
	struct Plane;
	struct Polygon;
	struct Triangle;
	struct Line;
	struct Ray;
	struct LineSegment;

	struct Transform3D;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HalfEdgeMesh {

		struct hEdge {
			uint16 vertIndex = -1;
			uint16 faceIndex = -1;
			uint16 nextIndex = -1;
			uint16 twinIndex = -1;
			bool duplicate = false;

			hEdge() {}
			hEdge(const uint16& v) : vertIndex(v) {}
		};

		struct hFace {
			uint16 edgeIndex = -1;
			HybridArray<byte, 4, byte> faceVerts;

			hFace() {}
			hFace(const uint16& e) : edgeIndex(e) {}
		};

		HybridArray<hEdge, 24, uint16> edges;
		HybridArray<hFace, 6, uint16> faces;

		uint16 nextEdge(const uint16& index) const
		{
			return this->edges[index].nextIndex;
		}

		uint16 twinEdge(const uint16& index) const
		{
			return this->edges[index].twinIndex;
		}

		void findTwin(const uint16& index)
		{
			for (uint16 x = 0, len = this->edges.size(); x < len; ++x) {
				HalfEdgeMesh::hEdge* e = &this->edges[x];
				if (e->twinIndex == (uint16)-1 && e->nextIndex != (uint16)-1) {
					if (this->edges[this->edges[index].nextIndex].vertIndex == e->vertIndex && this->edges[e->nextIndex].vertIndex == this->edges[index].vertIndex) {
						this->edges[index].twinIndex = x;
						e->twinIndex = index;
						e->duplicate = true;
						break;
					}
				}
			}
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ConvexHull {

		HalfEdgeMesh halfEdgeMesh;
		HybridArray<Vec3, 8, uint32> vertices;

		ConvexHull() {}
		explicit ConvexHull(const HybridArray<Polygon, 6, uint16>& polygons);
		~ConvexHull() {};

		Vec3 getSupportPoint(const Vec3& direction) const;
		void getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const;

		AABB toAABB();

		Polygon getFacePolygon(const uint16& index) const;
		HybridArray<Polygon, 6, uint16> getFacePolygons() const;
		
		Plane getFacePlane(const uint16& index) const;
		HybridArray<Plane, 6, uint16> getPlanes() const;
		
		Vec3 getFaceNormal(const uint16& index) const;
		HybridArray<Vec3, 6, uint16> getNormals() const;
		
		LineSegment getEdge(const uint16& index) const;
		HybridArray<LineSegment, 12, uint16> getEdges() const;

		Vec3 getCentroid() const;
		
		HybridArray<Vec3, 12, uint16> triangulate() const;

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
		bool intersects(const Capsule& capsule) const;
		bool intersects(const Sphere& sphere) const;
		bool intersects(const Plane& plane) const;
		bool intersects(const Polygon& polygon) const;
		bool intersects(const Triangle& triangle) const;
		bool intersects(const Line& line) const;
		bool intersects(const Ray& ray) const;
		bool intersects(const LineSegment& lineSegment) const;

		void transform(const Transform3D& t);
		ConvexHull transformed(const Transform3D& t) const;

		String toString() const
		{
			String string = "ConvexHull::numOfVertices: " + toString1(this->vertices.size()) + " (";
			for (uint16 x = 0, len = this->vertices.size(); x < len; ++x) {
				string += this->vertices[x].toString();
				if (x < len - 1) {
					string += ", ";
				}
				else {
					string += ")";
				}
			}
			return string;
		}
	};
}

#endif
