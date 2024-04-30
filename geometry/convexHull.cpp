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
#include"lineSegment.h"
#include"point.h"

#include"algorithms/SAT.h"
#include"algorithms/GJK.h"
#include"algorithms/CPToLS.h"
#include"../math/transform.h"

namespace mech {

	ConvexHull::ConvexHull(const HybridArray<Polygon, 6, uint16>& polygons)
	{
		if (polygons.size() > 0) {

			for(uint16 x = 0, len1 = polygons.size(); x < len1; ++x) {

				uint16 indexF = this->halfEdgeMesh.faces.size();
				this->halfEdgeMesh.faces.pushBack(HalfEdgeMesh::hFace());
				HalfEdgeMesh::hFace* face = &this->halfEdgeMesh.faces.back();

				uint16 prevEdgeIndex = -1;
				for(byte y = 0, len2 = polygons[x].vertices.size(); y < len2; ++y) {

					uint16 indexV = this->vertices.size();

					bool unique = true;
					for(byte z = 0, len3 = this->vertices.size(); z < len3; ++z) {
						if (this->vertices[z] == polygons[x].vertices[y]) {
							indexV = z;
							unique = false;
							break;
						}
					}
					if (unique) {
						this->vertices.pushBack(polygons[x].vertices[y]);
					}

					uint16 indexE = this->halfEdgeMesh.edges.size();
					this->halfEdgeMesh.edges.pushBack(HalfEdgeMesh::hEdge());
					HalfEdgeMesh::hEdge* edge = &this->halfEdgeMesh.edges.back();
					edge->vertIndex = indexV;
					edge->faceIndex = indexF;

					face->faceVerts.pushBack(indexV);

					if (isAValidIndex(prevEdgeIndex)) {
						this->halfEdgeMesh.edges[prevEdgeIndex].nextIndex = indexE;
						this->halfEdgeMesh.findTwin(prevEdgeIndex);
					}

					prevEdgeIndex = indexE;
					if (y == 0) {
						face->edgeIndex = indexE;
					}
					else if (y == len2 - 1) {
						edge->nextIndex = face->edgeIndex;
						this->halfEdgeMesh.findTwin(indexE);
					}
				}
			}
		}
	}

	Vec3 ConvexHull::getSupportPoint(const Vec3& direction) const
	{
		Vec3 support = nanVEC3;
		decimal maxDotProduct = -decimalMAX;
		for (uint16 i = 0, len = this->vertices.size(); i < len; i++) {

			decimal dot = dotProduct(direction, this->vertices[i]);
			if (dot > maxDotProduct) {
				support = this->vertices[i];
				maxDotProduct = dot;
			}
		}

		return support;
	}

	void ConvexHull::getSupportPoints(const Vec3& direction, Vec3& min, Vec3& max) const
	{
		decimal d1 = -decimalMAX;
		decimal d2 = decimalMAX;
		for (uint16 i = 0, len = this->vertices.size(); i < len; i++) {

			decimal dot = dotProduct(direction, this->vertices[i]);
			if (dot > d1) {
				max = this->vertices[i];
				d1 = dot;
			}
			if (dot < d2) {
				min = this->vertices[i];
				d2 = dot;
			}
		}
	}

	AABB ConvexHull::toAABB()
	{
		AABB aabb(this->vertices[0], this->vertices[0]);
		for (uint16 i = 1, len = this->vertices.size(); i < len; i++) {
			aabb.min = minVec(aabb.min, this->vertices[i]);
			aabb.max = maxVec(aabb.max, this->vertices[i]);
		}
		return aabb;
	}

	Polygon ConvexHull::getFacePolygon(const uint16& index) const
	{
		DynamicArray<Vec3, byte> verts;
		for(uint16 x = 0, len = this->halfEdgeMesh.faces[index].faceVerts.size(); x < len; ++x) {
			verts.pushBack(this->vertices[this->halfEdgeMesh.faces[index].faceVerts[x]]);
		}
		return Polygon(verts);
	}

	HybridArray<Polygon, 6, uint16> ConvexHull::getFacePolygons() const
	{
		HybridArray<Polygon, 6, uint16> p;
		for (auto it = this->halfEdgeMesh.faces.begin(), end = this->halfEdgeMesh.faces.end(); it != end; ++it) {
			p.pushBack(this->getFacePolygon(it.index()));
		}
		return p;
	}

	Plane ConvexHull::getFacePlane(const uint16& index) const
	{
		Vec3 normal = this->getFaceNormal(index);
		return Plane(normal, dotProduct(normal, vertices[this->halfEdgeMesh.faces[index].faceVerts[0]]));
	}

	HybridArray<Plane, 6, uint16> ConvexHull::getPlanes() const
	{
		HybridArray<Plane, 6, uint16> p;
		for (auto it = this->halfEdgeMesh.faces.begin(), end = this->halfEdgeMesh.faces.end(); it != end; ++it) {
			p.pushBack(this->getFacePlane(it.index()));
		}
		return p;
	}

	Vec3 ConvexHull::getFaceNormal(const uint16& index) const
	{
		uint16 index1 = this->halfEdgeMesh.faces[index].faceVerts[0];
		uint16 index2 = this->halfEdgeMesh.faces[index].faceVerts[1];
		uint16 index3 = this->halfEdgeMesh.faces[index].faceVerts[2];
		return normalise(crossProduct(this->vertices[index2] - this->vertices[index1], this->vertices[index3] - this->vertices[index2]));
	}

	HybridArray<Vec3, 6, uint16> ConvexHull::getNormals() const
	{
		HybridArray<Vec3, 6, uint16> n;
		for (auto it = this->halfEdgeMesh.faces.begin(), end = this->halfEdgeMesh.faces.end(); it != end; ++it) {
			n.pushBack(this->getFaceNormal(it.index()));
		}
		return n;
	}

	LineSegment ConvexHull::getEdge(const uint16& index) const
	{
		return LineSegment(this->vertices[this->halfEdgeMesh.edges[index].vertIndex], this->vertices[this->halfEdgeMesh.edges[this->halfEdgeMesh.nextEdge(index)].vertIndex]);
	}

	HybridArray<LineSegment, 12, uint16> ConvexHull::getEdges() const
	{
		HybridArray<LineSegment, 12, uint16> l;
		for (auto it = this->halfEdgeMesh.edges.begin(), end = this->halfEdgeMesh.edges.end(); it != end; ++it) {
			if (this->halfEdgeMesh.edges[it.index()].duplicate) continue;
			l.pushBack(this->getEdge(it.index()));
		}
		return l;
	}

	Vec3 ConvexHull::getCentroid() const
	{
		int len = this->vertices.size();

		Vec3 centerVertex;
		for(uint16 i = 0; i < len; ++i) {
			centerVertex += this->vertices[i];
		}

		centerVertex /= len;

		return centerVertex;
	}

	HybridArray<Vec3, 12, uint16> ConvexHull::triangulate() const
	{
		HybridArray<Vec3, 12, uint16> v;
		for (auto it = this->halfEdgeMesh.faces.begin(), end = this->halfEdgeMesh.faces.end(); it != end; ++it) {

			HybridArray<Triangle, 2, byte> t = this->getFacePolygon(it.index()).triangulate();

			for(uint16 y = 0, len2 = t.size(); y < len2; ++y) {
				v.pushBack(t[y].a);
				v.pushBack(t[y].b);
				v.pushBack(t[y].c);
			}
		}
		return v;
	}

	Vec3 ConvexHull::closestPoint(const Vec3& point)const
	{
		if (this->contains(point)) {
			return point;
		}

		return GJKDistance(*this, Point(point)).closest1;
	}

	Vec3 ConvexHull::closestPoint(const LineSegment& lineSegment) const
	{
		return closestPointToLineSegment(*this, lineSegment);
	}

	bool ConvexHull::contains(const Vec3& point) const
	{
		for (auto it = this->halfEdgeMesh.faces.begin(), end = this->halfEdgeMesh.faces.end(); it != end; ++it) {
			if (this->getFacePlane(it.index()).getDistanceFromPlane(point) > decimal(0.0)) return false;
		}

		return true;
	}

	bool ConvexHull::contains(const ConvexHull& convexHull) const
	{
		for (uint32 i = 0, len1 = convexHull.vertices.size(); i < len1; ++i) {
			if (this->contains(convexHull.vertices[i]) == false) return false;
		}

		return true;
	}

	bool ConvexHull::contains(const AABB& aabb) const
	{
		StackArray<Vec3, 8> v = aabb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool ConvexHull::contains(const OBB& obb) const
	{
		StackArray<Vec3, 8> v = obb.getVertices();
		for (byte i = 0; i < 8; ++i) {
			if (this->contains(v[i]) == false) return false;
		}

		return true;
	}

	bool ConvexHull::contains(const Capsule& capsule) const
	{
		if (this->contains(capsule.capsuleLine) == false) return false;

		decimal rSq = square(capsule.radius);
		return magnitudeSq(capsule.pointA - this->closestPoint(capsule.pointA)) < rSq && magnitudeSq(capsule.pointB - this->closestPoint(capsule.pointB)) < rSq;
	}

	bool ConvexHull::contains(const Sphere& sphere) const
	{
		if (this->contains(sphere.center) == false) return false;
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) < square(sphere.radius);
	}

	bool ConvexHull::contains(const Polygon& polygon) const
	{
		for(uint32 i = 0, len1 = polygon.vertices.size(); i < len1; ++i) {
			if (this->contains(polygon.vertices[i]) == false) return false;
		}

		return true;
	}

	bool ConvexHull::contains(const Triangle& triangle) const
	{
		return this->contains(triangle.a) && this->contains(triangle.b) && this->contains(triangle.c);
	}

	bool ConvexHull::contains(const LineSegment& lineSegment) const
	{
		return this->contains(lineSegment.pointA) && this->contains(lineSegment.pointB);
	}

	bool ConvexHull::intersects(const ConvexHull& convexHull) const
	{
		//return GJKOverlap(*this, convexHull); //inaccurate

		HybridArray<Vec3, 6, uint16> normals = this->getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, convexHull, normals[x]) == false) return false;
		}

		HybridArray<LineSegment, 12, uint16> edges1 = this->getEdges();
		HybridArray<LineSegment, 12, uint16> edges2 = convexHull.getEdges();
		for (uint16 x = 0, len1 = edges1.size(); x < len1; ++x) {
			Vec3 d1 = edges1[x].getDirection();
			for (uint16 y = 0, len2 = edges2.size(); y < len2; ++y) {
				if (SATOverlap(*this, convexHull, crossProduct(d1, edges2[y].getDirection())) == false) return false;
			}
		}

		return true;
	}

	bool ConvexHull::intersects(const AABB& aabb) const
	{
		return GJKOverlap(*this, aabb);
	}

	bool ConvexHull::intersects(const OBB& obb) const
	{
		return GJKOverlap(*this, obb);
	}

	bool ConvexHull::intersects(const Capsule& capsule) const 
	{
		Vec3 closest = this->closestPoint(capsule.capsuleLine);
		return magnitudeSq(closest - capsule.capsuleLine.closestPoint(closest)) <= square(capsule.radius);
	}

	bool ConvexHull::intersects(const Sphere& sphere) const
	{
		return magnitudeSq(sphere.center - this->closestPoint(sphere.center)) <= square(sphere.radius);
	}

	bool ConvexHull::intersects(const Plane& plane) const
	{
		return SATOverlap(*this, plane, plane.normal);
	}

	bool ConvexHull::intersects(const Polygon& polygon) const
	{
		return GJKOverlap(*this, polygon);
	}

	bool ConvexHull::intersects(const Triangle& triangle) const
	{
		//return GJKOverlap(*this, triangle); //inaccurate

		if (SATOverlap(*this, triangle, crossProduct(triangle.b - triangle.a, triangle.c - triangle.a)) == false) return false;

		HybridArray<Vec3, 6, uint16> normals = this->getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, triangle, normals[x]) == false) return false;
		}

		return true;
	}

	bool ConvexHull::intersects(const Line& line) const
	{
		HybridArray<Vec3, 6, uint16> normals = this->getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, line, normals[x]) == false) return false;
		}

		HybridArray<LineSegment, 12, uint16> edges = this->getEdges();
		for (uint16 x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, line, crossProduct(line.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool ConvexHull::intersects(const Ray& ray) const
	{
		HybridArray<Vec3, 6, uint16> normals = this->getNormals();
		for (uint16 x = 0, len = normals.size(); x < len; ++x) {
			if (SATOverlap(*this, ray, normals[x]) == false) return false;
		}

		HybridArray<LineSegment, 12, uint16> edges = this->getEdges();
		for (uint16 x = 0, len = edges.size(); x < len; ++x) {
			if (SATOverlap(*this, ray, crossProduct(ray.direction, edges[x].getDirection())) == false) return false;
		}

		return true;
	}

	bool ConvexHull::intersects(const LineSegment& lineSegment) const
	{
		return GJKOverlap(*this, lineSegment);
	}

	void ConvexHull::transform(const Transform3D& t)
	{
		for (uint16 i = 0, len = this->vertices.size(); i < len; i++) {
			this->vertices[i] = t * this->vertices[i];
		}
	}

	ConvexHull ConvexHull::transformed(const Transform3D& t) const
	{
		ConvexHull c = *this;
		c.transform(t);
		return c;
	}
}
