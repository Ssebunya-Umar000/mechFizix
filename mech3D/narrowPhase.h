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

#ifndef NARROWPHASE_H
#define NARROWPHASE_H

#include"physicsData.h"
#include"geometry/plane.h"
#include"geometry/algorithms/GJK.h"

namespace mech {

	struct NarrowPhase {

		PhysicsData* physicsData = nullptr;

		NarrowPhase() {}
		NarrowPhase(const NarrowPhase&) = delete;
		NarrowPhase& operator=(const NarrowPhase&) = delete;

		void generateContacts(const Sphere& sphere1, const Sphere& sphere2, ContactManifold& manifold)
		{
			if (magnitudeSq(sphere2.center - sphere1.center) < square(sphere1.radius + sphere2.radius)) {
				manifold.flag = CollisionFlag::PENETRATING;

				manifold.contactPoints[manifold.numPoints].normal = normalise(sphere2.center - sphere1.center);
				manifold.contactPoints[manifold.numPoints].position[0] = sphere1.center + manifold.contactPoints[manifold.numPoints].normal * sphere1.radius;
				manifold.contactPoints[manifold.numPoints].position[1] = sphere2.center - manifold.contactPoints[manifold.numPoints].normal * sphere2.radius;
				manifold.contactPoints[manifold.numPoints].ID = 1;
				++manifold.numPoints;
			}
		}

		void generateContacts(const Sphere& sphere, const Capsule& capsule, ContactManifold& manifold)
		{
			Vec3 capsulePoint = capsule.capsuleLine.closestPoint(sphere.center);

			if (magnitudeSq(capsulePoint - sphere.center) < square(sphere.radius + capsule.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				manifold.contactPoints[manifold.numPoints].normal = normalise(capsulePoint - sphere.center);
				manifold.contactPoints[manifold.numPoints].position[0] = sphere.center + manifold.contactPoints[manifold.numPoints].normal * sphere.radius;
				manifold.contactPoints[manifold.numPoints].position[1] = capsulePoint - manifold.contactPoints[manifold.numPoints].normal * capsule.radius;
				manifold.contactPoints[manifold.numPoints].ID = 1;
				++manifold.numPoints;
			}
		}

		void generateContacts(const Sphere& sphere, const ConvexHull& convexHull, ContactManifold& manifold)
		{
			Vec3 closest = convexHull.closestPoint(sphere.center);

			if (magnitudeSq(closest - sphere.center) < square(sphere.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				manifold.contactPoints[manifold.numPoints].normal = normalise(closest - sphere.center);
				if (dotProduct(manifold.contactPoints[manifold.numPoints].normal, closest - sphere.center) < decimal(0.0)) {
					manifold.contactPoints[manifold.numPoints].normal = -manifold.contactPoints[manifold.numPoints].normal;
				}

				manifold.contactPoints[manifold.numPoints].position[0] = sphere.center + manifold.contactPoints[manifold.numPoints].normal * sphere.radius;
				manifold.contactPoints[manifold.numPoints].position[1] = closest;
				manifold.contactPoints[manifold.numPoints].ID = 1;
				++manifold.numPoints;
			}
		}

		void generateMeshContacts(const Sphere& sphere, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				if (sphere.intersects(triangles[x])) {

					Vec3 closest = triangles[x].closestPoint(sphere.center);

					if (magnitudeSq(closest - sphere.center) < sphere.radius * sphere.radius) {

						manifold.flag = CollisionFlag::PENETRATING;

						manifold.contactPoints[manifold.numPoints].normal = -triangles[x].getNormal();
						manifold.contactPoints[manifold.numPoints].position[0] = sphere.center + manifold.contactPoints[manifold.numPoints].normal * sphere.radius;
						manifold.contactPoints[manifold.numPoints].position[1] = closest;
						manifold.contactPoints[manifold.numPoints].ID = 1;
						++manifold.numPoints;
					}
				}
			}
		}

		void generateTerrainContacts(const Sphere& sphere, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				if (sphere.intersects(triangles[x])) {

					manifold.flag = CollisionFlag::PENETRATING;

					Vec3 closest = triangles[x].closestPoint(sphere.center);

					manifold.contactPoints[manifold.numPoints].normal = -triangles[x].getNormal();
					manifold.contactPoints[manifold.numPoints].position[0] = sphere.center + manifold.contactPoints[manifold.numPoints].normal * sphere.radius;
					manifold.contactPoints[manifold.numPoints].position[1] = closest;
					manifold.contactPoints[manifold.numPoints].ID = 1;
					++manifold.numPoints;
				}
			}
		}

		void generateContacts(const Capsule& capsule1, const Capsule& capsule2, ContactManifold& manifold)
		{
			Vec3 closest1 = capsule1.capsuleLine.closestPoint(capsule2.capsuleLine);
			Vec3 closest2 = capsule2.capsuleLine.closestPoint(closest1);

			if (magnitudeSq(closest1 - closest2) < square(capsule1.radius + capsule2.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				Vec3 dir1 = capsule1.capsuleLine.getDirection();
				Vec3 dir2 = capsule2.capsuleLine.getDirection();
				if (almostEqual(dotProduct(crossProduct(dir1, dir2), capsule1.pointA - capsule2.pointA), decimal(0.0))) {

					if (magnitudeSq(dir1) > magnitudeSq(dir2)) {

						Vec3 n = normalise(dir1);

						Plane plane1 = Plane(n, dotProduct(capsule1.pointA, n));
						Plane plane2 = Plane(n, dotProduct(capsule1.pointB, n));

						Vec3 pt1 = plane1.clip(capsule2.capsuleLine);
						if (nanVec(pt1) == false) {

							if (capsule2.contains(capsule1.pointA)) {
								manifold.contactPoints[manifold.numPoints].normal = normalise(capsule1.pointA - pt1);
							}
							else {
								manifold.contactPoints[manifold.numPoints].normal = normalise(pt1 - capsule1.pointA);
							}
							manifold.contactPoints[manifold.numPoints].position[0] = capsule1.pointA + manifold.contactPoints[manifold.numPoints].normal * capsule1.radius;
							manifold.contactPoints[manifold.numPoints].position[1] = pt1 - manifold.contactPoints[manifold.numPoints].normal * capsule2.radius;
							manifold.contactPoints[manifold.numPoints].ID = 1;
							++manifold.numPoints;
						}

						Vec3 pt2 = plane2.clip(capsule2.capsuleLine);
						if (nanVec(pt2) == false) {

							if (capsule2.contains(capsule1.pointB)) {
								manifold.contactPoints[manifold.numPoints].normal = normalise(capsule1.pointB - pt2);
							}
							else {
								manifold.contactPoints[manifold.numPoints].normal = normalise(pt2 - capsule1.pointB);
							}
							manifold.contactPoints[manifold.numPoints].position[0] = capsule1.pointB + manifold.contactPoints[manifold.numPoints].normal * capsule1.radius;
							manifold.contactPoints[manifold.numPoints].position[1] = pt2 - manifold.contactPoints[manifold.numPoints].normal * capsule2.radius;
							manifold.contactPoints[manifold.numPoints].ID = 2;
							++manifold.numPoints;
						}
					}
					else {

						Vec3 n = normalise(dir2);

						Plane plane1 = Plane(n, dotProduct(capsule2.pointA, n));
						Plane plane2 = Plane(n, dotProduct(capsule2.pointB, n));

						Vec3 pt1 = plane1.clip(capsule1.capsuleLine);
						if (nanVec(pt1) == false) {

							if (capsule2.contains(pt1)) {
								manifold.contactPoints[manifold.numPoints].normal = normalise(pt1 - capsule2.pointA);
							}
							else {
								manifold.contactPoints[manifold.numPoints].normal = normalise(capsule2.pointA - pt1);
							}
							manifold.contactPoints[manifold.numPoints].position[0] = pt1 + manifold.contactPoints[manifold.numPoints].normal * capsule1.radius;
							manifold.contactPoints[manifold.numPoints].position[1] = capsule2.pointA - manifold.contactPoints[manifold.numPoints].normal * capsule2.radius;
							manifold.contactPoints[manifold.numPoints].ID = 3;
							++manifold.numPoints;
						}

						Vec3 pt2 = plane2.clip(capsule1.capsuleLine);
						if (nanVec(pt2) == false) {

							if (capsule2.contains(pt2)) {
								manifold.contactPoints[manifold.numPoints].normal = normalise(pt2 - capsule2.pointB);
							}
							else {
								manifold.contactPoints[manifold.numPoints].normal = normalise(capsule2.pointB - pt2);
							}
							manifold.contactPoints[manifold.numPoints].position[0] = pt2 + manifold.contactPoints[manifold.numPoints].normal * capsule1.radius;
							manifold.contactPoints[manifold.numPoints].position[1] = capsule2.pointB - manifold.contactPoints[manifold.numPoints].normal * capsule2.radius;
							manifold.contactPoints[manifold.numPoints].ID = 4;
							++manifold.numPoints;
						}
					}
				}
				else {

					manifold.contactPoints[manifold.numPoints].normal = normalise(closest2 - closest1);
					manifold.contactPoints[manifold.numPoints].position[0] = closest1 + manifold.contactPoints[manifold.numPoints].normal * capsule1.radius;
					manifold.contactPoints[manifold.numPoints].position[1] = closest2 - manifold.contactPoints[manifold.numPoints].normal * capsule2.radius;
					manifold.contactPoints[manifold.numPoints].ID = 5;
					++manifold.numPoints;
				}
			}
		}

		void generateContacts(const Capsule& capsule, const Sphere& sphere, ContactManifold& manifold)
		{
			generateContacts(sphere, capsule, manifold);
			manifold.revert();
		}

		void generateContacts(const Capsule& capsule, const ConvexHull& convexHull, ContactManifold& manifold)
		{
			Vec3 point1, point2, closestAxisNormal;
			decimal penetrationClosestAxis = decimalMAX;
			{
				Vec3 p2 = convexHull.closestPoint(capsule.capsuleLine);
				Vec3 p1 = capsule.capsuleLine.closestPoint(p2);
				
				point2 = convexHull.closestPoint(p1);
				closestAxisNormal = normalise(point2 - p1);
				point1 = capsule.getSupportPoint(closestAxisNormal);

				if (dotProduct(closestAxisNormal, point1 - capsule.pointA) < decimal(0.0)) {
					closestAxisNormal = -closestAxisNormal;
				}

				penetrationClosestAxis = dotProduct(point1 - point2, closestAxisNormal);
				if (penetrationClosestAxis < decimal(0.0)) {
					return;
				}
			}

			uint32 minFace = -1;
			Vec3 faceNormal = nanVEC3;
			Vec3 pointOnCapsule = nanVEC3;
			decimal minFacePenetration = decimalMAX;
			for (uint32 x = 0, len = convexHull.halfEdgeMesh.faces.size(); x < len; ++x) {

				Vec3 n = convexHull.getFaceNormal(x);
				Vec3 c = capsule.getSupportPoint(-n);

				decimal d = dotProduct(convexHull.vertices[convexHull.halfEdgeMesh.faces[x].faceVerts.front()] - c, n);
				if (d < decimal(0.0)) {
					return;
				}
				else {
					if (d < minFacePenetration) {
						minFace = x;
						minFacePenetration = d;
						faceNormal = n;
						pointOnCapsule = c;
					}
				}
			}

			if (penetrationClosestAxis < minFacePenetration) {

				manifold.contactPoints[manifold.numPoints].normal = closestAxisNormal;
				manifold.contactPoints[manifold.numPoints].position[0] = point1;
				manifold.contactPoints[manifold.numPoints].position[1] = point2;
				manifold.contactPoints[manifold.numPoints].ID = -1;
				++manifold.numPoints;
			}
			else {

				if (almostEqual(dotProduct(faceNormal, capsule.capsuleLine.getDirection()), decimal(0.0))) {

					Vec3 points[2] = { capsule.pointA, capsule.pointB };
					bool inside[2] = { true, true };
					StackArray<Vec3, 2> clipPoints(nanVEC3);

					uint16 edgeIndex = convexHull.halfEdgeMesh.faces[minFace].edgeIndex;
					do {
						Plane plane = convexHull.getFacePlane(convexHull.halfEdgeMesh.edges[convexHull.halfEdgeMesh.twinEdge(edgeIndex)].faceIndex);
						for (byte x = 0; x < 2; ++x) {
							if (plane.getDistanceFromPlane(points[x]) > decimal(0.0)) {
								inside[x] = false;
							}
						}

						Vec3 p = plane.clip(capsule.capsuleLine);
						if (nanVec(p) == false) {
							clipPoints.pushBack(p);
						}

						edgeIndex = convexHull.halfEdgeMesh.edges[edgeIndex].nextIndex;

					} while (edgeIndex != convexHull.halfEdgeMesh.faces[minFace].edgeIndex);

					for (byte x = 0; x < 2; ++x) {

						if (inside[x] == true) {
							
							Vec3 point = points[x] + (-faceNormal * capsule.radius);
							manifold.contactPoints[manifold.numPoints].normal = -faceNormal;
							manifold.contactPoints[manifold.numPoints].position[0] = point;
							manifold.contactPoints[manifold.numPoints].position[1] = convexHull.getFacePlane(minFace).closestPoint(point);
							manifold.contactPoints[manifold.numPoints].ID = pairingFunction(minFace, 0);
							++manifold.numPoints;
						}
					}

					for (byte x = 0; x < 2; ++x) {
						
						if (manifold.numPoints == 2) break;

						Vec3 point = clipPoints[x] + (-faceNormal * capsule.radius);
						manifold.contactPoints[manifold.numPoints].normal = -faceNormal;
						manifold.contactPoints[manifold.numPoints].position[0] = point;
						manifold.contactPoints[manifold.numPoints].position[1] = convexHull.getFacePlane(minFace).closestPoint(point);
						manifold.contactPoints[manifold.numPoints].ID = pairingFunction(minFace, 1);
						++manifold.numPoints;
					}
				}
				else {

					manifold.contactPoints[manifold.numPoints].normal = -faceNormal;
					manifold.contactPoints[manifold.numPoints].position[0] = pointOnCapsule;
					manifold.contactPoints[manifold.numPoints].position[1] = convexHull.getFacePlane(minFace).closestPoint(pointOnCapsule);
					manifold.contactPoints[manifold.numPoints].ID = pairingFunction(minFace, 2);
					++manifold.numPoints;
				}
			}

			manifold.flag = CollisionFlag::PENETRATING;
		}

		void generateMeshContacts(const Capsule& capsule, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			Vec3 dir = capsule.capsuleLine.getDirection();
			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				if (capsule.intersects(triangles[x])) {

					Plane plane = triangles[x].toPlane();

					if (almostEqual(dotProduct(dir, plane.normal), decimal(0.0))) {

						Vec3 closest[2] = { plane.closestPoint(capsule.pointA), plane.closestPoint(capsule.pointB) };
						for (byte y = 0; y < 2; ++y) {

							if (triangles[x].contains(closest[y])) {

								manifold.flag = CollisionFlag::PENETRATING;

								manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
								manifold.contactPoints[manifold.numPoints].position[0] = capsule.capsuleLine.closestPoint(closest[y]) + manifold.contactPoints[manifold.numPoints].normal * capsule.radius;
								manifold.contactPoints[manifold.numPoints].position[1] = closest[y];
								manifold.contactPoints[manifold.numPoints].ID = pairingFunction(y, x);
								++manifold.numPoints;
							}
						}
					}
					else {

						Vec3 closest = plane.closestPoint(capsule.capsuleLine);

						if (triangles[x].contains(closest)) {

							manifold.flag = CollisionFlag::PENETRATING;

							manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
							manifold.contactPoints[manifold.numPoints].position[0] = capsule.capsuleLine.closestPoint(closest) + manifold.contactPoints[manifold.numPoints].normal * capsule.radius;
							manifold.contactPoints[manifold.numPoints].position[1] = closest;
							manifold.contactPoints[manifold.numPoints].ID = pairingFunction(2, x);
							++manifold.numPoints;
						}
					}
				}
			}
		}

		void generateTerrainContacts(const Capsule& capsule, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			Vec3 dir = capsule.capsuleLine.getDirection();
			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				if (capsule.intersects(triangles[x])) {

					Plane plane = triangles[x].toPlane();

					if (almostEqual(dotProduct(dir, plane.normal), decimal(0.0))) {

						Vec3 closest[2] = { plane.closestPoint(capsule.pointA), plane.closestPoint(capsule.pointB) };
						for (byte y = 0; y < 2; ++y) {

							if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

							if (triangles[x].contains(closest[y])) {

								manifold.flag = CollisionFlag::PENETRATING;

								manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
								manifold.contactPoints[manifold.numPoints].position[0] = capsule.capsuleLine.closestPoint(closest[y]) + manifold.contactPoints[manifold.numPoints].normal * capsule.radius;
								manifold.contactPoints[manifold.numPoints].position[1] = closest[y];
								manifold.contactPoints[manifold.numPoints].ID = pairingFunction(y, x);
								++manifold.numPoints;
							}
						}
					}
					else {

						manifold.flag = CollisionFlag::PENETRATING;

						Vec3 closest = plane.closestPoint(capsule.capsuleLine);
						manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
						manifold.contactPoints[manifold.numPoints].position[0] = capsule.capsuleLine.closestPoint(closest) + manifold.contactPoints[manifold.numPoints].normal * capsule.radius;
						manifold.contactPoints[manifold.numPoints].position[1] = closest;
						manifold.contactPoints[manifold.numPoints].ID = pairingFunction(2, x);
						++manifold.numPoints;
					}
				}
			}
		}

		void generateContacts(const ConvexHull& convexHull1, const ConvexHull& convexHull2, ContactManifold& manifold)
		{
			struct HullVsHull {

				struct ReferenceFace {
					decimal minFacePenetration = decimalMAX;
					uint16 refFace = -1;
					byte ref = 0;
					bool separatingAxisFound = false;
				};

				struct MinimumEdges {
					LineSegment edge1 = LineSegment(nanVEC3, nanVEC3);
					LineSegment edge2 = LineSegment(nanVEC3, nanVEC3);
					Vec3 edgeNormal;
					decimal minEdgePenetration = decimalMAX;
					bool separatingAxisFound = false;
				};

				const ConvexHull& convexHull1;
				const ConvexHull& convexHull2;
				ContactManifold& manifold;

				HullVsHull(const ConvexHull& c1, const ConvexHull& c2, ContactManifold& m) : convexHull1(c1), convexHull2(c2), manifold(m) {}

				ReferenceFace getReferenceFace()
				{
					ReferenceFace ref;
					for (uint32 x = 0, len = this->convexHull1.halfEdgeMesh.faces.size(); x < len; ++x) {

						Vec3 normal = this->convexHull1.getFaceNormal(x);
						decimal d = dotProduct(this->convexHull1.vertices[this->convexHull1.halfEdgeMesh.faces[x].faceVerts.front()] - this->convexHull2.getSupportPoint(-normal), normal);

						if (d < decimal(0.0)) {
							ref.separatingAxisFound = true;
							return ref;
						}
						else {
							if (d < ref.minFacePenetration) {
								ref.minFacePenetration = d;
								ref.refFace = x;
								ref.ref = 1;
							}
						}
					}
					for (uint32 x = 0, len = this->convexHull2.halfEdgeMesh.faces.size(); x < len; ++x) {

						Vec3 normal = this->convexHull2.getFaceNormal(x);
						decimal d = dotProduct(this->convexHull2.vertices[this->convexHull2.halfEdgeMesh.faces[x].faceVerts.front()] - this->convexHull1.getSupportPoint(-normal), normal);

						if (d < decimal(0.0)) {
							ref.separatingAxisFound = true;
							return ref;
						}
						else {
							if (d < ref.minFacePenetration) {
								ref.minFacePenetration = d;
								ref.refFace = x;
								ref.ref = 2;
							}
						}
					}

					return ref;
				}

				uint16 getIncidentFace(const ConvexHull& incidentConvexHull, const Vec3& refFaceNormal)
				{
					uint16 incidentFace = -1;
					decimal least = decimalMAX;
					for (uint32 x = 0, len = incidentConvexHull.halfEdgeMesh.faces.size(); x < len; ++x) {
						decimal d = dotProduct(incidentConvexHull.getFaceNormal(x), refFaceNormal);
						if (d < least) {
							incidentFace = x;
							least = d;
						}
					}

					return incidentFace;
				}

				bool edgesBuildMinkowskiFace(const uint16& e1, const uint16& e2, const Vec3& dir1, const Vec3& dir2)
				{
					decimal adc = dotProduct(this->convexHull1.getFaceNormal(this->convexHull1.halfEdgeMesh.edges[e1].faceIndex), dir2);
					decimal bdc = dotProduct(this->convexHull1.getFaceNormal(this->convexHull1.halfEdgeMesh.edges[this->convexHull1.halfEdgeMesh.twinEdge(e1)].faceIndex), dir2);
					decimal cba = dotProduct(-this->convexHull2.getFaceNormal(this->convexHull2.halfEdgeMesh.edges[e2].faceIndex), dir1);
					decimal dba = dotProduct(-this->convexHull2.getFaceNormal(this->convexHull2.halfEdgeMesh.edges[this->convexHull2.halfEdgeMesh.twinEdge(e2)].faceIndex), dir1);

					return cba * dba < decimal(0.0) && adc* bdc < decimal(0.0) && cba* bdc > decimal(0.0);
				}

				MinimumEdges getMinimumEdges(const Vec3& center1)
				{
					MinimumEdges best;

					for (uint32 x = 0, len = this->convexHull1.halfEdgeMesh.edges.size(); x < len; ++x) {

						if (this->convexHull1.halfEdgeMesh.edges[x].duplicate) continue;

						for (uint32 y = 0, len = this->convexHull2.halfEdgeMesh.edges.size(); y < len; ++y) {

							if (this->convexHull2.halfEdgeMesh.edges[y].duplicate) continue;

							LineSegment e1 = this->convexHull1.getEdge(x);
							LineSegment e2 = this->convexHull2.getEdge(y);

							Vec3 dir1 = e1.getDirection();
							Vec3 dir2 = e2.getDirection();

							if (edgesBuildMinkowskiFace(x, y, dir1, dir2)) {

								Vec3 axis = crossProduct(dir1, dir2);

								if (almostEqual(magnitudeSq(axis), decimal(0.0))) continue;

								axis = normalise(axis);

								if (dotProduct(center1 - e1.start, axis) > decimal(0.0)) {
									axis = -axis;
								}

								decimal d = dotProduct(e2.start - e1.start, -axis);
								if (d < decimal(0.0)) {
									best.separatingAxisFound = true;
									return best;
								}
								else {
									if (d < best.minEdgePenetration) {
										best.edge1 = e1;
										best.edge2 = e2;
										best.edgeNormal = axis;
										best.minEdgePenetration = d;
									}
								}
							}
						}
					}

					return best;
				}

				void generateEdgeContact(const Vec3& edgeNormal, const LineSegment& edge1, const LineSegment& edge2)
				{
					this->manifold.contactPoints[this->manifold.numPoints].normal = edgeNormal;
					this->manifold.contactPoints[this->manifold.numPoints].position[0] = edge1.closestPoint(edge2);
					this->manifold.contactPoints[this->manifold.numPoints].position[1] = edge2.closestPoint(edge1);
					this->manifold.contactPoints[this->manifold.numPoints].ID = 0;
					++this->manifold.numPoints;
				}

				void generateFaceContacts(const ConvexHull& refConvexHull, const ConvexHull& incidentConvexHull, const uint16& refFace, const Vec3 & refFaceNormal, const uint16& incidentFace)
				{
					Plane refPlane = refConvexHull.getFacePlane(refFace);
					Polygon incidentPolygon = incidentConvexHull.getFacePolygon(incidentFace);

					uint16 edgeIndex = refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex;
					do {

						HybridArray<Vec3, 4, byte> pts;

						HybridArray<LineSegment, 4, byte> edges = refConvexHull.getFacePolygon(refConvexHull.halfEdgeMesh.edges[refConvexHull.halfEdgeMesh.twinEdge(edgeIndex)].faceIndex).getEdges();
						for (uint32 x = 0, len = edges.size(); x < len; ++x) {
							Vec3 p = incidentPolygon.clip(edges[x]);
							if (nanVec(p) == false) {
								pts.pushBack(p);
							}
						}

						for (uint32 x = 0, len = pts.size(); x < len; ++x) {

							bool pass = true;
							for (uint32 y = 0; y < this->manifold.numPoints; ++y) {
								if (this->manifold.contactPoints[y].position[1] == pts[x]) {
									pass = false;
									break;
								}
							}

							if (pass == true) {

								this->manifold.contactPoints[this->manifold.numPoints].normal = refFaceNormal;
								this->manifold.contactPoints[this->manifold.numPoints].position[0] = refPlane.closestPoint(pts[x]);
								this->manifold.contactPoints[this->manifold.numPoints].position[1] = pts[x];
								this->manifold.contactPoints[this->manifold.numPoints].ID = pairingFunction(refFace, pairingFunction(edgeIndex, x));
								++this->manifold.numPoints;
							}
						}

						edgeIndex = refConvexHull.halfEdgeMesh.edges[edgeIndex].nextIndex;

					} while (edgeIndex != refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex);

					for (uint32 x = 0, len = incidentPolygon.vertices.size(); x < len; ++x) {

						if (refPlane.getDistanceFromPlane(incidentPolygon.vertices[x]) < decimal(0.0)) {

							bool pass = true;
							uint16 edgeIndex = refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex;
							do {
								if (refConvexHull.getFacePlane(refConvexHull.halfEdgeMesh.edges[refConvexHull.halfEdgeMesh.twinEdge(edgeIndex)].faceIndex).getDistanceFromPlane(incidentPolygon.vertices[x]) > decimal(0.0)) {
									pass = false;
									break;
								}
								edgeIndex = refConvexHull.halfEdgeMesh.edges[edgeIndex].nextIndex;

							} while (edgeIndex != refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex);

							if (pass == true) {

								this->manifold.contactPoints[this->manifold.numPoints].ID = pairingFunction(refFace, x);
								this->manifold.contactPoints[this->manifold.numPoints].position[0] = refPlane.closestPoint(incidentPolygon.vertices[x]);
								this->manifold.contactPoints[this->manifold.numPoints].position[1] = incidentPolygon.vertices[x];
								this->manifold.contactPoints[this->manifold.numPoints].normal = refFaceNormal;
								++this->manifold.numPoints;
							}
						}
					}

					if (this->manifold.numPoints > MAXIMUM_CONTACT_POINTS) {
						this->manifold.enforce4Contacts();
					}
				}

				void contactsFromCache(HullVsHullContactCache& cache)
				{
					if (cache.cacheFlags & 0b00001000) {

						if (cache.ID1 == this->manifold.colliderID1.ID) {
							this->generateFaceContacts(this->convexHull1, this->convexHull2, cache.refFace, this->convexHull1.getFaceNormal(cache.refFace), cache.incidentFace);
						}
						else {
							this->generateFaceContacts(this->convexHull2, this->convexHull1, cache.refFace, this->convexHull2.getFaceNormal(cache.refFace), cache.incidentFace);
						}
					}
					else {

						if (cache.ID1 == this->manifold.colliderID1.ID) {
							this->generateFaceContacts(this->convexHull2, this->convexHull1, cache.refFace, this->convexHull2.getFaceNormal(cache.refFace), cache.incidentFace);
							this->manifold.revert();
						}
						else {
							this->generateFaceContacts(this->convexHull1, this->convexHull2, cache.refFace, this->convexHull1.getFaceNormal(cache.refFace), cache.incidentFace);
						}
					}

					if (this->manifold.numPoints > 0) {
						this->manifold.flag = CollisionFlag::PENETRATING;
					}
				}

				void contactsFromScratch(PhysicsData* physicsData, HullVsHullContactCache& cache)
				{
					/*
						--------------cache flags----------------
						cache is not empty               - 0b00000001
						hulls have not shifted alot      - 0b00000010
						minimum axis is a face normal    - 0b00000100
						referance face is on hull one    - 0b00001000
					*/
					cache.cacheFlags = 0;
					cache.cacheFlags |= 0b00000001;

					GJKResult gjk = GJKAlgorithm(this->convexHull1, this->convexHull2, true, cache.gjkAxis);

					if (gjk.overlap == true) {

						ReferenceFace reference = this->getReferenceFace();

						if (reference.separatingAxisFound == false) {

							MinimumEdges minEdges = this->getMinimumEdges(physicsData->convexHullColliders[this->manifold.colliderID1.colliderIndex].bound.getCenter());

							if (minEdges.separatingAxisFound == false) {

								if (reference.minFacePenetration <= (minEdges.minEdgePenetration + mathEPSILON)) {

									const ConvexHull& refConvexHull = reference.ref == 1 ? this->convexHull1 : this->convexHull2;
									const ConvexHull& incidentConvexHull = reference.ref == 1 ? this->convexHull2 : this->convexHull1;

									Vec3 normal = refConvexHull.getFaceNormal(reference.refFace);
									uint16 incidentFace = this->getIncidentFace(incidentConvexHull, normal);
									this->generateFaceContacts(refConvexHull, incidentConvexHull, reference.refFace, normal, incidentFace);

									cache.cacheFlags |= 0b00000100;
									cache.refFace = reference.refFace;
									cache.incidentFace = incidentFace;

									if (reference.ref == 1) {
										cache.cacheFlags |= 0b00001000;
									}
									else {
										this->manifold.revert();
									}
								}
								else {
									this->generateEdgeContact(minEdges.edgeNormal, minEdges.edge1, minEdges.edge2);
								}

								this->manifold.flag = CollisionFlag::PENETRATING;
							}
						}

						cache.gjkAxis = gjk.separatingAxis;
						cache.ID1 = this->manifold.colliderID1.ID;
					}
				}
			};

			HullVsHull hullVsHull(convexHull1, convexHull2, manifold);

			HullVsHullContactCache& cache = this->physicsData->hullVsHullContactCache.insert(manifold.ID)->second;

			Vec3 c1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound.getCenter();
			Vec3 c2 = this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].bound.getCenter();

			cache.cacheFlags |= (mathABS(magnitudeSq(c1 - c2) - magnitudeSq(cache.center1 - cache.center2)) < this->physicsData->minimalDispacement) ? 0b00000010 : 0;

			if ((cache.cacheFlags & 0b00000001) && (cache.cacheFlags & 0b00000010) && (cache.cacheFlags & 0b00000100)) {
				hullVsHull.contactsFromCache(cache);
			}
			else {
				cache.center1 = c1;
				cache.center2 = c2;
				hullVsHull.contactsFromScratch(this->physicsData, cache);
			}

			cache.retention = this->physicsData->framesToRetainCache;
		}

		void generateContacts(const ConvexHull& convexHull, const Sphere& sphere, ContactManifold& manifold)
		{
			generateContacts(sphere, convexHull, manifold);
			manifold.revert();
		}

		void generateContacts(const ConvexHull& convexHull, const Capsule& capsule, ContactManifold& manifold)
		{
			generateContacts(capsule, convexHull, manifold);
			manifold.revert();
		}

		void generateMeshContacts(const ConvexHull& convexHull, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			HybridArray<uint16, 8, uint16> registered;
			for (uint32 x = 0, len1 = triangles.size(); x < len1; ++x) {

				Plane plane = triangles[x].toPlane();

				for (uint32 y = 0, len2 = convexHull.vertices.size(); y < len2; ++y) {

					if (registered.find(y) == false) {

						if (plane.getDistanceFromPlane(convexHull.vertices[y]) < decimal(0.0)) {

							Vec3 closest = plane.closestPoint(convexHull.vertices[y]);

							if (triangles[x].contains(closest)) {
								registered.pushBack(y);

								manifold.flag = CollisionFlag::PENETRATING;

								manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
								manifold.contactPoints[manifold.numPoints].position[0] = convexHull.vertices[y];
								manifold.contactPoints[manifold.numPoints].position[1] = closest;
								manifold.contactPoints[manifold.numPoints].ID = y;
								++manifold.numPoints;
							}
						}
					}
				}
			}

			if (manifold.numPoints == 0) {

				for (uint32 x = 0, len1 = triangles.size(); x < len1; ++x) {

					if (convexHull.intersects(triangles[x])) {

						HybridArray<LineSegment, 12, uint16> edges = convexHull.getEdges();
						for (uint32 y = 0, len = edges.size(); y < len; ++y) {

							Vec3 point = triangles[x].clip(edges[y]);
							if (nanVec(point) == false) {

								manifold.flag = CollisionFlag::PENETRATING;

								Vec3 trinaglePoint;
								{
									StackArray<LineSegment, 3> triangleEdges = triangles[x].getEdges();
									decimal least = decimalMAX;
									for (byte z = 0; z < 3; ++z) {
										Vec3 c = triangleEdges[z].closestPoint(point);
										decimal d = magnitudeSq(c - point);
										if (d < least) {
											least = d;
											trinaglePoint = c;
										}
									}
								}

								manifold.contactPoints[manifold.numPoints].position[0] = point;
								manifold.contactPoints[manifold.numPoints].position[1] = trinaglePoint;
								manifold.contactPoints[manifold.numPoints].normal = normalise(manifold.contactPoints[manifold.numPoints].position[1] - manifold.contactPoints[manifold.numPoints].position[0]);
								manifold.contactPoints[manifold.numPoints].ID = y;
								++manifold.numPoints;
							}
						}
					}
				}
			}

			if (manifold.numPoints > MAXIMUM_CONTACT_POINTS) {
				manifold.enforce4Contacts();
			}
		}

		void generateTerrainContacts(const ConvexHull& convexHull, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			decimal midHeight = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound.getCenter().y;

			HybridArray<uint16, 8, uint16> registered;
			for (uint32 x = 0, len1 = triangles.size(); x < len1; ++x) {

				Plane plane = triangles[x].toPlane();

				for (uint32 y = 0, len2 = convexHull.vertices.size(); y < len2; ++y) {

					if (convexHull.vertices[y].y < midHeight && registered.find(y) == false) {

						if (plane.getDistanceFromPlane(convexHull.vertices[y]) < decimal(0.0)) {

							Vec3 closest = plane.closestPoint(convexHull.vertices[y]);

							if (triangles[x].contains(closest)) {
								
								registered.pushBack(y);

								manifold.flag = CollisionFlag::PENETRATING;

								manifold.contactPoints[manifold.numPoints].normal = -plane.normal;
								manifold.contactPoints[manifold.numPoints].position[0] = convexHull.vertices[y];
								manifold.contactPoints[manifold.numPoints].position[1] = closest;
								manifold.contactPoints[manifold.numPoints].ID = y;
								++manifold.numPoints;
							}
						}
					}
				}
			}

			if (manifold.numPoints > MAXIMUM_CONTACT_POINTS) {
				manifold.enforce4Contacts();
			}
		}
	};
}

#endif

