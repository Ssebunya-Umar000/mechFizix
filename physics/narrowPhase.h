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

#ifndef NARROWPHASE_H
#define NARROWPHASE_H

#include"physicsData.h"
#include"../geometry/plane.h"
#include"../geometry/algorithms/GJK.h"

namespace mech {

	struct NarrowPhase {

		PhysicsData* physicsData = nullptr;

		NarrowPhase() {}
		NarrowPhase(const NarrowPhase&) = delete;
		NarrowPhase& operator=(const NarrowPhase&) = delete;

		void generateContacts(const Sphere& sphere1, const Sphere& sphere2, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::SphereVsSphere");

			if (magnitudeSq(sphere2.center - sphere1.center) < square(sphere1.radius + sphere2.radius)) {
			
				manifold.flag = CollisionFlag::PENETRATING;
				Vec3 n = normalise(sphere2.center - sphere1.center);
				manifold.addContact(n, sphere1.center + n * sphere1.radius, sphere2.center - n * sphere2.radius, 1);
			}

			END_PROFILE;
		}

		void generateContacts(const Sphere& sphere, const Capsule& capsule, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::SphereVsCapsule");

			Vec3 capsulePoint = capsule.capsuleLine.closestPoint(sphere.center);

			if (magnitudeSq(capsulePoint - sphere.center) < square(sphere.radius + capsule.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;
				Vec3 n = normalise(capsulePoint - sphere.center);
				manifold.addContact(n, sphere.center + n * sphere.radius, capsulePoint - n * capsule.radius, 1);
			}

			END_PROFILE;
		}

		void generateContacts(const Sphere& sphere, const ConvexHull& convexHull, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::SphereVsConvexHull");

			Vec3 closest = convexHull.closestPoint(sphere.center);

			if (magnitudeSq(closest - sphere.center) < square(sphere.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				if (convexHull.contains(sphere.center)) {

					Vec3 convexHullPoint = nanVEC3;
					Vec3 faceNormal = nanVEC3;
					uint32 minFace = -1;
					{
						decimal minPenetration = decimalMAX;
						for (uint32 x = 0, len = convexHull.halfEdgeMesh.faces.size(); x < len; ++x) {

							Plane plane = convexHull.getFacePlane(x);
							Vec3 p = plane.closestPoint(sphere.center);
							decimal d = magnitudeSq(p - sphere.center);
							if (d < minPenetration) {
								minFace = x;
								minPenetration = d;
								convexHullPoint = p;
								faceNormal = plane.normal;
							}
						}
					}

					manifold.addContact(-faceNormal, sphere.center + -faceNormal * sphere.radius, convexHullPoint, 1);
				}
				else {

					Vec3 n = normalise(closest - sphere.center);
					manifold.addContact(n, sphere.center + n * sphere.radius, closest, 2);
				}
			}

			END_PROFILE;
		}

		void generateContacts(const Sphere& sphere, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::SphereVstriangles");

			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				Vec3 closest = triangles[x].closestPoint(sphere.center);

				if (magnitudeSq(closest - sphere.center) < square(sphere.radius)) {

					manifold.flag = CollisionFlag::PENETRATING;

					Vec3 n = normalise(closest - sphere.center);
					if (triangles[x].toPlane().getDistanceFromPlane(sphere.center) < decimal(0.0)) {
						n = -n;
					}
					manifold.addContact(n, sphere.center + n * sphere.radius, closest, 1);

					DEBUG_RENDERER_ADD(triangles[x], WHITE);
				}
			}

			END_PROFILE;
		}

		void generateContacts(const Capsule& capsule1, const Capsule& capsule2, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::CapsuleVsCapsule");

			Vec3 closest1 = capsule1.capsuleLine.closestPoint(capsule2.capsuleLine);
			Vec3 closest2 = capsule2.capsuleLine.closestPoint(closest1);

			if (magnitudeSq(closest1 - closest2) < square(capsule1.radius + capsule2.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				if (almostEqual(dotProduct(crossProduct(capsule1.capsuleLine.getDirection(), capsule2.capsuleLine.getDirection()), capsule1.pointA - capsule2.pointA), decimal(0.0))) {

					Vec3 cA1 = capsule1.capsuleLine.closestPoint(capsule2.pointA);
					Vec3 cA2 = capsule2.capsuleLine.closestPoint(cA1);
					Vec3 n1 = normalise(cA2 - cA1);
					manifold.addContact(n1, cA1 + n1 * capsule1.radius, cA2 - n1 * capsule2.radius, 1);

					Vec3 cB1 = capsule1.capsuleLine.closestPoint(capsule2.pointB);
					Vec3 cB2 = capsule2.capsuleLine.closestPoint(cB1);
					Vec3 n2 = normalise(cB2 - cB1);
					manifold.addContact(n2, cB1 + n2 * capsule1.radius, cB2 - n2 * capsule2.radius, 2);
				}
				else {

					Vec3 n = normalise(closest2 - closest1);
					manifold.addContact(n, closest1 + n * capsule1.radius, closest2 - n * capsule2.radius, 3);
				}
			}

			END_PROFILE;
		}

		void generateContacts(const Capsule& capsule, const Sphere& sphere, ContactManifold& manifold)
		{
			generateContacts(sphere, capsule, manifold);
			manifold.revert();
		}

		void generateContacts(const Capsule& capsule, const ConvexHull& convexHull, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::CapsuleVsConvexHull");

			Vec3 closetOnHull = convexHull.closestPoint(capsule.capsuleLine);
			Vec3 closestOnLine = capsule.capsuleLine.closestPoint(closetOnHull);

			if (magnitudeSq(closetOnHull - closestOnLine) < square(capsule.radius)) {

				manifold.flag = CollisionFlag::PENETRATING;

				uint32 minFace = -1;
				Vec3 faceNormal = nanVEC3;
				Vec3 pointOnCapsule = nanVEC3;
				{
					decimal minPenetration = decimalMAX;
					for (uint32 x = 0, len = convexHull.halfEdgeMesh.faces.size(); x < len; ++x) {

						Vec3 n = convexHull.getFaceNormal(x);
						Vec3 c = capsule.getSupportPoint(-n);

						decimal d = dotProduct(convexHull.vertices[convexHull.halfEdgeMesh.faces[x].faceVerts.front()] - c, n);
						if (d < minPenetration) {
							minFace = x;
							minPenetration = d;
							faceNormal = n;
							pointOnCapsule = c;
						}
					}
				}

				if (almostEqual(magnitudeSq(closestOnLine - closetOnHull), decimal(0.0))) {

					if (convexHull.contains(capsule.capsuleLine.pointA) == false && convexHull.contains(capsule.capsuleLine.pointB) == false) {

						Vec3 edgePoint;
						Vec3 capsuleLinePoint;
						{
							decimal min = decimalMAX;
							for (uint32 x = 0, len = convexHull.halfEdgeMesh.edges.size(); x < len; ++x) {

								if (convexHull.halfEdgeMesh.edges[x].duplicate == true) continue;

								Vec3 p2 = convexHull.getEdge(x).closestPoint(capsule.capsuleLine);
								Vec3 p1 = capsule.capsuleLine.closestPoint(p2);

								decimal d = magnitudeSq(p2 - p1);
								if (d < min) {
									min = d;
									capsuleLinePoint = p1;
									edgePoint = p2;
								}
							}
						}

						Vec3 n = normalise(edgePoint - capsuleLinePoint);
						manifold.addContact(n, capsuleLinePoint + n * capsule.radius, edgePoint, 1e10);
					}
					else {
						manifold.addContact(-faceNormal, pointOnCapsule, convexHull.getFacePlane(minFace).closestPoint(pointOnCapsule), 2e10);
					}
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
							if (isNanVec(p) == false) {
								clipPoints.pushBack(p);
							}

							edgeIndex = convexHull.halfEdgeMesh.edges[edgeIndex].nextIndex;

						} while (edgeIndex != convexHull.halfEdgeMesh.faces[minFace].edgeIndex);

						for (byte x = 0; x < 2; ++x) {

							if (inside[x] == true) {
								Vec3 p = points[x] + (-faceNormal * capsule.radius);
								manifold.addContact(-faceNormal, p, convexHull.getFacePlane(minFace).closestPoint(p), pairingFunction(pairingFunction(minFace, 0), x));
							}
							else {

								if (clipPoints.empty() == false) {
									Vec3 p = clipPoints.back() + (-faceNormal * capsule.radius);
									manifold.addContact(-faceNormal, p, convexHull.getFacePlane(minFace).closestPoint(p), pairingFunction(pairingFunction(minFace, 1), x));
									clipPoints.popBack();
								}
							}
						}
					}
					else {
						Vec3 n = normalise(closetOnHull - closestOnLine);
						manifold.addContact(n, closestOnLine + n * capsule.radius, closetOnHull, 3e10);
					}
				}
			}

			END_PROFILE;
		}

		void generateContacts(const Capsule& capsule, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold)
		{
			BEGIN_PROFILE("NarrowPhase::CapsuleVstriangles");

			Vec3 dir = capsule.capsuleLine.getDirection();
			for (uint32 x = 0, len = triangles.size(); x < len; ++x) {

				if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

				Vec3 closest2 = triangles[x].closestPoint(capsule.capsuleLine);
				Vec3 closest1 = capsule.capsuleLine.closestPoint(closest2);
				
				if (magnitudeSq(closest2 - closest1) < square(capsule.radius)) {

					Plane plane = triangles[x].toPlane();

					if (almostEqual(dotProduct(dir, plane.normal), decimal(0.0))) {

						Vec3 closest[2] = { plane.closestPoint(capsule.pointA), plane.closestPoint(capsule.pointB) };
						for (byte y = 0; y < 2; ++y) {

							if (manifold.numPoints == MAXIMUM_CONTACT_POINTS) break;

							if (triangles[x].contains(closest[y])) {
								manifold.flag = CollisionFlag::PENETRATING;
								manifold.addContact(-plane.normal, capsule.capsuleLine.closestPoint(closest[y]) + -plane.normal * capsule.radius, closest[y], pairingFunction(y, x));

								DEBUG_RENDERER_ADD(triangles[x], WHITE);
							}
						}
					}
					else {

						manifold.flag = CollisionFlag::PENETRATING;
						Vec3 n = normalise(closest2 - closest1);
						manifold.addContact(n, closest1 + n * capsule.radius, closest2, pairingFunction(2, x));

						DEBUG_RENDERER_ADD(triangles[x], WHITE);
					}
				}
			}

			END_PROFILE;
		}

		void generateContacts(const ConvexHull& convexHull1, const ConvexHull& convexHull2, ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			struct TaskExecutor {

				struct ClosestFaces {
					decimal penetration = decimalMAX;
					uint16 refFace = -1;
					uint16 incidentFace = -1;
					bool separatingAxisFound = false;
				};

				struct ClosestEdges {
					Vec3 normal;
					decimal penetration = decimalMAX;
					uint16 edge1 = -1;
					uint16 edge2 = -1;
					bool separatingAxisFound = false;
				};

				ClosestFaces getClosestFaces(const ConvexHull& convexHull1, const ConvexHull& convexHull2)
				{
					ClosestFaces c;
					for (uint32 x = 0, len = convexHull1.halfEdgeMesh.faces.size(); x < len; ++x) {

						Plane p = convexHull1.getFacePlane(x);
						decimal d = -p.getDistanceFromPlane(convexHull2.getSupportPoint(-p.normal));
				
						if (d < mathEPSILON) {
							c.separatingAxisFound = true;
							return c;
						}
						else {
							if (d < c.penetration) {
								c.penetration = d;
								c.refFace = x;
							}
						}
					}

					uint16 incidentFace = -1;
					decimal least = decimalMAX;
					Plane refPlane = convexHull1.getFacePlane(c.refFace);
					for (uint32 x = 0, len = convexHull2.halfEdgeMesh.faces.size(); x < len; ++x) {

						decimal d = dotProduct(convexHull2.getFaceNormal(x), refPlane.normal) + refPlane.getDistanceFromPlane(convexHull2.getFacePolygon(x).getSupportPoint(-refPlane.normal));
						if (d < least) {
							least = d;
							c.incidentFace = x;
						}
					}

					return c;
				}

				bool edgesBuildMinkowskiFace(const ConvexHull& convexHull1, const ConvexHull& convexHull2, const uint16& e1, const uint16& e2, const Vec3& dir1, const Vec3& dir2)
				{
					decimal adc = dotProduct(convexHull1.getFaceNormal(convexHull1.halfEdgeMesh.edges[e1].faceIndex), dir2);
					decimal bdc = dotProduct(convexHull1.getFaceNormal(convexHull1.halfEdgeMesh.edges[convexHull1.halfEdgeMesh.twinEdge(e1)].faceIndex), dir2);
					decimal cba = dotProduct(-convexHull2.getFaceNormal(convexHull2.halfEdgeMesh.edges[e2].faceIndex), dir1);
					decimal dba = dotProduct(-convexHull2.getFaceNormal(convexHull2.halfEdgeMesh.edges[convexHull2.halfEdgeMesh.twinEdge(e2)].faceIndex), dir1);

					return cba * dba < decimal(0.0) && adc* bdc < decimal(0.0) && cba* bdc > decimal(0.0);
				}

				ClosestEdges getClosestEdges(const ConvexHull& convexHull1, const ConvexHull& convexHull2, const Vec3& center1)
				{
					ClosestEdges c;

					for (uint32 x = 0, len = convexHull1.halfEdgeMesh.edges.size(); x < len; ++x) {

						if (convexHull1.halfEdgeMesh.edges[x].duplicate == true) continue;
						LineSegment e1 = convexHull1.getEdge(x);
						Vec3 dir1 = e1.getDirection();

						for (uint32 y = 0, len = convexHull2.halfEdgeMesh.edges.size(); y < len; ++y) {

							if (convexHull2.halfEdgeMesh.edges[y].duplicate == true) continue;
							Vec3 dir2 = convexHull2.getEdge(y).getDirection();

							if (edgesBuildMinkowskiFace(convexHull1, convexHull2 ,x, y, dir1, dir2)) {

								Vec3 axis = normalise(crossProduct(dir1, dir2));
								if (almostEqual(magnitudeSq(axis), decimal(0.0))) continue;
								if (dotProduct(center1 - e1.pointA, axis) > decimal(0.0)) {
									axis = -axis;
								}
								decimal d = dotProduct(axis, e1.pointA - convexHull2.getSupportPoint(-axis));

								if (d < decimal(0.0)) {
									c.separatingAxisFound = true;
									return c;
								}
								else {
									if (d < c.penetration) {
										c.penetration = d;
										c.edge1 = x;
										c.edge2 = y;
										c.normal = axis;
									}
								}
							}
						}
					}

					return c;
				}

				void generateEdgeContact(const ConvexHull& convexHull1, const ConvexHull& convexHull2, const ClosestEdges& c, ContactManifold& manifold)
				{
					LineSegment e1 = convexHull1.getEdge(c.edge1);
					LineSegment e2 = convexHull2.getEdge(c.edge2);
					Vec3 p1 = e1.closestPoint(e2);
					manifold.addContact(c.normal, p1, e2.closestPoint(p1), 1e20);
				}

				void generateContactsBelowFace(ContactManifold& manifold, const ConvexHull& refConvexHull, const uint32& refFace, const Plane& refPlane, const Polygon& refPolygon, const Polygon& incidentPolygon)
				{
					for (uint32 x = 0, len = incidentPolygon.vertices.size(); x < len; ++x) {

						if (refPlane.getDistanceFromPlane(incidentPolygon.vertices[x]) < decimal(0.0)) {

							bool pass = true;
							uint16 edgeIndex = refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex;
							do {
								if (refConvexHull.getFacePlane(refConvexHull.halfEdgeMesh.edges[refConvexHull.halfEdgeMesh.edges[edgeIndex].twinIndex].faceIndex).getDistanceFromPlane(incidentPolygon.vertices[x]) > decimal(0.0)) {
									pass = false;
									break;
								}
								edgeIndex = refConvexHull.halfEdgeMesh.edges[edgeIndex].nextIndex;
							} while (edgeIndex != refConvexHull.halfEdgeMesh.faces[refFace].edgeIndex);

							if (pass == true) {
								Vec3 p = refPlane.closestPoint(incidentPolygon.vertices[x]);
								if (refPolygon.contains(p)) {
									manifold.addContact(refPlane.normal, p, incidentPolygon.vertices[x], pairingFunction(refFace, x));
								}
							}
						}
					}
				}

				void generateFaceContacts(PhysicsData* physicsData, const ConvexHull& convexHull1, const ConvexHull& convexHull2, const ClosestFaces& c, ContactManifold& manifold, const ColliderIdentifier& identifier1)
				{
					Plane refPlane = convexHull1.getFacePlane(c.refFace);
					Polygon refPolygon = convexHull1.getFacePolygon(c.refFace);
					Polygon incidentPolygon = convexHull2.getFacePolygon(c.incidentFace);

					generateContactsBelowFace(manifold, convexHull1, c.refFace, refPlane, refPolygon, incidentPolygon);
					if (manifold.numPoints == 0) {
						generateContactsBelowFace(manifold, convexHull2, c.incidentFace, convexHull2.getFacePlane(c.incidentFace), incidentPolygon, refPolygon);
					}
			
					HybridArray<LineSegment, 4, byte> edges = incidentPolygon.getEdges();
					uint16 edgeIndex = convexHull1.halfEdgeMesh.faces[c.refFace].edgeIndex;
					do {

						Polygon polygon = convexHull1.getFacePolygon(convexHull1.halfEdgeMesh.edges[convexHull1.halfEdgeMesh.edges[edgeIndex].twinIndex].faceIndex);
						for (uint32 x = 0, len = edges.size(); x < len; ++x) {
					
							Vec3 p = polygon.clip(edges[x]);
							if (isNanVec(p) == false) {
								manifold.addContact(refPlane.normal, refPlane.closestPoint(p), p, pairingFunction(c.refFace, pairingFunction(edgeIndex, x)));
							}
						}

						edgeIndex = convexHull1.halfEdgeMesh.edges[edgeIndex].nextIndex;

					} while (edgeIndex != convexHull1.halfEdgeMesh.faces[c.refFace].edgeIndex);

					ASSERT(manifold.numPoints > 0, "manifold can not be empty!!");

					if (manifold.numPoints > MAXIMUM_CONTACT_POINTS) {
						manifold.enforce4Contacts(physicsData->convexHullColliders[identifier1.colliderIndex].centerOfMass);
					}
				}

				void contactsFromCache(HullVsHullContactCache& cache, PhysicsData* physicsData, const ConvexHull& convexHull1, const ConvexHull& convexHull2, ContactManifold& manifold, const ColliderIdentifier& identifier1)
				{
					ClosestFaces c;
					if (cache.ID1 == identifier1.colliderID) {
						c.refFace = cache.refFace;
						c.incidentFace = cache.incidentFace;
						generateFaceContacts(physicsData, convexHull1, convexHull2, c, manifold, identifier1);
					}
					else {
						c.incidentFace = cache.refFace;
						c.refFace = cache.incidentFace;
						generateFaceContacts(physicsData, convexHull2, convexHull1, c, manifold, identifier1);
					}

					if (manifold.numPoints > 0) {
						manifold.flag = CollisionFlag::PENETRATING;
					}
					else {
						cache.cacheFlags = 0;
					}
				}

				void contactsFromScratch(HullVsHullContactCache& cache, PhysicsData* physicsData, const ConvexHull& convexHull1, const ConvexHull& convexHull2, ContactManifold& manifold, const ColliderIdentifier& identifier1)
				{
					/*
						--------------cache flags----------------
						cache is not empty               - 0b00000001
						minimum axis is a face normal    - 0b00000010
					*/
					cache.cacheFlags = 0;
					cache.cacheFlags |= 0b00000001;

					ClosestFaces cFaces = getClosestFaces(convexHull1, convexHull2);

					if (cFaces.separatingAxisFound == false) {

						ClosestEdges cEdges = getClosestEdges(convexHull1, convexHull2, physicsData->convexHullColliders[identifier1.colliderIndex].centerOfMass);

						if (cEdges.separatingAxisFound == false) {

							if (cFaces.penetration <= cEdges.penetration) {

								generateFaceContacts(physicsData, convexHull1, convexHull2, cFaces, manifold, identifier1);

								cache.cacheFlags |= 0b00000010;
								cache.refFace = cFaces.refFace;
								cache.incidentFace = cFaces.incidentFace;
								cache.ID1 = identifier1.colliderID;
							}
							else {
								generateEdgeContact(convexHull1, convexHull2, cEdges, manifold);
							}

							manifold.flag = CollisionFlag::PENETRATING;
						}
					}

					cache.ID1 = identifier1.colliderID;
				}
			};

			BEGIN_PROFILE("NarrowPhase::ConvexHullVsConvexHull");

			HullVsHullContactCache& cache = this->physicsData->hullVsHullContactCache[this->physicsData->hullVsHullContactCache.insert(manifold.ID)].second;

			Vec3 c1 = this->physicsData->convexHullColliders[identifier1.colliderIndex].centerOfMass;
			Vec3 c2 = this->physicsData->convexHullColliders[identifier2.colliderIndex].centerOfMass;

			TaskExecutor ex;
			if ((cache.cacheFlags & 0b0000000) && (cache.cacheFlags & 0b00000010) && (mathABS(magnitudeSq(c1 - c2) - magnitudeSq(cache.center1 - cache.center2)) < this->physicsData->settings.minimalDispacement)) {
				ex.contactsFromCache(cache, this->physicsData, convexHull1, convexHull2, manifold, identifier1);
			}
			else {
				cache.center1 = c1;
				cache.center2 = c2;
				ex.contactsFromScratch(cache, this->physicsData, convexHull1, convexHull2, manifold, identifier1);
			}

			cache.retention = this->physicsData->settings.framesToRetainCache;

			END_PROFILE;
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

		void generateContacts(const ConvexHull& convexHull, const HybridArray<Triangle, 24, uint16>& triangles, ContactManifold& manifold, const ColliderIdentifier& identifier1)
		{
			BEGIN_PROFILE("NarrowPhase::ConvexHullVstriangles");

			HybridArray<uint16, 8, uint16> registered;
			
			for (uint32 x = 0, len1 = triangles.size(); x < len1; ++x) {

				if (convexHull.intersects(triangles[x])) {

					Plane plane = triangles[x].toPlane();

					for (uint32 y = 0, len2 = convexHull.vertices.size(); y < len2; ++y) {
						
						if (registered.find(y) == false) {

							if (plane.getDistanceFromPlane(convexHull.vertices[y]) < decimal(0.0)) {

								Vec3 closest = plane.closestPoint(convexHull.vertices[y]);

								if (triangles[x].contains(closest)) {
									manifold.flag = CollisionFlag::PENETRATING;
									manifold.addContact(-plane.normal, convexHull.vertices[y], closest, y);
									registered.pushBack(y);

									DEBUG_RENDERER_ADD(triangles[x], WHITE);
								}
							}
						}
					}
				}
			}

			if (manifold.numPoints > MAXIMUM_CONTACT_POINTS) {
				manifold.enforce4Contacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].bound.getCenter());
			}

			END_PROFILE;
		}
	};
}

#endif

