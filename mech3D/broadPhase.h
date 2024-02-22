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

#ifndef BROADPHASE_H
#define BROADPHASE_H

#include"narrowPhase.h"
#include"collision/timeOfImpact.h"
#include"../containers/sortedArray.h"
#include"geometry/ray.h"

namespace mech {

	struct BroadPhase {

		PhysicsData* physicsData = nullptr;
		NarrowPhase* narrowPhase = nullptr;
		ConstraintSolver* constraintSolver = nullptr;

		void (BroadPhase::* discreteCollisionFunctionPointers[20]) (ContactManifold&) = {};
		TOIResult (BroadPhase::* toiFuctionPointers[20]) (const AABB&, const ColliderIdentifier&, const ColliderIdentifier&, const Transform3DRange&, const Transform3DRange&) = {};

		BroadPhase()
		{
			this->discreteCollisionFunctionPointers[0]  = &BroadPhase::convexHullVsConvexHullDiscrete;
			this->discreteCollisionFunctionPointers[4]  = &BroadPhase::convexHullVsSphereDiscrete;
			this->discreteCollisionFunctionPointers[8]  = &BroadPhase::convexHullVsCapsuleDiscrete;
			this->discreteCollisionFunctionPointers[12] = &BroadPhase::convexHullVsTriangleMeshDiscrete;
			this->discreteCollisionFunctionPointers[16] = &BroadPhase::convexHullVsHeightFieldDiscrete;
			this->discreteCollisionFunctionPointers[1]  = &BroadPhase::sphereVsConvexHullDiscrete;
			this->discreteCollisionFunctionPointers[5]  = &BroadPhase::sphereVsSphereDiscrete;
			this->discreteCollisionFunctionPointers[9]  = &BroadPhase::sphereVsCapsuleDiscrete;
			this->discreteCollisionFunctionPointers[13] = &BroadPhase::sphereVsTriangleMeshDiscrete;
			this->discreteCollisionFunctionPointers[17] = &BroadPhase::sphereVsHeightFieldDiscrete;
			this->discreteCollisionFunctionPointers[2]  = &BroadPhase::capsuleVsConvexHullDiscrete;
			this->discreteCollisionFunctionPointers[6]  = &BroadPhase::capsuleVsSphereDiscrete;
			this->discreteCollisionFunctionPointers[10] = &BroadPhase::capsuleVsCapsuleDiscrete;
			this->discreteCollisionFunctionPointers[14] = &BroadPhase::capsuleVsTriangleMeshDiscrete;
			this->discreteCollisionFunctionPointers[18] = &BroadPhase::capsuleVsHeightFieldDiscrete;

			this->toiFuctionPointers[0] = &BroadPhase::convexHullVsConvexHullTOI;
			this->toiFuctionPointers[4] = &BroadPhase::convexHullVsSphereTOI;
			this->toiFuctionPointers[8] = &BroadPhase::convexHullVsCapsuleTOI;
			this->toiFuctionPointers[12] = &BroadPhase::convexHullVsTriangleMeshTOI;
			this->toiFuctionPointers[16] = &BroadPhase::convexHullVsHeightFieldTOI;
			this->toiFuctionPointers[1] = &BroadPhase::sphereVsConvexHullTOI;
			this->toiFuctionPointers[5] = &BroadPhase::sphereVsSphereTOI;
			this->toiFuctionPointers[9] = &BroadPhase::sphereVsCapsuleTOI;
			this->toiFuctionPointers[13] = &BroadPhase::sphereVsTriangleMeshTOI;
			this->toiFuctionPointers[17] = &BroadPhase::sphereVsHeightFieldTOI;
			this->toiFuctionPointers[2] = &BroadPhase::capsuleVsConvexHullTOI;
			this->toiFuctionPointers[6] = &BroadPhase::capsuleVsSphereTOI;
			this->toiFuctionPointers[10] = &BroadPhase::capsuleVsCapsuleTOI;
			this->toiFuctionPointers[14] = &BroadPhase::capsuleVsTriangleMeshTOI;
			this->toiFuctionPointers[18] = &BroadPhase::capsuleVsHeightFieldTOI;
		}

		BroadPhase(const BroadPhase&) = delete;
		BroadPhase& operator=(const BroadPhase&) = delete;

		void detectCollision(ContactManifold& manifold)
		{
			uint32 functionIndex = (uint32)(manifold.colliderID1.type) + ((uint32)(manifold.colliderID2.type) * 4);
			(this->*discreteCollisionFunctionPointers[functionIndex])(manifold);

			if (manifold.flag == CollisionFlag::PENETRATING) {
				this->constraintSolver->add(manifold);
			}

			this->physicsData->finishedCollisions.insert(Pair<uint32, CollisionFlag>(manifold.ID, manifold.flag));
		}

		void discreteCollisionDetection(PhysicsObject& phyObject)
		{
			ColliderIdentifier colliderID1 = phyObject.getIdentifier();

			//update collider status in the octree
			this->physicsData->octree.repositionCollider(physicsData, colliderID1);

			//update collider status in its island if their is one
			if (isAValidIndex(phyObject.islandIndex)) {

				bool removeFromIsland = true;
				for (auto it = this->physicsData->islands[phyObject.islandIndex].begin(), end = this->physicsData->islands[phyObject.islandIndex].end(); it != end; ++it) {
					 
					if (it.data().first == phyObject.colliderID) continue;

					if (phyObject.disabledCollisions.find(it.data().first)) {
						removeFromIsland = false;
						continue;
					}

					uint32 manifoldID = pairingFunction(phyObject.colliderID, it.data().first);
					Pair<uint32, CollisionFlag>* ptr = this->physicsData->finishedCollisions.find(manifoldID);
					if (ptr == nullptr) {

						ContactManifold manifold = ContactManifold(manifoldID, colliderID1, it.data().second);
						this->detectCollision(manifold);

						if (manifold.flag != CollisionFlag::NOTCOLLIDING) {
							removeFromIsland = false;
						}
					}
					else {
						if (ptr->second != CollisionFlag::NOTCOLLIDING) {
							removeFromIsland = false;
						}
					}
				}

				if (removeFromIsland == true) {

					if (this->physicsData->islands[phyObject.islandIndex].size() == 2) {

						uint32 index = phyObject.islandIndex;
						for (auto it = this->physicsData->islands[index].begin(), end = this->physicsData->islands[index].end(); it != end; ++it) {
							this->physicsData->physicsObjects[it.data().second.objectIndex].islandIndex = -1;
							this->physicsData->physicsObjects[it.data().second.objectIndex].rigidBody.setMotionToMax();
						}

						this->physicsData->islands.eraseDataAtIndex(index);
					}
					else {
						this->physicsData->islands[phyObject.islandIndex].eraseData(phyObject.colliderID);

						phyObject.islandIndex = -1;
						phyObject.rigidBody.setMotionToMax();
					}
				}
			}

			//look for collisions
			for (byte x = 0, len = phyObject.nodesIntersected.size(); x < len; ++x) {

				Octree::Node& node = this->physicsData->octree.nodes[phyObject.nodesIntersected[x]];

				if (node.evaluate(physicsData, colliderID1)) {

					for (auto it = node.staticColliders.begin(), end = node.staticColliders.end(); it != end; ++it) {
						
						uint32 manifoldID = pairingFunction(phyObject.colliderID, it.data().first);
						if (this->physicsData->finishedCollisions.find(manifoldID) == false) {
							ContactManifold manifold = ContactManifold(manifoldID, colliderID1, it.data().second);
							this->detectCollision(manifold);
						}
					}

					for (auto it = node.dynamicColliders.begin(), end = node.dynamicColliders.end(); it != end; ++it) {

						if (phyObject.disabledCollisions.find(it.data().first)) continue;

						uint32 manifoldID = pairingFunction(phyObject.colliderID, it.data().first);
						if (this->physicsData->finishedCollisions.find(manifoldID) == false) {

							ContactManifold manifold = ContactManifold(manifoldID, colliderID1, it.data().second);
							this->detectCollision(manifold);

							if (manifold.flag != CollisionFlag::NOTCOLLIDING) {
								phyObject.addToIsland(physicsData, it.data().second.objectIndex);
							}
						}
					}
				}
				else {
					node.dynamicColliders.eraseData(colliderID1.ID);

					if (node.dynamicColliders.empty() && (node.staticColliders.empty() || (node.staticColliders.size() == 1 && node.staticColliders.find(this->physicsData->heightFieldCollider.identifier.ID)))) {
						this->physicsData->octree.terminate(phyObject.nodesIntersected[x]);
					}

					phyObject.nodesIntersected.eraseDataAtIndex(x);
					--x;
					--len;
				}
			}

			if (phyObject.nodesIntersected.empty()) {

				if (isAValidIndex(phyObject.islandIndex)) {
					this->physicsData->islands[phyObject.islandIndex].eraseData(phyObject.colliderID);
				}

				this->physicsData->physicsObjects.eraseDataAtIndex(phyObject.objectIndex);
			}
		}

		void continousCollisionDetection(PhysicsObject& phyObject)
		{
			struct CCD {

				void registerIntersectingNodes(const uint32& nodeIndex, PhysicsData* physicsData, const AABB& aabbCast, HybridArray<uint16, 8, uint16>& nodes)
				{
					for (byte x = 0; x < 8; ++x) {

						if (isAValidIndex(physicsData->octree.nodes[nodeIndex].children[x].first)) {

							uint32 childIndex = physicsData->octree.nodes[nodeIndex].children[x].second;

							if (physicsData->octree.nodes[childIndex].bound.intersects(aabbCast)) {

								if (physicsData->octree.nodes[childIndex].children.empty()) {
									nodes.pushBack(childIndex);
								}
								else {
									registerIntersectingNodes(childIndex, physicsData, aabbCast, nodes);
								}

								if (physicsData->octree.nodes[childIndex].bound.contains(aabbCast)) {
									break;
								}
							}
						}
					}
				}
			};

			ColliderIdentifier colliderID1 = phyObject.getIdentifier();
			
			const AABB& currentAABB = this->physicsData->getColliderAABB(colliderID1);
			AABB prevAABB = currentAABB.transformed(getInverse(phyObject.rigidBody.transform) * phyObject.rigidBody.prevTransform);
			AABB aabbCast = AABB(minVec(prevAABB.min, currentAABB.min), maxVec(prevAABB.max, currentAABB.max));

			SortedArray<decimal, uint16> hitTimes;
			HybridArray<uint16, 8, uint16> intersectingNodes;

			CCD ccd;
			ccd.registerIntersectingNodes(this->physicsData->octree.parentNode, physicsData, aabbCast, intersectingNodes);

			Transform3DRange tA = Transform3DRange(phyObject.rigidBody.prevTransform, phyObject.rigidBody.transform);

			for (auto it1 = intersectingNodes.begin(), end1 = intersectingNodes.end(); it1 != end1; ++it1) {

				for (auto it2 = physicsData->octree.nodes[it1.data()].staticColliders.begin(), end2 = physicsData->octree.nodes[it1.data()].staticColliders.end(); it2 != end2; ++it2) {

					uint32 functionIndex = (uint32)(colliderID1.type) + ((uint32)(it2.data().second.type) * 4);
					TOIResult r = (this->*toiFuctionPointers[functionIndex])(aabbCast, colliderID1, it2.data().second, tA, Transform3DRange());
			
					if (r.state != TOIState::separated) {
						hitTimes.insert(r.t);
					}
				}

				for (auto it2 = physicsData->octree.nodes[it1.data()].dynamicColliders.begin(), end2 = physicsData->octree.nodes[it1.data()].dynamicColliders.end(); it2 != end2; ++it2) {

					if (phyObject.disabledCollisions.find(it2.data().first)) continue;

					Transform3DRange tB = Transform3DRange(physicsData->physicsObjects[it2.data().second.objectIndex].rigidBody.prevTransform, physicsData->physicsObjects[it2.data().second.objectIndex].rigidBody.transform);
					
					uint32 functionIndex = (uint32)(colliderID1.type) + ((uint32)(it2.data().second.type) * 4);
					TOIResult r = (this->*toiFuctionPointers[functionIndex])(aabbCast, colliderID1, it2.data().second, tA, tB);

					if (r.state != TOIState::separated) {
						hitTimes.insert(r.t);
					}
				}
			}

			if (hitTimes.empty() == false) {
				
				phyObject.rigidBody.subStep(physicsData, hitTimes.minimum() + this->physicsData->configurations.timeOfImpactBias);
				
				for (byte x = 0, len = phyObject.nodesIntersected.size(); x < len; ++x) {
					this->physicsData->octree.nodes[phyObject.nodesIntersected[x]].dynamicColliders.eraseData(phyObject.colliderID);
				}
				phyObject.nodesIntersected.clear();

				this->physicsData->octree.add(physicsData, colliderID1);
			}

			this->discreteCollisionDetection(phyObject);
		}

		void convexHullVsConvexHullDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void convexHullVsSphereDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void convexHullVsCapsuleDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void convexHullVsTriangleMeshDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].collider.intersects(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		void convexHullVsHeightFieldDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->heightFieldCollider.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		void sphereVsConvexHullDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsSphereDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsCapsuleDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsTriangleMeshDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].collider.intersects(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		void sphereVsHeightFieldDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->heightFieldCollider.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		void capsuleVsConvexHullDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->convexHullColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsSphereDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->sphereColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsCapsuleDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].material;
			if (this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material2 = this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].collider, this->physicsData->capsuleColliders[manifold.colliderID2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsTriangleMeshDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].collider.intersects(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->triangleMeshColliders[manifold.colliderID2.colliderIndex].material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		void capsuleVsHeightFieldDiscrete(ContactManifold& manifold)
		{
			manifold.material1 = this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].material;
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].bound, triangles)) {
				manifold.material2 = this->physicsData->heightFieldCollider.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[manifold.colliderID1.colliderIndex].collider, triangles, manifold);
			}
		}

		TOIResult triangleTOI(const HybridArray<Triangle, 24, uint16>& triangles, const ColliderIdentifier& id)
		{
			Vec3 origin = this->physicsData->physicsObjects[id.objectIndex].rigidBody.prevTransform.position;
			Vec3 direction = this->physicsData->physicsObjects[id.objectIndex].rigidBody.transform.position - this->physicsData->physicsObjects[id.objectIndex].rigidBody.prevTransform.position;
			Ray ray = Ray(origin, direction);

			SortedArray<decimal, byte> toiResults;
			for (uint16 x = 0, len = triangles.size(); x < len; ++x) {
				decimal t = ray.rayCastTime(triangles[x]);
				if (t >= decimal(0.0)) {
					toiResults.insert(t);
				}
			}

			TOIResult r;
			if (toiResults.empty() == false) {
				r.t = toiResults.minimum();
				r.state = TOIState::touching;
			}

			return r;
		}

		TOIResult convexHullVsConvexHullTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->convexHullColliders[id2.colliderIndex].bound)) {
				return timeOfImpact(this->physicsData->convexHullColliders[id1.colliderIndex].collider, this->physicsData->convexHullColliders[id2.colliderIndex].collider, t1, t2);
			}
			return TOIResult();
		}

		TOIResult convexHullVsSphereTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->convexHullColliders[id2.colliderIndex].bound)) {
				return timeOfImpact(this->physicsData->convexHullColliders[id1.colliderIndex].collider, this->physicsData->sphereColliders[id2.colliderIndex].collider, t1, t2);
				//return timeOfImpact(this->physicsData->convexHullColliders[id1.colliderIndex].collider, this->physicsData->sphereColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult convexHullVsCapsuleTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->convexHullColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->convexHullColliders[id1.colliderIndex].collider, this->physicsData->capsuleColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->convexHullColliders[id1.colliderIndex].collider, this->physicsData->capsuleColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult convexHullVsTriangleMeshTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[id2.colliderIndex].collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}

		TOIResult convexHullVsHeightFieldTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}

		TOIResult sphereVsConvexHullTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->convexHullColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].collider, this->physicsData->convexHullColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].bound, this->physicsData->convexHullColliders[id2.colliderIndex].collider, t1, t2);
			}
			return TOIResult();
		}

		TOIResult sphereVsSphereTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->sphereColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].collider, this->physicsData->sphereColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].bound, this->physicsData->sphereColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult sphereVsCapsuleTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->capsuleColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].collider, this->physicsData->capsuleColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->sphereColliders[id1.colliderIndex].bound, this->physicsData->capsuleColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult sphereVsTriangleMeshTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[id2.colliderIndex].collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}

		TOIResult sphereVsHeightFieldTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}

		TOIResult capsuleVsConvexHullTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->convexHullColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].collider, this->physicsData->convexHullColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].bound, this->physicsData->convexHullColliders[id2.colliderIndex].collider, t1, t2);
			}
			return TOIResult();
		}

		TOIResult capsuleVsSphereTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->sphereColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].collider, this->physicsData->sphereColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].bound, this->physicsData->sphereColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult capsuleVsCapsuleTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			if (aabbCast.intersects(this->physicsData->capsuleColliders[id2.colliderIndex].bound)) {
				//return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].collider, this->physicsData->capsuleColliders[id2.colliderIndex].collider, t1, t2);
				return timeOfImpact(this->physicsData->capsuleColliders[id1.colliderIndex].bound, this->physicsData->capsuleColliders[id2.colliderIndex].bound, t1, t2);
			}
			return TOIResult();
		}

		TOIResult capsuleVsTriangleMeshTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->triangleMeshColliders[id2.colliderIndex].collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}

		TOIResult capsuleVsHeightFieldTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			HybridArray<Triangle, 24, uint16> triangles;
			if (this->physicsData->heightFieldCollider.collider.intersects(aabbCast, triangles)) {
				return this->triangleTOI(triangles, id1);
			}

			return TOIResult();
		}
	};
}

#endif

