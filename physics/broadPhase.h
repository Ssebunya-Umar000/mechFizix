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

#ifndef BROADPHASE_H
#define BROADPHASE_H

#include"narrowPhase.h"
#include"timeOfImpact.h"

namespace mech {

#define CONTINOUS_COLLISION_THRESHOLD decimal(1.35)

	struct BroadPhase {

		PhysicsData* physicsData = nullptr;
		NarrowPhase* narrowPhase = nullptr;
		TimeOfImpact* timeOfImpact = nullptr;
		ConstraintSolver* constraintSolver = nullptr;

		void (BroadPhase::* manifoldPtrs[30]) (ContactManifold&, const ColliderIdentifier&, const ColliderIdentifier&) = {};
		TOIResult(BroadPhase::* toiPtrs[30]) (const AABB&, const ColliderIdentifier&, const ColliderIdentifier&, const Transform3DRange&, const Transform3DRange&) = {};
		const decimal& (BroadPhase::* radiusPtrs[4]) (const ColliderIdentifier&) = {};

		BroadPhase(const BroadPhase&) = delete;
		BroadPhase& operator=(const BroadPhase&) = delete;

		BroadPhase()
		{
			this->manifoldPtrs[0]  = &BroadPhase::convexHullVsConvexHullManifold;
			this->manifoldPtrs[5]  = &BroadPhase::convexHullVsSphereManifold;
			this->manifoldPtrs[10] = &BroadPhase::convexHullVsCapsuleManifold;
			this->manifoldPtrs[15] = &BroadPhase::otherVsCompoundManifold;
			this->manifoldPtrs[20] = &BroadPhase::convexHullVsTriangleMeshManifold;
			this->manifoldPtrs[25] = &BroadPhase::convexHullVsHeightFieldManifold;
			this->manifoldPtrs[1]  = &BroadPhase::sphereVsConvexHullManifold;
			this->manifoldPtrs[6]  = &BroadPhase::sphereVsSphereManifold;
			this->manifoldPtrs[11] = &BroadPhase::sphereVsCapsuleManifold;
			this->manifoldPtrs[16] = &BroadPhase::otherVsCompoundManifold;
			this->manifoldPtrs[21] = &BroadPhase::sphereVsTriangleMeshManifold;
			this->manifoldPtrs[26] = &BroadPhase::sphereVsHeightFieldManifold;
			this->manifoldPtrs[2]  = &BroadPhase::capsuleVsConvexHullManifold;
			this->manifoldPtrs[7]  = &BroadPhase::capsuleVsSphereManifold;
			this->manifoldPtrs[12] = &BroadPhase::capsuleVsCapsuleManifold;
			this->manifoldPtrs[17] = &BroadPhase::otherVsCompoundManifold;
			this->manifoldPtrs[22] = &BroadPhase::capsuleVsTriangleMeshManifold;
			this->manifoldPtrs[27] = &BroadPhase::capsuleVsHeightFieldManifold;
			this->manifoldPtrs[3]  = &BroadPhase::compoundVsOtherManifold;
			this->manifoldPtrs[8]  = &BroadPhase::compoundVsOtherManifold;
			this->manifoldPtrs[13] = &BroadPhase::compoundVsOtherManifold;
			this->manifoldPtrs[18] = &BroadPhase::compoundVsOtherManifold;
			this->manifoldPtrs[23] = &BroadPhase::compoundVsOtherManifold;
			this->manifoldPtrs[28] = &BroadPhase::compoundVsOtherManifold;

			this->toiPtrs[0] = &BroadPhase::commonTOI;
			this->toiPtrs[5] = &BroadPhase::commonTOI;
			this->toiPtrs[10] = &BroadPhase::commonTOI;
			this->toiPtrs[15] = &BroadPhase::otherVsCompoundTOI;
			this->toiPtrs[20] = &BroadPhase::otherVsTriangleMeshTOI;
			this->toiPtrs[25] = &BroadPhase::otherVsHeightFieldTOI;
			this->toiPtrs[1] = &BroadPhase::commonTOI;
			this->toiPtrs[6] = &BroadPhase::commonTOI;
			this->toiPtrs[11] = &BroadPhase::commonTOI;
			this->toiPtrs[16] = &BroadPhase::otherVsCompoundTOI;
			this->toiPtrs[21] = &BroadPhase::otherVsTriangleMeshTOI;
			this->toiPtrs[26] = &BroadPhase::otherVsHeightFieldTOI;
			this->toiPtrs[2] = &BroadPhase::commonTOI;
			this->toiPtrs[7] = &BroadPhase::commonTOI;
			this->toiPtrs[12] = &BroadPhase::commonTOI;
			this->toiPtrs[17] = &BroadPhase::otherVsCompoundTOI;
			this->toiPtrs[22] = &BroadPhase::otherVsTriangleMeshTOI;
			this->toiPtrs[27] = &BroadPhase::otherVsHeightFieldTOI;
			this->toiPtrs[3]  = &BroadPhase::compoundVsOtherTOI;
			this->toiPtrs[8]  = &BroadPhase::compoundVsOtherTOI;
			this->toiPtrs[13] = &BroadPhase::compoundVsOtherTOI;
			this->toiPtrs[18] = &BroadPhase::compoundVsOtherTOI;
			this->toiPtrs[23] = &BroadPhase::compoundVsOtherTOI;
			this->toiPtrs[28] = &BroadPhase::compoundVsOtherTOI;

			this->radiusPtrs[0] = &BroadPhase::getRadiusConvexHull;
			this->radiusPtrs[1] = &BroadPhase::getRadiusSphere;
			this->radiusPtrs[2] = &BroadPhase::getRadiusCapsule;
			this->radiusPtrs[3] = &BroadPhase::getRadiusCompound;
		}

		void handle(PhysicsObject& phyObject, const decimal& deltaTime)
		{
			const ColliderIdentifier& identifier1 = this->physicsData->colliderIdentifiers[phyObject.rigidBody.colliderID];

			if ((magnitudeSq(phyObject.rigidBody.getDisplacement()) / (this->*radiusPtrs[(uint32)(identifier1.type)])(identifier1)) >= CONTINOUS_COLLISION_THRESHOLD) {
				this->continousCollisionDetection(phyObject, identifier1, deltaTime);
			}
			else {

				this->physicsData->octree.updateEntityDiscrete(identifier1.colliderID, this->physicsData->getColliderAABB(identifier1.colliderID), this->physicsData->physicsObjects[identifier1.objectIndex].nodesIntersected);
			
				this->updateIsland(phyObject, identifier1);
				
				this->discreteCollisionDetection(phyObject, identifier1);
			}
		}

		void discreteCollisionDetection(PhysicsObject& phyObject, const ColliderIdentifier& identifier1)
		{
			BEGIN_PROFILE("BroadPhase::discreteCollisionDetection");

			for (byte x = 0, len = phyObject.nodesIntersected.size(); x < len; ++x) {

				Octree::Node& node = this->physicsData->octree.nodes[phyObject.nodesIntersected[x]];
				for (auto it = node.entities.begin(), end = node.entities.end(); it != end; ++it) {

					if (phyObject.disabledCollisions.find(it.data())) continue;

					uint32 manifoldID = pairingFunction(identifier1.colliderID, it.data());
					if (this->physicsData->finishedCollisions.find(manifoldID) == false) {

						const ColliderIdentifier& id2 = this->physicsData->colliderIdentifiers[it.data()];

						ContactManifold manifold = ContactManifold(manifoldID);
						this->detectCollision(manifold, identifier1, id2);

						if (id2.state == ColliderMotionState::dynamic && manifold.flag != CollisionFlag::NOTCOLLIDING) {
							phyObject.addToIsland(physicsData, id2.colliderID);
						}
					}
				}
			}

			if (phyObject.nodesIntersected.empty()) {

				if (isAValidIndex(phyObject.islandIndex)) {
					this->physicsData->islands[phyObject.islandIndex].eraseData(identifier1.colliderID);
				}

				this->physicsData->erase(identifier1.colliderID);
			}

			END_PROFILE;
		}

		void continousCollisionDetection(PhysicsObject& phyObject, const ColliderIdentifier& identifier1, const decimal& deltaTime)
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

			BEGIN_PROFILE("BroadPhase::continousCollisionDetection");

			const AABB& currentAABB = this->physicsData->getColliderAABB(identifier1.colliderID);
			AABB prevAABB = currentAABB.transformed(getInverse(phyObject.rigidBody.transform) * phyObject.rigidBody.prevTransform);
			AABB aabbCast = AABB(minVec(prevAABB.min, currentAABB.min), maxVec(prevAABB.max, currentAABB.max));

			HybridArray<uint16, 8, uint16> intersectingNodes;

			CCD ccd;
			ccd.registerIntersectingNodes(0, physicsData, aabbCast, intersectingNodes);

			Transform3DRange tA = Transform3DRange(phyObject.rigidBody.prevTransform, phyObject.rigidBody.transform);

			TOIResult hit;
			HashTable<uint32, uint32> finished;
			for (auto it1 = intersectingNodes.begin(), end1 = intersectingNodes.end(); it1 != end1; ++it1) {

				for (auto it2 = physicsData->octree.nodes[it1.data()].entities.begin(), end2 = physicsData->octree.nodes[it1.data()].entities.end(); it2 != end2; ++it2) {

					if (finished.find(it2.data()) || phyObject.disabledCollisions.find(it2.data())) continue;

					const ColliderIdentifier& id2 = this->physicsData->colliderIdentifiers[it2.data()];

					Transform3DRange tB;
					if (id2.state == ColliderMotionState::dynamic) {
						tB = Transform3DRange(physicsData->physicsObjects[id2.objectIndex].rigidBody.prevTransform, physicsData->physicsObjects[id2.objectIndex].rigidBody.transform);
					}
					
					uint32 functionIndex = (uint32)(identifier1.type) + ((uint32)(id2.type) * 5);
					TOIResult r = (this->*toiPtrs[functionIndex])(aabbCast, identifier1, id2, tA, tB);
					if (r.state == TOIState::overlaping && r.t < hit.t) {
						hit = r;
					}

					finished.insert(it2.data());
				}
			}

			if (hit.state == TOIState::overlaping) {
				phyObject.rigidBody.subStep(physicsData, mathMIN(hit.t + (decimal(5.0) / (magnitudeSq(phyObject.rigidBody.linearVelocity) * deltaTime)), decimal(1.0)));
			}

			this->physicsData->octree.updateEntityContinous(identifier1.colliderID, currentAABB, this->physicsData->physicsObjects[identifier1.objectIndex].nodesIntersected);

			this->discreteCollisionDetection(phyObject, identifier1);

			END_PROFILE;
		}

		void detectCollision(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			uint32 functionIndex = (uint32)(identifier1.type) + ((uint32)(identifier2.type) * 5);
			(this->*manifoldPtrs[functionIndex])(manifold, identifier1, identifier2);

			if (manifold.flag == CollisionFlag::PENETRATING) {
				this->constraintSolver->add(manifold, identifier1.objectIndex, identifier2.objectIndex);
			}

			this->physicsData->finishedCollisions.insert(Pair<uint32, CollisionFlag>(manifold.ID, manifold.flag));
		}

		void updateIsland(PhysicsObject& phyObject, const ColliderIdentifier& identifier1)
		{
			BEGIN_PROFILE("BroadPhase::updateIsland");

			if (isAValidIndex(phyObject.islandIndex)) {

				bool removeFromIsland = true;
				for (auto it = this->physicsData->islands[phyObject.islandIndex].begin(), end = this->physicsData->islands[phyObject.islandIndex].end(); it != end; ++it) {

					if (phyObject.disabledCollisions.find(it.data())) continue;

					uint32 manifoldID = pairingFunction(identifier1.colliderID, it.data());
					Pair<uint32, CollisionFlag>* ptr = this->physicsData->finishedCollisions.find(manifoldID);
					if (ptr == nullptr) {

						ContactManifold manifold = ContactManifold(manifoldID);
						this->detectCollision(manifold, identifier1, this->physicsData->colliderIdentifiers[it.data()]);

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
							this->physicsData->physicsObjects[this->physicsData->colliderIdentifiers[it.data()].objectIndex].islandIndex = -1;
							this->physicsData->physicsObjects[this->physicsData->colliderIdentifiers[it.data()].objectIndex].rigidBody.setMotionToMax();
						}

						this->physicsData->islands.eraseDataAtIndex(index);
					}
					else {
						this->physicsData->islands[phyObject.islandIndex].eraseData(identifier1.colliderID);

						phyObject.islandIndex = -1;
						phyObject.rigidBody.setMotionToMax();
					}
				}
			}

			END_PROFILE;
		}

		const decimal& getRadiusConvexHull(const ColliderIdentifier& identifier) { return this->physicsData->convexHullColliders[identifier.colliderIndex].convexRadius; }
		const decimal& getRadiusSphere(const ColliderIdentifier& identifier) { return this->physicsData->sphereColliders[identifier.colliderIndex].collider.radius; }
		const decimal& getRadiusCapsule(const ColliderIdentifier& identifier) { return this->physicsData->capsuleColliders[identifier.colliderIndex].convexRadius; }
		const decimal& getRadiusCompound(const ColliderIdentifier& identifier) { return this->physicsData->compoundColliders[identifier.colliderIndex].convexRadius; }

		void convexHullVsConvexHullManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->convexHullColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider, this->physicsData->convexHullColliders[identifier2.colliderIndex].collider, manifold, identifier1, identifier2);
			}
		}

		void convexHullVsSphereManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->convexHullColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider, this->physicsData->sphereColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void convexHullVsCapsuleManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->convexHullColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider, this->physicsData->capsuleColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void convexHullVsTriangleMeshManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.intersects(this->physicsData->convexHullColliders[identifier1.colliderIndex].bound)) {
			
				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.getTrianglesOverlapped(this->physicsData->convexHullColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider, triangles, manifold, identifier1);
			}
		}

		void convexHullVsHeightFieldManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->convexHullColliders[identifier1.colliderIndex].bound)) {

				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->heightFieldCollider.collider.getTrianglesOverlapped(this->physicsData->convexHullColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider, triangles, manifold, identifier1);
			}
		}

		void sphereVsConvexHullManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->sphereColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[identifier1.colliderIndex].collider, this->physicsData->convexHullColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsSphereManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->sphereColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[identifier1.colliderIndex].collider, this->physicsData->sphereColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsCapsuleManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->sphereColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[identifier1.colliderIndex].collider, this->physicsData->capsuleColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void sphereVsTriangleMeshManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.intersects(this->physicsData->sphereColliders[identifier1.colliderIndex].bound)) {
			
				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.getTrianglesOverlapped(this->physicsData->sphereColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[identifier1.colliderIndex].collider, triangles, manifold);
			}
		}

		void sphereVsHeightFieldManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->sphereColliders[identifier1.colliderIndex].bound)) {

				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->heightFieldCollider.collider.getTrianglesOverlapped(this->physicsData->sphereColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->sphereColliders[identifier1.colliderIndex].collider, triangles, manifold);
			}
		}

		void capsuleVsConvexHullManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->capsuleColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->convexHullColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[identifier1.colliderIndex].collider, this->physicsData->convexHullColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsSphereManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->capsuleColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->sphereColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[identifier1.colliderIndex].collider, this->physicsData->sphereColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsCapsuleManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->capsuleColliders[identifier1.colliderIndex].bound.intersects(this->physicsData->capsuleColliders[identifier2.colliderIndex].bound)) {
				manifold.flag = CollisionFlag::PROXIMAL;
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[identifier1.colliderIndex].collider, this->physicsData->capsuleColliders[identifier2.colliderIndex].collider, manifold);
			}
		}

		void capsuleVsTriangleMeshManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.intersects(this->physicsData->capsuleColliders[identifier1.colliderIndex].bound)) {
			
				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->triangleMeshColliders[identifier2.colliderIndex].collider.getTrianglesOverlapped(this->physicsData->capsuleColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[identifier1.colliderIndex].collider, triangles, manifold);
			}
		}

		void capsuleVsHeightFieldManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			if (this->physicsData->heightFieldCollider.collider.intersects(this->physicsData->capsuleColliders[identifier1.colliderIndex].bound)) {

				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->heightFieldCollider.collider.getTrianglesOverlapped(this->physicsData->capsuleColliders[identifier1.colliderIndex].bound, triangles);
				manifold.material1 = identifier1.material;
				manifold.material2 = identifier2.material;
				this->narrowPhase->generateContacts(this->physicsData->capsuleColliders[identifier1.colliderIndex].collider, triangles, manifold);
			}
		}

		void compoundVsOtherManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			for (auto it = this->physicsData->compoundColliders[identifier1.colliderIndex].components.begin(), end = this->physicsData->compoundColliders[identifier1.colliderIndex].components.end(); it != end; ++it) {

				uint32 functionIndex = (uint32)(this->physicsData->colliderIdentifiers[it.data()].type) + ((uint32)(identifier2.type) * 5);

				ContactManifold newManifold(pairingFunction(this->physicsData->colliderIdentifiers[it.data()].colliderID, identifier2.colliderID));
				(this->*manifoldPtrs[functionIndex])(newManifold, this->physicsData->colliderIdentifiers[it.data()], identifier2);

				if (newManifold.flag == CollisionFlag::PENETRATING) {
					this->constraintSolver->add(newManifold, this->physicsData->colliderIdentifiers[it.data()].objectIndex, identifier2.objectIndex);
				}
			}
		}

		void otherVsCompoundManifold(ContactManifold& manifold, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2)
		{
			for (auto it = this->physicsData->compoundColliders[identifier2.colliderIndex].components.begin(), end = this->physicsData->compoundColliders[identifier2.colliderIndex].components.end(); it != end; ++it) {

				uint32 functionIndex = (uint32)(identifier1.type) + ((uint32)(this->physicsData->colliderIdentifiers[it.data()].type) * 5);

				ContactManifold newManifold(pairingFunction(identifier1.colliderID, this->physicsData->colliderIdentifiers[it.data()].colliderID));
				(this->*manifoldPtrs[functionIndex])(newManifold, identifier1, this->physicsData->colliderIdentifiers[it.data()]);

				if (newManifold.flag == CollisionFlag::PENETRATING) {
					this->constraintSolver->add(newManifold, identifier1.objectIndex, this->physicsData->colliderIdentifiers[it.data()].objectIndex);
				}
			}
		}

		TOIResult commonTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			return this->timeOfImpact->toi(aabbCast, id1, id2, t1, t2);
		}

		TOIResult otherVsTriangleMeshTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			TOIResult toiResult;
			if (this->physicsData->triangleMeshColliders[id2.colliderIndex].collider.intersects(aabbCast)) {

				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->triangleMeshColliders[id2.colliderIndex].collider.getTrianglesOverlapped(aabbCast, triangles);
				
				for (uint16 x = 0, len = triangles.size(); x < len; ++x) {

					TOIResult r = this->timeOfImpact->toi(id1, triangles[x], t1);
					if (r.state == TOIState::overlaping && r.t < toiResult.t) {
						toiResult = r;
					}
				}
			}

			return toiResult;
		}

		TOIResult otherVsHeightFieldTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			TOIResult toiResult;
			if (this->physicsData->heightFieldCollider.collider.intersects(aabbCast)) {

				HybridArray<Triangle, 24, uint16> triangles;
				this->physicsData->heightFieldCollider.collider.getTrianglesOverlapped(aabbCast, triangles);

				for (uint16 x = 0, len = triangles.size(); x < len; ++x) {

					TOIResult r = this->timeOfImpact->toi(id1, triangles[x], t1);
					if (r.state == TOIState::overlaping && r.t < toiResult.t) {
						r.t += 0.001;
						toiResult = r;
					}
				}
			}

			return toiResult;
		}

		TOIResult compoundVsOtherTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			TOIResult toiResult;
			for (auto it = this->physicsData->compoundColliders[id2.colliderIndex].components.begin(), end = this->physicsData->compoundColliders[id2.colliderIndex].components.end(); it != end; ++it) {

				uint32 functionIndex = (uint32)(this->physicsData->colliderIdentifiers[it.data()].type) + ((uint32)(id2.type) * 5);
				TOIResult r = (this->*toiPtrs[functionIndex])(aabbCast, this->physicsData->colliderIdentifiers[it.data()], id2, t1, t2);
				if (r.state == TOIState::overlaping && r.t < toiResult.t) {
					toiResult = r;
				}
			}

			return toiResult;
		}

		TOIResult otherVsCompoundTOI(const AABB& aabbCast, const ColliderIdentifier& id1, const ColliderIdentifier& id2, const Transform3DRange& t1, const Transform3DRange& t2)
		{
			TOIResult toiResult;
			for (auto it = this->physicsData->compoundColliders[id2.colliderIndex].components.begin(), end = this->physicsData->compoundColliders[id2.colliderIndex].components.end(); it != end; ++it) {

				uint32 functionIndex = (uint32)(id1.type) + ((uint32)(this->physicsData->colliderIdentifiers[it.data()].type) * 5);
				TOIResult r = (this->*toiPtrs[functionIndex])(aabbCast, id1, this->physicsData->colliderIdentifiers[it.data()], t1, t2);
				if (r.state == TOIState::overlaping && r.t < toiResult.t) {
					toiResult = r;
				}
			}

			return toiResult;
		}
	};
}

#endif

