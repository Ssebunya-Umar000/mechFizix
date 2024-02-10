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

namespace mech {

	struct BroadPhase {

		NarrowPhase narrowPhase;

		BroadPhase() {}
		BroadPhase(const BroadPhase&) = delete;
		BroadPhase& operator=(const BroadPhase&) = delete;

		void discreteCollisionDetection()
		{
			struct DCD {

				NarrowPhase& narrowPhase;

				DCD(NarrowPhase& n) : narrowPhase(n) {}

				void updateIslands()
				{
					PhysicsData* physicsData = this->narrowPhase.physicsData;

					for (auto it1 = physicsData->islands.begin(), end1 = physicsData->islands.end(); it1 != end1;) {

						auto temp1 = it1;
						++it1;

						AVLTree<Pair<uint32, CollisionFlag>, uint32> finished;

						for (auto it2 = temp1.data().begin(), end2 = temp1.data().end(); it2 != end2;) {

							auto temp2 = it2;
							++it2;

							PhysicsObject& phyObject = physicsData->physicsObjects[temp2.data().second.objectIndex];

							if (phyObject.rigidBody.isActive() == false) continue;

							auto it3 = temp2;
							++it3;

							bool removeFromIsland = true;
							if (it3 == end2) {
								removeFromIsland = false;
							}

							while (it3 != end2) {

								auto temp3 = it3;
								++it3;

								if (phyObject.disabledCollisions.find(temp3.data().first)) {
									removeFromIsland = false;
								}
								else {

									uint32 id = pairingFunction(phyObject.colliderID, temp3.data().second.ID);
									Pair<uint32, CollisionFlag>* ptr = finished.find(id);
									if (ptr == nullptr) {

										CollisionFlag flag = this->dispatch(id, phyObject.getIdentifier(), temp3.data().second);
										if (flag != CollisionFlag::NOTCOLLIDING) {
											removeFromIsland = false;
										}

										finished.insert(Pair<uint32, CollisionFlag>(id, flag));
									}
									else {
										if (ptr->second != CollisionFlag::NOTCOLLIDING) {
											removeFromIsland = false;
										}
									}
								}
							}

							if (removeFromIsland == true) {

								if (temp1.data().size() == 2) {

									for (auto it3 = temp1.data().begin(), end3 = temp1.data().end(); it3 != end3; ++it3) {
										physicsData->physicsObjects[it3.data().second.objectIndex].rigidBody.islandIndex = -1;
										physicsData->physicsObjects[it3.data().second.objectIndex].rigidBody.motion = physicsData->maxMotion;
									}

									physicsData->islands.eraseDataAtIndex(temp1.index());

									break;
								}
								else {
									temp1.data().eraseData(phyObject.colliderID);

									phyObject.rigidBody.islandIndex = -1;
									phyObject.rigidBody.motion = physicsData->maxMotion;
								}
							}
						}
					}
				}

				void generateCollisionPairs()
				{
					PhysicsData* physicsData = this->narrowPhase.physicsData;

					for (auto it1 = physicsData->activeOctreeNodes.begin(), end1 = physicsData->activeOctreeNodes.end(); it1 != end1;) {

						auto temp1 = it1;
						++it1;

						if (temp1.isValid() == false) continue;

						bool firstIteration = true;
						Octree::Node & node = physicsData->octree.nodes[temp1.data()];
						for (auto it2 = node.dynamicColliders.begin(), end2 = node.dynamicColliders.end(); it2 != end2;) {

							auto temp2 = it2;
							++it2;

							PhysicsObject& phyObject = physicsData->physicsObjects[temp2.data().second.objectIndex];

							if (phyObject.rigidBody.isActive() == false) continue;

							if (firstIteration == true) {
								if (node.intersects(physicsData, phyObject.getIdentifier()) == false) {
									phyObject.nodesIntersected.eraseData(temp1.data());
									node.dynamicColliders.eraseData(phyObject.colliderID);
									continue;
								}
							}

							for (auto it3 = node.staticColliders.begin(), end3 = node.staticColliders.end(); it3 != end3; ++it3) {
								uint32 id = pairingFunction(phyObject.colliderID, it3.data().second.ID);
								if (physicsData->manifoldIDs.find(id)) continue;
								this->dispatch(id, phyObject.getIdentifier(), it3.data().second);
							}

							auto it3 = temp2;
							++it3;
							while (it3 != end2) {

								auto temp3 = it3;
								++it3;

								if (firstIteration == true) {
									if (node.intersects(physicsData, temp3.data().second) == false) {
										physicsData->physicsObjects[temp3.data().second.objectIndex].nodesIntersected.eraseData(temp1.data());
										node.dynamicColliders.eraseData(temp3.data().second.ID);
										if (temp3 == it2) {
											++it2;
										}
										continue;
									}
								}

								if (phyObject.disabledCollisions.find(temp3.data().first)) continue;

								uint32 id = pairingFunction(phyObject.colliderID, temp3.data().second.ID);

								if (physicsData->manifoldIDs.find(id)) continue;

								if (this->dispatch(id, phyObject.getIdentifier(), temp3.data().second) != CollisionFlag::NOTCOLLIDING) {
									phyObject.addToIsland(physicsData, temp3.data().second.objectIndex);
								}
							}

							firstIteration = false;

							if (phyObject.nodesIntersected.empty()) {
								//deal with objects attached to this in constraints
								if (isAValidIndex(phyObject.rigidBody.islandIndex)) {
									physicsData->islands[phyObject.rigidBody.islandIndex].eraseData(phyObject.colliderID);
								}
							}
						}

						if (node.dynamicColliders.empty() && (node.staticColliders.empty() || (node.staticColliders.size() == 1 && node.staticColliders.find(physicsData->heightFieldCollider.identifier.ID)))) {
							physicsData->octree.terminate(temp1.data());
							physicsData->activeOctreeNodes.eraseData(temp1.data());
						}
					}
				}

				CollisionFlag dispatch(const uint32& manifoldId, const ColliderIdentifier& colliderID1, const ColliderIdentifier& colliderID2)
				{
					PhysicsData* physicsData = this->narrowPhase.physicsData;

					physicsData->manifoldIDs.insert(manifoldId);

					ContactManifold manifold = ContactManifold(manifoldId, colliderID1, colliderID2);

					if (colliderID1.type == ColliderType::sphere) {

						manifold.properties1 = physicsData->sphereColliders[colliderID1.colliderIndex].properties;

						if (colliderID2.type == ColliderType::sphere) {
							if (physicsData->sphereColliders[colliderID1.colliderIndex].bound.intersects(physicsData->sphereColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->sphereColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->sphereColliders[colliderID1.colliderIndex].collider, physicsData->sphereColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::capsule) {
							if (physicsData->sphereColliders[colliderID1.colliderIndex].bound.intersects(physicsData->capsuleColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->capsuleColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->sphereColliders[colliderID1.colliderIndex].collider, physicsData->capsuleColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::convexHull) {
							if (physicsData->sphereColliders[colliderID1.colliderIndex].bound.intersects(physicsData->convexHullColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->convexHullColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->sphereColliders[colliderID1.colliderIndex].collider, physicsData->convexHullColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::triangleMesh) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->triangleMeshColliders[colliderID2.colliderIndex].collider.intersects(physicsData->sphereColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->triangleMeshColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateMeshContacts(physicsData->sphereColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::heightField) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->heightFieldCollider.collider.intersects(physicsData->sphereColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->heightFieldCollider.properties;
								this->narrowPhase.generateTerrainContacts(physicsData->sphereColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
					}
					else if (colliderID1.type == ColliderType::capsule) {

						manifold.properties1 = physicsData->capsuleColliders[colliderID1.colliderIndex].properties;

						if (colliderID2.type == ColliderType::sphere) {
							if (physicsData->capsuleColliders[colliderID1.colliderIndex].bound.intersects(physicsData->sphereColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->sphereColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->capsuleColliders[colliderID1.colliderIndex].collider, physicsData->sphereColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::capsule) {
							if (physicsData->capsuleColliders[colliderID1.colliderIndex].bound.intersects(physicsData->capsuleColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->capsuleColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->capsuleColliders[colliderID1.colliderIndex].collider, physicsData->capsuleColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::convexHull) {
							if (physicsData->capsuleColliders[colliderID1.colliderIndex].bound.intersects(physicsData->convexHullColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->convexHullColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->capsuleColliders[colliderID1.colliderIndex].collider, physicsData->convexHullColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::triangleMesh) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->triangleMeshColliders[colliderID2.colliderIndex].collider.intersects(physicsData->capsuleColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->triangleMeshColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateMeshContacts(physicsData->capsuleColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::heightField) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->heightFieldCollider.collider.intersects(physicsData->capsuleColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->heightFieldCollider.properties;
								this->narrowPhase.generateTerrainContacts(physicsData->capsuleColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
					}
					else if (colliderID1.type == ColliderType::convexHull) {

						manifold.properties1 = physicsData->convexHullColliders[colliderID1.colliderIndex].properties;

						if (colliderID2.type == ColliderType::sphere) {
							if (physicsData->convexHullColliders[colliderID1.colliderIndex].bound.intersects(physicsData->sphereColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->sphereColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->convexHullColliders[colliderID1.colliderIndex].collider, physicsData->sphereColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::capsule) {
							if (physicsData->convexHullColliders[colliderID1.colliderIndex].bound.intersects(physicsData->capsuleColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->capsuleColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->convexHullColliders[colliderID1.colliderIndex].collider, physicsData->capsuleColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::convexHull) {
							if (physicsData->convexHullColliders[colliderID1.colliderIndex].bound.intersects(physicsData->convexHullColliders[colliderID2.colliderIndex].bound)) {
								manifold.flag = CollisionFlag::PROXIMAL;
								manifold.properties2 = physicsData->convexHullColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateContacts(physicsData->convexHullColliders[colliderID1.colliderIndex].collider, physicsData->convexHullColliders[colliderID2.colliderIndex].collider, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::triangleMesh) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->triangleMeshColliders[colliderID2.colliderIndex].collider.intersects(physicsData->convexHullColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->triangleMeshColliders[colliderID2.colliderIndex].properties;
								this->narrowPhase.generateMeshContacts(physicsData->convexHullColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
						else if (colliderID2.type == ColliderType::heightField) {
							HybridArray<Triangle, 24, uint16> triangles;
							if (physicsData->heightFieldCollider.collider.intersects(physicsData->convexHullColliders[colliderID1.colliderIndex].bound, triangles)) {
								manifold.properties2 = physicsData->heightFieldCollider.properties;
								this->narrowPhase.generateTerrainContacts(physicsData->convexHullColliders[colliderID1.colliderIndex].collider, triangles, manifold);
							}
						}
					}

					if (manifold.flag == CollisionFlag::PENETRATING) {
						assert(manifold.numPoints > 0 && manifold.numPoints <= 4);
						physicsData->contactConstraints.pushBack(ContactConstraint(physicsData, manifold));
					}

					return manifold.flag;
				}
			};

			DCD dcd(this->narrowPhase);

			dcd.updateIslands();
			dcd.generateCollisionPairs();

			this->narrowPhase.physicsData->manifoldIDs.shallowClear();
		}

		void continousCollisionDetection(PhysicsObject& phyObject)
		{
			struct CCD {

				AABB overallAABB;
				SortedArray<Pair<decimal, uint16>, uint16> nodeIndicies;
				SortedArray<decimal, uint16> hitTimeFractions;

				CCD(const AABB& a) : overallAABB(a) {}

				void registerIntersectingNodes(PhysicsData* physicsData, const uint32& nodeIndex, const Vec3& position1Center)
				{
					if (physicsData->octree.nodes[nodeIndex].bound.intersects(this->overallAABB)) {

						if (physicsData->octree.nodes[nodeIndex].children.empty()) {
							this->nodeIndicies.insert(Pair<decimal, uint16>(magnitudeSq(position1Center - physicsData->octree.nodes[nodeIndex].bound.getCenter()), nodeIndex));
						}
						else {
							for (byte x = 0, len = physicsData->octree.nodes[nodeIndex].children.size(); x < len; ++x) {
								registerIntersectingNodes(physicsData, physicsData->octree.nodes[nodeIndex].children[x].second, position1Center);
							}
						}
					}
				}

				bool dispatch(PhysicsData* physicsData, const const AABB& aaabbAtOrigin, const ColliderIdentifier& colliderID2, const Transform3DPair& tA, const Transform3DPair& tB)
				{
					TOIResult toiResult;
					bool hitFound = false;

					if (colliderID2.type != ColliderType::triangleMesh && colliderID2.type != ColliderType::heightField) {

						Mat4x4 mat(decimal(1.0));
						if (isAValidIndex(colliderID2.objectIndex)) {
							mat = getInverse(physicsData->physicsObjects[colliderID2.objectIndex].rigidBody.transform.toMatrix());
						}

						if (colliderID2.type == ColliderType::sphere) {
							toiResult = timeOfImpact(aaabbAtOrigin, physicsData->sphereColliders[colliderID2.colliderIndex].bound.transformed(mat), tA, tB);
						}
						else if (colliderID2.type == ColliderType::capsule) {
							toiResult = timeOfImpact(aaabbAtOrigin, physicsData->capsuleColliders[colliderID2.colliderIndex].bound.transformed(mat), tA, tB);

						}
						else if (colliderID2.type == ColliderType::convexHull) {
							toiResult = timeOfImpact(aaabbAtOrigin, physicsData->convexHullColliders[colliderID2.colliderIndex].bound.transformed(mat), tA, tB);
						}

						if (toiResult.state == TOIState::overlaping || toiResult.state == TOIState::touching) {
							hitFound = true;
							this->hitTimeFractions.insert(toiResult.t);
						}
					}
					else {

						HybridArray<Triangle, 24, uint16> triangles;
						bool intersectionFound = false;
						
						if (colliderID2.type == ColliderType::triangleMesh) {
							if (physicsData->triangleMeshColliders[colliderID2.colliderIndex].collider.intersects(this->overallAABB, triangles)) {
								intersectionFound = true;
							}
						}
						else if (colliderID2.type == ColliderType::heightField) {
							if (physicsData->heightFieldCollider.collider.intersects(this->overallAABB, triangles)) {
								intersectionFound = true;
							}
						}

						if (intersectionFound == true) {
							for (byte x = 0, len = triangles.size(); x < len; ++x) {
								
								toiResult = timeOfImpact(aaabbAtOrigin, triangles[x], tA, tB);
								if (toiResult.state == TOIState::overlaping || toiResult.state == TOIState::touching) {
									hitFound = true;
									this->hitTimeFractions.insert(toiResult.t);
								}
							}
						}
					}

					return hitFound;
				}
			};

			PhysicsData* physicsData = this->narrowPhase.physicsData;

			for (byte x = 0, len = phyObject.nodesIntersected.size(); x < len; ++x) {
				physicsData->octree.nodes[phyObject.nodesIntersected[x]].dynamicColliders.eraseData(phyObject.colliderID);
			}
			phyObject.nodesIntersected.clear();

			ColliderIdentifier colliderID1 = phyObject.getIdentifier();
			AABB currentAABB;
			if (colliderID1.type == ColliderType::convexHull) {
				currentAABB = physicsData->convexHullColliders[colliderID1.colliderIndex].bound;
			}
			if (colliderID1.type == ColliderType::sphere) {
				currentAABB = physicsData->sphereColliders[colliderID1.colliderIndex].bound;
			}
			if (colliderID1.type == ColliderType::capsule) {
				currentAABB = physicsData->capsuleColliders[colliderID1.colliderIndex].bound;
			}

			AABB prevAABB = currentAABB.transformed(getInverse(phyObject.rigidBody.transform.toMatrix()) * phyObject.rigidBody.prevTransform3D.toMatrix());

			CCD ccd(AABB(minVec(prevAABB.min, currentAABB.min), maxVec(prevAABB.max, currentAABB.max)));
			
			Vec3 position1Center = prevAABB.getCenter();
			for (byte x = 0, len = physicsData->octree.nodes[physicsData->octree.parentNode].children.size(); x < len; ++x) {
				ccd.registerIntersectingNodes(physicsData, physicsData->octree.nodes[physicsData->octree.parentNode].children[x].second, position1Center);
			}

			AABB aabbAtOrigin = currentAABB.transformed(getInverse(phyObject.rigidBody.transform.toMatrix()));
			Transform3DPair tA;
			tA.transform1 = phyObject.rigidBody.prevTransform3D;
			tA.transform2 = phyObject.rigidBody.transform;
			for (auto it1 = ccd.nodeIndicies.begin(), end1 = ccd.nodeIndicies.end(); it1 != end1; ++it1) {

				bool hitFound = false;
				
				for (auto it2 = physicsData->octree.nodes[it1.data().second].staticColliders.begin(), end2 = physicsData->octree.nodes[it1.data().second].staticColliders.end(); it2 != end2; ++it2) {
					if (ccd.dispatch(physicsData, aabbAtOrigin, it2.data().second, tA, Transform3DPair()) == true) {
						hitFound = true;
					}
				}

				if (hitFound == true) break;

				for (auto it2 = physicsData->octree.nodes[it1.data().second].dynamicColliders.begin(), end2 = physicsData->octree.nodes[it1.data().second].dynamicColliders.end(); it2 != end2; ++it2) {
					
					Transform3DPair tB;
					tB.transform1 = physicsData->physicsObjects[it2.data().second.objectIndex].rigidBody.prevTransform3D;
					tB.transform2 = physicsData->physicsObjects[it2.data().second.objectIndex].rigidBody.transform;

					if (ccd.dispatch(physicsData, aabbAtOrigin, it2.data().second, tA, tB) == true) {
						hitFound = true;
					}
				}

				if (hitFound == true) break;
			}

			if (ccd.hitTimeFractions.empty() == false) {
				phyObject.rigidBody.subStep(physicsData, ccd.hitTimeFractions.minimum());
			}

			physicsData->octree.add(physicsData, colliderID1);
		}
	};
}

#endif

