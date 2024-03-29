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

#include"octree.h"

#include"physicsData.h"

namespace mech {
	
	void insertObject(PhysicsData* physicsData, Octree* octree, const uint16& nodeIndex, const ColliderIdentifier& colliderID)
	{
		if (isAValidIndex(colliderID.objectIndex)) {
			octree->nodes[nodeIndex].dynamicColliders.insert(Pair<uint32, ColliderIdentifier>(colliderID.ID, colliderID));

			if (physicsData->physicsObjects[colliderID.objectIndex].nodesIntersected.find(nodeIndex) == false) {
				physicsData->physicsObjects[colliderID.objectIndex].nodesIntersected.pushBack(nodeIndex);
			}
		}
		else {
			octree->nodes[nodeIndex].staticColliders.pushBack(Pair<uint32, ColliderIdentifier>(colliderID.ID, colliderID));
		}
	}

	void heightFieldTest(PhysicsData* physicsData, Octree* octree, const uint16& nodeIndex)
	{
		if (octree->nodes[nodeIndex].evaluate(physicsData, physicsData->heightFieldCollider.identifier)) {
			octree->nodes[nodeIndex].staticColliders.pushBack(Pair<uint32, ColliderIdentifier>(physicsData->heightFieldCollider.identifier.ID, physicsData->heightFieldCollider.identifier));
		}
	}

	uint16 insertChild(Octree* octree, const uint16& parentIndex, const AABB& aabb, const byte& key)
	{
		uint16 childIndex = octree->nodes.insert(Octree::Node(aabb));
		octree->nodes[childIndex].key = key;
		octree->nodes[childIndex].parent = parentIndex;

		octree->nodes[parentIndex].children[key] = Pair<byte, uint16>(key, childIndex);
		++octree->nodes[parentIndex].children.size();

		return childIndex;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	byte Octree::Node::evaluate(PhysicsData* physicsData, const ColliderIdentifier& colliderID)
	{
		if (colliderID.type == ColliderType::heightField) {
			const decimal& h = physicsData->heightFieldCollider.collider.maxHeight;
			return (this->bound.min.y <= h && this->bound.min.y >= -h) || (this->bound.max.y <= h && this->bound.max.y >= -h);
		}
		else {
			
			const AABB& bound = physicsData->getColliderAABB(colliderID);
			if (this->bound.intersects(bound)) {
				if (this->bound.contains(bound)) {
					return 2;
				}
				return 1;
			}
		}

		return 0;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Octree::initialise(const AABB& bounds, const byte& inDepth)
	{
		this->nodes.insert(Node(bounds));
		this->depth = inDepth;
	}

	void Octree::add(PhysicsData* physicsData, const ColliderIdentifier& colliderID)
	{
		struct Helper {
			Octree* octree;

			Helper(Octree* o) : octree(o) {}

			void addCollider(PhysicsData* physicsData, const uint16& nodeIndex, const ColliderIdentifier& colliderID, byte depth)
			{
				--depth;

				for (byte x = 0; x < 8; ++x) {
					
					if (isAValidIndex(this->octree->nodes[nodeIndex].children[x].first)) {

						uint16 childIndex = this->octree->nodes[nodeIndex].children[x].second;

						byte b = this->octree->nodes[childIndex].evaluate(physicsData, colliderID);
						if (b != 0) {

							if (depth == 0) {
								insertObject(physicsData, this->octree, childIndex, colliderID);
							}
							else {
								addCollider(physicsData, childIndex, colliderID, depth);
							}
						}

						if (b == 2) break;
					}
					else {
						AABB aabb = this->octree->nodes[nodeIndex].bound.partitionTo8(x);

						byte b = Node(aabb).evaluate(physicsData, colliderID);
						if (b != 0) {

							uint16 childIndex = insertChild(this->octree, nodeIndex, aabb, x);

							if (depth == 0) {
								insertObject(physicsData, this->octree, childIndex, colliderID);
								heightFieldTest(physicsData, this->octree, childIndex);
							}
							else {
								addCollider(physicsData, childIndex, colliderID, depth);
							}
						}

						if (b == 2) break;
					}
				}
			}
		};

		assert(this->nodes.empty() == false);

		Helper helper(this);

		if (this->nodes[this->parentNode].evaluate(physicsData, colliderID) != 0) {
			helper.addCollider(physicsData, this->parentNode, colliderID, this->depth);
		}
	}

	void Octree::repositionCollider(PhysicsData* physicsData, const ColliderIdentifier& colliderID)
	{
		struct Helper {
			
			Octree* octree;

			Helper(Octree* o) : octree(o) {}

			AABB neighbourBound(const uint16& nodeIndex, const uint16& sideIndex, const Vec3& dimensions)
			{
				switch (sideIndex) {
				case 0:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, -dimensions.z));

				case 1:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), dimensions.y, -dimensions.z));

				case 2:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, -dimensions.z));

				case 3:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, decimal(0.0)));

				case 4:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), dimensions.y, decimal(0.0)));

				case 5:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, decimal(0.0)));

				case 6:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, dimensions.z));

				case 7:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), dimensions.y, dimensions.z));

				case 8:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, dimensions.y, dimensions.z));

				case 9:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, decimal(0.0), -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, decimal(0.0), -dimensions.z));

				case 10:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), decimal(0.0), -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), decimal(0.0), -dimensions.z));

				case 11:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, decimal(0.0), -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, decimal(0.0), -dimensions.z));

				case 12:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, decimal(0.0), decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, decimal(0.0), decimal(0.0)));

				case 13:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, decimal(0.0), decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(-dimensions.x, decimal(0.0), decimal(0.0)));

				case 14:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, decimal(0.0), dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, decimal(0.0), dimensions.z));

				case 15:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), decimal(0.0), dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), decimal(0.0), dimensions.z));

				case 16:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, decimal(0.0), -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, decimal(0.0), -dimensions.z));

				case 17:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, -dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, -dimensions.y, -dimensions.z));

				case 18:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), -dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), -dimensions.y, -dimensions.z));

				case 19:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, -dimensions.y, -dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(-dimensions.x, -dimensions.y, -dimensions.z));

				case 20:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, -dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, -dimensions.y, decimal(0.0)));

				case 21:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), -dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), -dimensions.y, decimal(0.0)));

				case 22:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, -dimensions.y, decimal(0.0)), this->octree->nodes[nodeIndex].bound.max + Vec3(-dimensions.x, -dimensions.y, decimal(0.0)));

				case 23:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(dimensions.x, -dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(dimensions.x, -dimensions.y, dimensions.z));

				case 24:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(decimal(0.0), -dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(decimal(0.0), -dimensions.y, dimensions.z));

				case 25:
					return AABB(this->octree->nodes[nodeIndex].bound.min + Vec3(-dimensions.x, -dimensions.y, dimensions.z), this->octree->nodes[nodeIndex].bound.max + Vec3(-dimensions.x, -dimensions.y, dimensions.z));
				}

				return AABB();
			}

			StackArray<byte, 8> getValidNodes(const uint16& nodeIndex, const Vec3& dimensions, const AABB& aabb)
			{
				StackArray<byte, 8> valid;

				if (neighbourBound(nodeIndex, 12, dimensions).intersects(aabb)) {
					valid.pushBack(12);

					if (neighbourBound(nodeIndex, 10, dimensions).intersects(aabb)) {
						valid.pushBack(10);

						if (neighbourBound(nodeIndex, 9, dimensions).intersects(aabb)) {
							valid.pushBack(9);
						}

						if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
							valid.pushBack(4);

							if (neighbourBound(nodeIndex, 0, dimensions).intersects(aabb)) {
								valid.pushBack(0);
							}
							if (neighbourBound(nodeIndex, 1, dimensions).intersects(aabb)) {
								valid.pushBack(1);
							}
							if (neighbourBound(nodeIndex, 3, dimensions).intersects(aabb)) {
								valid.pushBack(3);
							}
						}
						else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
							valid.pushBack(21);

							if (neighbourBound(nodeIndex, 18, dimensions).intersects(aabb)) {
								valid.pushBack(18);
							}
							if (neighbourBound(nodeIndex, 17, dimensions).intersects(aabb)) {
								valid.pushBack(17);
							}
							if (neighbourBound(nodeIndex, 20, dimensions).intersects(aabb)) {
								valid.pushBack(20);
							}
						}
					}
					else if (neighbourBound(nodeIndex, 15, dimensions).intersects(aabb)) {
						valid.pushBack(15);

						if (neighbourBound(nodeIndex, 14, dimensions).intersects(aabb)) {
							valid.pushBack(14);
						}

						if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
							valid.pushBack(4);

							if (neighbourBound(nodeIndex, 7, dimensions).intersects(aabb)) {
								valid.pushBack(7);
							}
							if (neighbourBound(nodeIndex, 6, dimensions).intersects(aabb)) {
								valid.pushBack(6);
							}
							if (neighbourBound(nodeIndex, 3, dimensions).intersects(aabb)) {
								valid.pushBack(3);
							}
						}
						else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
							valid.pushBack(21);

							if (neighbourBound(nodeIndex, 24, dimensions).intersects(aabb)) {
								valid.pushBack(24);
							}
							if (neighbourBound(nodeIndex, 23, dimensions).intersects(aabb)) {
								valid.pushBack(23);
							}
							if (neighbourBound(nodeIndex, 20, dimensions).intersects(aabb)) {
								valid.pushBack(20);
							}
						}
					}
					else if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
						valid.pushBack(4);

						if (neighbourBound(nodeIndex, 3, dimensions).intersects(aabb)) {
							valid.pushBack(3);
						}
					}
					else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
						valid.pushBack(21);

						if (neighbourBound(nodeIndex, 20, dimensions).intersects(aabb)) {
							valid.pushBack(20);
						}
					}
				}
				else if (neighbourBound(nodeIndex, 13, dimensions).intersects(aabb)) {
					valid.pushBack(13);

					if (neighbourBound(nodeIndex, 10, dimensions).intersects(aabb)) {
						valid.pushBack(10);

						if (neighbourBound(nodeIndex, 11, dimensions).intersects(aabb)) {
							valid.pushBack(11);
						}

						if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
							valid.pushBack(4);

							if (neighbourBound(nodeIndex, 1, dimensions).intersects(aabb)) {
								valid.pushBack(1);
							}
							if (neighbourBound(nodeIndex, 2, dimensions).intersects(aabb)) {
								valid.pushBack(2);
							}
							if (neighbourBound(nodeIndex, 5, dimensions).intersects(aabb)) {
								valid.pushBack(5);
							}
						}
						else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
							valid.pushBack(21);

							if (neighbourBound(nodeIndex, 18, dimensions).intersects(aabb)) {
								valid.pushBack(18);
							}
							if (neighbourBound(nodeIndex, 19, dimensions).intersects(aabb)) {
								valid.pushBack(19);
							}
							if (neighbourBound(nodeIndex, 22, dimensions).intersects(aabb)) {
								valid.pushBack(22);
							}
						}
					}
					else if (neighbourBound(nodeIndex, 15, dimensions).intersects(aabb)) {
						valid.pushBack(15);

						if (neighbourBound(nodeIndex, 16, dimensions).intersects(aabb)) {
							valid.pushBack(16);
						}

						if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
							valid.pushBack(4);

							if (neighbourBound(nodeIndex, 5, dimensions).intersects(aabb)) {
								valid.pushBack(5);
							}
							if (neighbourBound(nodeIndex, 8, dimensions).intersects(aabb)) {
								valid.pushBack(8);
							}
							if (neighbourBound(nodeIndex, 7, dimensions).intersects(aabb)) {
								valid.pushBack(7);
							}
						}
						else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
							valid.pushBack(21);

							if (neighbourBound(nodeIndex, 22, dimensions).intersects(aabb)) {
								valid.pushBack(22);
							}
							if (neighbourBound(nodeIndex, 25, dimensions).intersects(aabb)) {
								valid.pushBack(25);
							}
							if (neighbourBound(nodeIndex, 24, dimensions).intersects(aabb)) {
								valid.pushBack(24);
							}
						}
					}
					else if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
						valid.pushBack(4);

						if (neighbourBound(nodeIndex, 5, dimensions).intersects(aabb)) {
							valid.pushBack(5);
						}
					}
					else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
						valid.pushBack(21);

						if (neighbourBound(nodeIndex, 22, dimensions).intersects(aabb)) {
							valid.pushBack(22);
						}
					}
				}
				else if (neighbourBound(nodeIndex, 10, dimensions).intersects(aabb)) {
					valid.pushBack(10);

					if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
						valid.pushBack(4);

						if (neighbourBound(nodeIndex, 1, dimensions).intersects(aabb)) {
							valid.pushBack(1);
						}
					}
					else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
						valid.pushBack(21);

						if (neighbourBound(nodeIndex, 18, dimensions).intersects(aabb)) {
							valid.pushBack(18);
						}
					}
				}
				else if (neighbourBound(nodeIndex, 15, dimensions).intersects(aabb)) {
					valid.pushBack(15);

					if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
						valid.pushBack(4);

						if (neighbourBound(nodeIndex, 7, dimensions).intersects(aabb)) {
							valid.pushBack(7);
						}
					}
					else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
						valid.pushBack(21);

						if (neighbourBound(nodeIndex, 24, dimensions).intersects(aabb)) {
							valid.pushBack(24);
						}
					}
				}
				else if (neighbourBound(nodeIndex, 4, dimensions).intersects(aabb)) {
					valid.pushBack(4);
				}
				else if (neighbourBound(nodeIndex, 21, dimensions).intersects(aabb)) {
					valid.pushBack(21);
				}

				return valid;
			}

			bool specialContains(const AABB& aabb, const Vec3& point)
			{
				return point.x >= aabb.min.x && point.x <= aabb.max.x &&
					point.y >= aabb.min.y && point.y <= aabb.max.y &&
					point.z >= aabb.min.z && point.z <= aabb.max.z;
			}

			uint16 getChildInternal(PhysicsData* physicsData, const uint16& nodeIndex, const Vec3& otherCenter, const byte& depth, const byte& key1, const byte& key2)
			{
				uint16 childIndex = -1;

				AABB box = this->octree->nodes[nodeIndex].bound.partitionTo8(key1);
				if (specialContains(box, otherCenter)) {

					if (isAValidIndex(this->octree->nodes[nodeIndex].children[key1].first)) {
						childIndex = this->octree->nodes[nodeIndex].children[key1].second;
					}
					else {
						childIndex = insertChild(this->octree, nodeIndex, box, key1);
						if (depth == 0) {
							heightFieldTest(physicsData, this->octree, childIndex);
						}
					}
				}
				else {

					if (isAValidIndex(this->octree->nodes[nodeIndex].children[key2].first)) {
						childIndex = this->octree->nodes[nodeIndex].children[key2].second;
					}
					else {
						childIndex = insertChild(this->octree, nodeIndex, this->octree->nodes[nodeIndex].bound.partitionTo8(key2), key2);
						if (depth == 0) {
							heightFieldTest(physicsData, this->octree, childIndex);
						}
					}
				}

				if (depth != 0) {
					childIndex = getChild(physicsData, childIndex, otherCenter, depth);
				}

				return childIndex;
			}

			uint16 getChild(PhysicsData* physicsData, const uint16& nodeIndex, const Vec3& otherCenter, byte depth)
			{
				--depth;

				Vec3 c = this->octree->nodes[nodeIndex].bound.getCenter();

				if (specialContains(AABB(this->octree->nodes[nodeIndex].bound.min, Vec3(this->octree->nodes[nodeIndex].bound.max.x, c.y, this->octree->nodes[nodeIndex].bound.max.z)), otherCenter)) {

					if (specialContains(AABB(this->octree->nodes[nodeIndex].bound.min, Vec3(c.x, c.y, this->octree->nodes[nodeIndex].bound.max.z)), otherCenter)) {
						return getChildInternal(physicsData, nodeIndex, otherCenter, depth, 0, 4);
					}
					else {
						return getChildInternal(physicsData, nodeIndex, otherCenter, depth, 1, 5);
					}
				}
				else {

					if (specialContains(AABB(Vec3(this->octree->nodes[nodeIndex].bound.min.x, c.y, this->octree->nodes[nodeIndex].bound.min.z), Vec3(c.x, this->octree->nodes[nodeIndex].bound.max.y, this->octree->nodes[nodeIndex].bound.max.z)), otherCenter)) {
						return getChildInternal(physicsData, nodeIndex, otherCenter, depth, 2, 6);
					}
					else {
						return getChildInternal(physicsData, nodeIndex, otherCenter, depth, 3, 7);
					}
				}

				return -1;
			}
		};

		uint16 nodeIndex = physicsData->physicsObjects[colliderID.objectIndex].nodesIntersected[0];

		if (this->nodes[nodeIndex].evaluate(physicsData, colliderID) == 2) return;

		Helper helper(this);

		Vec3 dimensions = this->nodes[nodeIndex].bound.getDimensions();

		StackArray<byte, 8> valid = helper.getValidNodes(nodeIndex, dimensions, physicsData->getColliderAABB(colliderID));

		for (byte x = 0, len = valid.size(); x < len; ++x) {
			Vec3 nodeCenter = helper.neighbourBound(nodeIndex, valid[x], dimensions).getCenter();
			if(helper.specialContains(this->nodes[this->parentNode].bound, nodeCenter)) {
				insertObject(physicsData, this, helper.getChild(physicsData, this->parentNode, nodeCenter, this->depth), colliderID);
			}
		}
	}

	void Octree::terminate(const uint16& index)
	{
		uint16 parentIndex = this->nodes[index].parent;
		if (isAValidIndex(parentIndex)) {

			this->nodes[parentIndex].children[this->nodes[index].key] = Pair<byte, uint16>(-1, -1);
			--this->nodes[parentIndex].children.size();

			this->nodes.eraseDataAtIndex(index);

			if (this->nodes[parentIndex].children.empty()) {
				this->terminate(parentIndex);
			}
		}
	}
}
