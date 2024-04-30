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

#include"octree.h"

namespace mech {
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	uint16 insertChild(Octree* octree, const uint16& parentIndex, const AABB& bound, const byte& key)
	{
		uint16 childIndex = octree->nodes.insert(Octree::Node(bound));
		octree->nodes[childIndex].key = key;
		octree->nodes[childIndex].parent = parentIndex;

		octree->nodes[parentIndex].children[key] = Pair<byte, uint16>(key, childIndex);
		octree->nodes[parentIndex].children.setSize(octree->nodes[parentIndex].children.size() + 1);

		return childIndex;
	}

	struct BoundGenerator {
		AABB (BoundGenerator::* boundPtrs[26]) (const AABB&, const Vec3&) = {};

		BoundGenerator()
		{
			this->boundPtrs[0] = &BoundGenerator::f0;
			this->boundPtrs[1] = &BoundGenerator::f1;
			this->boundPtrs[2] = &BoundGenerator::f2;
			this->boundPtrs[3] = &BoundGenerator::f3;
			this->boundPtrs[4] = &BoundGenerator::f4;
			this->boundPtrs[5] = &BoundGenerator::f5;
			this->boundPtrs[6] = &BoundGenerator::f6;
			this->boundPtrs[7] = &BoundGenerator::f7;
			this->boundPtrs[8] = &BoundGenerator::f8;
			this->boundPtrs[9] = &BoundGenerator::f9;
			this->boundPtrs[10] = &BoundGenerator::f10;
			this->boundPtrs[11] = &BoundGenerator::f11;
			this->boundPtrs[12] = &BoundGenerator::f12;
			this->boundPtrs[13] = &BoundGenerator::f13;
			this->boundPtrs[14] = &BoundGenerator::f14;
			this->boundPtrs[15] = &BoundGenerator::f15;
			this->boundPtrs[16] = &BoundGenerator::f16;
			this->boundPtrs[17] = &BoundGenerator::f17;
			this->boundPtrs[18] = &BoundGenerator::f18;
			this->boundPtrs[19] = &BoundGenerator::f19;
			this->boundPtrs[20] = &BoundGenerator::f20;
			this->boundPtrs[21] = &BoundGenerator::f21;
			this->boundPtrs[22] = &BoundGenerator::f22;
			this->boundPtrs[23] = &BoundGenerator::f23;
			this->boundPtrs[24] = &BoundGenerator::f24;
			this->boundPtrs[25] = &BoundGenerator::f25;
		}

		AABB generateBound(const AABB& bound, const byte& index, const Vec3& dimensions) { return (this->*boundPtrs[index])(bound, dimensions); }

		AABB f0(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, d.y, -d.z), b.max + Vec3(d.x, d.y, -d.z)); }
		AABB f1(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), d.y, -d.z), b.max + Vec3(decimal(0.0), d.y, -d.z)); }
		AABB f2(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, d.y, -d.z), b.max + Vec3(d.x, d.y, -d.z)); }
		AABB f3(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, d.y, decimal(0.0)), b.max + Vec3(d.x, d.y, decimal(0.0))); }
		AABB f4(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), d.y, decimal(0.0)), b.max + Vec3(decimal(0.0), d.y, decimal(0.0))); }
		AABB f5(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, d.y, decimal(0.0)), b.max + Vec3(d.x, d.y, decimal(0.0))); }
		AABB f6(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, d.y, d.z), b.max + Vec3(d.x, d.y, d.z)); }
		AABB f7(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), d.y, d.z), b.max + Vec3(decimal(0.0), d.y, d.z)); }
		AABB f8(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, d.y, d.z), b.max + Vec3(d.x, d.y, d.z)); }
		AABB f9(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, decimal(0.0), -d.z), b.max + Vec3(d.x, decimal(0.0), -d.z)); }
		AABB f10(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), decimal(0.0), -d.z), b.max + Vec3(decimal(0.0), decimal(0.0), -d.z)); }
		AABB f11(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, decimal(0.0), -d.z), b.max + Vec3(d.x, decimal(0.0), -d.z)); }
		AABB f12(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, decimal(0.0), decimal(0.0)), b.max + Vec3(d.x, decimal(0.0), decimal(0.0))); }
		AABB f13(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, decimal(0.0), decimal(0.0)), b.max + Vec3(-d.x, decimal(0.0), decimal(0.0))); }
		AABB f14(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, decimal(0.0), d.z), b.max + Vec3(d.x, decimal(0.0), d.z)); }
		AABB f15(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), decimal(0.0), d.z), b.max + Vec3(decimal(0.0), decimal(0.0), d.z)); }
		AABB f16(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, decimal(0.0), -d.z), b.max + Vec3(d.x, decimal(0.0), -d.z)); }
		AABB f17(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, -d.y, -d.z), b.max + Vec3(d.x, -d.y, -d.z)); }
		AABB f18(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), -d.y, -d.z), b.max + Vec3(decimal(0.0), -d.y, -d.z)); }
		AABB f19(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, -d.y, -d.z), b.max + Vec3(-d.x, -d.y, -d.z)); }
		AABB f20(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, -d.y, decimal(0.0)), b.max + Vec3(d.x, -d.y, decimal(0.0))); }
		AABB f21(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), -d.y, decimal(0.0)), b.max + Vec3(decimal(0.0), -d.y, decimal(0.0))); }
		AABB f22(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, -d.y, decimal(0.0)), b.max + Vec3(-d.x, -d.y, decimal(0.0))); }
		AABB f23(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(d.x, -d.y, d.z), b.max + Vec3(d.x, -d.y, d.z)); }
		AABB f24(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(decimal(0.0), -d.y, d.z), b.max + Vec3(decimal(0.0), -d.y, d.z)); }
		AABB f25(const AABB& b, const Vec3& d) { return AABB(b.min + Vec3(-d.x, -d.y, d.z), b.max + Vec3(-d.x, -d.y, d.z)); }
	};

	BoundGenerator boundGenerator;
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	byte Octree::Node::evaluateEntity(const AABB& entityAABB)
	{
		if (this->bound.intersects(entityAABB)) {
			if (this->bound.contains(entityAABB)) {
				return 2;
			}
			return 1;
		}

		return 0;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void Octree::initialise(const AABB& bounds, OctreeToHeightFieldLink* ohl, const byte& inDepth)
	{
		this->nodes.insert(Node(bounds));
		this->depth = inDepth;

		AABB a = bounds;
		for (byte x = 0; x < this->depth; ++x) {
			a = a.partitionTo8(0);
		}
		this->acceptableRadiusSq = a.getRadiusSq() * decimal(2.0 / 3.0);

		this->heightFieldLink = ohl;
	}

	StackArray<uint16, 8> Octree::addEntity(const uint32& entityID, const AABB& entityAABB)
	{
		struct TaskExecuter {

			void findNode(Octree* octree, const uint16& nodeIndex, const uint32& entityID, const AABB& entityAABB, byte depth, StackArray<uint16, 8>& nodes)
			{
				++depth;

				for (byte x = 0; x < 8; ++x) {

					byte evaluation = 0;
					
					if (isAValidIndex(octree->nodes[nodeIndex].children[x].first)) {

						uint16 childIndex = octree->nodes[nodeIndex].children[x].second;

						evaluation = octree->nodes[childIndex].evaluateEntity(entityAABB);
						if (evaluation != 0) {

							if (depth == octree->depth) {
								octree->nodes[childIndex].entities.insert(entityID);
								nodes.pushBack(childIndex);
							}
							else {
								findNode(octree, childIndex, entityID, entityAABB, depth, nodes);
							}
						}
					}
					else {
						AABB a = octree->nodes[nodeIndex].bound.partitionTo8(x);

						evaluation = Node(a).evaluateEntity(entityAABB);
						if (evaluation != 0) {

							uint16 childIndex = insertChild(octree, nodeIndex, a, x);

							if (depth == octree->depth) {
								octree->nodes[childIndex].entities.insert(entityID);
								nodes.pushBack(childIndex);
								if (octree->heightFieldLink->intersects(a)) {
									octree->nodes[childIndex].entities.insert(octree->heightFieldLink->heightFieldID);
								}
							}
							else {
								findNode(octree, childIndex, entityID, entityAABB, depth, nodes);
							}
						}
					}

					if (evaluation == 2) break;
				}
			}
		};

		ASSERT(this->nodes.empty() == false, "no parent node!!, initialise octree first");

		StackArray<uint16, 8> n;
		TaskExecuter ex;
		if (this->nodes[0].evaluateEntity(entityAABB) != 0) {
			ex.findNode(this, 0, entityID, entityAABB, 0, n);
		}

		return n;
	}

	void Octree::updateEntityDiscrete(const uint32& entityID, const AABB& entityAABB, StackArray<uint16, 8>& referenceNodes)
	{
		struct TaskExecuter {

			StackArray<byte, 8> getValidNodes(const AABB& bound, const Vec3& dimensions, const AABB& entityAABB)
			{
				StackArray<byte, 8> valid;

				if (boundGenerator.generateBound(bound, 12, dimensions).intersects(entityAABB)) {
					valid.pushBack(12);

					if (boundGenerator.generateBound(bound, 10, dimensions).intersects(entityAABB)) {
						valid.pushBack(10);

						if (boundGenerator.generateBound(bound, 9, dimensions).intersects(entityAABB)) {
							valid.pushBack(9);
						}

						if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
							valid.pushBack(4);

							if (boundGenerator.generateBound(bound, 0, dimensions).intersects(entityAABB)) {
								valid.pushBack(0);
							}
							if (boundGenerator.generateBound(bound, 1, dimensions).intersects(entityAABB)) {
								valid.pushBack(1);
							}
							if (boundGenerator.generateBound(bound, 3, dimensions).intersects(entityAABB)) {
								valid.pushBack(3);
							}
						}
						else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
							valid.pushBack(21);

							if (boundGenerator.generateBound(bound, 18, dimensions).intersects(entityAABB)) {
								valid.pushBack(18);
							}
							if (boundGenerator.generateBound(bound, 17, dimensions).intersects(entityAABB)) {
								valid.pushBack(17);
							}
							if (boundGenerator.generateBound(bound, 20, dimensions).intersects(entityAABB)) {
								valid.pushBack(20);
							}
						}
					}
					else if (boundGenerator.generateBound(bound, 15, dimensions).intersects(entityAABB)) {
						valid.pushBack(15);

						if (boundGenerator.generateBound(bound, 14, dimensions).intersects(entityAABB)) {
							valid.pushBack(14);
						}

						if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
							valid.pushBack(4);

							if (boundGenerator.generateBound(bound, 7, dimensions).intersects(entityAABB)) {
								valid.pushBack(7);
							}
							if (boundGenerator.generateBound(bound, 6, dimensions).intersects(entityAABB)) {
								valid.pushBack(6);
							}
							if (boundGenerator.generateBound(bound, 3, dimensions).intersects(entityAABB)) {
								valid.pushBack(3);
							}
						}
						else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
							valid.pushBack(21);

							if (boundGenerator.generateBound(bound, 24, dimensions).intersects(entityAABB)) {
								valid.pushBack(24);
							}
							if (boundGenerator.generateBound(bound, 23, dimensions).intersects(entityAABB)) {
								valid.pushBack(23);
							}
							if (boundGenerator.generateBound(bound, 20, dimensions).intersects(entityAABB)) {
								valid.pushBack(20);
							}
						}
					}
					else if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
						valid.pushBack(4);

						if (boundGenerator.generateBound(bound, 3, dimensions).intersects(entityAABB)) {
							valid.pushBack(3);
						}
					}
					else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
						valid.pushBack(21);

						if (boundGenerator.generateBound(bound, 20, dimensions).intersects(entityAABB)) {
							valid.pushBack(20);
						}
					}
				}
				else if (boundGenerator.generateBound(bound, 13, dimensions).intersects(entityAABB)) {
					valid.pushBack(13);

					if (boundGenerator.generateBound(bound, 10, dimensions).intersects(entityAABB)) {
						valid.pushBack(10);

						if (boundGenerator.generateBound(bound, 11, dimensions).intersects(entityAABB)) {
							valid.pushBack(11);
						}

						if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
							valid.pushBack(4);

							if (boundGenerator.generateBound(bound, 1, dimensions).intersects(entityAABB)) {
								valid.pushBack(1);
							}
							if (boundGenerator.generateBound(bound, 2, dimensions).intersects(entityAABB)) {
								valid.pushBack(2);
							}
							if (boundGenerator.generateBound(bound, 5, dimensions).intersects(entityAABB)) {
								valid.pushBack(5);
							}
						}
						else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
							valid.pushBack(21);

							if (boundGenerator.generateBound(bound, 18, dimensions).intersects(entityAABB)) {
								valid.pushBack(18);
							}
							if (boundGenerator.generateBound(bound, 19, dimensions).intersects(entityAABB)) {
								valid.pushBack(19);
							}
							if (boundGenerator.generateBound(bound, 22, dimensions).intersects(entityAABB)) {
								valid.pushBack(22);
							}
						}
					}
					else if (boundGenerator.generateBound(bound, 15, dimensions).intersects(entityAABB)) {
						valid.pushBack(15);

						if (boundGenerator.generateBound(bound, 16, dimensions).intersects(entityAABB)) {
							valid.pushBack(16);
						}

						if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
							valid.pushBack(4);

							if (boundGenerator.generateBound(bound, 5, dimensions).intersects(entityAABB)) {
								valid.pushBack(5);
							}
							if (boundGenerator.generateBound(bound, 8, dimensions).intersects(entityAABB)) {
								valid.pushBack(8);
							}
							if (boundGenerator.generateBound(bound, 7, dimensions).intersects(entityAABB)) {
								valid.pushBack(7);
							}
						}
						else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
							valid.pushBack(21);

							if (boundGenerator.generateBound(bound, 22, dimensions).intersects(entityAABB)) {
								valid.pushBack(22);
							}
							if (boundGenerator.generateBound(bound, 25, dimensions).intersects(entityAABB)) {
								valid.pushBack(25);
							}
							if (boundGenerator.generateBound(bound, 24, dimensions).intersects(entityAABB)) {
								valid.pushBack(24);
							}
						}
					}
					else if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
						valid.pushBack(4);

						if (boundGenerator.generateBound(bound, 5, dimensions).intersects(entityAABB)) {
							valid.pushBack(5);
						}
					}
					else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
						valid.pushBack(21);

						if (boundGenerator.generateBound(bound, 22, dimensions).intersects(entityAABB)) {
							valid.pushBack(22);
						}
					}
				}
				else if (boundGenerator.generateBound(bound, 10, dimensions).intersects(entityAABB)) {
					valid.pushBack(10);

					if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
						valid.pushBack(4);

						if (boundGenerator.generateBound(bound, 1, dimensions).intersects(entityAABB)) {
							valid.pushBack(1);
						}
					}
					else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
						valid.pushBack(21);

						if (boundGenerator.generateBound(bound, 18, dimensions).intersects(entityAABB)) {
							valid.pushBack(18);
						}
					}
				}
				else if (boundGenerator.generateBound(bound, 15, dimensions).intersects(entityAABB)) {
					valid.pushBack(15);

					if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
						valid.pushBack(4);

						if (boundGenerator.generateBound(bound, 7, dimensions).intersects(entityAABB)) {
							valid.pushBack(7);
						}
					}
					else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
						valid.pushBack(21);

						if (boundGenerator.generateBound(bound, 24, dimensions).intersects(entityAABB)) {
							valid.pushBack(24);
						}
					}
				}
				else if (boundGenerator.generateBound(bound, 4, dimensions).intersects(entityAABB)) {
					valid.pushBack(4);
				}
				else if (boundGenerator.generateBound(bound, 21, dimensions).intersects(entityAABB)) {
					valid.pushBack(21);
				}

				return valid;
			}

			uint16 findRootParent(Octree* octree, const uint16& nodeIndex, const Vec3& nodeCenter, byte& depth)
			{
				if (octree->nodes[nodeIndex].bound.contains(nodeCenter)) {
					return nodeIndex;
				}

				++depth;
				return findRootParent(octree, octree->nodes[nodeIndex].parent, nodeCenter, depth);
			}

			uint16 findChild(Octree* octree, const uint16& parent, const Vec3& nodeCenter, byte& depth)
			{
				uint16 childIndex = -1;
				for (byte x = 0; x < 8; ++x) {

					if (isAValidIndex(octree->nodes[parent].children[x].first)) {

						if (octree->nodes[octree->nodes[parent].children[x].second].bound.contains(nodeCenter)) {
							childIndex = octree->nodes[parent].children[x].second;
							break;
						}
					}
					else {
						AABB bound = octree->nodes[parent].bound.partitionTo8(x);
						if (bound.contains(nodeCenter)) {
							childIndex = insertChild(octree, parent, bound, x);
							if (depth == 0) {
								if (octree->heightFieldLink->intersects(bound)) {
									octree->nodes[childIndex].entities.insert(octree->heightFieldLink->heightFieldID);
								}
							}
							break;
						}
					}
				}

				if (depth == 0) {
					return childIndex;
				}

				--depth;
				return findChild(octree, childIndex, nodeCenter, depth);
			}
		};

		BEGIN_PROFILE("Octree::repositionEntity");

		ASSERT(entityAABB.getRadiusSq() < this->acceptableRadiusSq, "collider is too large to be correctly handled!!, consider; -partitioning collider to smaller colliders, -scaling up the size of the octree, - reducing the depth of the octree");

		bool in[2] = { false, false };
		for (byte x = 0, len = referenceNodes.size(); x < len; ++x) {
			
			Node& node = this->nodes[referenceNodes[x]];
			if (node.evaluateEntity(entityAABB) == 0) {
				node.entities.eraseData(entityID);
				if (this->isNodeEmpty(referenceNodes[x])) {
					terminateNode(referenceNodes[x]);
				}
				referenceNodes.eraseDataAtIndex(x);
				--x; --len;
			}
			else {
				in[0] = in[0] == false ? node.bound.contains(entityAABB.min) : true;
				in[1] = in[1] == false ? node.bound.contains(entityAABB.max) : true;
			}
		}

		if (in[0] == false || in[1] == false) {

			TaskExecuter ex;

			Vec3 dimensions = this->nodes[referenceNodes[0]].bound.getDimensions();
			StackArray<byte, 8> valid = ex.getValidNodes(this->nodes[referenceNodes[0]].bound, dimensions, entityAABB);

			for (byte x = 0, len1 = valid.size(); x < len1; ++x) {

				Vec3 nodeCenter = boundGenerator.generateBound(this->nodes[referenceNodes[0]].bound, valid[x], dimensions).getCenter();
				if (this->nodes[0].bound.contains(nodeCenter)) {

					byte d = 0;
					uint16 rootParent = ex.findRootParent(this, this->nodes[referenceNodes[0]].parent, nodeCenter, d);
					uint16 childIndex = ex.findChild(this, rootParent, nodeCenter, d);

					if (referenceNodes.find(childIndex) == false) {
						this->nodes[childIndex].entities.insert(entityID);
						referenceNodes.pushBack(childIndex);
					}
				}
			}
		}

		END_PROFILE;
	}

	void Octree::updateEntityContinous(const uint32& entityID, const AABB& entityAABB, StackArray<uint16, 8>& referenceNodes)
	{
		for (byte x = 0, len = referenceNodes.size(); x < len; ++x) {
			this->nodes[referenceNodes[x]].entities.eraseData(entityID);
			if (this->isNodeEmpty(referenceNodes[x])) {
				this->terminateNode(referenceNodes[x]);
			}
		}
		referenceNodes.clear();
		referenceNodes = this->addEntity(entityID, entityAABB);
	}

	bool Octree::isNodeEmpty(const uint16& index)
	{
		return this->nodes[index].entities.empty() || (this->nodes[index].entities.size() == 1 && this->nodes[index].entities.find(this->heightFieldLink->heightFieldID));
	}

	void Octree::terminateNode(const uint16& index)
	{
		uint16 parentIndex = this->nodes[index].parent;
		if (isAValidIndex(parentIndex)) {

			this->nodes[parentIndex].children[this->nodes[index].key] = Pair<byte, uint16>(-1, -1);
			this->nodes[parentIndex].children.setSize(this->nodes[parentIndex].children.size() - 1);

			this->nodes.eraseDataAtIndex(index);

			if (this->nodes[parentIndex].children.empty()) {
				this->terminateNode(parentIndex);
			}
		}
	}
}
