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

#ifndef OCTREE_H
#define OCTREE_H

#include"../geometry/aabb.h"
#include"../containers/rigidArray.h"
#include"../containers/hashTable.h"

namespace mech {

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct OctreeToHeightFieldLink {
		uint32 heightFieldID = -1;
		virtual bool intersects(const AABB& aabb) = 0;
	};

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct Octree  {

		struct Node {

			byte key = -1;
			uint16 parent = -1;
			AABB bound;
			StackArray<Pair<byte, uint16>, 8> children; //StackArray<Pair<child ID, child index>, ...
			HashTable<uint32, uint16> entities; //HashTable<component id, ...

			Node() {}
			Node(const AABB& aabb) : bound(aabb), children(Pair<byte, uint16>(-1, -1)) {}

			byte evaluateEntity(const AABB& entityAABB); //return 0 - no intersection, return 1 - intersection NO containment, return 2 - intersection and containment
		};

		RigidArray<Node, uint16> nodes;
		OctreeToHeightFieldLink* heightFieldLink = nullptr;
		decimal acceptableRadiusSq = decimal(0.0);
		byte depth = 0;

		void initialise(const AABB& bounds, OctreeToHeightFieldLink* ohl, const byte& inDepth);
		StackArray<uint16, 8> addEntity(const uint32& entityID, const AABB& entityAABB); //returns indicies at which nodes containing component are located
		void updateEntityDiscrete(const uint32& entityID, const AABB& entityAABB, StackArray<uint16, 8>& referenceNodes);
		void updateEntityContinous(const uint32& entityID, const AABB& entityAABB, StackArray<uint16, 8>& referenceNodes);
		bool isNodeEmpty(const uint16& index);
		void terminateNode(const uint16& index);
	};
}

#endif

