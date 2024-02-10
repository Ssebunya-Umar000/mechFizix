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

#ifndef OCTREE_H
#define OCTREE_H

#include"geometry/aabb.h"
#include"collision/collider.h"
#include"../containers/rigidArray.h"
#include"../containers/hashTable.h"

namespace mech {

	struct PhysicsData;

	struct Octree  {

		struct Node {

			byte key = 255;
			uint16 parent = -1;
			AABB bound;
			StackArray<Pair<byte, uint16>, 8> children; //StackArray<Pair<child ID, child index>, ...
			HashTable<Pair<uint32, ColliderIdentifier>, uint16> dynamicColliders; //HashTable<Pair<collider id, identifier>, ...
			DynamicArray<Pair<uint32, ColliderIdentifier>, uint16> staticColliders; //DynamicArray<Pair<collider id, identifier>, ...

			Node() {}
			Node(const AABB& aabb) : bound(aabb), children(Pair<byte, uint16>(-1, -1)) {}

			bool intersects(PhysicsData* physicsData, const ColliderIdentifier& colliderID);
		};

		RigidArray<Node, uint16> nodes;
		uint16 parentNode = 0;
		byte depth = 0;
		bool initialised = false;

		Octree() {}

		void initialise(const AABB& bounds, const byte& inDepth);
		void add(PhysicsData* physicsData, const ColliderIdentifier& colliderID);
		void repositionCollider(PhysicsData* physicsData, const ColliderIdentifier& colliderID);
		void terminate(const uint16& index);
	};
}

#endif

