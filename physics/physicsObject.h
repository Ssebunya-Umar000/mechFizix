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

#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

#include"rigidBody.h"
#include"../containers/stackArray.h"

namespace mech {

	struct PhysicsObject {

		RigidBody rigidBody;
		StackArray<uint16, 8> nodesIntersected; //StackArray<node index, ...
		StackArray<uint32, 4> disabledCollisions; //StackArray<collider ID, ...
		uint32 islandIndex = -1;

		PhysicsObject() {}

		void initialise(PhysicsData* physicsData, const uint32& id, const decimal& mass, const Mat3x3& tensor, const Transform3D& offset);
		void diableCollision(PhysicsData* physicsData, const uint32& otherID);
		void addToIsland(PhysicsData* physicsData, const uint32& otherID);
	};
}

#endif

