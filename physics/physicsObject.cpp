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

#include"physicsObject.h"

#include"physicsData.h"

namespace mech {

	void PhysicsObject::diableCollision(PhysicsData* physicsData, const uint32& otherID)
	{
		this->disabledCollisions.pushBack(otherID);
		physicsData->physicsObjects[physicsData->colliderIdentifiers[otherID].objectIndex].disabledCollisions.pushBack(this->rigidBody.colliderID);
	}

	void PhysicsObject::addToIsland(PhysicsData* physicsData, const uint32& otherID)
	{
		PhysicsObject& otherObject = physicsData->physicsObjects[physicsData->colliderIdentifiers[otherID].objectIndex];

		if (isAValidIndex(this->islandIndex) == false && isAValidIndex(otherObject.islandIndex) == false) {

			this->islandIndex = otherObject.islandIndex = physicsData->islands.insert(AVLTree<uint32, uint32>());
			physicsData->islands[this->islandIndex].insert(this->rigidBody.colliderID);
			physicsData->islands[otherObject.islandIndex].insert(otherObject.rigidBody.colliderID);
		}
		else if (this->islandIndex != otherObject.islandIndex) {

			if (isAValidIndex(this->islandIndex) == true && isAValidIndex(otherObject.islandIndex) == true) {

				uint32 index = otherObject.islandIndex;
				for (auto it = physicsData->islands[otherObject.islandIndex].begin(), end = physicsData->islands[otherObject.islandIndex].end(); it != end; ++it) {
					physicsData->islands[this->islandIndex].insert(it.data());
					physicsData->physicsObjects[physicsData->colliderIdentifiers[it.data()].objectIndex].islandIndex = this->islandIndex;
				}
				physicsData->islands.eraseDataAtIndex(index);
			}
			else if (isAValidIndex(this->islandIndex) == true) {
				physicsData->islands[this->islandIndex].insert(otherObject.rigidBody.colliderID);
				otherObject.islandIndex = this->islandIndex;
			}
			else {
				physicsData->islands[otherObject.islandIndex].insert(this->rigidBody.colliderID);
				this->islandIndex = otherObject.islandIndex;
			}
		}
	}

	void PhysicsObject::initialise(PhysicsData* physicsData, const const uint32& id, const decimal& mass, const Mat3x3& tensor, const Transform3D& offset)
	{
		this->rigidBody.colliderID = id;

		this->rigidBody.setTensor(tensor);
		this->rigidBody.setMass(mass);
		this->rigidBody.setTransform(offset);

		this->disabledCollisions.pushBack(id);
	}
}
