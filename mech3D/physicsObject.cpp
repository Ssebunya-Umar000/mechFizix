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

#include"physicsObject.h"

#include"physicsData.h"

namespace mech {

	void PhysicsObject::diableCollision(PhysicsData* physicsData, const uint32& otherIndex)
	{
		PhysicsObject& other = physicsData->physicsObjects[otherIndex];

		this->disabledCollisions.pushBack(other.colliderID);
		other.disabledCollisions.pushBack(this->colliderID);

		this->addToIsland(physicsData, otherIndex);
	}

	void PhysicsObject::addToIsland(PhysicsData* physicsData, const uint32& otherIndex)
	{
		PhysicsObject& otherObject = physicsData->physicsObjects[otherIndex];

		if (isAValidIndex(this->rigidBody.islandIndex) == false && isAValidIndex(otherObject.rigidBody.islandIndex) == false) {

			this->rigidBody.islandIndex = otherObject.rigidBody.islandIndex = physicsData->islands.insert(Island());
			physicsData->islands[this->rigidBody.islandIndex].insert(Pair<uint32, ColliderIdentifier>(this->colliderID, this->getIdentifier()));
			physicsData->islands[otherObject.rigidBody.islandIndex].insert(Pair<uint32, ColliderIdentifier>(otherObject.colliderID, otherObject.getIdentifier()));
		}
		else if (this->rigidBody.islandIndex != otherObject.rigidBody.islandIndex) {

			if (isAValidIndex(this->rigidBody.islandIndex) == true && isAValidIndex(otherObject.rigidBody.islandIndex) == true) {

				uint32 index = otherObject.rigidBody.islandIndex;
				for (auto it = physicsData->islands[otherObject.rigidBody.islandIndex].begin(), end = physicsData->islands[otherObject.rigidBody.islandIndex].end(); it != end; ++it) {
					physicsData->islands[this->rigidBody.islandIndex].insert(it.data());
					physicsData->physicsObjects[it.data().second.objectIndex].rigidBody.islandIndex = this->rigidBody.islandIndex;
				}
				physicsData->islands.eraseDataAtIndex(index);
			}
			else if (isAValidIndex(this->rigidBody.islandIndex) == true) {
				physicsData->islands[this->rigidBody.islandIndex].insert(Pair<uint32, ColliderIdentifier>(otherObject.colliderID, otherObject.getIdentifier()));
				otherObject.rigidBody.islandIndex = this->rigidBody.islandIndex;
			}
			else {
				physicsData->islands[otherObject.rigidBody.islandIndex].insert(Pair<uint32, ColliderIdentifier>(this->colliderID, this->getIdentifier()));
				this->rigidBody.islandIndex = otherObject.rigidBody.islandIndex;
			}
		}
	}

	ColliderIdentifier PhysicsObject::getIdentifier()
	{
		ColliderIdentifier id;
		id.colliderIndex = this->rigidBody.colliderIndex;
		id.ID = this->colliderID;
		id.objectIndex = this->objectIndex;
		id.type = this->rigidBody.colliderType;

		return id;
	}

	Mat3x3 calculateTensor(const decimal& density, const Capsule* capsule)
	{
		Mat3x3 tensor;

		decimal height = capsule->capsuleLine.getLength();
		decimal rSq = capsule->radius * capsule->radius;

		decimal cylinderMass = mathPI * height * rSq * density;
		decimal sphereMass = mathPI * decimal(2.0) * (decimal(1.0) / decimal(3.0)) * rSq * capsule->radius * density;

		tensor.rowXcol(1, 1) = rSq * cylinderMass * decimal(0.5);
		tensor.rowXcol(0, 0) = tensor.rowXcol(2, 2) = tensor.rowXcol(1, 1) * decimal(0.5) + cylinderMass * height * height * (decimal(1.0) / decimal(12.0));

		decimal temp0 = sphereMass * decimal(2.0) * rSq / decimal(5.0); 
		tensor.rowXcol(1, 1) += temp0 * decimal(2.0);

		decimal temp1 = height * decimal(0.5); 
		decimal temp2 = temp0 + sphereMass * (temp1 * temp1 + decimal(3.0) * (decimal(1.0) / decimal(8.0)) * height * capsule->radius);

		tensor.rowXcol(0, 0) += temp2 * decimal(2.0); 
		tensor.rowXcol(2, 2) += temp2 * decimal(2.0);

		return tensor;
	}

	//solid sphere
	Mat3x3 calculateTensor(const decimal& mass, const Sphere* sphere)
	{
		Mat3x3 tensor;

		decimal v = (decimal(2.0) / decimal(5.0)) * mass * sphere->radius * sphere->radius;

		tensor.rowXcol(0, 0) = v;
		tensor.rowXcol(1, 1) = v;
		tensor.rowXcol(2, 2) = v;

		return tensor;
	}

	Mat3x3 calculateTensor(const decimal& mass, const DynamicArray<Vec3, uint32>& points)
	{
		Mat3x3 tensor;
		for (uint32 x = 0, len = points.size(); x < len; ++x) {
			
			tensor.rowXcol(0, 0) += (points[x].y * points[x].y + points[x].z * points[x].z);
			tensor.rowXcol(1, 1) += (points[x].x * points[x].x + points[x].z * points[x].z);
			tensor.rowXcol(2, 2) += (points[x].x * points[x].x + points[x].y * points[x].y);
			
			tensor.rowXcol(0, 1) -= (points[x].x * points[x].y);
			tensor.rowXcol(0, 2) -= (points[x].x * points[x].z);
			tensor.rowXcol(1, 2) -= (points[x].y * points[x].z);
		}

		return tensor * mass;
	}

	void PhysicsObject::initialise(PhysicsData* physicsData, const ColliderIdentifier& identifier, const decimal& density, const Vec3& pos)
	{
		this->colliderID = identifier.ID;
		this->objectIndex = identifier.objectIndex;
		this->rigidBody.colliderType = identifier.type;
		this->rigidBody.colliderIndex = identifier.colliderIndex;
		this->rigidBody.motion = physicsData->maxMotion;

		this->disabledCollisions.pushBack(this->colliderID);

		decimal mass = 0;

		if (this->rigidBody.colliderType == ColliderType::convexHull) {

			physicsData->convexHullColliders[this->rigidBody.colliderIndex].transform(translationMatrix(pos));
			mass = density * physicsData->convexHullColliders[this->rigidBody.colliderIndex].getVolume();
			this->rigidBody.setTensor(calculateTensor(mass, physicsData->convexHullColliders[this->rigidBody.colliderIndex].collider.vertices.toDynamicArray()));
		}
		if (this->rigidBody.colliderType == ColliderType::sphere) {

			physicsData->sphereColliders[this->rigidBody.colliderIndex].transform(translationMatrix(pos));
			mass = density * physicsData->sphereColliders[this->rigidBody.colliderIndex].getVolume();
			this->rigidBody.setTensor(calculateTensor(mass, &physicsData->sphereColliders[this->rigidBody.colliderIndex].collider));
		}
		if (this->rigidBody.colliderType == ColliderType::capsule) {

			physicsData->capsuleColliders[this->rigidBody.colliderIndex].transform(translationMatrix(pos));
			mass = density * physicsData->capsuleColliders[this->rigidBody.colliderIndex].getVolume();
			this->rigidBody.setTensor(calculateTensor(density, &physicsData->capsuleColliders[this->rigidBody.colliderIndex].collider));
		}

		this->rigidBody.setMass(mass);
		this->rigidBody.setPosition(pos);
	}
}
