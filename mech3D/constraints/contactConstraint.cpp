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

#include"constraints.h"

#include"../physicsData.h"

namespace mech {

	ContactConstraint::ContactConstraint(PhysicsData* physicsData, const ContactManifold& manifold)
	{
		this->bodies[0] = &physicsData->physicsObjects[manifold.colliderID1.objectIndex].rigidBody;
		this->bodies[1] = isAValidIndex(manifold.colliderID2.objectIndex) ? &physicsData->physicsObjects[manifold.colliderID2.objectIndex].rigidBody : nullptr;

		this->impulseCacheID = manifold.ID;

		this->frictionCoefficient = manifold.properties1.frictionSqrt * manifold.properties2.frictionSqrt;
		decimal coeficientOfRestitution = manifold.properties2.restitution > manifold.properties1.restitution ? manifold.properties1.restitution : manifold.properties2.restitution;

		this->contactPointCount = manifold.numPoints;

		for (uint32 x = 0; x < this->contactPointCount; ++x) {

			this->contactData[x].ID = manifold.contactPoints[x].ID;

			StackArray<decimal, 2> invMass;
			StackArray<Vec3, 2> r;
			StackArray<Mat3x3, 2> invI;
			Vec3 deltaVelocity;
			for (uint32 y = 0; y < 2; ++y) if (this->bodies[y]) {

				invMass[y] = this->bodies[y]->invMass;
				r[y] = manifold.contactPoints[x].position[y] - this->bodies[y]->transform.position;
				invI[y] = this->bodies[y]->invInertiaTensor;

				deltaVelocity += this->bodies[y]->linearVelocity + crossProduct(this->bodies[y]->angularVelocity, r[y]) * (y == 0 ? -1 : 1);
			}

			this->contactData[x].normal = manifold.contactPoints[x].normal;
			this->contactData[x].tangent1 = getPerpendicularVector(manifold.contactPoints[x].normal);
			this->contactData[x].tangent2 = crossProduct(manifold.contactPoints[x].normal, this->contactData[x].tangent1);

			this->contactData[x].penetration = dotProduct(this->contactData[x].normal, manifold.contactPoints[x].position[1] - manifold.contactPoints[x].position[0]);

			decimal normalVelocity = mathABS(dotProduct(this->contactData[x].normal, deltaVelocity));
			decimal penetrationBias = coeficientOfRestitution * (normalVelocity > physicsData->minVelocityForRestitution ? normalVelocity : decimal(0.0));

			this->contactData[x].penetrationConstraint.initialise(this->contactData[x].normal, this->bodies, r, invI, invMass, penetrationBias);
			this->contactData[x].frictionConstraint1.initialise(this->contactData[x].tangent1, this->bodies, r, invI, invMass, decimal(0.0));
			this->contactData[x].frictionConstraint2.initialise(this->contactData[x].tangent2, this->bodies, r, invI, invMass, decimal(0.0));
		}
	}

	void ContactConstraint::warmStart(PhysicsData* physicsData)
	{
		ImpulseCache& impulseCache = physicsData->contactImpulseCache.insert(this->impulseCacheID)->second;
	
		if (impulseCache.retention != 0) {

			StackArray<Vec3, 2> deltaLinVel;
			StackArray<Vec3, 2> deltaAngVel;

			for (uint32 x = 0, len = impulseCache.impulses.size(); x < len; ++x) {

				Pair<uint32, ImpulseCache::Impulse>* impPtr = impulseCache.impulses.find(this->contactData[x].ID);

				if (impPtr) {

					StackArray<decimal, 2> invMass;
					for (uint32 y = 0; y < 2; ++y) if (this->bodies[y]) {
						invMass[y] = this->bodies[y]->invMass;
					}

					//friction1
					this->contactData[x].frictionConstraint1.totalLambda = dotProduct(this->contactData[x].tangent1, impPtr->second.frictionImpulse1);
					this->contactData[x].frictionConstraint1.warmStart(this->contactData[x].tangent1, deltaLinVel, deltaAngVel, invMass);

					//friction2
					this->contactData[x].frictionConstraint2.totalLambda = dotProduct(this->contactData[x].tangent2, impPtr->second.frictionImpulse2);
					this->contactData[x].frictionConstraint2.warmStart(this->contactData[x].tangent2, deltaLinVel, deltaAngVel, invMass);

					//penetration
					this->contactData[x].penetrationConstraint.totalLambda = impPtr->second.antiPenetrationImpulse;
					this->contactData[x].penetrationConstraint.warmStart(this->contactData[x].normal, deltaLinVel, deltaAngVel, invMass);
				}
			}

			for (uint32 y = 0; y < 2; ++y) if (this->bodies[y]) {
				this->bodies[y]->updateLinearAndAngularVelocity(deltaLinVel[y], deltaAngVel[y]);
			}
		}

		impulseCache.impulses.clear();
	}

	void ContactConstraint::solve(PhysicsData* physicsData, const decimal& baumgarteFactor, const bool& solvePosition, const bool& lastIteration)
	{
		for (uint32 x = 0; x < this->contactPointCount; ++x) {

			StackArray<decimal, 2> invMass;
			for (uint32 y = 0; y < 2; ++y) if (this->bodies[y]) {
				invMass[y] = this->bodies[y]->invMass;
			}

			decimal maxFriction = this->contactData[x].penetrationConstraint.totalLambda * this->frictionCoefficient;

			//friction1
			this->contactData[x].frictionConstraint1.solveVelocity(this->contactData[x].tangent1, this->bodies, invMass, -maxFriction, maxFriction);

			//friction2
			this->contactData[x].frictionConstraint2.solveVelocity(this->contactData[x].tangent2, this->bodies, invMass, -maxFriction, maxFriction);

			//penetration
			this->contactData[x].penetrationConstraint.solveVelocity(this->contactData[x].normal, this->bodies, invMass, decimal(0.0), decimalMAX);

			//position resolution
			if (solvePosition) {
				if (this->contactData[x].penetration < -physicsData->linearSlop) {
					this->contactData[x].penetrationConstraint.solvePosition(this->contactData[x].normal, this->bodies, invMass, baumgarteFactor, this->contactData[x].penetration);
				}
			}
		}

		if (lastIteration) {

			//cache impulses
			ImpulseCache& impulseCache = physicsData->contactImpulseCache.find(this->impulseCacheID)->second;
			for (uint32 x = 0; x < this->contactPointCount; ++x) {

				ImpulseCache::Impulse impulse;
				impulse.antiPenetrationImpulse = this->contactData[x].penetrationConstraint.totalLambda;
				impulse.frictionImpulse1 = this->contactData[x].tangent1 * this->contactData[x].frictionConstraint1.totalLambda;
				impulse.frictionImpulse2 = this->contactData[x].tangent2 * this->contactData[x].frictionConstraint2.totalLambda;

				impulseCache.impulses.pushBack(Pair<uint32, ImpulseCache::Impulse>(this->contactData[x].ID, impulse));
			}
			impulseCache.retention = physicsData->framesToRetainCache;
			
			this->bodies[0]->activate(physicsData);
		}
	}
}