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

#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

#include"physicsData.h"

namespace mech {

	struct ConstraintSolver {
		
		PhysicsData* physicsData = nullptr;

		ConstraintSolver() {}
		ConstraintSolver(const ConstraintSolver&) = delete;
		ConstraintSolver& operator=(const ConstraintSolver&) = delete;

		void solve(const decimal& deltaTime)
		{
			BEGIN_PROFILE("ConstraintSolver::solve");

			for (byte iteration = 0; iteration < this->physicsData->settings.velocityIterations; ++iteration) {

				bool firstIteration = iteration == 0;
				bool solvePosition = iteration >= (this->physicsData->settings.velocityIterations - this->physicsData->settings.positionIterations);

				//contact constraints
				for (uint32 x = 0, len = this->physicsData->contactConstraints.size(); x < len; ++x) {
					if (firstIteration) {
						this->physicsData->contactConstraints[x].warmStart(physicsData);
					}
					bool lastIteration = iteration == this->physicsData->settings.velocityIterations - 1;
					this->physicsData->contactConstraints[x].solve(physicsData, this->physicsData->settings.baumgarteFactor, this->physicsData->settings.linearSlop, solvePosition, lastIteration);
				}

				//hinge constraints
				for (auto it = this->physicsData->hingeConstraints.begin(), end = this->physicsData->hingeConstraints.end(); it != end; ++it) {
					
					if (firstIteration) {

						if (it.data().isValid(physicsData) == false) {
							this->physicsData->hingeConstraints.eraseDataAtIndex(it.index());
							continue;
						}

						it.data().warmStart(physicsData);
					}
					it.data().solve(physicsData, this->physicsData->settings.baumgarteFactor, solvePosition);
				}

				//cone constraints
				for (auto it = this->physicsData->coneConstraints.begin(), end = this->physicsData->coneConstraints.end(); it != end; ++it) {

					if (firstIteration) {

						if (it.data().isValid(physicsData) == false) {
							this->physicsData->coneConstraints.eraseDataAtIndex(it.index());
							continue;
						}

						it.data().warmStart(physicsData);
					}
					it.data().solve(physicsData, this->physicsData->settings.baumgarteFactor, solvePosition);
				}

				//motor constraints
				for (auto it = this->physicsData->motorConstraints.begin(), end = this->physicsData->motorConstraints.end(); it != end; ++it) {

					if (firstIteration) {

						if (it.data().isValid(physicsData) == false) {
							this->physicsData->motorConstraints.eraseDataAtIndex(it.index());
							continue;
						}

						it.data().warmStart(physicsData);
					}
					it.data().solve(physicsData, deltaTime, this->physicsData->settings.baumgarteFactor, solvePosition);
				}
			}

			END_PROFILE;
		}

		void add(const ContactManifold& manifold, const uint32& objectIndex1, const uint32& objectIndex2)
		{
			this->physicsData->contactConstraints.pushBack(ContactConstraint(this->physicsData, manifold, objectIndex1, objectIndex2));
		}

		void add(const HingeConstraint::Parameters& parameters)
		{
			if (parameters.disableCollisions == true) {
				this->physicsData->physicsObjects[this->physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex].diableCollision(this->physicsData, parameters.colliderID2);
			}
			this->physicsData->hingeConstraints.insert(HingeConstraint(this->physicsData, parameters));
		}

		void add(const ConeConstraint::Parameters& parameters)
		{
			if (parameters.disableCollisions == true) {
				this->physicsData->physicsObjects[this->physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex].diableCollision(this->physicsData, parameters.colliderID2);
			}
			this->physicsData->coneConstraints.insert(ConeConstraint(this->physicsData, parameters));
		}

		void add(const MotorConstraint::Parameters& parameters)
		{
			if (parameters.disableCollisions == true) {
				this->physicsData->physicsObjects[this->physicsData->colliderIdentifiers[parameters.colliderID1].objectIndex].diableCollision(this->physicsData, parameters.colliderID2);
			}
			this->physicsData->motorConstraints.insert(MotorConstraint(this->physicsData, parameters));
		}
	};
}

#endif

