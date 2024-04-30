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

#ifndef TIMEOTIMPACT_H
#define TIMEOFIMPACT_H

#include"physicsData.h"
#include"../geometry/algorithms/GJK.h"

namespace mech {

#define MAXIMUM_ITERATIONS 20

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class TOIState : byte { overlaping = 1 << 0, separated = 1 << 1 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TOIResult {
		decimal t = decimalMAX;
		TOIState state = TOIState::separated;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TimeOfImpact {

		struct DistanceResult {
			bool overlap = false;
			Vec3 closest1 = nanVEC3;
			Vec3 closest2 = nanVEC3;
		};

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		struct SeparationEvaluator {

			Vec3 axis;
			Vec3 support1 = nanVEC3;
			Vec3 support2 = nanVEC3;
			Transform3DRange transform1;
			Transform3DRange transform2;
			TimeOfImpact* timeOfImpact = nullptr;
			Vec3(TimeOfImpact::** supportPtrs) (const ColliderIdentifier&, const Vec3&) = nullptr;

			virtual DistanceResult distance(const decimal& t) = 0;
			virtual void setSupportPoints(const Vec3& axis) = 0;

			void renewAxis(const DistanceResult& d)
			{
				this->axis = normalise(d.closest2 - d.closest1);
			}

			decimal calculateSeparation(const decimal& t)
			{
				Transform3D tA = this->transform1.interpolate(t);
				Vec3 a = tA.orientation * this->axis;
				this->setSupportPoints(a);
				return dotProduct(this->transform2.interpolate(t) * this->support2 - tA * this->support1, a);
			}

			decimal reCalculateSeparation(const decimal & t)
			{
				Transform3D tA = this->transform1.interpolate(t);
				return dotProduct(this->transform2.interpolate(t) * this->support2 - tA * this->support1, tA.orientation * this->axis);
			}

		};

		struct EvaluatorCommon : public SeparationEvaluator {

			ColliderIdentifier identifier1;
			ColliderIdentifier identifier2;
			DistanceResult(TimeOfImpact::** distancePtrs) (const ColliderIdentifier&, const ColliderIdentifier&, const Transform3DRange&, const Transform3DRange&, const decimal&) = nullptr;

			DistanceResult distance(const decimal& t) override
			{
				return (timeOfImpact->*distancePtrs[(uint32)(identifier1.type) + ((uint32)(identifier2.type) * 3)])(identifier1, identifier2, transform1, transform2, t);
			}

			void setSupportPoints(const Vec3 & axis) override
			{
				this->support1 = (timeOfImpact->*supportPtrs[(uint32)(identifier1.type)])(identifier1, axis);
				this->support2 = (timeOfImpact->*supportPtrs[(uint32)(identifier2.type)])(identifier2, -axis);
			}
		};

		struct EvaluatorTriangle : public SeparationEvaluator {

			ColliderIdentifier identifier1;
			Triangle triangle;
			DistanceResult(TimeOfImpact::** triangleDistancePtrs) (const ColliderIdentifier&, const Triangle&, const Transform3DRange&, const decimal&) = nullptr;

			DistanceResult distance(const decimal& t) override
			{
				return (timeOfImpact->*triangleDistancePtrs[(uint32)(identifier1.type)])(identifier1, triangle, transform1, t);
			}

			void setSupportPoints(const Vec3& axis) override
			{
				this->support1 = (timeOfImpact->*supportPtrs[(uint32)(identifier1.type)])(identifier1, axis);
				this->support2 = triangle.getSupportPoint(-axis);
			}
		};

		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		PhysicsData* physicsData = nullptr;
		
		bool (TimeOfImpact::* overlapTestPtrs[3]) (const AABB&, const ColliderIdentifier&) = {};
		DistanceResult (TimeOfImpact::* distancePtrs[9]) (const ColliderIdentifier&, const ColliderIdentifier&, const Transform3DRange&, const Transform3DRange&, const decimal&) = {};
		DistanceResult (TimeOfImpact::* triangleDistancePtrs[3]) (const ColliderIdentifier&, const Triangle&, const Transform3DRange&, const decimal&) = {};
		Vec3 (TimeOfImpact::* supportPtrs[3]) (const ColliderIdentifier&, const Vec3&) = {};

		TimeOfImpact()
		{
			this->overlapTestPtrs[0] = &TimeOfImpact::overlapConvexHull;
			this->overlapTestPtrs[1] = &TimeOfImpact::overlapSphere;
			this->overlapTestPtrs[2] = &TimeOfImpact::overlapCapsule;

			this->distancePtrs[0] = &TimeOfImpact::convexHullVsConvexHullDistance;
			this->distancePtrs[1] = &TimeOfImpact::sphereVsConvexHullDistance;
			this->distancePtrs[2] = &TimeOfImpact::capsuleVsConvexHullDistance;
			this->distancePtrs[3] = &TimeOfImpact::convexHullVsSphereDistance;
			this->distancePtrs[4] = &TimeOfImpact::sphereVsSphereDistance;
			this->distancePtrs[5] = &TimeOfImpact::capsuleVsSphereDistance;
			this->distancePtrs[6] = &TimeOfImpact::convexHullVsCapsuleDistance;
			this->distancePtrs[7] = &TimeOfImpact::sphereVsCapsuleDistance;
			this->distancePtrs[8] = &TimeOfImpact::capsuleVsCapsuleDistance;

			this->triangleDistancePtrs[0] = &TimeOfImpact::convexHullVsTriangleDistance;
			this->triangleDistancePtrs[1] = &TimeOfImpact::sphereVsTriangleDistance;
			this->triangleDistancePtrs[2] = &TimeOfImpact::capsuleVsTriangleDistance;

			this->supportPtrs[0] = &TimeOfImpact::convexHullSupport;
			this->supportPtrs[1] = &TimeOfImpact::sphereSupport;
			this->supportPtrs[2] = &TimeOfImpact::capsuleSupport;
		}

		TOIResult toiFunction(SeparationEvaluator* sEvaluator, const decimal& tolerance)
		{
			BEGIN_PROFILE("TimeOfImpact::toiFunction");

			TOIResult result;

			decimal t1 = decimal(0.0);

			byte toiIterations = 0;
			while (toiIterations < MAXIMUM_ITERATIONS) {

				DistanceResult r = sEvaluator->distance(t1);

				if (r.overlap == true || magnitudeSq(r.closest2 - r.closest1) <= tolerance) {
					result.state = TOIState::overlaping;
					result.t = t1;
					break;
				}

				bool terminate = false;

				sEvaluator->renewAxis(r);

				decimal t2 = decimal(1.0);
				byte deepestPointIterations = 0;
				while (deepestPointIterations < MAXIMUM_ITERATIONS) {

					decimal s2 = sEvaluator->calculateSeparation(t2);

					if (s2 > tolerance) {
						result.state = TOIState::separated;
						result.t = t2;
						terminate = true;
						break;
					}

					if (s2 > -tolerance) {
						t1 = t2;
						break;
					}

					decimal s1 = sEvaluator->reCalculateSeparation(t1);

					if (s1 < -tolerance || s1 <= tolerance) {
						result.state = TOIState::overlaping;
						result.t = t1;
						terminate = true;
						break;
					}

					decimal rootT1 = t1;
					decimal rootT2 = t2;
					byte rootIterations = 0;
					while (rootIterations < MAXIMUM_ITERATIONS) {

						decimal t;
						if (rootIterations & 1) {
							t = rootT1 + (-s1) * (rootT2 - rootT1) / (s2 - s1);
						}
						else {
							t = decimal(0.5) * (rootT1 + rootT2);
						}

						decimal s = sEvaluator->reCalculateSeparation(t);

						if (mathABS(s) < tolerance) {
							t2 = t;
							break;
						}

						if (s > decimal(0.0)) {
							rootT1 = t;
							s1 = s;
						}
						else {
							rootT2 = t;
							s2 = s;
						}

						++rootIterations;
					}

					ASSERT(rootIterations < MAXIMUM_ITERATIONS, "root finder has failed!!");

					++deepestPointIterations;
				}

				if (terminate == true) break;
				++toiIterations;
			}

			ASSERT(toiIterations < MAXIMUM_ITERATIONS, "toiFunction has failed!!");

			END_PROFILE;
			return result;
		}

		TOIResult toi(const AABB& aabbCast, const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2)
		{
			if ((this->*overlapTestPtrs[(uint32)(identifier2.type)])(aabbCast, identifier2)) {
				
				EvaluatorCommon evaluator;

				evaluator.identifier1 = identifier1;
				evaluator.identifier2 = identifier2;
				evaluator.transform1 = transform1;
				evaluator.transform2 = transform2;
				evaluator.timeOfImpact = this;
				evaluator.distancePtrs = this->distancePtrs;
				evaluator.supportPtrs = this->supportPtrs;

				return this->toiFunction(&evaluator, decimal(0.01));
			}

			return TOIResult();
		}

		TOIResult toi(const ColliderIdentifier& identifier, const Triangle& triangle, const Transform3DRange& transform)
		{
			EvaluatorTriangle evaluator;

			evaluator.triangle = triangle;
			evaluator.identifier1 = identifier;
			evaluator.transform1 = transform;
			evaluator.timeOfImpact = this;
			evaluator.supportPtrs = this->supportPtrs;
			evaluator.triangleDistancePtrs = this->triangleDistancePtrs;

			return this->toiFunction(&evaluator, decimal(0.01));
		}

		bool overlapConvexHull(const AABB& aabbCast, const ColliderIdentifier& identifier)
		{
			return aabbCast.intersects(this->physicsData->convexHullColliders[identifier.colliderIndex].bound);
		}

		bool overlapSphere(const AABB& aabbCast, const ColliderIdentifier& identifier)
		{
			return aabbCast.intersects(this->physicsData->sphereColliders[identifier.colliderIndex].bound);
		}

		bool overlapCapsule(const AABB& aabbCast, const ColliderIdentifier& identifier)
		{
			return aabbCast.intersects(this->physicsData->capsuleColliders[identifier.colliderIndex].bound);
		}

		DistanceResult convexHullVsConvexHullDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			GJKDistanceResult r = GJKDistance(this->physicsData->convexHullColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t)), this->physicsData->convexHullColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t)));

			DistanceResult s;
			s.closest1 = r.closest1;
			s.closest2 = r.closest2;
			s.overlap = r.overlap;
			return s;
		}

		DistanceResult convexHullVsSphereDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Sphere sphere = this->physicsData->sphereColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			DistanceResult s;
			s.closest1 = this->physicsData->convexHullColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t)).closestPoint(sphere.center);
			s.closest2 = sphere.closestPoint(s.closest1);
			s.overlap = magnitudeSq(s.closest1 - sphere.center) < square(sphere.radius);
			return s;
		}

		DistanceResult convexHullVsCapsuleDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Capsule capsule = this->physicsData->capsuleColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			DistanceResult s;
			s.closest1 = this->physicsData->convexHullColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t)).closestPoint(capsule.capsuleLine);
			s.closest2 = capsule.closestPoint(s.closest1);
			s.overlap = magnitudeSq(s.closest1 - capsule.capsuleLine.closestPoint(s.closest1)) < square(capsule.radius);
			return s;
		}

		DistanceResult sphereVsConvexHullDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Sphere sphere = this->physicsData->sphereColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));

			DistanceResult s;
			s.closest2 = this->physicsData->convexHullColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t)).closestPoint(sphere.center);
			s.closest1 = sphere.closestPoint(s.closest2);
			s.overlap = magnitudeSq(s.closest2 - sphere.center) < square(sphere.radius);
			return s;
		}

		DistanceResult sphereVsSphereDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Sphere sphere1 = this->physicsData->sphereColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));
			Sphere sphere2 = this->physicsData->sphereColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			DistanceResult s;
			s.closest1 = sphere1.closestPoint(sphere2.center);
			s.closest2 = sphere2.closestPoint(s.closest1);
			s.overlap = magnitudeSq(sphere1.center - sphere2.center) < square(sphere1.radius + sphere2.radius);
			return s;
		}

		DistanceResult sphereVsCapsuleDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Sphere sphere = this->physicsData->sphereColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));
			Capsule capsule = this->physicsData->capsuleColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			DistanceResult s;
			s.closest2 = capsule.closestPoint(sphere.center);
			s.closest1 = sphere.closestPoint(s.closest2);
			s.overlap = magnitudeSq(sphere.center - capsule.capsuleLine.closestPoint(sphere.center)) < square(sphere.radius + capsule.radius);
			return s;
		}

		DistanceResult capsuleVsConvexHullDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Capsule capsule = this->physicsData->capsuleColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));

			DistanceResult s;
			s.closest2 = this->physicsData->convexHullColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t)).closestPoint(capsule.capsuleLine);
			s.closest1 = capsule.closestPoint(s.closest2);
			s.overlap = magnitudeSq(s.closest2 - capsule.capsuleLine.closestPoint(s.closest2)) < square(capsule.radius);
			return s;
		}

		DistanceResult capsuleVsSphereDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Capsule capsule = this->physicsData->capsuleColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));
			Sphere sphere = this->physicsData->sphereColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			DistanceResult s;
			s.closest1 = capsule.closestPoint(sphere.center);
			s.closest2 = sphere.closestPoint(s.closest1);
			s.overlap = magnitudeSq(sphere.center - capsule.capsuleLine.closestPoint(sphere.center)) < square(sphere.radius + capsule.radius);
			return s;
		}

		DistanceResult capsuleVsCapsuleDistance(const ColliderIdentifier& identifier1, const ColliderIdentifier& identifier2, const Transform3DRange& transform1, const Transform3DRange& transform2, const decimal& t)
		{
			Capsule capsule1 = this->physicsData->capsuleColliders[identifier1.colliderIndex].collider.transformed(transform1.interpolate(t));
			Capsule capsule2 = this->physicsData->capsuleColliders[identifier2.colliderIndex].collider.transformed(transform2.interpolate(t));

			Vec3 c = capsule2.capsuleLine.closestPoint(capsule1.capsuleLine);

			DistanceResult s;
			s.closest1 = capsule1.closestPoint(c);
			s.closest2 = capsule1.closestPoint(s.closest1);
			s.overlap = magnitudeSq(capsule1.capsuleLine.closestPoint(c) - c) < square(capsule1.radius + capsule2.radius);
			return s;
		}

		DistanceResult convexHullVsTriangleDistance(const ColliderIdentifier& identifier, const Triangle& triangle, const Transform3DRange& transform, const decimal& t)
		{
			GJKDistanceResult r = GJKDistance(this->physicsData->convexHullColliders[identifier.colliderIndex].collider.transformed(transform.interpolate(t)), triangle);

			DistanceResult s;
			s.closest1 = r.closest1;
			s.closest2 = r.closest2;
			s.overlap = r.overlap;
			return s;
		}

		DistanceResult sphereVsTriangleDistance(const ColliderIdentifier& identifier, const Triangle& triangle, const Transform3DRange& transform, const decimal& t)
		{
			Sphere sphere = this->physicsData->sphereColliders[identifier.colliderIndex].collider.transformed(transform.interpolate(t));

			DistanceResult s;
			s.closest2 = triangle.closestPoint(sphere.center);
			s.closest1 = sphere.closestPoint(s.closest2);
			s.overlap = magnitudeSq(s.closest2 - sphere.center) < square(sphere.radius);
			return s;
		}

		DistanceResult capsuleVsTriangleDistance(const ColliderIdentifier& identifier, const Triangle& triangle, const Transform3DRange& transform, const decimal& t)
		{
			Capsule capsule = this->physicsData->capsuleColliders[identifier.colliderIndex].collider.transformed(transform.interpolate(t));

			DistanceResult s;
			s.closest2 = triangle.closestPoint(capsule.capsuleLine);
			s.closest1 = capsule.closestPoint(s.closest2);
			s.overlap = magnitudeSq(s.closest2 - capsule.capsuleLine.closestPoint(s.closest2)) < square(capsule.radius);
			return s;
		}

		Vec3 convexHullSupport(const ColliderIdentifier& identifier, const Vec3& direction)
		{
			return this->physicsData->convexHullColliders[identifier.colliderIndex].collider.getSupportPoint(direction);
		}

		Vec3 sphereSupport(const ColliderIdentifier& identifier, const Vec3& direction)
		{
			return this->physicsData->sphereColliders[identifier.colliderIndex].collider.getSupportPoint(direction);
		}

		Vec3 capsuleSupport(const ColliderIdentifier& identifier, const Vec3& direction)
		{
			return this->physicsData->capsuleColliders[identifier.colliderIndex].collider.getSupportPoint(direction);
		}
	};
}

#endif


