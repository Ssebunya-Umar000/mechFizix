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

//https://t.me/mechFizix - Telegram (if you have a question, remark or centerOfMassplaint)
//@ssebunya_umar - X(twitter)

#ifndef COLLIDER_H
#define COLLIDER_H

#include"../../math/transform.h"
#include"../../geometry/aabb.h"
#include"../../geometry/convexHull.h"
#include"../../geometry/sphere.h"
#include"../../geometry/capsule.h"
#include"../../geometry/polygon.h"
#include"../../geometry/triangleMesh.h"
#include"../heightField.h"
#include"../material.h"

namespace mech {

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class ColliderType : byte { convexHull = 0, sphere = 1, capsule = 2, compound = 3, triangleMesh = 4, heightField = 5, noType = 6 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class ColliderMotionState : byte { motionless = 0, dynamic = 1 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ColliderIdentifier {
		PhysicsMaterial material;
		uint32 colliderID = -1;
		uint32 colliderIndex = -1;
		uint32 objectIndex = -1;
		ColliderType type = ColliderType::noType;
		ColliderMotionState state = ColliderMotionState::motionless;

		ColliderIdentifier() {}
		ColliderIdentifier(const ColliderType& t) : type(t) {}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ConvexHullCollider {

		ConvexHull collider;
		AABB bound = AABB(nanVEC3, nanVEC3);
		Vec3 centerOfMass;
		decimal convexRadius = decimalNAN;

		ConvexHullCollider() {}
		ConvexHullCollider(const ConvexHull& convexHull) : collider(convexHull)
		{
			this->bound = this->collider.toAABB();
			this->centerOfMass = this->collider.getCentroid();
			this->convexRadius = magnitude(this->bound.getCenter() - this->bound.max);
		}

		void transform(const Transform3D& t)
		{
			this->centerOfMass = t * this->centerOfMass;
			this->bound.max = this->centerOfMass;
			this->bound.min = this->centerOfMass;

			for (uint32 i = 0, len = this->collider.vertices.size(); i < len; ++i) {

				this->collider.vertices[i] = t * this->collider.vertices[i];

				this->bound.max = maxVec(this->collider.vertices[i], this->bound.max);
				this->bound.min = minVec(this->collider.vertices[i], this->bound.min);
			}
		}

		decimal getVolume()
		{
			decimal volume = decimal(0.0);
			for (uint32 x = 0, len = this->collider.halfEdgeMesh.faces.size(); x < len; ++x) {

				Polygon face = this->collider.getFacePolygon(x);
				volume += dotProduct(face.vertices[0], face.getNormal()) * face.calculateArea();
			}
			return mathABS(volume) / decimal(3.0);
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct SphereCollider {

		Sphere collider;
		AABB bound = AABB(nanVEC3, nanVEC3);

		SphereCollider() {}
		SphereCollider(const Sphere& sphere) : collider(sphere)
		{
			this->bound = this->collider.toAABB();
		}

		void transform(const Transform3D& t)
		{
			this->collider.transform(t);
			this->bound = this->collider.toAABB();
		}

		decimal getVolume() { return decimal(4.0) * mathPI * this->collider.radius * this->collider.radius * this->collider.radius / decimal(3.0); } 
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct CapsuleCollider {

		Capsule collider;
		AABB bound = AABB(nanVEC3, nanVEC3);
		Vec3 centerOfMass;
		decimal convexRadius = decimalNAN;

		CapsuleCollider() {}
		CapsuleCollider(const Capsule& capsule) : collider(capsule)
		{
			this->bound = this->collider.toAABB();
			this->centerOfMass = this->collider.capsuleLine.getCenter();
			this->convexRadius = magnitude(this->bound.getCenter() - this->bound.max);
		}

		void transform(const Transform3D& t)
		{
			this->collider.transform(t);
			this->bound = this->collider.toAABB();
			this->centerOfMass = t * this->centerOfMass;
		}

		decimal getVolume() { return mathPI * this->collider.radius * this->collider.radius * this->collider.capsuleLine.getLength() + decimal(4.0) * mathPI * this->collider.radius * this->collider.radius * this->collider.radius / decimal(3.0); } 
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct CompoundCollider {

		HybridArray<uint32, 4, byte> components;
		AABB bound = AABB(nanVEC3, nanVEC3);
		decimal convexRadius = decimalNAN;

		CompoundCollider() {}
		CompoundCollider(const uint32& id) {}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TriangleMeshCollider {

		TriangleMesh collider;
		AABB bound = AABB(nanVEC3, nanVEC3);

		TriangleMeshCollider() {}
		TriangleMeshCollider(const TriangleMesh& mesh) : collider(mesh)
		{
			this->bound = this->collider.bvh.nodes[this->collider.bvh.parentNode].bound;
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HeightFieldCollider {
		HeightField collider;
	};
}

#endif
