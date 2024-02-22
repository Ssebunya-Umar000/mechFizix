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

#ifndef COLLIDER_H
#define COLLIDER_H

#include"../geometry/aabb.h"
#include"../geometry/convexHull.h"
#include"../geometry/sphere.h"
#include"../geometry/capsule.h"
#include"../geometry/polygon.h"
#include"../geometry/triangleMesh.h"
#include"../heightField.h"
#include"../../core/material.h"
#include"../../math/transform.h"

namespace mech {

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class ColliderType : byte { convexHull = 0, sphere = 1, capsule = 2, triangleMesh = 3, heightField = 4, noType = 5 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ColliderIdentifier {
		uint32 ID = -1;
		uint32 colliderIndex = -1;
		uint32 objectIndex = -1;
		ColliderType type = ColliderType::noType;

		ColliderIdentifier() {}
		ColliderIdentifier(const ColliderType& t) : type(t)
		{ 
			static uint32 colliderCount = 0;
			this->ID = colliderCount;
			++colliderCount;
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct ConvexHullCollider {

		AABB bound = AABB(nanVEC3, nanVEC3);
		PhysicsMaterial material;
		ColliderIdentifier identifier;
		ConvexHull collider;
		Vec3 com;
		decimal convexRadius = decimalNAN;

		ConvexHullCollider() {}
		ConvexHullCollider(const ConvexHull& convexHull) : collider(convexHull), identifier(ColliderType::convexHull)
		{
			this->bound = this->collider.toAABB();
			this->com = this->collider.getCentroid();
			this->convexRadius = magnitude(this->bound.getCenter() - this->bound.max);
		}

		void transform(const Transform3D& t)
		{
			this->com = t * this->com;
			this->collider.vertices[0] = t * this->collider.vertices[0];

			this->bound.max = this->collider.vertices[0];
			this->bound.min = this->collider.vertices[0];

			for (uint32 i = 1, len = this->collider.vertices.size(); i < len; ++i) {

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

		AABB bound = AABB(nanVEC3, nanVEC3);
		PhysicsMaterial material;
		ColliderIdentifier identifier;
		Sphere collider;

		SphereCollider() {}
		SphereCollider(const Sphere& sphere) : collider(sphere), identifier(ColliderType::sphere)
		{
			this->bound = this->collider.toAABB();
		}

		void transform(const Transform3D& t)
		{
			this->collider.center = t * this->collider.center;
			this->bound = this->collider.toAABB();
		}

		decimal getVolume() { return decimal(4.0) * mathPI * this->collider.radius * this->collider.radius * this->collider.radius / decimal(3.0); } 
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct CapsuleCollider {

		AABB bound = AABB(nanVEC3, nanVEC3);
		PhysicsMaterial material;
		ColliderIdentifier identifier;
		Capsule collider;
		Vec3 com;
		decimal convexRadius = decimalNAN;

		CapsuleCollider() {}
		CapsuleCollider(const Capsule& capsule) : collider(capsule), identifier(ColliderType::capsule)
		{
			this->bound = this->collider.toAABB();
			this->com = this->collider.capsuleLine.getCenter();
			this->convexRadius = magnitude(this->bound.getCenter() - this->bound.max);
		}

		void transform(const Transform3D& t)
		{
			this->collider.pointA = t * this->collider.pointA;
			this->collider.pointB = t * this->collider.pointB;
			this->com = t * this->com;
			this->bound = this->collider.toAABB();
		}

		decimal getVolume() { return mathPI * this->collider.radius * this->collider.radius * this->collider.capsuleLine.getLength() + decimal(4.0) * mathPI * this->collider.radius * this->collider.radius * this->collider.radius / decimal(3.0); } 
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct TriangleMeshCollider {

		AABB bound = AABB(nanVEC3, nanVEC3);
		PhysicsMaterial material;
		ColliderIdentifier identifier;
		TriangleMesh collider;

		TriangleMeshCollider() {}
		TriangleMeshCollider(const TriangleMesh& mesh) : collider(mesh), identifier(ColliderType::triangleMesh)
		{
			this->bound = this->collider.bvh.nodes[this->collider.bvh.parentNode].bound;
		}
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HeightFieldCollider {

		PhysicsMaterial material;
		ColliderIdentifier identifier;
		HeightField collider;

		HeightFieldCollider() : identifier(ColliderType::heightField) {}
	};
}

#endif
