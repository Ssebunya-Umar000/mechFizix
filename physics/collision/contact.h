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

#ifndef CONTACT_H
#define CONTACT_H

#include"collider.h"
#include"../../core/debugRenderer.h"

namespace mech {

#define MAXIMUN_MANIFOLD_CONTACT_POINTS 8

	//////////////////////////////////////////////////////////////////////////////////////////
	enum class CollisionFlag : byte { NOTCOLLIDING = 1 << 0, PROXIMAL = 1 << 1, PENETRATING = 1 << 2};

	//////////////////////////////////////////////////////////////////////////////////////////
	struct HullVsHullContactCache {
		Vec3 center1 = nanVEC3;
		Vec3 center2 = nanVEC3;
		uint32 ID1 = -1;
		uint16 refFace = -1;
		uint16 incidentFace = -1;
		byte cacheFlags = 0;
		byte retention = 0;
	};

	//////////////////////////////////////////////////////////////////////////////////////////
	struct ContactPoint {
		Vec3 position[2];
		Vec3 normal;
		uint32 ID = -1;
	};

	//////////////////////////////////////////////////////////////////////////////////////////
	struct ContactManifold {
		ContactPoint contactPoints[MAXIMUN_MANIFOLD_CONTACT_POINTS] = {};
		PhysicsMaterial material1;
		PhysicsMaterial material2;
		uint32 ID;
		CollisionFlag flag = CollisionFlag::NOTCOLLIDING;
		byte numPoints = 0;

		ContactManifold(const uint32 id) : ID(id) {}

		void addContact(const Vec3& normal, const Vec3& p1, const Vec3& p2, const uint32& id)
		{
			this->contactPoints[this->numPoints].normal = normal;
			this->contactPoints[this->numPoints].position[0] = p1;
			this->contactPoints[this->numPoints].position[1] = p2;
			this->contactPoints[this->numPoints].ID = id;
			++this->numPoints;

			DEBUG_RENDERER_ADD(this->contactPoints[this->numPoints - 1].position[0], BLACK);
			DEBUG_RENDERER_ADD(this->contactPoints[this->numPoints - 1].position[1], WHITE);
			DEBUG_RENDERER_ADD(LineSegment(this->contactPoints[this->numPoints - 1].position[0], this->contactPoints[this->numPoints - 1].position[0] + this->contactPoints[this->numPoints - 1].normal * decimal(0.5)), YELLOW);
		}

		void revert()
		{
			for (byte x = 0; x < this->numPoints; ++x) {
				this->contactPoints[x].normal = -this->contactPoints[x].normal;

				Vec3 temp = this->contactPoints[x].position[0];
				this->contactPoints[x].position[0] = this->contactPoints[x].position[1];
				this->contactPoints[x].position[1] = temp;
			}
		}

		void enforce4Contacts(const Vec3& center1)
		{
			Vec3 faceNormal1;
			{
				Vec3 average;
				for (byte x = 0; x < this->numPoints; ++x) {
					average += this->contactPoints[x].position[0];
				}
				average /= this->numPoints;

				faceNormal1 = average - center1;
			}

			StackArray<byte, 4> points(-1);
			{
				decimal l = -decimalMAX;
				for (byte x = 0; x < this->numPoints; ++x) {
					decimal p = dotProduct(faceNormal1, this->contactPoints[x].position[0] - center1);
					if (p > l) {
						l = p;
						points[0] = x;
					}
				}
			}

			{
				decimal l = -decimalMAX;
				for (byte x = 0; x < this->numPoints; ++x) {

					if (points.find(x) == false) {
						decimal d = magnitudeSq(this->contactPoints[points[0]].position[0] - this->contactPoints[x].position[0]);
						if (d > l) {
							l = d;
							points[1] = x;
						}
					}
				}
			}

			{
				decimal l1 = -decimalMAX;
				decimal l2 = -decimalMAX;
				for (byte x = 0; x < this->numPoints; ++x) {

					if (points.find(x) == false) {
						decimal d = dotProduct(crossProduct(this->contactPoints[points[0]].position[0] - this->contactPoints[x].position[0], this->contactPoints[points[1]].position[0] - this->contactPoints[x].position[0]), faceNormal1);

						if (d < decimal(0.0)) {
							if (d > l1) {
								l1 = d;
								points[2] = x;
							}
						}
						else {
							if (d > l2) {
								l2 = d;
								points[3] = x;
							}
						}
					}
				}
			}

			ContactPoint newPoints[4];
			for (byte x = 0; x < 4; ++x) {
				ASSERT(isAValidIndex(points[x]), "index is invalid");
				newPoints[x] = this->contactPoints[points[x]];
			}

			this->numPoints = 4;
			for (byte x = 0; x < 4; ++x) {
				this->contactPoints[x] = newPoints[x];
			}
		}
	};
}

#endif

