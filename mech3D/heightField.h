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

#ifndef HEIGHTFIELD_H
#define HEIGHTFIELD_H

#include"geometry/triangle.h"
#include"geometry/aabb.h"
#include"../containers/hybridArray.h"

namespace mech {

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class TriangulationMode : byte { none = 0, oneToFour = 1 << 0, twoTothree = 1 << 1 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HeightField {

		struct Parameters {
			decimal* heightData = nullptr;
			decimal gridSize = decimal(0.0);
			decimal xTranslation = decimal(0.0);
			decimal zTranslation = decimal(0.0);
			decimal maxHeight = decimal(0.0);
			uint16 numOfGridsX = 0;
			uint16 numOfGridsZ = 0;
			TriangulationMode triangulationMode = TriangulationMode::none;
		};

		decimal* heightData = nullptr;
		decimal gridSize = decimal(0.0);
		decimal xTranslation = decimal(0.0);
		decimal zTranslation = decimal(0.0);
		decimal maxHeight = decimal(0.0);
		uint16 numOfGridsX = 0;
		uint16 numOfGridsZ = 0;
		TriangulationMode triangulationMode = TriangulationMode::none;

		void initialise(const Parameters& params)
		{
			this->heightData = params.heightData;
			this->gridSize = params.gridSize;
			this->xTranslation = params.xTranslation;
			this->zTranslation = params.zTranslation;
			this->maxHeight = params.maxHeight;
			this->numOfGridsX = params.numOfGridsX;
			this->numOfGridsZ = params.numOfGridsZ;
			this->triangulationMode = params.triangulationMode;
		}

		bool intersects(const AABB& aabb, HybridArray<Triangle, 24, uint16>& triangles) const
		{
			assert(this->heightData != nullptr);

			bool hit = false;

			uint32 minX = -1;
			int32 maxX = -1;
			uint32 minZ = -1;
			int32 maxZ = -1;

			Vec3 points[4] = { aabb.min, aabb.max, Vec3(aabb.max.x, aabb.min.y, aabb.min.z), Vec3(aabb.min.x, aabb.max.y, aabb.max.z) };
			for (byte i = 0; i < 4; ++i) {

				decimal heightFieldX = points[i].x - this->xTranslation;
				decimal heightFieldZ = points[i].z - this->zTranslation;

				int32 gridX = int(heightFieldX / this->gridSize);
				int32 gridZ = int(heightFieldZ / this->gridSize);

				if (gridX < 0 || gridX > this->numOfGridsX || gridZ < 0 || gridZ > this->numOfGridsZ) continue;

				minX = gridX < minX ? gridX : minX;
				maxX = gridX > maxX ? gridX : maxX;

				minZ = gridZ < minZ ? gridZ : minZ;
				maxZ = gridZ > maxZ ? gridZ : maxZ;
			}

			if (maxX < 0) return hit;

			for (uint32 z = minZ; z <= maxZ; ++z) {

				decimal zCord1 = z * this->gridSize + this->zTranslation;
				decimal zCord2 = zCord1 + this->gridSize;

				for (uint32 x = minX; x <= maxX; ++x) {

					decimal xCord1 = x * this->gridSize + this->xTranslation;
					decimal xCord2 = xCord1 + this->gridSize;

					Vec3 a1 = Vec3(xCord1, this->heightData[(z *       this->numOfGridsZ) +   x     ], zCord1);
					Vec3 a2 = Vec3(xCord2, this->heightData[(z *       this->numOfGridsZ) +  (x + 1)], zCord1);
					Vec3 a3 = Vec3(xCord1, this->heightData[((z + 1) * this->numOfGridsZ) +   x     ], zCord2);
					Vec3 a4 = Vec3(xCord2, this->heightData[((z + 1) * this->numOfGridsZ) +  (x + 1)], zCord2);

					Triangle t[2];
					if (this->triangulationMode == TriangulationMode::oneToFour) {
						t[0] = Triangle(a1, a3, a4);
						t[1] = Triangle(a4, a2, a1);
					}
					else {
						t[0] = Triangle(a1, a2, a3);
						t[1] = Triangle(a3, a2, a4);
					}

					for (byte i = 0; i < 2; ++i) {
						if (aabb.intersects(t[i])) {
							triangles.pushBack(t[i]);
							hit = true;
						}
					}
				}
			}

			return hit;
		}

		/*decimal getHeight(const decimal& x, const decimal& z) const
		{
			assert(this->heightData != nullptr);

			decimal heightFieldX = x - this->xTranslation;
			decimal heightFieldZ = z - this->zTranslation;

			int32 gridX = int(heightFieldX / this->gridSize);
			int32 gridZ = int(heightFieldZ / this->gridSize);

			if (gridX < 0 || gridX > this->numOfGridsX || gridZ < 0 || gridZ > this->numOfGridsZ) return decimalNAN;

			gridX = gridX == this->numOfGridsX ? gridX - 1 : gridX;
			gridZ = gridZ == this->numOfGridsZ ? gridZ - 1 : gridZ;

			decimal xpos = heightFieldX - gridX * this->gridSize;
			decimal zpos = heightFieldZ - gridZ * this->gridSize;

			Vec3 p1, p2, p3;
			if (this->triangulationMode == TriangulationMode::oneToFour) {

				if (xpos / this->gridSize >= (1 - zpos / this->gridSize)) {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +  (gridX + 1)], this->gridSize);
				}
				else {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +  (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +   gridX     ], decimal(0.0));
				}
			}
			else {

				if (xpos / this->gridSize <= (1 - zpos / this->gridSize)) {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +   gridX     ], decimal(0.0));
				}
				else {
					p1 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +   gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfGridsZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfGridsZ) +  (gridX + 1)], this->gridSize);
				}
			}

			decimal det = (p2.z - p3.z) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.z - p3.z);

			decimal l1 = ((p2.z - p3.z) * (xpos - p3.x) + (p3.x - p2.x) * (zpos - p3.z)) / det;
			decimal l2 = ((p3.z - p1.z) * (xpos - p3.x) + (p1.x - p3.x) * (zpos - p3.z)) / det;
			decimal l3 = decimal(1.0) - l1 - l2;

			return l1 * p1.y + l2 * p2.y + l3 * p3.y;
		}*/
	};
}

#endif
