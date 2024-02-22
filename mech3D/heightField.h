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

	/*
		--------------heightFeild cell-----------------
		(1)------(3)
		 |        |
		 |        |
		 |        |
		(2)------(4)
		
		the data passed to the heightField is a one dimensional array of height values for each of the points
		in the heightField, the data is assumed to be in the order below,,,,,,,,,,
											zxxxxxxxxx
											z
											z
											z

		TriangleDiagonalMode::oneToFour - diagonal connects point 1 to point 4
		TriangleDiagonalMode::twoTothree - diagonal connects point 2 to point 2
	*/
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	enum class TriangleDiagonalMode : byte { none = 0, oneToFour = 1 << 0, twoTothree = 1 << 1 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HeightField {

		struct Parameters {
			decimal* heightData = nullptr;
			decimal gridSize = decimal(0.0);
			decimal xTranslation = decimal(0.0);
			decimal zTranslation = decimal(0.0);
			decimal maxHeight = decimal(0.0);
			uint16 numOfCellsAlongX = 0;
			uint16 numOfCellsAlongZ = 0;
			TriangleDiagonalMode diagonalMode = TriangleDiagonalMode::none;
		};

		decimal* heightData = nullptr;
		decimal gridSize = decimal(0.0);
		decimal xTranslation = decimal(0.0);
		decimal zTranslation = decimal(0.0);
		decimal maxHeight = decimal(0.0);
		uint16 numOfCellsAlongX = 0;
		uint16 numOfCellsAlongZ = 0;
		TriangleDiagonalMode diagonalMode = TriangleDiagonalMode::none;

		void initialise(const Parameters& params)
		{
			this->heightData = params.heightData;
			this->gridSize = params.gridSize;
			this->xTranslation = params.xTranslation;
			this->zTranslation = params.zTranslation;
			this->maxHeight = params.maxHeight;
			this->numOfCellsAlongX = params.numOfCellsAlongX;
			this->numOfCellsAlongZ = params.numOfCellsAlongZ;
			this->diagonalMode = params.diagonalMode;
		}

		bool intersects(const AABB& aabb, HybridArray<Triangle, 24, uint16>& triangles) const
		{
			assert(this->heightData != nullptr);

			bool hit = false;

			decimal minX = decimalMAX;
			decimal maxX = -decimalMAX;
			decimal minZ = decimalMAX;
			decimal maxZ = -decimalMAX;

			Vec3 points[4] = { aabb.min, aabb.max, Vec3(aabb.max.x, aabb.min.y, aabb.min.z), Vec3(aabb.min.x, aabb.max.y, aabb.max.z) };
			for (byte i = 0; i < 4; ++i) {

				int32 gridX = int32((points[i].x - this->xTranslation) / this->gridSize);
				int32 gridZ = int32((points[i].z - this->zTranslation) / this->gridSize);

				if (gridX < 0 || gridX > this->numOfCellsAlongX || gridZ < 0 || gridZ > this->numOfCellsAlongZ) continue;

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

					Vec3 a1 = Vec3(xCord1, this->heightData[(z *       this->numOfCellsAlongZ) + x],       zCord1);
					Vec3 a2 = Vec3(xCord1, this->heightData[((z + 1) * this->numOfCellsAlongZ) + x],       zCord2);
					Vec3 a3 = Vec3(xCord2, this->heightData[(z *       this->numOfCellsAlongZ) + (x + 1)], zCord1);
					Vec3 a4 = Vec3(xCord2, this->heightData[((z + 1) * this->numOfCellsAlongZ) + (x + 1)], zCord2);

					Triangle t[2];
					if (this->diagonalMode == TriangleDiagonalMode::oneToFour) {
						t[0] = Triangle(a1, a2, a4);
						t[1] = Triangle(a4, a3, a1);
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

			if (gridX < 0 || gridX > this->numOfCellsAlongX || gridZ < 0 || gridZ > this->numOfCellsAlongZ) return decimalNAN;

			gridX = gridX == this->numOfCellsAlongX ? gridX - 1 : gridX;
			gridZ = gridZ == this->numOfCellsAlongZ ? gridZ - 1 : gridZ;

			decimal xpos = heightFieldX - gridX * this->gridSize;
			decimal zpos = heightFieldZ - gridZ * this->gridSize;

			Vec3 p1, p2, p3;
			if (this->diagonalMode == TriangleDiagonalMode::oneToFour) {

				if (xpos / this->gridSize >= (1 - zpos / this->gridSize)) {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +  (gridX + 1)], this->gridSize);
				}
				else {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +  (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +   gridX     ], decimal(0.0));
				}
			}
			else {

				if (xpos / this->gridSize <= (1 - zpos / this->gridSize)) {
					p1 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +         gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +   gridX     ], decimal(0.0));
				}
				else {
					p1 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +   gridX     ], decimal(0.0));
					p2 = Vec3(decimal(0.0),   this->heightData[(gridZ * this->numOfCellsAlongZ) +        (gridX + 1)], this->gridSize);
					p3 = Vec3(this->gridSize, this->heightData[((gridZ + 1) * this->numOfCellsAlongZ) +  (gridX + 1)], this->gridSize);
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
