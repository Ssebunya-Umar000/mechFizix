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

#ifndef HEIGHTFIELD_H
#define HEIGHTFIELD_H

#include"../geometry/triangle.h"
#include"../geometry/aabb.h"
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
	enum class HeightFieldType : byte { none = 0, flat = 1 << 0, bumpy = 1 << 1 };

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct FlatTerrainParameters {
		Vec2 min;
		Vec2 max;
		float height = 0.0;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct BumpyTerrainParameters {
		float* heightData = nullptr;
		float gridSize = 0.0;
		float drift = 0.0;
		uint16 numOfCellsAlongXandZ = 0;
		TriangleDiagonalMode diagonalMode = TriangleDiagonalMode::none;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	struct HeightField {

		BumpyTerrainParameters* bumpy = nullptr;
		FlatTerrainParameters* flat = nullptr;
		HeightFieldType heightFieldType = HeightFieldType::none;

		void initialise(BumpyTerrainParameters* params)
		{
			this->bumpy = params;
			this->heightFieldType = HeightFieldType::bumpy;
		}

		void initialise(FlatTerrainParameters* params)
		{
			this->flat = params;
			this->heightFieldType = HeightFieldType::flat;
		}

		void gridIndicies(const AABB& aabb, float& minX, float& maxX, float& minZ, float& maxZ) const
		{
			minX = FLT_MAX;
			maxX = -FLT_MAX;
			minZ = FLT_MAX;
			maxZ = -FLT_MAX;

			Vec3 points[2] = { aabb.min, aabb.max };
			for (byte i = 0; i < 2; ++i) {

				float gridX = (points[i].x - this->bumpy->drift) / this->bumpy->gridSize;
				minX = gridX < minX ? gridX : minX;
				maxX = gridX > maxX ? gridX : maxX;

				float gridZ = (points[i].z - this->bumpy->drift) / this->bumpy->gridSize;
				minZ = gridZ < minZ ? gridZ : minZ;
				maxZ = gridZ > maxZ ? gridZ : maxZ;
			}

			minX = mathMAX(minX, 0.0);
			minZ = mathMAX(minZ, 0.0);
			maxX = mathMIN(maxX, this->bumpy->numOfCellsAlongXandZ - 1);
			maxZ = mathMIN(maxZ, this->bumpy->numOfCellsAlongXandZ - 1);
		}

		bool intersects(const AABB& aabb) const
		{
			if (this->heightFieldType == HeightFieldType::bumpy) {
				
				float minX, maxX, minZ, maxZ;
				gridIndicies(aabb, minX, maxX, minZ, maxZ);
				if (maxX < 0) return false;

				float maxHeight = -FLT_MAX;
				float minHeight = FLT_MAX;

				for (uint32 z = minZ; z <= maxZ; ++z) {
					for (uint32 x = minX; x <= maxX; ++x) {

						float h[4];
						h[0] = this->bumpy->heightData[(z * this->bumpy->numOfCellsAlongXandZ) + x];
						h[1] = this->bumpy->heightData[((z + 1) * this->bumpy->numOfCellsAlongXandZ) + x];
						h[2] = this->bumpy->heightData[(z * this->bumpy->numOfCellsAlongXandZ) + (x + 1)];
						h[3] = this->bumpy->heightData[((z + 1) * this->bumpy->numOfCellsAlongXandZ) + (x + 1)];

						for (byte i = 0; i < 4; ++i) {
							maxHeight = mathMAX(h[i], maxHeight);
							minHeight = mathMIN(h[i], minHeight);
						}
					}
				}

				return (aabb.min.y <= maxHeight && aabb.min.y >= minHeight) ||
					(aabb.max.y <= maxHeight && aabb.max.y >= minHeight) ||
					(aabb.max.y >= maxHeight && aabb.min.y <= minHeight);
			}
			else if (this->heightFieldType == HeightFieldType::flat) {

				return (aabb.min.y <= this->flat->height && aabb.min.y >= this->flat->height) ||
					(aabb.max.y <= this->flat->height && aabb.max.y >= this->flat->height) ||
					(aabb.max.y >= this->flat->height && aabb.min.y <= this->flat->height);
			}

			ASSERT(false, "heightFeild was not initialised");
			return false;
		}

		void getTrianglesOverlapped(const AABB& aabb, HybridArray<Triangle, 24, uint16>& triangles) const
		{
			if (this->heightFieldType == HeightFieldType::bumpy) {

				float minX, maxX, minZ, maxZ;
				gridIndicies(aabb, minX, maxX, minZ, maxZ);
				if (maxX < 0) return;

				for (uint32 z = minZ; z <= maxZ; ++z) {

					float zCord1 = z * this->bumpy->gridSize + this->bumpy->drift;
					float zCord2 = zCord1 + this->bumpy->gridSize;

					for (uint32 x = minX; x <= maxX; ++x) {

						float xCord1 = x * this->bumpy->gridSize + this->bumpy->drift;
						float xCord2 = xCord1 + this->bumpy->gridSize;

						Vec3 a1 = Vec3(xCord1, this->bumpy->heightData[(z * this->bumpy->numOfCellsAlongXandZ) + x], zCord1);
						Vec3 a2 = Vec3(xCord1, this->bumpy->heightData[((z + 1) * this->bumpy->numOfCellsAlongXandZ) + x], zCord2);
						Vec3 a3 = Vec3(xCord2, this->bumpy->heightData[(z * this->bumpy->numOfCellsAlongXandZ) + (x + 1)], zCord1);
						Vec3 a4 = Vec3(xCord2, this->bumpy->heightData[((z + 1) * this->bumpy->numOfCellsAlongXandZ) + (x + 1)], zCord2);

						Triangle t[2];
						if (this->bumpy->diagonalMode == TriangleDiagonalMode::oneToFour) {
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
							}
						}
					}
				}
			}
			else if (this->heightFieldType == HeightFieldType::flat) {

				Triangle t[2] = {Triangle(Vec3(this->flat->min.x, this->flat->height, this->flat->min.y), Vec3(this->flat->min.x, this->flat->height, this->flat->max.y), Vec3(this->flat->max.x, this->flat->height, this->flat->max.y)), Triangle(Vec3(this->flat->min.x, this->flat->height, this->flat->min.y), Vec3(this->flat->max.x, this->flat->height, this->flat->max.y), Vec3(this->flat->max.x, this->flat->height, this->flat->min.y))};
				for (byte i = 0; i < 2; ++i) {
					if (aabb.intersects(t[i])) {
						triangles.pushBack(t[i]);
					}
				}
			}

			ASSERT(false, "heightFeild was not initialised");
		}

		float getHeight(const float& x, const float& z) const
		{
			if (this->heightFieldType == HeightFieldType::bumpy) {
				
				float heightFieldX = x - this->bumpy->drift;
				float heightFieldZ = z - this->bumpy->drift;

				int32 gridX = int(heightFieldX / this->bumpy->gridSize);
				int32 gridZ = int(heightFieldZ / this->bumpy->gridSize);

				if (gridX < 0 || gridX > this->bumpy->numOfCellsAlongXandZ || gridZ < 0 || gridZ > this->bumpy->numOfCellsAlongXandZ) return decimalNAN;

				gridX = gridX == this->bumpy->numOfCellsAlongXandZ ? gridX - 1 : gridX;
				gridZ = gridZ == this->bumpy->numOfCellsAlongXandZ ? gridZ - 1 : gridZ;

				float xpos = heightFieldX - gridX * this->bumpy->gridSize;
				float zpos = heightFieldZ - gridZ * this->bumpy->gridSize;

				Vec3 p1, p2, p3;
				if (this->bumpy->diagonalMode == TriangleDiagonalMode::oneToFour) {

					if (xpos / this->bumpy->gridSize >= (1 - zpos / this->bumpy->gridSize)) {
						p1 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
						p2 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
						p3 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
					}
					else {
						p1 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
						p2 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
						p3 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
					}
				}
				else {

					if (xpos / this->bumpy->gridSize <= (1 - zpos / this->bumpy->gridSize)) {
						p1 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
						p2 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
						p3 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
					}
					else {
						p1 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + gridX], float(0.0));
						p2 = Vec3(float(0.0), this->bumpy->heightData[(gridZ * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
						p3 = Vec3(this->bumpy->gridSize, this->bumpy->heightData[((gridZ + 1) * this->bumpy->numOfCellsAlongXandZ) + (gridX + 1)], this->bumpy->gridSize);
					}
				}

				float det = (p2.z - p3.z) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.z - p3.z);

				float l1 = ((p2.z - p3.z) * (xpos - p3.x) + (p3.x - p2.x) * (zpos - p3.z)) / det;
				float l2 = ((p3.z - p1.z) * (xpos - p3.x) + (p1.x - p3.x) * (zpos - p3.z)) / det;
				float l3 = float(1.0) - l1 - l2;

				return l1 * p1.y + l2 * p2.y + l3 * p3.y;
			}
			else if (this->heightFieldType == HeightFieldType::flat) {
				return this->flat->height;
			}

			ASSERT(false, "heightFeild was not initialised");
		}
	};
}

#endif
